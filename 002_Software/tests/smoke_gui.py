"""Open the real viewer with test frames, check failure states, then close it."""

from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np
import open3d as o3d

from main import Viewer
from protocol import FrameParser
from serial_link import ReceiverState
from samples import wire_frame


class TestReader:
    def __init__(self):
        self.state = ReceiverState(None, 0, 0, 0, "")
        self.closed = False

    def snapshot(self):
        return self.state

    def close(self):
        self.closed = True


def run():
    reader = TestReader()
    viewer = Viewer(reader, "TEST DATA")

    def pump():
        deadline = time.monotonic() + 0.4
        while time.monotonic() < deadline:
            assert not viewer.closed, "window closed unexpectedly"
            viewer.root.update()
            time.sleep(0.01)

    def show(data, age=0, error=""):
        frame = FrameParser().feed(data)[0]
        frame.received_at = time.monotonic() - age
        reader.state = ReceiverState(frame, reader.state.frames + 1, 0, 5, error)
        pump()

    try:
        pump()
        assert "Waiting" in viewer.status.get()
        show(wire_frame())  # A flat first frame must not fit the camera to just the axes.
        distances = np.full((8, 8), 400)
        distances[2:6, 2:6] = 250
        distances[3:5, 3:5] = 150
        show(wire_frame(distance=distances))
        assert viewer._has_cloud and viewer.cells[27].cget("text") == "150"
        assert "Valid 64/64" in viewer.status.get()
        assert len(viewer.cloud.points) == 64
        camera = viewer.visualizer.get_view_control().convert_to_pinhole_camera_parameters()
        points = np.asarray(viewer.cloud.points)
        projected = camera.intrinsic.intrinsic_matrix @ (camera.extrinsic @ np.c_[points, np.ones(len(points))].T)[:3]
        screen = projected[:2] / projected[2]
        assert (screen >= 0).all() and (screen[0] < 640).all() and (screen[1] < 480).all(), "initial view clips points"
        assert viewer.grid.winfo_rootx() + viewer.grid.winfo_width() <= viewer.canvas.winfo_rootx()
        artifact_dir = Path(__file__).resolve().parents[1] / "artifacts"
        artifact_dir.mkdir(exist_ok=True)
        path = artifact_dir / "gui_depth_preview.png"
        viewer.image.write(str(path), format="png")
        image = np.asarray(o3d.io.read_image(str(path))).astype(float)
        assert image.std() > 5, "3D renderer produced an empty image"
        assert np.count_nonzero((image[:, :, 0] > 100) & (image[:, :, 0] > image[:, :, 1] * 1.3)) > 1000
        control = viewer.visualizer.get_view_control()
        before = control.convert_to_pinhole_camera_parameters().extrinsic.copy()
        viewer.canvas.event_generate("<ButtonPress-1>", x=300, y=200)
        viewer.canvas.event_generate("<B1-Motion>", x=350, y=230)
        pump()
        rotated = control.convert_to_pinhole_camera_parameters().extrinsic.copy()
        assert not np.allclose(before, rotated), "drag did not rotate the camera"
        viewer.canvas.event_generate("<MouseWheel>", delta=120)
        pump()
        assert not np.allclose(rotated, control.convert_to_pinhole_camera_parameters().extrinsic), "wheel did not zoom"
        show(wire_frame(status=np.full((8, 8), 255), targets=np.zeros((8, 8))))
        assert not viewer._has_cloud and all(label.cget("text") == "--" for label in viewer.cells)
        status = np.full((8, 8), 255)
        status[0, 0] = 9
        targets = np.zeros((8, 8))
        targets[0, 0] = 1
        show(wire_frame(status=status, targets=targets))
        assert viewer._has_cloud and "Valid 1/64" in viewer.status.get()
        assert len(viewer.cloud.points) == 1
        show(wire_frame(), age=3)
        assert not viewer._has_cloud and "timeout" in viewer.status.get()
        show(wire_frame(sequence=0))
        assert viewer._has_cloud and "counter 0" in viewer.status.get()
        for sequence in range(30):
            frame = FrameParser().feed(wire_frame(sequence, distance=distances + sequence))[0]
            reader.state = ReceiverState(frame, reader.state.frames + 1, 0, 5, "")
            deadline = time.monotonic() + 0.2
            while time.monotonic() < deadline:
                viewer.root.update()
                time.sleep(0.01)
        assert viewer._has_cloud and len(viewer.cloud.points) == 64
        assert viewer.cells[27].cget("text") == "179"
        show(wire_frame(), error="test unplug")
        assert not viewer._has_cloud and "Disconnected" in viewer.status.get()
        print("GUI PASS: grid, 3D readback, orbit, zoom, empty, partial, timeout, recovery, 30 updates, disconnect")
    finally:
        viewer.root.tk.call(viewer.root.protocol("WM_DELETE_WINDOW"))
        viewer.close()  # Closing twice must also be harmless during main() cleanup.
        assert reader.closed, "close callback did not release the receiver"


if __name__ == "__main__":
    run()
