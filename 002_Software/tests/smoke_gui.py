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

    def pump(duration=0.4):
        deadline = time.monotonic() + duration
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
        assert viewer.capture_button.winfo_viewable(), "ground capture button is not visible"
        assert "Waiting" in viewer.status.get()
        boundary = np.full((8, 8), 1001)
        boundary[0, 0] = 1000
        show(wire_frame(distance=boundary))
        assert viewer.cells[0].cget("text") == "1000"
        assert all(label.cget("text") == "--" for label in viewer.cells[1:])
        assert len(viewer.cloud.points) == 1 and "Valid 1/64 (0-1000 mm)" in viewer.status.get()
        viewer._camera_ready = False
        show(wire_frame())  # A flat first frame must not fit the camera to just the axes.
        distances = np.full((8, 8), 400)
        distances[2:6, 2:6] = 250
        distances[3:5, 3:5] = 150
        show(wire_frame(distance=distances))
        assert viewer._has_cloud and viewer.cells[27].cget("text") == "150"
        assert "Valid 64/64" in viewer.status.get()
        assert len(viewer.cloud.points) == 64
        assert viewer._has_surface and len(viewer.surface.triangles) == 98
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
        viewer.capture_button.invoke()
        ground = np.arange(64).reshape(8, 8) + 600
        for sequence in range(30):
            frame = FrameParser().feed(wire_frame(sequence, distance=ground + (sequence % 3 - 1)))[0]
            reader.state = ReceiverState(frame, reader.state.frames + 1, 0, 5, "")
            deadline = time.monotonic() + 0.4
            while viewer.sampler is not None and viewer.sampler.count <= sequence and time.monotonic() < deadline:
                viewer.root.update()
                time.sleep(0.01)
        pump(0.1)
        assert viewer.sampler is None and viewer.ground_mm is not None, (viewer.sampler.count if viewer.sampler else None, viewer.status.get())
        np.testing.assert_allclose(viewer.ground_mm, ground)
        assert viewer.relative_mode.get() and "RELATIVE RISE" in viewer.value_title.get()
        current = ground.copy()
        current[2:6, 2:6] -= 100
        current[3:5, 3:5] -= 50
        current[0, 0] += 50
        show(wire_frame(distance=current))
        assert viewer.cells[34].cget("text") == "100"
        assert viewer.cells[0].cget("text") == "-50"
        assert viewer._has_surface and len(viewer.surface.triangles) == 98
        np.testing.assert_allclose(np.asarray(viewer.surface.vertices), np.asarray(viewer.cloud.points))
        camera = control.convert_to_pinhole_camera_parameters()
        points = np.asarray(viewer.surface.vertices)
        projected = camera.intrinsic.intrinsic_matrix @ (camera.extrinsic @ np.c_[points, np.ones(len(points))].T)[:3]
        screen = projected[:2] / projected[2]
        assert (screen >= 0).all() and (screen[0] < 640).all() and (screen[1] < 480).all(), "relative surface clips points"
        viewer.image.write(str(artifact_dir / "gui_surface_preview.png"), format="png")
        viewer.relative_mode.set(False)
        viewer._mode_changed()
        assert viewer.cells[34].cget("text") == str(current[4, 2])
        viewer.relative_mode.set(True)
        viewer._mode_changed()
        assert viewer.cells[34].cget("text") == "100"
        previous_reference = viewer.ground_mm.copy()
        viewer.capture_button.invoke()
        viewer._last_sample_at = time.monotonic() - 3
        pump(0.1)
        assert viewer.sampler is None and "2 seconds" in viewer.status.get()
        np.testing.assert_allclose(viewer.ground_mm, previous_reference)
        viewer.capture_button.invoke()
        viewer._sampling_started -= 16
        pump(0.1)
        assert viewer.sampler is None and "15 second" in viewer.status.get()
        np.testing.assert_allclose(viewer.ground_mm, previous_reference)
        show(wire_frame(status=np.full((8, 8), 255), targets=np.zeros((8, 8))))
        assert not viewer._has_cloud and all(label.cget("text") == "--" for label in viewer.cells)
        assert not viewer._has_surface, "invalid frame left an old surface visible"
        status = np.full((8, 8), 255)
        status[0, 0] = 9
        targets = np.zeros((8, 8))
        targets[0, 0] = 1
        show(wire_frame(status=status, targets=targets))
        assert viewer._has_cloud and "Valid 1/64" in viewer.status.get()
        assert len(viewer.cloud.points) == 1
        assert not viewer._has_surface, "isolated point must not produce a surface"
        show(wire_frame(), age=3)
        assert not viewer._has_cloud and "timeout" in viewer.status.get()
        assert not viewer._has_surface
        show(wire_frame(sequence=0))
        assert viewer._has_cloud and "counter 0" in viewer.status.get()
        viewer.relative_mode.set(False)
        viewer._mode_changed()
        for sequence in range(30):
            frame = FrameParser().feed(wire_frame(sequence, distance=distances + sequence))[0]
            reader.state = ReceiverState(frame, reader.state.frames + 1, 0, 5, "")
            deadline = time.monotonic() + 0.2
            while time.monotonic() < deadline:
                viewer.root.update()
                time.sleep(0.01)
        assert viewer._has_cloud and len(viewer.cloud.points) == 64
        assert viewer.cells[27].cget("text") == "179"
        previous_reference = viewer.ground_mm.copy()
        viewer.capture_button.invoke()
        show(wire_frame(), error="test unplug")
        np.testing.assert_allclose(viewer.ground_mm, previous_reference)
        assert not viewer._has_cloud and "Disconnected" in viewer.status.get()
        assert not viewer._has_surface
        print("GUI PASS: grid, colored surface, 3D readback, orbit, zoom, ground capture, relative/raw modes, empty, partial, timeout, recovery, disconnect")
    finally:
        viewer.root.tk.call(viewer.root.protocol("WM_DELETE_WINDOW"))
        viewer.close()  # Closing twice must also be harmless during main() cleanup.
        assert reader.closed, "close callback did not release the receiver"


if __name__ == "__main__":
    run()
