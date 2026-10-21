"""Live 8x8 distance grid and region-index depth preview."""

import argparse
import json
import sys
import time
import tkinter as tk

import numpy as np
import serial

from serial_link import SerialReader
from ground_reference import GroundSampler
from protocol import MAX_DISTANCE_MM


def display_data(frame, ground_mm=None):
    if ground_mm is None:
        return frame.distance_mm.astype(float), frame.valid
    valid = frame.valid & np.isfinite(ground_mm)
    return ground_mm - frame.distance_mm, valid


def preview_data(frame, ground_mm=None):
    """Keep the firmware's row/column order; X/Y are NOT physical dimensions."""
    values, valid = display_data(frame, ground_mm)
    if ground_mm is None:
        t = np.clip(values / MAX_DISTANCE_MM, 0, 1)
        near, far = np.array([0.95, 0.48, 0.20]), np.array([0.12, 0.38, 0.68])
        colors = near + t[..., None] * (far - near)
        z = -values / 100
    else:
        t = np.clip(values / 200.0, 0, 1)
        blue, orange = np.array([0.12, 0.38, 0.68]), np.array([0.95, 0.48, 0.20])
        colors = blue + t[..., None] * (orange - blue)
        z = values / 100
    colors[~valid] = [0.22, 0.24, 0.27]
    rows, cols = np.indices((8, 8))
    # shortcut: region-index geometry; replace only when physical projection is specified.
    points = np.stack((cols - 3.5, 3.5 - rows, z), axis=-1)
    return points[valid], colors[valid], colors


def surface_triangles(valid):
    """Join neighboring measurements; indices refer to preview_data's valid points."""
    indices = np.full(valid.shape, -1, dtype=np.int32)
    indices[valid] = np.arange(np.count_nonzero(valid))
    triangles = []
    # shortcut: straight joins only approximate edges; finer shapes need more measurements.
    for row in range(7):
        for col in range(7):
            a, b = indices[row, col:col + 2]
            c, d = indices[row + 1, col:col + 2]
            for face in ((a, c, b), (b, c, d)):
                if min(face) >= 0:
                    triangles.append(face)
    return np.asarray(triangles, dtype=np.int32).reshape(-1, 3)


class Viewer:
    def __init__(self, reader, port):
        import open3d as o3d

        self.o3d = o3d
        self.reader = reader
        self.port = port
        self.started = time.monotonic()
        self._last_frame = None
        self._last_status = None
        self.ground_mm = None
        self.sampler = None
        self._sampling_started = None
        self._last_sample_at = None
        self._capture_previous_frame = None
        self._calibration_note = ""
        self._has_cloud = False
        self._has_surface = False
        self._camera_ready = False
        self._dirty = True
        self._drag = None
        self._job = None
        self.closed = False
        self.root = tk.Tk()
        self.root.title("SpinDepth3D | Live distance")
        self.root.geometry("1180x720")
        self.root.minsize(1120, 660)
        self.root.configure(background="#141a22")
        self.root.option_add("*Font", "{Segoe UI} 11")
        self.relative_mode = tk.BooleanVar(master=self.root, value=False)

        # Filament crashes on this AMD driver; keep all rendering on the legacy path.
        self.visualizer = o3d.visualization.Visualizer()
        if not self.visualizer.create_window("SpinDepth3D renderer", 640, 480, visible=False):
            self.root.destroy()
            raise RuntimeError("Cannot create the Open3D legacy rendering context")
        option = self.visualizer.get_render_option()
        option.background_color = np.array([0.065, 0.08, 0.10])
        option.point_size = 6
        option.mesh_show_back_face = True
        option.mesh_show_wireframe = True
        self.cloud = o3d.geometry.PointCloud()
        self.surface = o3d.geometry.TriangleMesh()
        self.axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1)
        self.visualizer.add_geometry(self.axes)

        self.status = tk.StringVar()
        tk.Label(self.root, textvariable=self.status, background="#141a22", foreground="#d4dce5",
                 anchor="w", justify="left", padx=16, pady=12).pack(side="bottom", fill="x")
        toolbar = tk.Frame(self.root, background="#141a22")
        toolbar.pack(side="top", fill="x", padx=16, pady=(12, 0))
        self.capture_button = tk.Button(toolbar, text="采集地面", command=self._start_ground_capture)
        self.capture_button.pack(side="left")
        tk.Checkbutton(toolbar, text="显示相对起伏", variable=self.relative_mode,
                       command=self._mode_changed, background="#141a22", foreground="white",
                       activebackground="#141a22", activeforeground="white",
                       selectcolor="#141a22", disabledforeground="#7c8794").pack(side="left", padx=12)
        content = tk.Frame(self.root, background="#141a22")
        content.pack(fill="both", expand=True, padx=16, pady=(8, 16))
        left = tk.Frame(content, background="#141a22")
        left.pack(side="left", fill="y", padx=(0, 24))
        right = tk.Frame(content, background="#141a22")
        right.pack(side="left", fill="both", expand=True)
        self.value_title = tk.StringVar(value="DISTANCE / mm")
        tk.Label(left, textvariable=self.value_title, background="#141a22", foreground="white",
                 font=("Segoe UI", 15, "bold"), anchor="w").pack(fill="x", pady=(0, 12))
        tk.Label(right, text="SURFACE PREVIEW / 8 x 8", background="#141a22", foreground="white",
                 font=("Segoe UI", 15, "bold"), anchor="w").pack(fill="x", pady=(0, 12))
        tk.Label(left, text="Firmware order\nRows 0-7: top to bottom\nColumns 0-7: left to right",
                 background="#141a22", foreground="#a8b8c8", justify="left").pack(anchor="w", pady=(0, 16))
        self.grid = tk.Frame(left, background="#141a22")
        self.grid.pack(anchor="w")
        self.cells = []
        for i in range(64):
            label = tk.Label(self.grid, text="--", width=5, padx=2, pady=10)
            label.grid(row=i // 8, column=i % 8, padx=1, pady=1, sticky="nsew")
            self.cells.append(label)
        self.legend = tk.StringVar(value=f"Near 0 mm: orange\nFar {MAX_DISTANCE_MM} mm: blue\nGray / -- : invalid or outside range")
        tk.Label(left, textvariable=self.legend, background="#141a22", foreground="#a8b8c8",
                 justify="left").pack(anchor="w", pady=16)
        self.canvas = tk.Canvas(right, width=640, height=480, background="#11141a", highlightthickness=0)
        self.canvas.pack(expand=True)
        self._image_item = self.canvas.create_image(0, 0, anchor="nw")
        self.image = None
        self.preview_note = tk.StringVar(value="X/Y: region indices | Z = -distance / 100\nApproximate surface | Drag: orbit | Wheel: zoom")
        tk.Label(right, textvariable=self.preview_note,
                 background="#141a22", foreground="#a8b8c8", justify="left").pack(anchor="w", pady=(12, 0))
        self.canvas.bind("<ButtonPress-1>", self._start_drag)
        self.canvas.bind("<B1-Motion>", self._rotate)
        self.canvas.bind("<MouseWheel>", self._zoom)
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self._show_frame(None)
        self._job = self.root.after(30, self._tick)

    def _show_frame(self, frame, reset_camera=False):
        if reset_camera:
            self._camera_ready = False
        if self._has_cloud:
            self.visualizer.remove_geometry(self.cloud, reset_bounding_box=False)
            self._has_cloud = False
        if self._has_surface:
            self.visualizer.remove_geometry(self.surface, reset_bounding_box=False)
            self._has_surface = False
        if frame is None:
            colors = np.full((8, 8, 3), 0.24)
            for label in self.cells:
                label.configure(text="--")
        else:
            ground = self.ground_mm if self.relative_mode.get() else None
            values, valid = display_data(frame, ground)
            points, point_colors, colors = preview_data(frame, ground)
            for i, label in enumerate(self.cells):
                row, col = divmod(i, 8)
                label.configure(text=str(int(np.rint(values[row, col]))) if valid[row, col] else "--")
            if len(points):
                self.cloud.points = self.o3d.utility.Vector3dVector(points)
                self.cloud.colors = self.o3d.utility.Vector3dVector(point_colors)
                self.visualizer.add_geometry(self.cloud, reset_bounding_box=not self._camera_ready)
                self._has_cloud = True
                triangles = surface_triangles(valid)
                if len(triangles):
                    self.surface.vertices = self.o3d.utility.Vector3dVector(points)
                    self.surface.vertex_colors = self.o3d.utility.Vector3dVector(point_colors)
                    self.surface.triangles = self.o3d.utility.Vector3iVector(triangles)
                    self.surface.compute_vertex_normals()
                    self.visualizer.add_geometry(self.surface, reset_bounding_box=False)
                    self._has_surface = True
                if not self._camera_ready:
                    front = np.array([0.5, -0.7, 0.7])
                    front /= np.linalg.norm(front)
                    up = np.cross(front, np.cross([0, 0, 1], front))
                    up /= np.linalg.norm(up)
                    # Legacy bounds ignore flat/single-point clouds when merged with the axes.
                    view = json.loads(self.visualizer.get_view_status())
                    view["trajectory"][0].update(
                        boundingbox_min=[-4, -4, min(0, float(points[:, 2].min())) - 1],
                        boundingbox_max=[4, 4, max(1, float(points[:, 2].max()) + 1)],
                        lookat=[0, 0, float(np.median(points[:, 2]))], front=front.tolist(),
                        up=up.tolist(), zoom=0.85)
                    self.visualizer.set_view_status(json.dumps(view))
                    self._camera_ready = True
        for label, color in zip(self.cells, colors.reshape(-1, 3)):
            linear = np.where(color <= 0.04045, color / 12.92, ((color + 0.055) / 1.055) ** 2.4)
            ink = 0 if np.dot(linear, [0.2126, 0.7152, 0.0722]) > 0.179 else 1
            background = "#" + "".join(f"{int(c * 255):02x}" for c in color)
            label.configure(background=background, foreground="black" if ink == 0 else "white")
        self._dirty = True

    def _mode_changed(self):
        if self.relative_mode.get() and self.ground_mm is None:
            self.relative_mode.set(False)
            return
        relative = self.relative_mode.get()
        self.value_title.set("RELATIVE RISE / mm" if relative else "DISTANCE / mm")
        self.legend.set("0 mm: blue\n200 mm: orange\nGray / -- : invalid/outside range\nor no ground reference" if relative else
                        f"Near 0 mm: orange\nFar {MAX_DISTANCE_MM} mm: blue\nGray / -- : invalid or outside range")
        self.preview_note.set("X/Y: region indices | Z = relative rise / 100\nApproximate surface | Drag: orbit | Wheel: zoom"
                              if relative else "X/Y: region indices | Z = -distance / 100\nApproximate surface | Drag: orbit | Wheel: zoom")
        self._show_frame(self._last_frame, reset_camera=True)

    def _start_ground_capture(self):
        if self.reader.snapshot().error:
            self._calibration_note = "Ground capture failed: serial disconnected."
            return
        self.sampler = GroundSampler()
        self._sampling_started = time.monotonic()
        self._last_sample_at = self._sampling_started
        self._capture_previous_frame = self.reader.snapshot().frame
        self._calibration_note = ""
        self.capture_button.configure(state="disabled")

    def _finish_ground_capture(self, success, note):
        self.sampler = None
        self.capture_button.configure(state="normal")
        self._calibration_note = note
        if success is not None:
            self.ground_mm = success
            self.relative_mode.set(True)
            self._mode_changed()

    def _render(self):
        self.visualizer.update_renderer()
        pixels = (np.clip(np.asarray(self.visualizer.capture_screen_float_buffer(do_render=True)), 0, 1) * 255).astype(np.uint8)
        height, width = pixels.shape[:2]
        ppm = f"P6\n{width} {height}\n255\n".encode() + pixels.tobytes()
        self.image = tk.PhotoImage(data=ppm, format="PPM")
        self.canvas.itemconfigure(self._image_item, image=self.image)
        self._dirty = False

    def _start_drag(self, event):
        self._drag = (event.x, event.y)

    def _rotate(self, event):
        if self._drag is not None:
            x, y = self._drag
            self.visualizer.get_view_control().rotate(event.x - x, event.y - y)
            self._drag = (event.x, event.y)
            self._dirty = True

    def _zoom(self, event):
        self.visualizer.get_view_control().scale(event.delta / 120)
        self._dirty = True

    def _tick(self):
        if self.closed:
            return
        if not self.visualizer.poll_events():
            self.close()
            return
        state = self.reader.snapshot()
        now = time.monotonic()
        stale = now - (state.frame.received_at if state.frame else self.started) >= 2
        if self.sampler is not None:
            if state.error:
                self._finish_ground_capture(None, "Ground capture failed: serial disconnected; previous reference kept.")
            elif now - self._sampling_started >= 15:
                self._finish_ground_capture(None, "Ground capture failed: 15 second limit; previous reference kept.")
            elif now - self._last_sample_at >= 2:
                self._finish_ground_capture(None, "Ground capture failed: no complete frame for 2 seconds; previous reference kept.")
            elif state.frame is not None and state.frame is not self._capture_previous_frame:
                if self.sampler.add(state.frame):
                    self._last_sample_at = now
                if self.sampler.complete:
                    ground = self.sampler.result()
                    if ground is None:
                        self._finish_ground_capture(None, "Ground capture failed: no zones had 24 valid samples; previous reference kept.")
                    else:
                        coverage = int(np.isfinite(ground).sum())
                        self._finish_ground_capture(ground, f"Ground reference ready: {coverage}/64 zones.")
        if state.error:
            message = f"Disconnected: {state.error}\nReconnect the device, then restart this program."
        elif stale:
            message = f"{self.port} | Data timeout: no complete frame for 2 seconds."
        elif state.frame is None:
            message = f"{self.port} | Waiting for a complete frame..."
        else:
            message = (f"{self.port} | {state.fps:.1f} fps | Valid {np.count_nonzero(state.frame.valid)}/64"
                       f" (0-{MAX_DISTANCE_MM} mm)"
                       f" | Sensor counter {state.frame.sequence}")
        if self.sampler is not None:
            message += f" | Ground {self.sampler.count}/30"
        elif self.ground_mm is not None:
            message += f" | Ground {np.isfinite(self.ground_mm).sum()}/64 zones"
        detail = f"Received {state.frames} | Parse errors {state.errors}"
        if self.sampler is not None:
            detail = f"Ground sampling {self.sampler.count}/30 | {detail}"
        elif self._calibration_note:
            detail = f"{self._calibration_note} | {detail}"
        message += f"\n{detail}"
        shown = None if stale or state.error else state.frame
        if shown is not self._last_frame:
            self._show_frame(shown)
            self._last_frame = shown
        if message != self._last_status:
            self.status.set(message)
            self._last_status = message
        if self._dirty:
            self._render()
        self._job = self.root.after(30, self._tick)

    def run(self):
        self.root.mainloop()

    def close(self):
        if self.closed:
            return
        self.closed = True
        if self._job is not None:
            self.root.after_cancel(self._job)
        try:
            self.reader.close()
        finally:
            try:
                self.visualizer.destroy_window()
            finally:
                self.root.destroy()


def main():
    parser = argparse.ArgumentParser(description="Live STM32 8x8 distance and region-grid preview")
    parser.add_argument("--port", required=True, help="CH340 serial port, e.g. COM5 (115200 8N1)")
    args = parser.parse_args()
    reader = None
    viewer = None
    try:
        reader = SerialReader(args.port)
        viewer = Viewer(reader, args.port)
        viewer.run()
        return 0
    except (serial.SerialException, OSError, RuntimeError, tk.TclError) as exc:
        print(f"Cannot start viewer: {exc}", file=sys.stderr)
        return 1
    finally:
        if viewer is not None:
            viewer.close()
        elif reader is not None:
            reader.close()


if __name__ == "__main__":
    sys.exit(main())
