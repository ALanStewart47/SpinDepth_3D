"""Live 8x8 distance grid and region-index depth preview."""

import argparse
import json
import sys
import time
import tkinter as tk

import numpy as np
import serial

from serial_link import SerialReader


def preview_data(frame):
    """Keep the firmware's row/column order; X/Y are NOT physical dimensions."""
    valid = frame.valid
    distance = frame.distance_mm.astype(float)
    t = np.clip(distance / 1000.0, 0, 1)
    near = np.array([0.95, 0.48, 0.20])
    far = np.array([0.12, 0.38, 0.68])
    colors = near + t[..., None] * (far - near)
    colors[~valid] = [0.22, 0.24, 0.27]
    rows, cols = np.indices((8, 8))
    # shortcut: region-index geometry; replace only when physical projection is specified.
    points = np.stack((cols - 3.5, 3.5 - rows, -distance / 100), axis=-1)
    return points[valid], colors[valid], colors


class Viewer:
    def __init__(self, reader, port):
        import open3d as o3d

        self.o3d = o3d
        self.reader = reader
        self.port = port
        self.started = time.monotonic()
        self._last_frame = None
        self._last_status = None
        self._has_cloud = False
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

        # Filament crashes on this AMD driver; keep all rendering on the legacy path.
        self.visualizer = o3d.visualization.Visualizer()
        if not self.visualizer.create_window("SpinDepth3D renderer", 640, 480, visible=False):
            self.root.destroy()
            raise RuntimeError("Cannot create the Open3D legacy rendering context")
        option = self.visualizer.get_render_option()
        option.background_color = np.array([0.065, 0.08, 0.10])
        option.point_size = 14
        self.cloud = o3d.geometry.PointCloud()
        self.axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1)
        self.visualizer.add_geometry(self.axes)

        self.status = tk.StringVar()
        tk.Label(self.root, textvariable=self.status, background="#141a22", foreground="#d4dce5",
                 anchor="w", justify="left", padx=16, pady=12).pack(side="bottom", fill="x")
        content = tk.Frame(self.root, background="#141a22")
        content.pack(fill="both", expand=True, padx=16, pady=16)
        left = tk.Frame(content, background="#141a22")
        left.pack(side="left", fill="y", padx=(0, 24))
        right = tk.Frame(content, background="#141a22")
        right.pack(side="left", fill="both", expand=True)
        for parent, text in ((left, "DISTANCE / mm"), (right, "REGION GRID PREVIEW")):
            tk.Label(parent, text=text, background="#141a22", foreground="white",
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
        tk.Label(left, text="Near 0 mm: orange\nFar 1000 mm: blue\nGray / -- : no valid target",
                 background="#141a22", foreground="#a8b8c8", justify="left").pack(anchor="w", pady=16)
        self.canvas = tk.Canvas(right, width=640, height=480, background="#11141a", highlightthickness=0)
        self.canvas.pack(expand=True)
        self._image_item = self.canvas.create_image(0, 0, anchor="nw")
        self.image = None
        tk.Label(right, text="X/Y: region indices | Z = -distance / 100\n100 mm per display unit | Drag: orbit | Wheel: zoom",
                 background="#141a22", foreground="#a8b8c8", justify="left").pack(anchor="w", pady=(12, 0))
        self.canvas.bind("<ButtonPress-1>", self._start_drag)
        self.canvas.bind("<B1-Motion>", self._rotate)
        self.canvas.bind("<MouseWheel>", self._zoom)
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self._show_frame(None)
        self._job = self.root.after(30, self._tick)

    def _show_frame(self, frame):
        if self._has_cloud:
            self.visualizer.remove_geometry(self.cloud, reset_bounding_box=False)
            self._has_cloud = False
        if frame is None:
            colors = np.full((8, 8, 3), 0.24)
            for label in self.cells:
                label.configure(text="--")
        else:
            points, point_colors, colors = preview_data(frame)
            valid = frame.valid
            for i, label in enumerate(self.cells):
                row, col = divmod(i, 8)
                label.configure(text=str(frame.distance_mm[row, col]) if valid[row, col] else "--")
            if len(points):
                self.cloud.points = self.o3d.utility.Vector3dVector(points)
                self.cloud.colors = self.o3d.utility.Vector3dVector(point_colors)
                self.visualizer.add_geometry(self.cloud, reset_bounding_box=not self._camera_ready)
                self._has_cloud = True
                if not self._camera_ready:
                    front = np.array([0.5, -0.7, 0.7])
                    front /= np.linalg.norm(front)
                    up = np.cross(front, np.cross([0, 0, 1], front))
                    up /= np.linalg.norm(up)
                    # Legacy bounds ignore flat/single-point clouds when merged with the axes.
                    view = json.loads(self.visualizer.get_view_status())
                    view["trajectory"][0].update(
                        boundingbox_min=[-4, -4, float(points[:, 2].min()) - 1],
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
        if state.error:
            message = f"Disconnected: {state.error}\nReconnect the device, then restart this program."
        elif stale:
            message = f"{self.port} | Data timeout: no complete frame for 2 seconds."
        elif state.frame is None:
            message = f"{self.port} | Waiting for a complete frame..."
        else:
            message = (f"{self.port} | {state.fps:.1f} fps | Valid {np.count_nonzero(state.frame.valid)}/64"
                       f" | Sensor counter {state.frame.sequence}")
        message += f"\nReceived {state.frames} | Parse errors {state.errors}"
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
