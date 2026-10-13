"""``vision_web_stream`` -- browser display for the annotated vision stream.

The car's camera is already reachable as a plain MJPEG page (``http://<ip>:81/``)
and ``mycar_camera`` republishes it as ``/camera/image_raw/compressed``.
``vision_detector`` draws the boxes and republishes ``/vision/image/compressed``;
this node turns *that* topic back into a web page so the operator can watch the
detected frames **in a browser, without Qt**.

Routes
------

===========================  ============================================
``/``                        HTML page: live video + detection legend
``/stream.mjpg``             ``multipart/x-mixed-replace`` MJPEG stream
``/snapshot.jpg``            the latest annotated frame as a single JPEG
``/detections.json``         latest detections (id / label / score / box)
``/healthz``                 plain-text liveness probe
===========================  ============================================

It is intentionally dependency-free (only ``http.server`` from the standard
library + OpenCV, which is already required by ``vision_detector``), so it is a
drop-in fallback when ``ros-humble-web-video-server`` cannot be installed.

Typical use::

    ros2 run mycar_vision vision_web_stream --ros-args -p port:=8081
    # then open http://<machine-ip>:8081/
"""

from __future__ import annotations

import json
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler
from http.server import ThreadingHTTPServer
from typing import Dict, List, Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from vision_msgs.msg import Detection2DArray

#: Multipart boundary used for the MJPEG stream.
BOUNDARY = "mycarvision"

_PAGE_TEMPLATE = """<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width, initial-scale=1"/>
<title>mycar vision</title>
<style>
  :root {{ color-scheme: dark; }}
  * {{ box-sizing: border-box; }}
  body {{
    margin: 0; background: #10141c; color: #e6edf3;
    font: 14px/1.5 -apple-system, "Segoe UI", "Noto Sans SC", sans-serif;
  }}
  header {{
    display: flex; align-items: center; gap: 12px;
    padding: 12px 18px; background: #161c27;
    border-bottom: 1px solid #232b3a;
  }}
  header h1 {{ margin: 0; font-size: 16px; font-weight: 600; letter-spacing: .3px; }}
  .tag {{
    font-size: 12px; padding: 2px 8px; border-radius: 999px;
    background: #1f6feb22; color: #6cb6ff; border: 1px solid #1f6feb55;
  }}
  .tag.off {{ background: #f8514922; color: #ff7b72; border-color: #f8514955; }}
  main {{ padding: 18px; max-width: 1100px; margin: 0 auto; }}
  .stage {{
    position: relative; background: #000; border: 1px solid #232b3a;
    border-radius: 10px; overflow: hidden; line-height: 0;
  }}
  .stage img {{ width: 100%; height: auto; display: block; }}
  .stats {{
    display: grid; grid-template-columns: repeat(auto-fit, minmax(150px, 1fr));
    gap: 10px; margin-top: 14px;
  }}
  .card {{
    background: #161c27; border: 1px solid #232b3a; border-radius: 8px;
    padding: 10px 12px;
  }}
  .card .k {{ font-size: 11px; text-transform: uppercase; letter-spacing: .6px; color: #8b949e; }}
  .card .v {{ font-size: 20px; font-weight: 600; margin-top: 2px; }}
  table {{ width: 100%; border-collapse: collapse; margin-top: 14px; font-size: 13px; }}
  th, td {{ text-align: left; padding: 7px 10px; border-bottom: 1px solid #232b3a; }}
  th {{ color: #8b949e; font-weight: 500; font-size: 11px; text-transform: uppercase; }}
  td.num {{ font-variant-numeric: tabular-nums; }}
  .dot {{ display: inline-block; width: 9px; height: 9px; border-radius: 2px; margin-right: 7px; }}
  footer {{ color: #6e7681; font-size: 12px; padding: 4px 18px 22px; text-align: center; }}
  code {{ background: #1c2431; padding: 1px 6px; border-radius: 4px; }}
</style>
</head>
<body>
<header>
  <h1>mycar vision</h1>
  <span class="tag" id="live">CONNECTING</span>
  <span class="tag" id="src">{image_topic}</span>
</header>
<main>
  <div class="stage"><img id="video" src="/stream.mjpg" alt="annotated stream"/></div>
  <div class="stats">
    <div class="card"><div class="k">Objects</div><div class="v" id="count">-</div></div>
    <div class="card"><div class="k">Stream fps</div><div class="v" id="fps">-</div></div>
    <div class="card"><div class="k">Frame age</div><div class="v" id="age">-</div></div>
    <div class="card"><div class="k">Resolution</div><div class="v" id="res">-</div></div>
  </div>
  <table>
    <thead><tr><th>Class</th><th>Source</th><th>Score</th><th>Center (px)</th><th>Size (px)</th></tr></thead>
    <tbody id="rows"><tr><td colspan="5" style="color:#6e7681">waiting for /vision/detections ...</td></tr></tbody>
  </table>
</main>
<footer>Annotated by <code>vision_detector</code> &middot; streamed by <code>vision_web_stream</code></footer>
<script>
const COLORS = {{
  person: "#ff8c00", bottle: "#ffd700",
  blue_block: "#1e78e6", green_block: "#3cc83c"
}};
function esc(s) {{
  return String(s).replace(/[&<>"]/g, c => ({{"&":"&amp;","<":"&lt;",">":"&gt;",'"':"&quot;"}})[c]);
}}
async function tick() {{
  try {{
    const r = await fetch("/detections.json", {{cache: "no-store"}});
    const d = await r.json();
    document.getElementById("live").textContent = "LIVE";
    document.getElementById("live").className = "tag";
    document.getElementById("count").textContent = d.count;
    document.getElementById("fps").textContent = d.fps.toFixed(1);
    document.getElementById("res").textContent = d.width + "x" + d.height;
    const age = d.age;
    document.getElementById("age").textContent =
      age < 0 ? "-" : age.toFixed(2) + " s";
    const tb = document.getElementById("rows");
    if (!d.items.length) {{
      tb.innerHTML = '<tr><td colspan="5" style="color:#6e7681">no object</td></tr>';
    }} else {{
      tb.innerHTML = d.items.map(it => {{
        const c = COLORS[it.label] || "#8b949e";
        return "<tr>" +
          '<td><span class="dot" style="background:' + c + '"></span>' + esc(it.label) + "</td>" +
          "<td>" + esc(it.source) + "</td>" +
          '<td class="num">' + it.score.toFixed(3) + "</td>" +
          '<td class="num">' + it.cx.toFixed(0) + ", " + it.cy.toFixed(0) + "</td>" +
          '<td class="num">' + it.w.toFixed(0) + " x " + it.h.toFixed(0) + "</td>" +
        "</tr>";
      }}).join("");
    }}
  }} catch (e) {{
    document.getElementById("live").textContent = "OFFLINE";
    document.getElementById("live").className = "tag off";
  }}
}}
setInterval(tick, 500);
tick();
document.getElementById("video").addEventListener("error", () => {{
  setTimeout(() => {{ document.getElementById("video").src = "/stream.mjpg?t=" + Date.now(); }}, 1500);
}});
</script>
</body>
</html>
"""


def _video_qos() -> QoSProfile:
    """Match the publisher side (BestEffort, keep-last 1)."""
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


class FrameHub:
    """Thread-safe "latest value" slot shared by the ROS and HTTP threads."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._jpeg: Optional[bytes] = None
        self._jpeg_stamp: float = 0.0
        self._width: int = 0
        self._height: int = 0
        self._items: List[Dict[str, object]] = []
        self._items_stamp: float = 0.0
        self._serial: int = 0
        self._fps_window: List[float] = []

    # -- writer side (ROS callbacks) ---------------------------------------
    def push_frame(self, jpeg: bytes, width: int, height: int) -> None:
        now = time.monotonic()
        with self._lock:
            self._jpeg = jpeg
            self._jpeg_stamp = now
            self._width = width
            self._height = height
            self._serial += 1
            self._fps_window.append(now)
            cutoff = now - 2.0
            while self._fps_window and self._fps_window[0] < cutoff:
                self._fps_window.pop(0)

    def push_detections(self, items: List[Dict[str, object]]) -> None:
        with self._lock:
            self._items = items
            self._items_stamp = time.monotonic()

    # -- reader side (HTTP handlers) ---------------------------------------
    def snapshot(self) -> Tuple[Optional[bytes], int]:
        """Return ``(jpeg, serial)``; ``serial`` lets callers detect new frames."""
        with self._lock:
            return self._jpeg, self._serial

    def wait_for_frame(self, last_serial: int, timeout: float) -> Tuple[Optional[bytes], int]:
        """Block until a frame newer than ``last_serial`` shows up."""
        deadline = time.monotonic() + timeout
        while True:
            with self._lock:
                if self._serial != last_serial and self._jpeg is not None:
                    return self._jpeg, self._serial
            if time.monotonic() >= deadline:
                with self._lock:
                    return self._jpeg, self._serial
            time.sleep(0.005)

    def status(self) -> Dict[str, object]:
        with self._lock:
            now = time.monotonic()
            window = [t for t in self._fps_window if t > now - 2.0]
            fps = len(window) / 2.0 if len(window) > 1 else 0.0
            age = (now - self._jpeg_stamp) if self._jpeg is not None else -1.0
            return {
                "count": len(self._items),
                "items": list(self._items),
                "fps": fps,
                "age": age,
                "width": self._width,
                "height": self._height,
                "frame_serial": self._serial,
                "items_age": (now - self._items_stamp) if self._items else -1.0,
            }


class _Handler(BaseHTTPRequestHandler):
    """Serves the built-in page, the MJPEG stream and the JSON status."""

    server_version = "mycar-vision/1.0"
    protocol_version = "HTTP/1.1"

    # ``hub`` / ``image_topic`` / ``fps`` are injected by the node.
    hub: FrameHub
    image_topic: str = "/vision/image/compressed"
    fps: float = 15.0

    # -- helpers ----------------------------------------------------------
    def log_message(self, fmt: str, *args) -> None:  # noqa: A003 - stdlib API
        """Silence per-request logging; the node logs a summary instead."""
        return

    def _send_text(self, body: str, status: HTTPStatus = HTTPStatus.OK, ctype: str = "text/plain"):
        payload = body.encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", f"{ctype}; charset=utf-8")
        self.send_header("Content-Length", str(len(payload)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(payload)

    def _send_json(self, obj: Dict[str, object]) -> None:
        payload = json.dumps(obj, ensure_ascii=False).encode("utf-8")
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(payload)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(payload)

    # -- routes -----------------------------------------------------------
    def do_GET(self) -> None:  # noqa: N802 - stdlib API
        path = self.path.split("?", 1)[0]
        try:
            if path in ("/", "/index.html"):
                self._serve_page()
            elif path in ("/stream.mjpg", "/stream"):
                self._serve_stream()
            elif path == "/snapshot.jpg":
                self._serve_snapshot()
            elif path == "/detections.json":
                self._send_json(self.hub.status())
            elif path == "/healthz":
                self._send_text("ok")
            else:
                self._send_text("not found", HTTPStatus.NOT_FOUND)
        except (BrokenPipeError, ConnectionResetError):
            pass  # client navigated away mid-stream: perfectly normal

    def _serve_page(self) -> None:
        self._send_text(
            _PAGE_TEMPLATE.format(image_topic=self.image_topic),
            ctype="text/html",
        )

    def _serve_snapshot(self) -> None:
        jpeg, _ = self.hub.snapshot()
        if jpeg is None:
            self._send_text("no frame yet", HTTPStatus.SERVICE_UNAVAILABLE)
            return
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "image/jpeg")
        self.send_header("Content-Length", str(len(jpeg)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(jpeg)

    def _serve_stream(self) -> None:
        self.send_response(HTTPStatus.OK)
        self.send_header("Age", "0")
        self.send_header("Cache-Control", "no-cache, private")
        self.send_header("Pragma", "no-cache")
        self.send_header(
            "Content-Type", f"multipart/x-mixed-replace; boundary={BOUNDARY}"
        )
        self.end_headers()

        period = 1.0 / max(0.5, self.fps)
        serial = -1
        while True:
            jpeg, serial_new = self.hub.wait_for_frame(serial, timeout=period * 3)
            if jpeg is None:
                time.sleep(period)
                continue
            if serial_new == serial:
                time.sleep(period)  # paused / no new frame: keep the socket warm
                continue
            serial = serial_new
            try:
                self.wfile.write(b"--" + BOUNDARY.encode() + b"\r\n")
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(b"Content-Length: " + str(len(jpeg)).encode() + b"\r\n\r\n")
                self.wfile.write(jpeg)
                self.wfile.write(b"\r\n")
                self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError, OSError):
                return
            time.sleep(period)


class VisionWebStream(Node):
    """Republish ``/vision/image/compressed`` as an MJPEG web page."""

    def __init__(self) -> None:
        super().__init__("vision_web_stream")

        self.declare_parameter("image_topic", "/vision/image/compressed")
        self.declare_parameter("detection_topic", "/vision/detections")
        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 8081)
        self.declare_parameter("fps", 15.0)
        self.declare_parameter("jpeg_quality", 75)
        self.declare_parameter("log_interval", 30.0)

        gp = self.get_parameter
        self.image_topic = gp("image_topic").get_parameter_value().string_value
        self.detection_topic = gp("detection_topic").get_parameter_value().string_value
        host = gp("host").get_parameter_value().string_value or "0.0.0.0"
        port = max(1, min(65535, gp("port").get_parameter_value().integer_value))
        fps = max(0.5, gp("fps").get_parameter_value().double_value)
        self.jpeg_quality = max(1, min(100, gp("jpeg_quality").get_parameter_value().integer_value))
        self.log_interval = max(1.0, gp("log_interval").get_parameter_value().double_value)

        self.hub = FrameHub()
        self._frames = 0
        self._last_log = time.monotonic()

        self.create_subscription(CompressedImage, self.image_topic, self._on_image, _video_qos())
        self.create_subscription(
            Detection2DArray, self.detection_topic, self._on_detections, 10
        )

        handler = type(
            "BoundHandler",
            (_Handler,),
            {"hub": self.hub, "image_topic": self.image_topic, "fps": fps},
        )
        self._server = ThreadingHTTPServer((host, port), handler)
        self._server.daemon_threads = True
        self._thread = threading.Thread(
            target=self._server.serve_forever, name="vision_web_stream_http", daemon=True
        )
        self._thread.start()

        self.get_logger().info(
            f"vision_web_stream ready | {self.image_topic} -> http://{host}:{port}/ "
            f"| stream fps<={fps:.1f}"
        )

    # -- ROS callbacks -----------------------------------------------------
    def _on_image(self, msg: CompressedImage) -> None:
        # The publisher already encoded JPEG, so the annotation can be forwarded
        # verbatim when the quality matches; otherwise re-encode once.
        if msg.format.lower().endswith("jpeg") and self.jpeg_quality >= 95:
            payload = bytes(msg.data)
            frame = None
        else:
            buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
            frame = cv2_imdecode(buf)
            if frame is None:
                self.get_logger().warn("cannot decode frame", throttle_duration_sec=5.0)
                return
            ok, encoded = cv2_imencode(frame, self.jpeg_quality)
            if not ok:
                return
            payload = encoded

        height, width = (frame.shape[0], frame.shape[1]) if frame is not None else self._cached_size(msg)
        self.hub.push_frame(payload, width, height)
        self._frames += 1

        now = time.monotonic()
        if now - self._last_log >= self.log_interval:
            st = self.hub.status()
            self.get_logger().info(
                f"served {self._frames} frames | in-rate {st['fps']:.1f} fps "
                f"| objects {st['count']}"
            )
            self._last_log = now

    def _cached_size(self, msg: CompressedImage) -> Tuple[int, int]:
        """Best-effort size when the JPEG is forwarded without decoding."""
        st = self.hub.status()
        return int(st["height"]), int(st["width"])

    def _on_detections(self, msg: Detection2DArray) -> None:
        items: List[Dict[str, object]] = []
        for det in msg.detections:
            raw_id = det.id or "unknown"
            parts = raw_id.split("#")
            label = parts[0]
            source = parts[1] if len(parts) > 1 else "?"
            score = 0.0
            if det.results:
                hyp = det.results[0].hypothesis
                if hyp.class_id:
                    label = hyp.class_id
                score = float(hyp.score)
            items.append(
                {
                    "id": raw_id,
                    "label": label,
                    "source": source,
                    "score": round(score, 4),
                    "cx": float(det.bbox.center.position.x),
                    "cy": float(det.bbox.center.position.y),
                    "w": float(det.bbox.size_x),
                    "h": float(det.bbox.size_y),
                }
            )
        self.hub.push_detections(items)

    # -- shutdown ----------------------------------------------------------
    def destroy_node(self) -> bool:
        try:
            self._server.shutdown()
            self._server.server_close()
        except Exception:  # pragma: no cover - shutdown must never raise
            pass
        return super().destroy_node()


# Small shims keep the module importable even if OpenCV is missing, and make the
# two hot calls easy to monkey-patch in tests.
def cv2_imdecode(buf: np.ndarray):
    import cv2

    return cv2.imdecode(buf, cv2.IMREAD_COLOR)


def cv2_imencode(frame: np.ndarray, quality: int) -> Tuple[bool, bytes]:
    import cv2

    ok, encoded = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
    return ok, (encoded.tobytes() if ok else b"")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = VisionWebStream()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
