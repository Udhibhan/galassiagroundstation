"""
sim_server.py

Web front end for pass_sim.py: open http://localhost:8001, pick a satellite, press Track.
Simulation only - this server never opens a CAN bus or moves a motor.

    python sim_server.py
"""

import json
import logging
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

import pass_sim

PORT = 8001
ROOT = Path(__file__).resolve().parent

logger = logging.getLogger(__name__)


class SimHandler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(ROOT), **kwargs)

    def _json(self, code: int, body) -> None:
        data = json.dumps(body).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        url = urlparse(self.path)
        q = {k: v[0] for k, v in parse_qs(url.query).items()}
        try:
            if url.path in ("/", "/index.html"):
                self.path = "/pass_sim.html"
            elif url.path == "/api/simulate":
                duration = min(max(float(q.get("duration", 120)), 10), 600)
                self._json(200, pass_sim.simulate(q.get("name", "LUMELITE-4"),
                                                  q.get("source", "celestrak"), duration))
                return
            elif url.path == "/api/search":
                self._json(200, pass_sim.search(q.get("q", "")))
                return
        except Exception as exc:
            logger.exception("request failed")
            self._json(500, {"error": str(exc)})
            return
        super().do_GET()


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    print(f"Pass simulator: http://localhost:{PORT}  (simulation only, no CAN)")
    ThreadingHTTPServer(("127.0.0.1", PORT), SimHandler).serve_forever()
