"""The seat's one port: the live view (static files) and TensorBoard under /tensorboard/ (proxied).
    python3 server.py <site_dir> <port> <tensorboard_port>"""
import http.server, socketserver, sys, urllib.request, urllib.error
from functools import partial

site, port, tb = sys.argv[1], int(sys.argv[2]), int(sys.argv[3])


class Handler(http.server.SimpleHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _proxy(self):
        body = self.rfile.read(int(self.headers.get("Content-Length") or 0)) or None
        req = urllib.request.Request(f"http://127.0.0.1:{tb}{self.path}", data=body, method=self.command,
                                     headers={k: v for k, v in self.headers.items() if k.lower() not in ("host", "connection")})
        try:
            with urllib.request.urlopen(req, timeout=60) as r:
                status, headers, data = r.status, r.headers.items(), r.read()
        except urllib.error.HTTPError as e:
            status, headers, data = e.code, e.headers.items(), e.read()
        except OSError:
            status, headers, data = 503, [("Content-Type", "text/plain")], b"TensorBoard is not running (tools/train/board.sh)"
        self.send_response(status)
        for k, v in headers:
            if k.lower() not in ("transfer-encoding", "connection", "content-length"):
                self.send_header(k, v)
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        if self.path.startswith("/tensorboard"):
            return self._proxy()
        return super().do_GET()

    def do_POST(self):
        if self.path.startswith("/tensorboard"):
            return self._proxy()
        self.send_error(405)


class Server(socketserver.ThreadingMixIn, http.server.HTTPServer):
    daemon_threads = True
    allow_reuse_address = True


Server(("127.0.0.1", port), partial(Handler, directory=site)).serve_forever()
