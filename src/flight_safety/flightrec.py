#!/usr/bin/env python3
"""
flightrec — control client for the webcam flight recorder.

Talks to recorder.py (running on the ml PC) over HTTP. Use it from the command
line or import it from your flight-experiment Python code.

CLI:
    ./flightrec.py start                 # begin recording
    ./flightrec.py start flight7         # begin, name the file "flight7"
    ./flightrec.py start --name flight7  # same
    ./flightrec.py stop                  # finalize + save
    ./flightrec.py status                # show state
    ./flightrec.py --host 192.168.50.12 --port 8088 start flight7

Set the default target once via env vars instead of flags:
    export FLIGHTREC_HOST=192.168.50.12
    export FLIGHTREC_PORT=8088

As a library (e.g. inside your flight node):
    from flightrec import RecorderClient
    rec = RecorderClient("192.168.50.12")
    rec.start("flight7")
    ...
    rec.stop()
"""
import argparse
import json
import os
import sys
from urllib.parse import quote
from urllib.request import urlopen, Request
from urllib.error import URLError, HTTPError


class RecorderClient:
    def __init__(self, host=None, port=None, timeout=15):
        self.host = host or os.environ.get("FLIGHTREC_HOST", "127.0.0.1")
        self.port = int(port or os.environ.get("FLIGHTREC_PORT", "8088"))
        self.timeout = timeout

    def _call(self, path):
        url = f"http://{self.host}:{self.port}{path}"
        req = Request(url, method="POST" if path != "/status" else "GET")
        try:
            with urlopen(req, timeout=self.timeout) as r:
                return json.loads(r.read().decode())
        except HTTPError as e:
            # The server returns JSON bodies on 4xx too (e.g. already recording).
            try:
                return json.loads(e.read().decode())
            except Exception:
                return {"ok": False, "msg": f"HTTP {e.code}"}
        except URLError as e:
            return {"ok": False, "msg": f"cannot reach recorder at {self.host}:{self.port} ({e.reason})"}

    def start(self, name=None):
        path = "/start"
        if name:
            path += "?name=" + quote(name)
        return self._call(path)

    def stop(self):
        return self._call("/stop")

    def status(self):
        return self._call("/status")


def main(argv=None):
    p = argparse.ArgumentParser(prog="flightrec", description="Control the webcam flight recorder.")
    p.add_argument("--host", default=None, help="recorder host (default $FLIGHTREC_HOST or 127.0.0.1)")
    p.add_argument("--port", default=None, help="recorder port (default $FLIGHTREC_PORT or 8088)")
    sub = p.add_subparsers(dest="cmd", required=True)

    s = sub.add_parser("start", help="begin recording")
    s.add_argument("name", nargs="?", default=None, help="file name label for this recording")
    s.add_argument("--name", dest="name_opt", default=None, help="file name label (alternative to positional)")

    sub.add_parser("stop", help="finalize and save")
    sub.add_parser("status", help="show recorder state")

    args = p.parse_args(argv)
    client = RecorderClient(args.host, args.port)

    if args.cmd == "start":
        name = args.name_opt or args.name
        resp = client.start(name)
    elif args.cmd == "stop":
        resp = client.stop()
    else:
        resp = client.status()

    print(json.dumps(resp, indent=2))
    return 0 if resp.get("ok") else 1


if __name__ == "__main__":
    sys.exit(main())
