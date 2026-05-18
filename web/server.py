"""
KinetiKey Dashboard Server — WiFi edition
The STM32 board POSTs JSON events directly to POST /event.
Browser clients connect to GET /events (SSE) for real-time updates.

Usage:
    pip install -r requirements.txt
    python server.py

Then open http://localhost:5000 in a browser.
Set mbed_app.json server-ip to THIS machine's IP on the same WiFi network.
Find it with:  ifconfig | grep "inet "   (macOS/Linux)
               ipconfig                  (Windows)
"""

import json
import queue
import threading

from flask import Flask, Response, jsonify, render_template, request

app = Flask(__name__)

# ─── Shared state ─────────────────────────────────────────────────────────────

board_state = {
    "mode":        "IDLE",
    "pin":         [],        # digits entered so far (ints 1-3)
    "total":       4,
    "last_digit":  None,      # int 1-3
    "last_shape":  None,      # "TRIANGLE" | "SQUARE" | "CIRCLE"
    "lock":        "locked",  # "locked" | "success" | "denied"
    "log":         [],
}
state_lock = threading.Lock()

sse_clients: list[queue.Queue] = []
sse_lock = threading.Lock()

MAX_LOG = 80

# ─── State updater ────────────────────────────────────────────────────────────

def update_state(ev: dict):
    with state_lock:
        t = ev.get("type")

        if t == "mode":
            board_state["mode"] = ev.get("mode", "IDLE")
            if ev.get("mode") == "RECORD":
                board_state["pin"] = []
                board_state["lock"] = "locked"
                board_state["last_digit"] = None
                board_state["last_shape"] = None

        elif t == "digit":
            board_state["pin"]        = ev.get("pin", [])
            board_state["last_digit"] = ev.get("digit")
            board_state["last_shape"] = ev.get("shape")
            board_state["mode"]       = ev.get("mode", board_state["mode"])

        elif t == "erase":
            board_state["pin"]  = ev.get("pin", [])
            board_state["mode"] = ev.get("mode", board_state["mode"])

        elif t == "unlock_result":
            r = ev.get("result", "denied")
            board_state["lock"] = r if r in ("success", "denied") else "locked"
            board_state["mode"] = "IDLE"

        board_state["log"].append(ev)
        if len(board_state["log"]) > MAX_LOG:
            board_state["log"].pop(0)


def broadcast(ev: dict):
    payload = json.dumps(ev)
    with sse_lock:
        dead = []
        for q in sse_clients:
            try:
                q.put_nowait(payload)
            except queue.Full:
                dead.append(q)
        for q in dead:
            sse_clients.remove(q)

# ─── Routes ───────────────────────────────────────────────────────────────────

@app.route("/")
def index():
    return render_template("index.html")


@app.route("/state")
def state():
    with state_lock:
        return jsonify(dict(board_state))


@app.route("/event", methods=["POST"])
def receive_event():
    """Called by the STM32 board over WiFi for every gesture event."""
    ev = request.get_json(silent=True, force=True)
    if not ev or not isinstance(ev, dict):
        return "bad request", 400
    update_state(ev)
    broadcast(ev)
    return "", 204


@app.route("/events")
def sse_stream():
    """Browser SSE stream — one persistent connection per tab."""
    def stream():
        q: queue.Queue = queue.Queue(maxsize=100)
        with sse_lock:
            sse_clients.append(q)
        try:
            with state_lock:
                snap = dict(board_state)
            yield f"data: {json.dumps({'type': 'snapshot', **snap})}\n\n"

            while True:
                try:
                    payload = q.get(timeout=20)
                    yield f"data: {payload}\n\n"
                except queue.Empty:
                    yield ": ping\n\n"
        finally:
            with sse_lock:
                if q in sse_clients:
                    sse_clients.remove(q)

    return Response(
        stream(),
        mimetype="text/event-stream",
        headers={"Cache-Control": "no-cache", "X-Accel-Buffering": "no"},
    )


# ─── Entry point ──────────────────────────────────────────────────────────────

if __name__ == "__main__":
    import argparse
    import socket

    parser = argparse.ArgumentParser(description="KinetiKey dashboard server")
    parser.add_argument("--host", default="0.0.0.0", help="Bind address (default: all interfaces)")
    parser.add_argument("--port", type=int, default=5000)
    args = parser.parse_args()

    # Print local IPs so user knows what to put in mbed_app.json server-ip
    print("=" * 55)
    print("  KinetiKey Dashboard Server")
    print("=" * 55)
    try:
        hostname = socket.gethostname()
        local_ip = socket.gethostbyname(hostname)
        print(f"  Local IP (set this in mbed_app.json):  {local_ip}")
    except Exception:
        print("  Could not detect local IP — check manually with ifconfig/ipconfig")
    print(f"  Dashboard URL: http://localhost:{args.port}")
    print(f"  Board POSTs to: http://<this-ip>:{args.port}/event")
    print("=" * 55)

    app.run(host=args.host, port=args.port, threaded=True)
