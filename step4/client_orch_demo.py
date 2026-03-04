import socket, json

HOST, PORT = "127.0.0.1", 50051

def send(w, r, vehicle, op, args=None):
    req = {"vehicle": vehicle, "op": op, "args": args or {}}
    w.write(json.dumps(req) + "\n")
    w.flush()
    resp = r.readline().strip()
    print("<<", resp)

with socket.create_connection((HOST, PORT), timeout=60) as s:
    s.settimeout(60)

    r = s.makefile("r", encoding="utf-8", newline="\n")
    w = s.makefile("w", encoding="utf-8", newline="\n")

    print("<<", r.readline().strip())

    send(w, r, "Drone1", "takeoff")
    send(w, r, "Drone1", "moveToZ", {"z": -2, "v": 1.0})
    send(w, r, "Drone1", "hover")
    send(w, r, "UGV1", "drive", {"throttle": 0.6, "steering": 0.0, "duration": 8.0})
    send(w, r, "UGV1", "stop")
    send(w, r, "Drone1", "land")
