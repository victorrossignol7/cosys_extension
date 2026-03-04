# client_uav1.py
import json
import socket

HOST, PORT = "127.0.0.1", 50051
CMD_TOPIC = "/uav1/cmd"
ODOM_TOPIC = "/uav1/odom"


def send(sock, topic, op, args=None):
    msg = {"topic": topic, "op": op, "args": args or {}}
    sock.sendall((json.dumps(msg) + "\n").encode("utf-8"))
    data = b""
    while not data.endswith(b"\n"):
        chunk = sock.recv(4096)
        if not chunk:
            raise RuntimeError("Server closed connection")
        data += chunk
    return json.loads(data.decode("utf-8"))


with socket.create_connection((HOST, PORT)) as sock:
    print("[uav1] connected to orchestrator")

    r = send(sock, CMD_TOPIC, "takeoff")
    print(f"[uav1] takeoff ok={r.get('ok')}")

    r = send(sock, CMD_TOPIC, "moveToZ", {"z": -2.0, "speed": 1.5})
    print(f"[uav1] moveToZ ok={r.get('ok')}")

    for i in range(6):
        r = send(sock, CMD_TOPIC, "moveByVelocityZ", {"vx": 1.0, "vy": 0.0, "z": -2.0, "duration": 0.6})
        if not r.get("ok"):
            print("[uav1] error:", r)
            break
        st = r["state"]["pos"]
        print(f"[uav1] {i} pos=({st['x']:.2f},{st['y']:.2f},{st['z']:.2f}) alt≈{st['alt']:.2f}")

        od = send(sock, ODOM_TOPIC, "get_state")
        if od.get("ok"):
            odpos = od["state"]["pos"]
            print(f"[uav1] odom alt≈{odpos['alt']:.2f}")

    r = send(sock, CMD_TOPIC, "land")
    print(f"[uav1] land ok={r.get('ok')}")
