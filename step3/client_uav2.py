# client_uav2.py
import json
import socket

HOST, PORT = "127.0.0.1", 50051
CMD_TOPIC = "/uav2/cmd"
ODOM_TOPIC = "/uav2/odom"


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
    print("[uav2] connected to orchestrator")

    r = send(sock, CMD_TOPIC, "takeoff")
    print(f"[uav2] takeoff ok={r.get('ok')}")

    r = send(sock, CMD_TOPIC, "moveToZ", {"z": -3.0, "speed": 1.5})
    print(f"[uav2] moveToZ ok={r.get('ok')}")

    for i in range(6):
        r = send(sock, CMD_TOPIC, "moveByVelocityZ", {"vx": 0.6, "vy": 0.6, "z": -3.0, "duration": 0.6})
        if not r.get("ok"):
            print("[uav2] error:", r)
            break
        st = r["state"]["pos"]
        print(f"[uav2] {i} pos=({st['x']:.2f},{st['y']:.2f},{st['z']:.2f}) alt≈{st['alt']:.2f}")

        od = send(sock, ODOM_TOPIC, "get_state")
        if od.get("ok"):
            odpos = od["state"]["pos"]
            print(f"[uav2] odom alt≈{odpos['alt']:.2f}")

    r = send(sock, CMD_TOPIC, "land")
    print(f"[uav2] land ok={r.get('ok')}")
