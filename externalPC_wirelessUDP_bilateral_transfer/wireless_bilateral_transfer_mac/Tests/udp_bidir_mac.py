# Mac UDP bilateral exchange with Raspberry Pi
# Run: python3 udp_bidir_mac.py
#
# Sends two variables (emg_level, gait_state) to Pi
# Receives two variables (torque_cmd, ankle_angle) from Pi

import socket, time, math, threading

PI_IP      = "10.152.63.189"
SEND_PORT  = 5006   # to Pi
RECV_PORT  = 5005   # from Pi
running = True

def recv_loop(sock):
    while running:
        try:
            data, _ = sock.recvfrom(1024)
            msg = data.decode().strip()
            vals = msg.split(",")
            if len(vals) >= 2:
                torque = float(vals[0])
                angle  = float(vals[1])
                print(f"[RX Pi] torque={torque:.2f} angle={angle:.2f}")
        except Exception:
            pass

def send_loop(sock, dest):
    freq = 50.0
    period = 1.0 / freq
    t0 = time.time()
    while running:
        t = time.time() - t0
        emg_level = 0.5 + 0.5 * math.sin(2 * math.pi * 0.7 * t)
        gait_state = 1.0 if int(t * 1.4) % 2 == 0 else 0.0
        msg = f"{emg_level:.3f},{gait_state:.3f}\n"
        sock.sendto(msg.encode(), dest)
        time.sleep(period)

if __name__ == "__main__":
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("", RECV_PORT))
    dest = (PI_IP, SEND_PORT)

    threading.Thread(target=recv_loop, args=(sock,), daemon=True).start()
    try:
        send_loop(sock, dest)
    except KeyboardInterrupt:
        running = False
        sock.close()
        print("Stopped.")