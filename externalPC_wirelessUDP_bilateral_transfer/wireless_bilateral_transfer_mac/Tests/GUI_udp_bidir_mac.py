import sys, socket, threading, time, math, csv
import numpy as np
from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg

PI_IP = "10.152.63.189"
SEND_PORT, RECV_PORT = 5006, 5005
FREQ_HZ = 50.0

class UDPWorker(QtCore.QThread):
    new_data = QtCore.pyqtSignal(float, float, float)  # time, torque, angle

    def __init__(self):
        super().__init__()
        self.running = True
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("", RECV_PORT))
        self.dest = (PI_IP, SEND_PORT)

    def run(self):
        recv_thread = threading.Thread(target=self.recv_loop, daemon=True)
        recv_thread.start()
        self.send_loop()

    def recv_loop(self):
        while self.running:
            try:
                data, _ = self.sock.recvfrom(1024)
                torque, angle = map(float, data.decode().strip().split(","))
                self.new_data.emit(time.time(), torque, angle)
            except:
                pass

    def send_loop(self):
        t0 = time.time()
        while self.running:
            t = time.time() - t0
            emg = 0.5 + 0.5 * math.sin(2*math.pi*0.7*t)
            gait = 1.0 if int(t*1.4) % 2 == 0 else 0.0
            msg = f"{emg:.3f},{gait:.3f}\n"
            self.sock.sendto(msg.encode(), self.dest)
            time.sleep(1.0 / FREQ_HZ)

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Exoskeleton UDP Monitor")
        self.resize(900, 600)

        # Widgets
        self.plot = pg.PlotWidget(title="Torque & Angle")
        self.torque_curve = self.plot.plot(pen='b', name="Torque")
        self.angle_curve = self.plot.plot(pen='r', name="Angle")

        self.record_btn = QtWidgets.QPushButton("Start Recording")
        self.filename = QtWidgets.QLineEdit("session.csv")
        self.record_btn.clicked.connect(self.toggle_record)

        ctrl = QtWidgets.QHBoxLayout()
        ctrl.addWidget(QtWidgets.QLabel("File:"))
        ctrl.addWidget(self.filename)
        ctrl.addWidget(self.record_btn)

        layout = QtWidgets.QVBoxLayout()
        layout.addWidget(self.plot)
        layout.addLayout(ctrl)

        widget = QtWidgets.QWidget()
        widget.setLayout(layout)
        self.setCentralWidget(widget)

        # Data buffers
        self.t_data, self.torque_data, self.angle_data = [], [], []
        self.recording = False
        self.csv_file = None
        self.csv_writer = None

        # Worker thread
        self.worker = UDPWorker()
        self.worker.new_data.connect(self.update_plot)
        self.worker.start()

    def toggle_record(self):
        if not self.recording:
            self.csv_file = open(self.filename.text(), "w", newline="")
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(["time", "torque", "angle"])
            self.recording = True
            self.record_btn.setText("Stop Recording")
        else:
            self.recording = False
            if self.csv_file:
                self.csv_file.close()
            self.record_btn.setText("Start Recording")

    def update_plot(self, t, torque, angle):
        self.t_data.append(t)
        self.torque_data.append(torque)
        self.angle_data.append(angle)
        if len(self.t_data) > 500:
            self.t_data = self.t_data[-500:]
            self.torque_data = self.torque_data[-500:]
            self.angle_data = self.angle_data[-500:]
        t0 = self.t_data[0]
        times = [ti - t0 for ti in self.t_data]
        self.torque_curve.setData(times, self.torque_data)
        self.angle_curve.setData(times, self.angle_data)
        if self.recording and self.csv_writer:
            self.csv_writer.writerow([t, torque, angle])

    def closeEvent(self, event):
        self.worker.running = False
        if self.csv_file:
            self.csv_file.close()
        event.accept()

app = QtWidgets.QApplication(sys.argv)
win = MainWindow()
win.show()
sys.exit(app.exec_())