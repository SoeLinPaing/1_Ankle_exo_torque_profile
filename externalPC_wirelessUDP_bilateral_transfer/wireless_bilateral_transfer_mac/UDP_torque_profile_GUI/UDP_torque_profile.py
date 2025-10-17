import sys, socket, struct, csv, time
from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg

# -------------------------------------------------------------
# Configuration
# -------------------------------------------------------------
RECV_PORT = 5005
NUM_FLOATS = 16
UPDATE_RATE_HZ = 30.0
DATA_TIMEOUT = 0.5
PLOT_WINDOW_SEC = 10.0

# -------------------------------------------------------------
# UDP Thread
# -------------------------------------------------------------
class UDPWorker(QtCore.QThread):
    new_data = QtCore.pyqtSignal(float, float, float, float, float, float,
                                 float, float, float, float, float,
                                 float, float, float, float, float)
    def __init__(self):
        super().__init__()
        self.running = True
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("", RECV_PORT))
        self.sock.settimeout(0.2)

    def run(self):
        while self.running:
            try:
                data, _ = self.sock.recvfrom(1024)
                if len(data) >= NUM_FLOATS * 4:
                    vals = struct.unpack("16f", data)
                    self.new_data.emit(*vals)
            except socket.timeout:
                continue
            except Exception as e:
                print("UDP error:", e)

# -------------------------------------------------------------
# Main GUI
# -------------------------------------------------------------
class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Exoskeleton UDP Telemetry (Stride + Status)")
        self.resize(1250, 900)

        # ---------- Left plots ----------
        self.plot_tau   = pg.PlotWidget(title="Torque (Nm)")
        self.plot_q     = pg.PlotWidget(title="Angle (deg)")
        self.plot_dq    = pg.PlotWidget(title="Angular Velocity (deg/s)")
        self.plot_ads   = pg.PlotWidget(title="ADS Voltage (V)")
        self.plot_phase = pg.PlotWidget(title="Gait Phase (0–1)")
        for p in [self.plot_q, self.plot_dq, self.plot_ads, self.plot_phase]:
            p.setXLink(self.plot_tau)

        self.curve_tau   = self.plot_tau.plot(pen='b')
        self.curve_q     = self.plot_q.plot(pen='r')
        self.curve_dq    = self.plot_dq.plot(pen='m')
        self.curve_ads   = self.plot_ads.plot(pen='g')
        self.curve_phase = self.plot_phase.plot(pen='k')

        left_layout = QtWidgets.QVBoxLayout()
        for w in [self.plot_tau, self.plot_q, self.plot_dq, self.plot_ads, self.plot_phase]:
            left_layout.addWidget(w)

        # ---------- Right panel ----------
        # Status indicator at top
        self.status_label = QtWidgets.QLabel("Receiving Data: ●")
        self.status_label.setStyleSheet("font-size:16px; color:gray;")
        self.last_data_time = 0.0

        # Stride info (moved to bottom)
        stride_title = QtWidgets.QLabel("<b>Stride Times (s)</b>")
        stride_title.setStyleSheet("font-size:17px;")

        self.stride_label = QtWidgets.QLabel("--")
        self.stride_label.setAlignment(QtCore.Qt.AlignTop)
        self.stride_label.setStyleSheet(
            "font-size:16px; font-family:monospace; border:1px solid #ccc; padding:5px;")
        self.stride_label.setMinimumHeight(120)

        self.avg_stride_label = QtWidgets.QLabel("<b>Average:</b> -- s")
        self.avg_stride_label.setStyleSheet(
            "font-size:18px; font-family:monospace; color:white; "
            "background-color:#222; border:1px solid #333; padding:8px; "
            "border-radius:6px;")
        self.avg_stride_label.setMinimumHeight(50)
        self.avg_stride_label.setAlignment(QtCore.Qt.AlignCenter)

        # Stride info box (bottom-anchored)
        stride_box = QtWidgets.QVBoxLayout()
        stride_box.addWidget(stride_title)
        stride_box.addWidget(self.stride_label)
        stride_box.addWidget(self.avg_stride_label)

        # Right column layout
        right_layout = QtWidgets.QVBoxLayout()
        right_layout.addWidget(self.status_label)
        right_layout.addStretch(1)             # push stride box to bottom
        right_layout.addLayout(stride_box)
        right_layout.setContentsMargins(10, 20, 10, 20)

        # ---------- Controls ----------
        ctrl_layout = QtWidgets.QHBoxLayout()
        self.filename = QtWidgets.QLineEdit("session.csv")
        self.record_btn = QtWidgets.QPushButton("Start Recording")
        ctrl_layout.addWidget(QtWidgets.QLabel("File:"))
        ctrl_layout.addWidget(self.filename)
        ctrl_layout.addWidget(self.record_btn)

        # ---------- Combine ----------
        main_layout = QtWidgets.QHBoxLayout()
        main_layout.addLayout(left_layout, 4)
        main_layout.addLayout(right_layout, 1)

        full_layout = QtWidgets.QVBoxLayout()
        full_layout.addLayout(main_layout)
        full_layout.addLayout(ctrl_layout)

        central = QtWidgets.QWidget()
        central.setLayout(full_layout)
        self.setCentralWidget(central)

        # ---------- Buffers ----------
        self.t, self.tau, self.q, self.dq, self.ads, self.phase = [], [], [], [], [], []
        self.stride_times = []
        self.recording = False
        self.csv_file = None
        self.csv_writer = None
        self.record_btn.clicked.connect(self.toggle_record)

        # ---------- UDP worker ----------
        self.worker = UDPWorker()
        self.worker.new_data.connect(self.update_data)
        self.worker.start()

        # ---------- Timer ----------
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(int(1000 / UPDATE_RATE_HZ))

    # ---------------------------------------------------------
    def update_stride_label(self, values):
        if values:
            txt = "\n".join(f"{v:6.3f}" for v in values[-5:])
            avg = sum(values[-5:]) / len(values[-5:])
            self.avg_stride_label.setText(f"<b>Average:</b> {avg:6.3f} s")
        else:
            txt = "--"
            self.avg_stride_label.setText("<b>Average:</b> -- s")
        self.stride_label.setText(txt)
        self.avg_stride_label.repaint()

    # ---------------------------------------------------------
    def toggle_record(self):
        if not self.recording:
            self.csv_file = open(self.filename.text(), "w", newline="")
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow([
                "t_sched","t_actual","jitter_ns",
                "tau_des_Nm","q_deg","dq_deg_s",
                "ads_voltage","gait_phase",
                "rise_time","peak_time","fall_time",
                "stride1","stride2","stride3","stride4","stride5"
            ])
            self.recording = True
            self.record_btn.setText("Stop Recording")
        else:
            self.recording = False
            if self.csv_file:
                self.csv_file.close()
            self.record_btn.setText("Start Recording")

    # ---------------------------------------------------------
    def update_data(self, t_sched, t_actual, jitter, tau, q, dq, adsV, gait_phase,
                    rise, peak, fall, s1, s2, s3, s4, s5):
        self.last_data_time = time.time()
        self.status_label.setStyleSheet("font-size:16px; color:green;")

        self.t.append(t_actual)
        self.tau.append(tau)
        self.q.append(q)
        self.dq.append(dq)
        self.ads.append(adsV)
        self.phase.append(gait_phase)

        self.stride_times = [s for s in [s1, s2, s3, s4, s5] if s > 0]
        self.update_stride_label(self.stride_times)

        while self.t and (self.t[-1] - self.t[0]) > PLOT_WINDOW_SEC:
            self.t.pop(0); self.tau.pop(0); self.q.pop(0)
            self.dq.pop(0); self.ads.pop(0); self.phase.pop(0)

        if self.recording and self.csv_writer:
            self.csv_writer.writerow([
                t_sched, t_actual, jitter, tau, q, dq, adsV, gait_phase,
                rise, peak, fall, s1, s2, s3, s4, s5
            ])

    # ---------------------------------------------------------
    def update_plot(self):
        if time.time() - self.last_data_time > DATA_TIMEOUT:
            self.status_label.setStyleSheet("font-size:16px; color:gray;")
        if not self.t:
            return
        tmin = max(0, self.t[-1] - PLOT_WINDOW_SEC)
        self.plot_tau.setXRange(tmin, self.t[-1], padding=0)
        self.curve_tau.setData(self.t, self.tau)
        self.curve_q.setData(self.t, self.q)
        self.curve_dq.setData(self.t, self.dq)
        self.curve_ads.setData(self.t, self.ads)
        self.curve_phase.setData(self.t, self.phase)

    # ---------------------------------------------------------
    def closeEvent(self, event):
        self.worker.running = False
        if self.csv_file:
            self.csv_file.close()
        event.accept()

# -------------------------------------------------------------
if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    pg.setConfigOptions(antialias=True, background='w', foreground='k')
    win = MainWindow()
    win.show()
    sys.exit(app.exec_())