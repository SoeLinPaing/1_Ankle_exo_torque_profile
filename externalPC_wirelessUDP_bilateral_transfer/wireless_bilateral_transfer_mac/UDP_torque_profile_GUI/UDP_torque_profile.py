import sys, socket, struct, csv, time
from PyQt5 import QtWidgets, QtCore, QtGui
import pyqtgraph as pg

# -------------------------------------------------------------
# Configuration
# -------------------------------------------------------------
RECV_PORT = 5005
NUM_FLOATS = 17  # includes peak_torque
UPDATE_RATE_HZ = 30.0
DATA_TIMEOUT = 0.5
PLOT_WINDOW_SEC = 10.0

RPI_IP = "10.152.46.85"
RPI_CMD_PORT = 5006

# -------------------------------------------------------------
# UDP Telemetry Receiver
# -------------------------------------------------------------
class UDPWorker(QtCore.QThread):
    new_data = QtCore.pyqtSignal(*(float,) * NUM_FLOATS)
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
                    vals = struct.unpack(f"{NUM_FLOATS}f", data)
                    self.new_data.emit(*vals)
            except socket.timeout:
                continue
            except Exception as e:
                print("UDP error:", e)

# -------------------------------------------------------------
# Toggle Switch Widget
# -------------------------------------------------------------
class ToggleSwitch(QtWidgets.QCheckBox):
    toggled_on = QtCore.pyqtSignal(bool)
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedSize(60, 30)
        self.setCursor(QtCore.Qt.PointingHandCursor)
        self.stateChanged.connect(lambda s: self.toggled_on.emit(s == QtCore.Qt.Checked))
    def mousePressEvent(self, e):
        self.setChecked(not self.isChecked()); e.accept()
    def paintEvent(self, e):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing)
        rect = self.rect(); radius = rect.height()/2; margin=3
        handle_r = radius - margin
        bg = QtGui.QColor("#4CAF50" if self.isChecked() else "#bbb")
        p.setBrush(bg); p.setPen(QtCore.Qt.NoPen)
        p.drawRoundedRect(rect, radius, radius)
        x = rect.width()-radius if self.isChecked() else radius
        p.setBrush(QtGui.QColor("white"))
        p.drawEllipse(QtCore.QPointF(x, radius), handle_r, handle_r)
        p.setPen(QtGui.QPen(QtCore.Qt.white))
        font = p.font(); font.setBold(True); font.setPointSize(9)
        p.setFont(font)
        p.drawText(rect, QtCore.Qt.AlignCenter, "ON" if self.isChecked() else "OFF")
        p.end()

# -------------------------------------------------------------
# Main GUI Window
# -------------------------------------------------------------
class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Exoskeleton UDP Telemetry + Motor Control")
        self.resize(1350, 900)

        # UDP control socket
        self.ctrl_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.RPI_ADDR = (RPI_IP, RPI_CMD_PORT)

        # ---------------------------------------------------------
        # Left plots
        # ---------------------------------------------------------
        self.plot_tau   = pg.PlotWidget(title="Torque (Nm)")
        self.plot_q     = pg.PlotWidget(title="Angle (deg)")
        self.plot_dq    = pg.PlotWidget(title="Angular Velocity (deg/s)")
        self.plot_ads   = pg.PlotWidget(title="ADS Voltage (V)")
        self.plot_phase = pg.PlotWidget(title="Gait Phase (0–1)")
        for p in [self.plot_q, self.plot_dq, self.plot_ads, self.plot_phase]:
            p.setXLink(self.plot_tau)
        self.curve_tau = self.plot_tau.plot(pen='b')
        self.curve_q   = self.plot_q.plot(pen='r')
        self.curve_dq  = self.plot_dq.plot(pen='m')
        self.curve_ads = self.plot_ads.plot(pen='g')
        self.curve_phase = self.plot_phase.plot(pen='k')

        left_layout = QtWidgets.QVBoxLayout()
        for w in [self.plot_tau, self.plot_q, self.plot_dq, self.plot_ads, self.plot_phase]:
            left_layout.addWidget(w)

        # ---------------------------------------------------------
        # Right layout
        # ---------------------------------------------------------
        right_layout = QtWidgets.QVBoxLayout()

        # Status
        self.status_label = QtWidgets.QLabel("Receiving Data: ●")
        self.status_label.setStyleSheet("font-size:16px; color:gray;")
        self.last_data_time = 0.0
        right_layout.addWidget(self.status_label)

        # ---------------------------------------------------------
        # Motor control box
        # ---------------------------------------------------------
        self.toggle_label = QtWidgets.QLabel("<b>Motor Disabled</b>")
        self.toggle_label.setStyleSheet("font-size:18px; color:red;")
        self.toggle_switch = ToggleSwitch()
        self.toggle_switch.toggled_on.connect(self.send_motor_command)

        motor_box = QtWidgets.QGroupBox("Motor Control")
        motor_layout = QtWidgets.QHBoxLayout()
        motor_layout.addWidget(self.toggle_switch)
        motor_layout.addWidget(self.toggle_label)
        motor_box.setLayout(motor_layout)
        right_layout.addWidget(motor_box)

        # ---------------------------------------------------------
        # Parameter input box
        # ---------------------------------------------------------
        input_box = QtWidgets.QGroupBox("Set Torque Profile Parameters")
        input_layout = QtWidgets.QGridLayout()
        self.in_peak_torque = QtWidgets.QDoubleSpinBox(); self.in_peak_torque.setRange(0,50); self.in_peak_torque.setValue(5.0)
        self.in_rise_time   = QtWidgets.QDoubleSpinBox(); self.in_rise_time.setRange(0,2); self.in_rise_time.setValue(0.7)
        self.in_peak_time   = QtWidgets.QDoubleSpinBox(); self.in_peak_time.setRange(0,2); self.in_peak_time.setValue(0.8)
        self.in_fall_time   = QtWidgets.QDoubleSpinBox(); self.in_fall_time.setRange(0,2); self.in_fall_time.setValue(0.1)

        for sb in [self.in_peak_torque, self.in_rise_time, self.in_peak_time, self.in_fall_time]:
            sb.setDecimals(3); sb.setSingleStep(0.05); sb.setFixedWidth(100)

        input_layout.addWidget(QtWidgets.QLabel("Peak Torque (Nm):"), 0, 0)
        input_layout.addWidget(self.in_peak_torque, 0, 1)
        input_layout.addWidget(QtWidgets.QLabel("Rise Time:"), 1, 0)
        input_layout.addWidget(self.in_rise_time, 1, 1)
        input_layout.addWidget(QtWidgets.QLabel("Peak Time:"), 2, 0)
        input_layout.addWidget(self.in_peak_time, 2, 1)
        input_layout.addWidget(QtWidgets.QLabel("Fall Time:"), 3, 0)
        input_layout.addWidget(self.in_fall_time, 3, 1)

        self.btn_set_params = QtWidgets.QPushButton("Set Parameters")
        self.btn_set_params.clicked.connect(self.send_params)
        input_layout.addWidget(self.btn_set_params, 4, 0, 1, 2)
        input_box.setLayout(input_layout)
        right_layout.addWidget(input_box)

        # ---------------------------------------------------------
        # Stride Times Box
        # ---------------------------------------------------------
        stride_box = QtWidgets.QGroupBox("Stride Times (s)")
        stride_box.setStyleSheet("""
            QGroupBox {
                font-size: 17px;
                font-weight: bold;
                border: 2px solid #666;
                border-radius: 8px;
                margin-top: 10px;
            }
        """)
        self.stride_label = QtWidgets.QLabel("--")
        self.stride_label.setAlignment(QtCore.Qt.AlignTop | QtCore.Qt.AlignHCenter)
        self.stride_label.setFixedHeight(120)
        self.stride_label.setStyleSheet("font-size:16px; font-family:monospace; padding:5px;")
        self.avg_stride_label = QtWidgets.QLabel("<b>Average:</b> -- s")
        self.avg_stride_label.setAlignment(QtCore.Qt.AlignCenter)
        self.avg_stride_label.setStyleSheet("font-size:16px; color:#111; padding:4px;")

        stride_layout = QtWidgets.QVBoxLayout()
        stride_layout.setContentsMargins(10, 15, 10, 10)  # <-- top margin increased
        stride_layout.setSpacing(10)  # <-- adds vertical gap between widgets
        stride_layout.addWidget(self.stride_label)
        stride_layout.addWidget(self.avg_stride_label)
        stride_box.setLayout(stride_layout)
        right_layout.addWidget(stride_box)

        # ---------------------------------------------------------
# Current Torque Parameters Box (proper title spacing fix)
# ---------------------------------------------------------
        param_box = QtWidgets.QGroupBox("Current Torque Profile Parameters")
        param_box.setStyleSheet("""
    QGroupBox {
        font-size: 17px;
        font-weight: bold;
        border: 2px solid #666;
        border-radius: 8px;
        margin-top: 20px;
    }
    QGroupBox::title {
        subcontrol-origin: margin;
        subcontrol-position: top center;
        padding: 6px 10px 12px 10px;  /* extra bottom padding pushes content down */
        background-color: transparent;
    }
""")

        self.param_peak_torque = QtWidgets.QLabel("--")
        self.param_rise_time   = QtWidgets.QLabel("--")
        self.param_peak_time   = QtWidgets.QLabel("--")
        self.param_fall_time   = QtWidgets.QLabel("--")

        for lbl in [self.param_peak_torque, self.param_rise_time, self.param_peak_time, self.param_fall_time]:
            lbl.setStyleSheet("font-size:16px; font-family:monospace; padding:2px;")

        param_layout = QtWidgets.QFormLayout()
        param_layout.setLabelAlignment(QtCore.Qt.AlignRight)
        param_layout.setContentsMargins(15, 25, 15, 15)  # more space under title
        param_layout.setVerticalSpacing(10)
        param_layout.addRow("Peak Torque (Nm):", self.param_peak_torque)
        param_layout.addRow("Rise Time:", self.param_rise_time)
        param_layout.addRow("Peak Time:", self.param_peak_time)
        param_layout.addRow("Fall Time:", self.param_fall_time)

        param_box.setLayout(param_layout)
        right_layout.addWidget(param_box)

        # ---------------------------------------------------------
        # Bottom recording layout
        # ---------------------------------------------------------
        ctrl_layout = QtWidgets.QHBoxLayout()
        self.filename = QtWidgets.QLineEdit("session.csv")
        self.record_btn = QtWidgets.QPushButton("Start Recording")
        ctrl_layout.addWidget(QtWidgets.QLabel("File:"))
        ctrl_layout.addWidget(self.filename)
        ctrl_layout.addWidget(self.record_btn)

        # Combine main layout
        main_layout = QtWidgets.QHBoxLayout()
        main_layout.addLayout(left_layout, 4)
        main_layout.addLayout(right_layout, 1)

        full_layout = QtWidgets.QVBoxLayout()
        full_layout.addLayout(main_layout)
        full_layout.addLayout(ctrl_layout)

        central = QtWidgets.QWidget()
        central.setLayout(full_layout)
        self.setCentralWidget(central)

        # ---------------------------------------------------------
        # Data and timers
        # ---------------------------------------------------------
        self.t, self.tau, self.q, self.dq, self.ads, self.phase = [], [], [], [], [], []
        self.recording = False; self.csv_file = None; self.csv_writer = None
        self.record_btn.clicked.connect(self.toggle_record)

        self.worker = UDPWorker()
        self.worker.new_data.connect(self.update_data)
        self.worker.start()

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(int(1000 / UPDATE_RATE_HZ))

    # ---------------------------------------------------------
    def send_motor_command(self, enabled):
        msg = b"ENABLE" if enabled else b"DISABLE"
        color = "green" if enabled else "red"
        text = "Enabled" if enabled else "Disabled"
        self.toggle_label.setText(f"<b>Motor {text}</b>")
        self.toggle_label.setStyleSheet(f"font-size:18px; color:{color};")
        self.ctrl_sock.sendto(msg, self.RPI_ADDR)

    # ---------------------------------------------------------
    def send_params(self):
        pt = self.in_peak_torque.value()
        r  = self.in_rise_time.value()
        p  = self.in_peak_time.value()
        f  = self.in_fall_time.value()
        msg = f"PARAM {pt:.3f} {r:.3f} {p:.3f} {f:.3f}".encode()
        self.ctrl_sock.sendto(msg, self.RPI_ADDR)
        QtWidgets.QMessageBox.information(
            self, "Parameters Sent",
            f"Sent to RPi:\nPeakTorque={pt:.2f}, Rise={r:.2f}, Peak={p:.2f}, Fall={f:.2f}"
        )

    # ---------------------------------------------------------
    def update_data(self, *vals):
        (t_sched, t_actual, jitter, tau, q, dq, adsV, gait_phase,
         peak_torque, rise, peak, fall, s1, s2, s3, s4, s5) = vals
        self.last_data_time = time.time()
        self.status_label.setStyleSheet("font-size:16px; color:green;")
        self.t.append(t_actual); self.tau.append(tau); self.q.append(q)
        self.dq.append(dq); self.ads.append(adsV); self.phase.append(gait_phase)
        strides = [s for s in [s1,s2,s3,s4,s5] if s>0]
        if strides:
            self.stride_label.setText("\n".join(f"{v:6.3f}" for v in strides[-5:]))
            self.avg_stride_label.setText(f"<b>Average:</b> {sum(strides)/len(strides):6.3f} s")
        else:
            self.stride_label.setText("--"); self.avg_stride_label.setText("<b>Average:</b> -- s")
        self.param_peak_torque.setText(f"{peak_torque:6.2f}")
        self.param_rise_time.setText(f"{rise:5.2f}")
        self.param_peak_time.setText(f"{peak:5.2f}")
        self.param_fall_time.setText(f"{fall:5.2f}")
        while self.t and (self.t[-1]-self.t[0])>PLOT_WINDOW_SEC:
            self.t.pop(0); self.tau.pop(0); self.q.pop(0)
            self.dq.pop(0); self.ads.pop(0); self.phase.pop(0)
        if self.recording and self.csv_writer: self.csv_writer.writerow(vals)

    # ---------------------------------------------------------
    def toggle_record(self):
        if not self.recording:
            self.csv_file=open(self.filename.text(),"w",newline="")
            self.csv_writer=csv.writer(self.csv_file)
            self.csv_writer.writerow([
                "t_sched","t_actual","jitter_ns","tau_des","q","dq",
                "adsV","gait_phase","peak_torque","rise","peak","fall",
                "stride1","stride2","stride3","stride4","stride5"])
            self.recording=True; self.record_btn.setText("Stop Recording")
        else:
            self.recording=False
            if self.csv_file: self.csv_file.close()
            self.record_btn.setText("Start Recording")

    def update_plot(self):
        if time.time()-self.last_data_time>DATA_TIMEOUT:
            self.status_label.setStyleSheet("font-size:16px; color:gray;")
        if not self.t: return
        tmin=max(0,self.t[-1]-PLOT_WINDOW_SEC)
        self.plot_tau.setXRange(tmin,self.t[-1],padding=0)
        self.curve_tau.setData(self.t,self.tau)
        self.curve_q.setData(self.t,self.q)
        self.curve_dq.setData(self.t,self.dq)
        self.curve_ads.setData(self.t,self.ads)
        self.curve_phase.setData(self.t,self.phase)

    def closeEvent(self, e):
        self.worker.running=False
        if self.csv_file: self.csv_file.close()
        e.accept()

# -------------------------------------------------------------
if __name__ == "__main__":
    app=QtWidgets.QApplication(sys.argv)
    pg.setConfigOptions(antialias=True,background='w',foreground='k')
    win=MainWindow(); win.show()
    sys.exit(app.exec_())