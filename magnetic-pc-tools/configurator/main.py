import sys
import struct
import hid
import time
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QLabel, QRadioButton, QSlider, 
                             QPushButton, QGroupBox, QMessageBox, QDoubleSpinBox)
from PyQt5.QtCore import Qt, QTimer

# Standard VIDs and PIDs for the device
# When in Joystick Mode
VID_JOY = 0x0483  # 1155
PID_JOY = 0x572B  # 22315
# When in SpaceMouse Mode
VID_SM = 0x256F
PID_SM = 0xC626

class MagneticConfigurator(QMainWindow):
    def __init__(self):
        super().__init__()
        self.device = None
        self.current_vid = None
        self.current_pid = None
        
        self.initUI()
        
        # Timer to auto-detect device connection
        self.timer = QTimer()
        self.timer.timeout.connect(self.check_connection)
        self.timer.start(1000)
        self.check_connection()

    def initUI(self):
        self.setWindowTitle('Magnetic Podium Configurator')
        self.setFixedSize(500, 500)
        self.setStyleSheet("QWidget { font-size: 14px; }")

        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)

        # Status Label
        self.status_label = QLabel("Status: Disconnected")
        self.status_label.setStyleSheet("color: red; font-weight: bold;")
        main_layout.addWidget(self.status_label)

        # Mode Selection
        mode_group = QGroupBox("USB Operating Mode")
        mode_layout = QHBoxLayout()
        self.radio_joy = QRadioButton("Generic Joystick (0x0483:0x572B)")
        self.radio_sm = QRadioButton("SpaceMouse 3D (0x256F:0xC626)")
        self.radio_joy.toggled.connect(lambda: self.switch_mode(0) if self.radio_joy.isChecked() else None)
        self.radio_sm.toggled.connect(lambda: self.switch_mode(1) if self.radio_sm.isChecked() else None)
        mode_layout.addWidget(self.radio_joy)
        mode_layout.addWidget(self.radio_sm)
        mode_group.setLayout(mode_layout)
        main_layout.addWidget(mode_group)

        # Sensitivities
        sens_group = QGroupBox("Sensitivities")
        self.sens_layout = QVBoxLayout()
        self.sliders = []
        self.spinboxes = []
        labels = ['X Pos', 'Y Pos', 'Z Pos', 'Pitch', 'Yaw', 'Roll']
        
        for i, label in enumerate(labels):
            h_layout = QHBoxLayout()
            lbl = QLabel(label)
            lbl.setFixedWidth(60)
            
            slider = QSlider(Qt.Horizontal)
            slider.setMinimum(1)
            slider.setMaximum(500)
            slider.setValue(100) # 1.0
            slider.setTickPosition(QSlider.TicksBelow)
            slider.setTickInterval(50)
            
            spinbox = QDoubleSpinBox()
            spinbox.setMinimum(0.01)
            spinbox.setMaximum(5.0)
            spinbox.setSingleStep(0.1)
            spinbox.setValue(1.0)
            spinbox.setDecimals(2)
            
            # Connect the signals
            slider.valueChanged.connect(lambda val, idx=i: self.sync_spinbox(idx, val))
            spinbox.valueChanged.connect(lambda val, idx=i: self.sync_slider(idx, val))
            
            # Update device on release
            slider.sliderReleased.connect(self.send_sensitivities)
            spinbox.editingFinished.connect(self.send_sensitivities)
            
            h_layout.addWidget(lbl)
            h_layout.addWidget(slider)
            h_layout.addWidget(spinbox)
            
            self.sliders.append(slider)
            self.spinboxes.append(spinbox)
            self.sens_layout.addLayout(h_layout)
            
        sens_group.setLayout(self.sens_layout)
        main_layout.addWidget(sens_group)

        # Calibration Actions
        calib_group = QGroupBox("Calibration Actions")
        calib_layout = QHBoxLayout()
        self.btn_calib = QPushButton("Trigger Offset Calibration")
        self.btn_save = QPushButton("Save Calibration to Flash")
        
        self.btn_calib.clicked.connect(self.trigger_calibration)
        self.btn_save.clicked.connect(self.save_calibration)
        
        calib_layout.addWidget(self.btn_calib)
        calib_layout.addWidget(self.btn_save)
        calib_group.setLayout(calib_layout)
        main_layout.addWidget(calib_group)
        
        self.update_ui_state(False)

    def sync_spinbox(self, idx, val):
        self.spinboxes[idx].blockSignals(True)
        self.spinboxes[idx].setValue(val / 100.0)
        self.spinboxes[idx].blockSignals(False)

    def sync_slider(self, idx, val):
        self.sliders[idx].blockSignals(True)
        self.sliders[idx].setValue(int(val * 100))
        self.sliders[idx].blockSignals(False)

    def check_connection(self):
        connected = False
        try:
            # Check for SpaceMouse
            devs = hid.enumerate(VID_SM, PID_SM)
            if devs:
                if self.current_vid != VID_SM:
                    self.connect_device(VID_SM, PID_SM, "SpaceMouse Mode")
                    self.radio_sm.blockSignals(True)
                    self.radio_sm.setChecked(True)
                    self.radio_sm.blockSignals(False)
                connected = True
            else:
                # Check for Joystick
                devs = hid.enumerate(VID_JOY, PID_JOY)
                if devs:
                    if self.current_vid != VID_JOY:
                        self.connect_device(VID_JOY, PID_JOY, "Joystick Mode")
                        self.radio_joy.blockSignals(True)
                        self.radio_joy.setChecked(True)
                        self.radio_joy.blockSignals(False)
                    connected = True
        except Exception as e:
            print(f"Error enumerating: {e}")
            
        if not connected and self.device is not None:
            self.disconnect_device()

    def connect_device(self, vid, pid, mode_name):
        try:
            if self.device:
                self.device.close()
            self.device = hid.device()
            self.device.open(vid, pid)
            self.current_vid = vid
            self.current_pid = pid
            
            self.status_label.setText(f"Status: Connected ({mode_name})")
            self.status_label.setStyleSheet("color: green; font-weight: bold;")
            self.update_ui_state(True)
            print(f"Connected to {mode_name}")
            
            # Send current sensitivities to synchronize
            self.send_sensitivities()
        except IOError as ex:
            print(f"Failed to open device: {ex}")
            self.disconnect_device()

    def disconnect_device(self):
        if self.device:
            try:
                self.device.close()
            except:
                pass
        self.device = None
        self.current_vid = None
        self.current_pid = None
        self.status_label.setText("Status: Disconnected")
        self.status_label.setStyleSheet("color: red; font-weight: bold;")
        self.update_ui_state(False)

    def update_ui_state(self, enabled):
        self.radio_joy.setEnabled(enabled)
        self.radio_sm.setEnabled(enabled)
        self.btn_calib.setEnabled(enabled)
        self.btn_save.setEnabled(enabled)
        for slider in self.sliders:
            slider.setEnabled(enabled)
        for spinbox in self.spinboxes:
            spinbox.setEnabled(enabled)

    def send_feature_report(self, payload):
        """Send a 32-byte Feature Report with ID 0x10"""
        if not self.device:
            return False
            
        # Payload must be exactly 32 bytes (STM32 side expects 32 bytes)
        # hidapi requires the first byte to be the Report ID
        report_id = 0x10
        padded_payload = list(payload)
        while len(padded_payload) < 32:
            padded_payload.append(0x00)
            
        data = [report_id] + padded_payload[:32]
        
        try:
            res = self.device.send_feature_report(data)
            if res < 0:
                print("Failed to send feature report")
                return False
            print(f"Sent {res} bytes: {[hex(x) for x in data[:8]]}...")
            return True
        except IOError as e:
            print(f"IOError sending feature report: {e}")
            self.disconnect_device()
            return False

    def switch_mode(self, mode):
        # mode 0 = Joystick, 1 = SpaceMouse
        print(f"Switching mode to {mode}")
        success = self.send_feature_report([0x01, mode])
        if success:
            # Reconnecting...
            self.disconnect_device()
            QMessageBox.information(self, "Mode Switch", 
                                    "Device is rebooting into the new USB mode. Please wait...",
                                    QMessageBox.Ok)

    def send_sensitivities(self):
        if not self.device:
            return
            
        # Read 6 floats from spinboxes
        vals = [sb.value() for sb in self.spinboxes]
        print(f"Sending sensitivities: {vals}")
        
        # Pack into floats (little-endian: '<')
        packed_floats = struct.pack('<ffffff', *vals)
        
        payload = [0x02] + list(packed_floats)
        self.send_feature_report(payload)

    def trigger_calibration(self):
        print("Triggering calibration")
        if self.send_feature_report([0x03]):
            QMessageBox.information(self, "Calibration", 
                                    "Calibration command sent. Keep the device absolutely still.",
                                    QMessageBox.Ok)

    def save_calibration(self):
        print("Saving calibration")
        if self.send_feature_report([0x04]):
            QMessageBox.information(self, "Calibration Saved", 
                                    "Stored to Flash.",
                                    QMessageBox.Ok)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    
    # Modern dark mode look
    from PyQt5.QtGui import QPalette, QColor
    palette = QPalette()
    palette.setColor(QPalette.Window, QColor(53, 53, 53))
    palette.setColor(QPalette.WindowText, Qt.white)
    palette.setColor(QPalette.Base, QColor(25, 25, 25))
    palette.setColor(QPalette.AlternateBase, QColor(53, 53, 53))
    palette.setColor(QPalette.ToolTipBase, Qt.white)
    palette.setColor(QPalette.ToolTipText, Qt.white)
    palette.setColor(QPalette.Text, Qt.white)
    palette.setColor(QPalette.Button, QColor(53, 53, 53))
    palette.setColor(QPalette.ButtonText, Qt.white)
    palette.setColor(QPalette.BrightText, Qt.red)
    palette.setColor(QPalette.Link, QColor(42, 130, 218))
    palette.setColor(QPalette.Highlight, QColor(42, 130, 218))
    palette.setColor(QPalette.HighlightedText, Qt.black)
    app.setPalette(palette)
    
    ex = MagneticConfigurator()
    ex.show()
    sys.exit(app.exec_())
