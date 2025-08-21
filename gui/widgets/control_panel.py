from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGroupBox, QLabel,
    QPushButton, QSlider, QSpinBox, QGridLayout,
    QMessageBox, QScrollArea, QDoubleSpinBox
)
from PyQt5.QtCore import Qt, pyqtSignal

class ActuatorControlWidget(QWidget):
    # Define specific signals for clarity
    pressure_command_sent = pyqtSignal(list)  # For direct pressure control
    length_command_sent = pyqtSignal(list)    # Signal for direct length control
    emergency_stop_sent = pyqtSignal()
    start_sequence_sent = pyqtSignal()
    stop_sequence_sent = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.init_ui()

    def init_ui(self):
        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        container = QWidget()
        layout = QVBoxLayout(container)
        
        # --- Direct Length Control ---
        length_group = QGroupBox("Direct Length Control")
        length_layout = QGridLayout(length_group)
        self.length_spinboxes = []
        
        # Add length controls for each actuator
        for i in range(3):
            label = QLabel(f"L{i+1} (mm):")
            spinbox = QDoubleSpinBox()
            spinbox.setRange(100.0, 300.0)  # Increased max range from 250 to 300mm
            spinbox.setValue(200.0)  # Default neutral position
            spinbox.setSingleStep(1.0)
            spinbox.setDecimals(1)
            self.length_spinboxes.append(spinbox)
            length_layout.addWidget(label, i, 0)
            length_layout.addWidget(spinbox, i, 1)
            
        # Add send button
        self.send_length_btn = QPushButton("Send Direct Length Command")
        self.send_length_btn.clicked.connect(self.send_length_command)
        length_layout.addWidget(self.send_length_btn, 3, 0, 1, 2)

        # --- Secondary Control: Direct Pressure ---
        pressure_group = QGroupBox("Direct Pressure Control (Manual Override)")
        pressure_layout = QGridLayout(pressure_group)
        self.actuator_sliders = []
        self.actuator_spinboxes = []
        for i in range(3):
            label = QLabel(f"Actuator {i+1}:")
            slider = QSlider(Qt.Horizontal)
            slider.setRange(0, 130)  # Range 0.0-13.0 kPa (multiplied by 10 for slider precision)
            slider.setValue(0)       # Default to 0 kPa (atmospheric pressure)
            spinbox = QDoubleSpinBox()
            spinbox.setRange(0.0, 13.0) # Range 0.0-13.0 kPa 
            spinbox.setValue(0.0)       # Default to 0 kPa (atmospheric pressure)
            spinbox.setSuffix(" kPa")
            spinbox.setSingleStep(0.1)  # Allow fine control with 0.1 kPa steps
            spinbox.setDecimals(1)      # Show one decimal place
            
            # Connect spinbox and slider with value conversion
            slider.valueChanged.connect(lambda val, sb=spinbox: sb.setValue(val/10.0))
            spinbox.valueChanged.connect(lambda val, sl=slider: sl.setValue(int(val*10)))
            
            pressure_layout.addWidget(label, i, 0)
            pressure_layout.addWidget(slider, i, 1)
            pressure_layout.addWidget(spinbox, i, 2)
            
            self.actuator_sliders.append(slider)
            self.actuator_spinboxes.append(spinbox)
        self.send_pressure_btn = QPushButton("Send Direct Pressure Command")
        self.send_pressure_btn.clicked.connect(self.send_pressure_command)
        pressure_layout.addWidget(self.send_pressure_btn, 3, 0, 1, 3)

        # --- Other Controls ---
        sequence_group = QGroupBox("Auto Sequence Control")
        sequence_layout = QHBoxLayout(sequence_group)
        self.sequence_status = QLabel("Manual Mode")
        self.sequence_status.setStyleSheet("color: #4CAF50; font-weight: bold;")
        sequence_layout.addWidget(self.sequence_status)
        self.start_sequence_btn = QPushButton("Start Auto Sequence")
        self.start_sequence_btn.clicked.connect(self.start_sequence)
        sequence_layout.addWidget(self.start_sequence_btn)
        self.stop_sequence_btn = QPushButton("Stop Auto Sequence")
        self.stop_sequence_btn.clicked.connect(self.stop_sequence)
        self.stop_sequence_btn.setEnabled(False)
        sequence_layout.addWidget(self.stop_sequence_btn)

        self.emergency_btn = QPushButton("EMERGENCY STOP")
        self.emergency_btn.setStyleSheet("background-color: red; color: white; font-weight: bold; font-size: 14pt; min-height: 40px;")
        self.emergency_btn.clicked.connect(self.emergency_stop)

        # Add widgets to layout in desired order
        layout.addWidget(length_group)     # Direct length control is primary
        layout.addWidget(pressure_group)   # Secondary/manual control
        layout.addWidget(sequence_group)
        layout.addWidget(self.emergency_btn)

        scroll_area.setWidget(container)
        main_layout = QVBoxLayout(self)
        main_layout.addWidget(scroll_area)

    def send_pressure_command(self):
        """Emit direct pressure command for the 3 actuators."""
        pressures = []
        print("ActuatorControlWidget: send_pressure_command called.")
        for i, spinbox in enumerate(self.actuator_spinboxes):
            val = spinbox.value() # Get QDoubleSpinBox value
            rounded_val = round(val, 1) # Round to 1 decimal place
            pressures.append(rounded_val)
            print(f"  Actuator {i+1}: Original Value={val}, Type={type(val)}, Rounded Value={rounded_val}, Type={type(rounded_val)}")
        
        # pressures = [round(spinbox.value(), 1) for spinbox in self.actuator_spinboxes]
        self.pressure_command_sent.emit(pressures)
        print(f"ActuatorControlWidget: pressure_command_sent emitted with pressures: {pressures}, Type: {type(pressures)}")
        # print(f"Direct Pressure command signal emitted: {pressures} kPa") # Original print
    
    def send_length_command(self):
        """Emit direct length command for the 3 actuators."""
        lengths = [spinbox.value() for spinbox in self.length_spinboxes]
        self.length_command_sent.emit(lengths)
        print(f"Direct Length command signal emitted: {lengths} mm")

    def emergency_stop(self):
        """Emit emergency stop signal."""
        self.emergency_stop_sent.emit()
        print("Emergency stop signal emitted")
        QMessageBox.warning(self, "Emergency", "Emergency stop activated! System halting.") # Keep immediate feedback

    def start_sequence(self):
        """Emit start sequence signal."""
        self.start_sequence_sent.emit()
        self.sequence_status.setText("Auto Sequence Running...")
        self.sequence_status.setStyleSheet("color: #FFA500; font-weight: bold;")
        self.start_sequence_btn.setEnabled(False)
        self.stop_sequence_btn.setEnabled(True)
        print("Start Auto Sequence signal emitted")

    def stop_sequence(self):
        """Emit stop sequence signal."""
        self.stop_sequence_sent.emit()
        self.sequence_status.setText("Manual Mode")
        self.sequence_status.setStyleSheet("color: #4CAF50; font-weight: bold;")
        self.start_sequence_btn.setEnabled(True)
        self.stop_sequence_btn.setEnabled(False)
        print("Stop Auto Sequence signal emitted")
