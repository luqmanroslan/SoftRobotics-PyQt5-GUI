import numpy as np
import pyqtgraph as pg
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGroupBox, QLabel,
    QPushButton, QCheckBox
)
from PyQt5.QtCore import QTimer, Qt

# --- New IMU Display Widget ---
class IMUDisplayWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0) # Remove margins to fit snugly in parent

        self.orientation_label = QLabel("Orientation: Roll=0.0°, Pitch=0.0°, Yaw=0.0°")
        # self.acceleration_label = QLabel("Acceleration: X=0.0, Y=0.0, Z=0.0") # Removed
        # self.gyroscope_label = QLabel("Gyroscope: X=0.0, Y=0.0, Z=0.0") # Removed

        layout.addWidget(self.orientation_label)
        # layout.addWidget(self.acceleration_label) # Removed
        # layout.addWidget(self.gyroscope_label) # Removed
        

    def update_display(self, imu_data):
        if not imu_data: # Do nothing if no data
            return

        roll = imu_data.get('imu_roll', 0.0)
        pitch = imu_data.get('imu_pitch', 0.0)
        yaw = imu_data.get('imu_yaw', 0.0)
        self.orientation_label.setText(f"Orientation: Roll={roll:.1f}°, Pitch={pitch:.1f}°, Yaw={yaw:.1f}°")

        # acc_x = imu_data.get('acc_x', 0.0) # Removed
        # acc_y = imu_data.get('acc_y', 0.0) # Removed
        # acc_z = imu_data.get('acc_z', 0.0) # Removed
        # self.acceleration_label.setText(f"Acceleration: X={acc_x:.2f}, Y={acc_y:.2f}, Z={acc_z:.2f}") # Removed

        # Handle optional gyroscope data - Section Removed
        # if 'gyro_x' in imu_data or 'gyro_y' in imu_data or 'gyro_z' in imu_data:
        #     gyro_x = imu_data.get('gyro_x', 0.0)
        #     gyro_y = imu_data.get('gyro_y', 0.0)
        #     gyro_z = imu_data.get('gyro_z', 0.0)
        #     self.gyroscope_label.setText(f"Gyroscope: X={gyro_x:.2f}, Y={gyro_y:.2f}, Z={gyro_z:.2f}")
        #     self.gyroscope_label.setVisible(True)
        # else:
        #     self.gyroscope_label.setVisible(False)


class SensorMonitorWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        
        # IMU Data Handling - MOVED EARLIER
        self.latest_imu_data = {}
        self.imu_update_timer = QTimer(self)
        self.imu_update_timer.timeout.connect(self.update_imu_gui)
        self.imu_update_interval = 150 # milliseconds (approx 6-7 Hz)

        self.init_ui() # init_ui is called AFTER IMU attributes are set
        self.max_points = 100  # Reduced from 200 for better performance
        self.timepoints = np.linspace(0, 10, self.max_points)
        # Initialize arrays for three pressure values
        self.pressure_data_1 = np.zeros(self.max_points)
        self.pressure_data_2 = np.zeros(self.max_points)
        self.pressure_data_3 = np.zeros(self.max_points)
        # Initialize arrays for six potentiometer values
        self.pot_data = [np.zeros(self.max_points) for _ in range(6)]
        
        # Add counter for throttling updates
        self.update_counter = 0
        self.update_frequency = 3  # Only update every N points

    def init_ui(self):
        layout = QVBoxLayout(self)
        
        pressure_group = QGroupBox("Pressure Sensors")
        pressure_layout = QVBoxLayout(pressure_group)
        self.pressure_plot = pg.PlotWidget()
        self.pressure_plot.setBackground('w')
        self.pressure_plot.setLabel('left', 'Pressure (kPa)')
        self.pressure_plot.setLabel('bottom', 'Time (s)')
        self.pressure_plot.showGrid(x=True, y=True)
        # Create three pressure curves with different colors
        self.pressure_curve_1 = self.pressure_plot.plot(pen=pg.mkPen(color='r', width=2), name='Actuator 1')
        self.pressure_curve_2 = self.pressure_plot.plot(pen=pg.mkPen(color='g', width=2), name='Actuator 2')
        self.pressure_curve_3 = self.pressure_plot.plot(pen=pg.mkPen(color='b', width=2), name='Actuator 3')
        # Add legend
        self.pressure_plot.addLegend()
        # Create labels for current pressure values
        self.pressure_value_1 = QLabel("Actuator 1: 0 kPa")
        self.pressure_value_2 = QLabel("Actuator 2: 0 kPa")
        self.pressure_value_3 = QLabel("Actuator 3: 0 kPa")
        for label in [self.pressure_value_1, self.pressure_value_2, self.pressure_value_3]:
            label.setStyleSheet("font-size: 14pt;")
            pressure_layout.addWidget(label)
        pressure_layout.addWidget(self.pressure_plot)
        
        # Create a grid layout for potentiometer plots
        pot_group = QGroupBox("Potentiometer Sensors")
        pot_layout = QVBoxLayout(pot_group)
        
        # Create a grid of 2x3 for the potentiometer plots
        pot_grid = QHBoxLayout()
        pot_left = QVBoxLayout()
        pot_right = QVBoxLayout()
        
        # Create plots and labels for each potentiometer
        self.pot_plots = []
        self.pot_curves = []
        self.pot_values = []
        colors = ['r', 'g', 'b', 'c', 'm', 'y']
        
        for i in range(6):
            pot_plot = pg.PlotWidget()
            pot_plot.setBackground('w')
            pot_plot.setLabel('left', f'Pot {i+1} (mm)')
            pot_plot.setLabel('bottom', 'Time (s)')
            pot_plot.showGrid(x=True, y=True)
            pot_curve = pot_plot.plot(pen=pg.mkPen(color=colors[i], width=2))
            pot_value = QLabel(f"Pot {i+1}: 0 mm")
            pot_value.setStyleSheet("font-size: 14pt;")
            
            self.pot_plots.append(pot_plot)
            self.pot_curves.append(pot_curve)
            self.pot_values.append(pot_value)
            
            # Add to appropriate layout (left or right)
            if i < 3:
                pot_left.addWidget(pot_value)
                pot_left.addWidget(pot_plot)
            else:
                pot_right.addWidget(pot_value)
                pot_right.addWidget(pot_plot)
        
        pot_grid.addLayout(pot_left)
        pot_grid.addLayout(pot_right)
        pot_layout.addLayout(pot_grid)
        
        layout.addWidget(pressure_group)
        layout.addWidget(pot_group)
        
        # --- IMU Data Sub-Panel ---
        self.imu_group = QGroupBox("IMU Data")
        imu_group_layout = QVBoxLayout(self.imu_group)
        
        self.toggle_imu_btn = QPushButton("Show IMU Data")
        self.toggle_imu_btn.setCheckable(True)
        self.toggle_imu_btn.setChecked(False) # Start hidden
        self.toggle_imu_btn.clicked.connect(self.toggle_imu_display)
        imu_group_layout.addWidget(self.toggle_imu_btn)
        
        self.imu_display_widget = IMUDisplayWidget()
        self.imu_display_widget.setVisible(False) # Start hidden
        imu_group_layout.addWidget(self.imu_display_widget)
        
        layout.addWidget(self.imu_group)
        # --- End IMU Data Sub-Panel ---
        
        log_group = QGroupBox("Data Logging")
        log_layout = QHBoxLayout(log_group)
        self.log_checkbox = QCheckBox("Log Sensor Data")
        self.log_file_btn = QPushButton("Select Log File")
        log_layout.addWidget(self.log_checkbox)
        log_layout.addWidget(self.log_file_btn)
        layout.addWidget(log_group)

        # Start the IMU GUI update timer
        self.imu_update_timer.start(self.imu_update_interval)

    def toggle_imu_display(self, checked):
        self.imu_display_widget.setVisible(checked)
        if checked:
            self.toggle_imu_btn.setText("Hide IMU Data")
            self.update_imu_gui() # Update immediately when shown
        else:
            self.toggle_imu_btn.setText("Show IMU Data")
            
    def update_imu_gui(self):
        if self.imu_display_widget.isVisible() and self.latest_imu_data:
            self.imu_display_widget.update_display(self.latest_imu_data)
    
    def update_data(self, data_dict):
        # Increment counter and only update visually every few calls
        self.update_counter += 1
        visual_update = (self.update_counter % self.update_frequency == 0)
        
        # --- Store IMU Data ---
        # Using the correct keys from your ESP32 output
        imu_keys = ['imu_roll', 'imu_pitch', 'imu_yaw'] 
        received_imu_data = False
        for key in imu_keys:
            if key in data_dict:
                self.latest_imu_data[key] = data_dict[key]
                received_imu_data = True
        # --- End Store IMU Data ---

        # Handle individual pressure values if available
        if 'pressure_1' in data_dict:
            # Update pressure 1
            p1 = data_dict['pressure_1']
            self.pressure_data_1 = np.roll(self.pressure_data_1, -1)
            self.pressure_data_1[-1] = p1
            if visual_update:
                self.pressure_curve_1.setData(self.timepoints, self.pressure_data_1)
            self.pressure_value_1.setText(f"Actuator 1: {p1:.2f} kPa")
            
            # Update pressure 2 if available
            if 'pressure_2' in data_dict:
                p2 = data_dict['pressure_2']
                self.pressure_data_2 = np.roll(self.pressure_data_2, -1)
                self.pressure_data_2[-1] = p2
                if visual_update:
                    self.pressure_curve_2.setData(self.timepoints, self.pressure_data_2)
                self.pressure_value_2.setText(f"Actuator 2: {p2:.2f} kPa")
            
            # Update pressure 3 if available
            if 'pressure_3' in data_dict:
                p3 = data_dict['pressure_3']
                self.pressure_data_3 = np.roll(self.pressure_data_3, -1)
                self.pressure_data_3[-1] = p3
                if visual_update:
                    self.pressure_curve_3.setData(self.timepoints, self.pressure_data_3)
                self.pressure_value_3.setText(f"Actuator 3: {p3:.2f} kPa")
        
        # Handle legacy single pressure format
        elif 'pressure' in data_dict:
            pressure = data_dict['pressure']
            self.pressure_data_1 = np.roll(self.pressure_data_1, -1)
            self.pressure_data_1[-1] = pressure
            if visual_update:
                self.pressure_curve_1.setData(self.timepoints, self.pressure_data_1)
            self.pressure_value_1.setText(f"Pressure: {pressure:.2f} kPa")
        
        # Update potentiometer values
        for i in range(6):
            pot_key = f'potentiometer_{i+1}'
            if pot_key in data_dict:
                value = data_dict[pot_key]
                self.pot_data[i] = np.roll(self.pot_data[i], -1)
                self.pot_data[i][-1] = value
                if visual_update:
                    self.pot_curves[i].setData(self.timepoints, self.pot_data[i])
                self.pot_values[i].setText(f"Pot {i+1}: {value:.2f} mm")

    def update_single_value(self, key, value):
        """
        Update a single sensor value.
        
        Args:
            key (str): Sensor key (e.g., 'potentiometer_1', 'pressure_1')
            value (float): New sensor value
        """
        data_dict = {key: value}
        self.update_data(data_dict) 