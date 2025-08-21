import pygame
import numpy as np
import math
import json
import os
from datetime import datetime
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QPushButton, 
                           QLabel, QMessageBox, QGroupBox, QSlider, QDoubleSpinBox,
                           QComboBox, QCheckBox, QTextEdit, QTabWidget)
from PyQt5.QtCore import QTimer, pyqtSignal, Qt
import time

class JoystickControlWidget(QWidget):
    # Signals
    pressure_command_sent = pyqtSignal(list)
    stop_ml_processor_requested = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        
        pygame.init()
        pygame.joystick.init()
        
        self.joystick = None
        self.joystick_timer = QTimer(self)
        self.joystick_timer.timeout.connect(self.read_joystick)
        self.joystick_poll_interval = 30  # milliseconds (approx 33Hz)

        # GUI Update Timer
        self.gui_update_timer = QTimer(self)
        self.gui_update_timer.timeout.connect(self._update_gui_displays)
        self.gui_update_interval = 80  # ms (approx 12.5 Hz)

        # Control modes (removed dance mode)
        self.CONTROL_MODE_INDIVIDUAL = "Individual Actuators"
        self.CONTROL_MODE_COORDINATE = "Coordinate Control"
        self.current_control_mode = self.CONTROL_MODE_COORDINATE

        # Target pressures for individual actuator mode
        self.target_pressures = [0.0, 0.0, 0.0] 
        self.min_pressure = 0.0  # kPa
        self.max_pressure = 13.0 # kPa
        self.pressure_sensitivity = 0.2

        # Virtual position tracking for coordinate mode
        self.virtual_position = {'x': 0.0, 'y': 0.0, 'z': 0.0}  # Relative position
        self.virtual_orientation = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}  # Degrees
        
        # Movement sensitivity (how much virtual position changes per joystick input)
        self.movement_sensitivity = {
            'translation': 2.0,    # units per update
            'rotation': 1.0        # degrees per update
        }

        # Geometric mapping parameters (these are the key tuning parameters)
        self.mapping_params = {
            'base_pressure': 6.5,      # Base pressure when centered (kPa)
            'tilt_sensitivity': 0.3,   # How much pressure changes per degree of tilt
            'height_sensitivity': 0.05, # How much pressure changes per unit of Z
            'lateral_sensitivity': 0.2, # How much pressure changes per unit of X/Y
        }

        # Haptic feedback settings
        self.haptic_enabled = True
        self.pressure_threshold = 13.0  # kPa threshold for vibration
        self.is_vibrating = False
        self.vibration_intensity = 0.8  # Vibration strength (0.0 to 1.0)
        self.vibration_duration = 200   # Duration in milliseconds
        
        # Continuous haptic feedback timer
        self.haptic_timer = QTimer(self)
        self.haptic_timer.timeout.connect(self._continuous_haptic_feedback)
        self.haptic_interval = 300  # Vibrate every 300ms while threshold exceeded

        # Data logging
        self.logging_enabled = False
        self.log_data = []

        self.init_ui()
        self.find_joystick()  # Simple USB joystick detection on startup

    def init_ui(self):
        layout = QVBoxLayout(self)
        
        # Create tabs for different control aspects
        self.tabs = QTabWidget()
        layout.addWidget(self.tabs)
        
        # Main control tab
        self.main_tab = QWidget()
        self.tabs.addTab(self.main_tab, "Control")
        self.init_main_tab()
        
        # Calibration tab
        self.calibration_tab = QWidget()
        self.tabs.addTab(self.calibration_tab, "Calibration")
        self.init_calibration_tab()
        
        # Logging tab
        self.logging_tab = QWidget()
        self.tabs.addTab(self.logging_tab, "Data Logging")
        self.init_logging_tab()

    def init_main_tab(self):
        layout = QVBoxLayout(self.main_tab)
        
        # Status
        self.status_label = QLabel("Joystick: Disconnected")
        layout.addWidget(self.status_label)
        
        # Control mode selection
        mode_group = QGroupBox("Control Mode")
        mode_layout = QHBoxLayout(mode_group)
        
        self.mode_combo = QComboBox()
        self.mode_combo.addItems([self.CONTROL_MODE_COORDINATE, self.CONTROL_MODE_INDIVIDUAL])
        self.mode_combo.currentTextChanged.connect(self.change_control_mode)
        mode_layout.addWidget(QLabel("Mode:"))
        mode_layout.addWidget(self.mode_combo)
        
        layout.addWidget(mode_group)
        
        # Enable/disable button
        self.enable_button = QPushButton("Enable Joystick Control")
        self.enable_button.setCheckable(True)
        self.enable_button.clicked.connect(self.toggle_joystick_control)
        layout.addWidget(self.enable_button)

        # Current state display
        self.state_group = QGroupBox("Current State")
        state_layout = QVBoxLayout(self.state_group)
        
        # Virtual position display
        self.position_display = QLabel("Position: X=0.0, Y=0.0, Z=0.0\nOrientation: Roll=0.0°, Pitch=0.0°, Yaw=0.0°")
        state_layout.addWidget(self.position_display)
        
        # Pressure display
        self.pressure_display = [QLabel(f"Actuator {i+1}: 0.0 kPa") for i in range(3)]
        for pd_label in self.pressure_display:
            state_layout.addWidget(pd_label)
            
        layout.addWidget(self.state_group)

        # Control mapping display
        self.mapping_label = QLabel(self.get_control_mapping_text())
        layout.addWidget(self.mapping_label)

    def init_calibration_tab(self):
        layout = QVBoxLayout(self.calibration_tab)
        
        # Movement sensitivity
        sens_group = QGroupBox("Movement Sensitivity")
        sens_layout = QVBoxLayout(sens_group)
        
        # Translation sensitivity
        trans_layout = QHBoxLayout()
        trans_layout.addWidget(QLabel("Translation Speed:"))
        self.trans_sens_spinbox = QDoubleSpinBox()
        self.trans_sens_spinbox.setRange(0.1, 10.0)
        self.trans_sens_spinbox.setValue(self.movement_sensitivity['translation'])
        self.trans_sens_spinbox.setSingleStep(0.1)
        self.trans_sens_spinbox.valueChanged.connect(self.update_translation_sensitivity)
        trans_layout.addWidget(self.trans_sens_spinbox)
        sens_layout.addLayout(trans_layout)
        
        # Rotation sensitivity
        rot_layout = QHBoxLayout()
        rot_layout.addWidget(QLabel("Rotation Speed:"))
        self.rot_sens_spinbox = QDoubleSpinBox()
        self.rot_sens_spinbox.setRange(0.1, 5.0)
        self.rot_sens_spinbox.setValue(self.movement_sensitivity['rotation'])
        self.rot_sens_spinbox.setSingleStep(0.1)
        self.rot_sens_spinbox.valueChanged.connect(self.update_rotation_sensitivity)
        rot_layout.addWidget(self.rot_sens_spinbox)
        sens_layout.addLayout(rot_layout)
        
        layout.addWidget(sens_group)
        
        # Haptic feedback controls
        haptic_group = QGroupBox("Haptic Feedback Settings")
        haptic_layout = QVBoxLayout(haptic_group)
        
        # Enable/disable haptic feedback
        self.haptic_checkbox = QCheckBox("Enable Haptic Feedback")
        self.haptic_checkbox.setChecked(self.haptic_enabled)
        self.haptic_checkbox.toggled.connect(self.toggle_haptic_feedback)
        haptic_layout.addWidget(self.haptic_checkbox)
        
        # Pressure threshold
        threshold_layout = QHBoxLayout()
        threshold_layout.addWidget(QLabel("Pressure Threshold (kPa):"))
        self.threshold_spinbox = QDoubleSpinBox()
        self.threshold_spinbox.setRange(5.0, 13.0)
        self.threshold_spinbox.setValue(self.pressure_threshold)
        self.threshold_spinbox.setSingleStep(0.1)
        self.threshold_spinbox.valueChanged.connect(self.update_pressure_threshold)
        threshold_layout.addWidget(self.threshold_spinbox)
        haptic_layout.addLayout(threshold_layout)
        
        # Vibration intensity
        intensity_layout = QHBoxLayout()
        intensity_layout.addWidget(QLabel("Vibration Intensity:"))
        self.intensity_spinbox = QDoubleSpinBox()
        self.intensity_spinbox.setRange(0.1, 1.0)
        self.intensity_spinbox.setValue(self.vibration_intensity)
        self.intensity_spinbox.setSingleStep(0.1)
        self.intensity_spinbox.valueChanged.connect(self.update_vibration_intensity)
        intensity_layout.addWidget(self.intensity_spinbox)
        haptic_layout.addLayout(intensity_layout)
        
        # Vibration interval (for continuous feedback)
        interval_layout = QHBoxLayout()
        interval_layout.addWidget(QLabel("Vibration Interval (ms):"))
        self.interval_spinbox = QDoubleSpinBox()
        self.interval_spinbox.setRange(100.0, 1000.0)
        self.interval_spinbox.setValue(self.haptic_interval)
        self.interval_spinbox.setSingleStep(50.0)
        self.interval_spinbox.setDecimals(0)
        self.interval_spinbox.valueChanged.connect(self.update_haptic_interval)
        interval_layout.addWidget(self.interval_spinbox)
        haptic_layout.addLayout(interval_layout)
        
        # Test haptic feedback button
        self.test_haptic_btn = QPushButton("Test Haptic Feedback")
        self.test_haptic_btn.clicked.connect(self.test_haptic_feedback)
        haptic_layout.addWidget(self.test_haptic_btn)
        
        layout.addWidget(haptic_group)
        
        # Mapping parameters
        mapping_group = QGroupBox("Pressure Mapping Parameters")
        mapping_layout = QVBoxLayout(mapping_group)
        
        # Base pressure
        base_layout = QHBoxLayout()
        base_layout.addWidget(QLabel("Base Pressure (kPa):"))
        self.base_pressure_spinbox = QDoubleSpinBox()
        self.base_pressure_spinbox.setRange(0.0, 13.0)
        self.base_pressure_spinbox.setValue(self.mapping_params['base_pressure'])
        self.base_pressure_spinbox.setSingleStep(0.1)
        self.base_pressure_spinbox.valueChanged.connect(self.update_base_pressure)
        base_layout.addWidget(self.base_pressure_spinbox)
        mapping_layout.addLayout(base_layout)
        
        # Tilt sensitivity
        tilt_layout = QHBoxLayout()
        tilt_layout.addWidget(QLabel("Tilt Sensitivity:"))
        self.tilt_sens_spinbox = QDoubleSpinBox()
        self.tilt_sens_spinbox.setRange(0.0, 1.0)
        self.tilt_sens_spinbox.setValue(self.mapping_params['tilt_sensitivity'])
        self.tilt_sens_spinbox.setSingleStep(0.01)
        self.tilt_sens_spinbox.valueChanged.connect(self.update_tilt_sensitivity)
        tilt_layout.addWidget(self.tilt_sens_spinbox)
        mapping_layout.addLayout(tilt_layout)
        
        # Height sensitivity
        height_layout = QHBoxLayout()
        height_layout.addWidget(QLabel("Height Sensitivity:"))
        self.height_sens_spinbox = QDoubleSpinBox()
        self.height_sens_spinbox.setRange(0.0, 0.5)
        self.height_sens_spinbox.setValue(self.mapping_params['height_sensitivity'])
        self.height_sens_spinbox.setSingleStep(0.01)
        self.height_sens_spinbox.valueChanged.connect(self.update_height_sensitivity)
        height_layout.addWidget(self.height_sens_spinbox)
        mapping_layout.addLayout(height_layout)
        
        # Lateral sensitivity
        lateral_layout = QHBoxLayout()
        lateral_layout.addWidget(QLabel("Lateral Sensitivity:"))
        self.lateral_sens_spinbox = QDoubleSpinBox()
        self.lateral_sens_spinbox.setRange(0.0, 1.0)
        self.lateral_sens_spinbox.setValue(self.mapping_params['lateral_sensitivity'])
        self.lateral_sens_spinbox.setSingleStep(0.01)
        self.lateral_sens_spinbox.valueChanged.connect(self.update_lateral_sensitivity)
        lateral_layout.addWidget(self.lateral_sens_spinbox)
        mapping_layout.addLayout(lateral_layout)
        
        layout.addWidget(mapping_group)
        
        # Reset buttons
        reset_group = QGroupBox("Reset Functions")
        reset_layout = QVBoxLayout(reset_group)
        
        self.reset_position_btn = QPushButton("Reset to Center Position")
        self.reset_position_btn.clicked.connect(self.reset_to_center)
        reset_layout.addWidget(self.reset_position_btn)
        
        self.reset_pressures_btn = QPushButton("Reset All Pressures to Zero")
        self.reset_pressures_btn.clicked.connect(self.reset_pressures)
        reset_layout.addWidget(self.reset_pressures_btn)
        
        layout.addWidget(reset_group)

    def init_logging_tab(self):
        layout = QVBoxLayout(self.logging_tab)
        
        # Logging controls
        log_control_group = QGroupBox("Data Logging")
        log_control_layout = QVBoxLayout(log_control_group)
        
        self.logging_checkbox = QCheckBox("Enable Data Logging")
        self.logging_checkbox.toggled.connect(self.toggle_logging)
        log_control_layout.addWidget(self.logging_checkbox)
        
        log_buttons_layout = QHBoxLayout()
        self.save_log_btn = QPushButton("Save Log to File")
        self.save_log_btn.clicked.connect(self.save_log_data)
        self.clear_log_btn = QPushButton("Clear Log Data")
        self.clear_log_btn.clicked.connect(self.clear_log_data)
        
        log_buttons_layout.addWidget(self.save_log_btn)
        log_buttons_layout.addWidget(self.clear_log_btn)
        log_control_layout.addLayout(log_buttons_layout)
        
        layout.addWidget(log_control_group)
        
        # Log display
        log_display_group = QGroupBox("Recent Log Entries")
        log_display_layout = QVBoxLayout(log_display_group)
        
        self.log_display = QTextEdit()
        self.log_display.setMaximumHeight(200)
        self.log_display.setReadOnly(True)
        log_display_layout.addWidget(self.log_display)
        
        layout.addWidget(log_display_group)

    def get_control_mapping_text(self):
        if self.current_control_mode == self.CONTROL_MODE_COORDINATE:
            haptic_status = f"Haptic Feedback: {'Enabled' if self.haptic_enabled else 'Disabled'} (Threshold: {self.pressure_threshold} kPa)"
            return f"""Coordinate Control Mapping:
Left Stick X/Y: Move left/right, forward/backward
Right Stick Y: Move up/down
Right Stick X: Rotate around vertical axis (Yaw)
R3 (Press Right Stick): Reset to Center Position

The robot will lean in the direction you want it to move!

{haptic_status}"""
        else:
            return """Individual Actuator Control Mapping:
Left Stick X: Actuator 1 pressure
Left Stick Y: Actuator 2 pressure  
Right Stick Y: Actuator 3 pressure
L1/R1: Decrease/Increase all pressures"""

    def change_control_mode(self, mode):
        self.current_control_mode = mode
        self.mapping_label.setText(self.get_control_mapping_text())
        print(f"Control mode changed to: {mode}")

    def find_joystick(self):
        """Find and initialize the first available USB joystick"""
        joystick_count = pygame.joystick.get_count()
        print(f"Scanning for joysticks... Found {joystick_count} device(s)")
        
        if joystick_count == 0:
            self.status_label.setText("Joystick: No joysticks found")
            self.joystick = None
            return

        # Try to initialize the first joystick
        try:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            
            controller_name = self.joystick.get_name()
            self.status_label.setText(f"Joystick: Found - {controller_name}")
            print(f"Initialized Joystick: {controller_name}")
            print(f"Number of axes: {self.joystick.get_numaxes()}")
            print(f"Number of buttons: {self.joystick.get_numbuttons()}")
            
        except pygame.error as e:
            print(f"Error initializing joystick: {e}")
            self.status_label.setText(f"Joystick: Error initializing controller - {e}")
            self.joystick = None

    def toggle_joystick_control(self, checked):
        if checked:
            if not self.joystick:
                print("No controller initialized, attempting to find one...")
                self.find_joystick()
                if not self.joystick:
                    self.status_label.setText("Joystick: Connection failed")
                    self.enable_button.setChecked(False)
                    QMessageBox.warning(self, "Controller Not Found", 
                                      "No controller detected!\n\n"
                                      "Please connect your USB controller and restart the application.")
                    return
            
            self.stop_ml_processor_requested.emit()
            print("Joystick control enabled. ML Processor stop requested.")
            
            # Reset to neutral state
            if self.current_control_mode == self.CONTROL_MODE_COORDINATE:
                self.reset_to_center() # This already updates displays
            else:
                self.target_pressures = [self.min_pressure] * 3
                # self.update_pressure_display() # No direct call, GUI timer will handle
                self._update_gui_displays() # Update once immediately

            self.joystick_timer.start(self.joystick_poll_interval)
            self.gui_update_timer.start(self.gui_update_interval) # Start GUI timer
            self.enable_button.setText("Disable Joystick Control")
            
            # Show status
            controller_name = self.joystick.get_name()
            self.status_label.setText(f"Joystick: Active - {controller_name}")
        else:
            self.joystick_timer.stop()
            self.gui_update_timer.stop() # Stop GUI timer
            self.enable_button.setText("Enable Joystick Control")
            
            if self.joystick:
                controller_name = self.joystick.get_name()
                self.status_label.setText(f"Joystick: Idle - {controller_name}")
            else:
                self.status_label.setText("Joystick: Disconnected")
            print("Joystick control disabled.")

    def read_joystick(self):
        if not self.joystick:
            return

        pygame.event.pump()

        # Read joystick inputs
        dead_zone = 0.15
        
        try:
            axis_lx = self.get_axis_value(0, dead_zone)  # Left stick X
            axis_ly = self.get_axis_value(1, dead_zone)  # Left stick Y  
            axis_rx = self.get_axis_value(2, dead_zone)  # Right stick X
            axis_ry = self.get_axis_value(3, dead_zone)  # Right stick Y
            
            # Triggers (usually start at -1.0 when released, go to 1.0 when pressed)
            trigger_l2 = (self.get_axis_value(4, 0.1) + 1.0) / 2.0 if self.joystick.get_numaxes() > 4 else 0.0
            trigger_r2 = (self.get_axis_value(5, 0.1) + 1.0) / 2.0 if self.joystick.get_numaxes() > 5 else 0.0
            
            # Buttons
            button_l1 = self.get_button_value(9)   # L1
            button_r1 = self.get_button_value(10)  # R1
            button_r3 = self.get_button_value(8)  # R3 (Press Right Stick) - Index 8 based on user feedback
            
            # If R3 doesn't work, you can print all button states here to find the correct index:
            # UNCOMMENT THE NEXT 3 LINES TO DEBUG BUTTON PRESSES
            # for i in range(self.joystick.get_numbuttons()):
            #     if self.joystick.get_button(i):
            #         print(f"Button {i} pressed - R3? Target was 12") # Added more context to print
            
        except pygame.error:
            print("Error reading joystick. Joystick might have been disconnected.")
            self.toggle_joystick_control(False)
            return

        # Process input based on control mode
        if self.current_control_mode == self.CONTROL_MODE_COORDINATE:
            self.process_coordinate_control(axis_lx, axis_ly, axis_rx, axis_ry, 
                                          trigger_l2, trigger_r2, button_l1, button_r1, button_r3)
        else:
            self.process_individual_control(axis_lx, axis_ly, axis_ry, button_l1, button_r1)

    def get_axis_value(self, axis_index, dead_zone):
        if self.joystick.get_numaxes() > axis_index:
            value = self.joystick.get_axis(axis_index)
            return value if abs(value) > dead_zone else 0.0
        return 0.0

    def get_button_value(self, button_index):
        if self.joystick.get_numbuttons() > button_index:
            return self.joystick.get_button(button_index)
        return False

    def process_coordinate_control(self, lx, ly, rx, ry, l2, r2, l1, r1, r3_pressed):
        """Process joystick input for coordinate-based control"""
        
        # Reset to center if R3 is pressed
        if r3_pressed:
            self.reset_to_center()
            # Potentially add a small cooldown or require button release to prevent multiple resets if held
            return # Skip other processing for this tick if reset

        # Update virtual position and orientation
        self.virtual_position['x'] += (-lx) * self.movement_sensitivity['translation'] # Inverted for intuitive control
        self.virtual_position['y'] += -ly * self.movement_sensitivity['translation']  # Inverted for intuitive control
        self.virtual_position['z'] += -ry * self.movement_sensitivity['translation']  # Inverted for intuitive control
        
        # Update virtual orientation
        self.virtual_orientation['yaw'] += rx * self.movement_sensitivity['rotation']
        
        # Removed L1/R1 pitch control
        # Removed L2/R2 roll control

        # Apply limits to prevent extreme values
        self.apply_virtual_limits()
        
        # Convert virtual position/orientation to actuator pressures using geometric mapping
        self.target_pressures = self.geometric_mapping_to_pressures()
        
        # Send pressure command
        self.pressure_command_sent.emit(self.target_pressures)
        
        # Check pressure threshold and trigger haptic feedback if needed
        self.check_pressure_threshold_and_vibrate()
        
        # Update displays - NO LONGER CALLED DIRECTLY HERE
        # self.update_position_display()
        # self.update_pressure_display()
        
        # Log data if enabled
        if self.logging_enabled:
            self.log_control_data([lx, ly, rx, ry, l2, r2, l1, r1])

    def process_individual_control(self, lx, ly, ry, l1, r1):
        """Process joystick input for individual actuator control (original method)"""
        
        delta_p = [0.0, 0.0, 0.0]
        
        # Individual actuator control
        delta_p[0] = lx * self.pressure_sensitivity
        delta_p[1] = (-ly) * self.pressure_sensitivity
        delta_p[2] = (-ry) * self.pressure_sensitivity
        
        # Overall pressure modification
        overall_mod = 0.0
        if l1:
            overall_mod = -1.0
        if r1:
            overall_mod = 1.0
            
        if overall_mod != 0.0:
            for i in range(3):
                delta_p[i] += overall_mod * self.pressure_sensitivity

        # Update pressures with limits
        for i in range(3):
            self.target_pressures[i] += delta_p[i]
            self.target_pressures[i] = round(max(self.min_pressure, 
                                               min(self.max_pressure, self.target_pressures[i])), 2)
        
        self.pressure_command_sent.emit(self.target_pressures)
        
        # Check pressure threshold and trigger haptic feedback if needed
        self.check_pressure_threshold_and_vibrate()
        
        # self.update_pressure_display() # NO LONGER CALLED DIRECTLY HERE
        
        # Log data if enabled
        if self.logging_enabled:
            self.log_control_data([lx, ly, ry, 0, 0, 0, l1, r1])

    def geometric_mapping_to_pressures(self):
        """
        Convert virtual position and orientation to actuator pressures using geometric principles.
        
        This assumes a 3-actuator system arranged in a triangle (120° apart).
        The key insight: to move in a direction, the robot needs to lean in that direction,
        which means reducing pressure on actuators in that direction.
        """
        
        # Get current virtual state
        x = self.virtual_position['x']
        y = self.virtual_position['y'] 
        z = self.virtual_position['z']
        roll = math.radians(self.virtual_orientation['roll'])
        pitch = math.radians(self.virtual_orientation['pitch'])
        yaw = math.radians(self.virtual_orientation['yaw'])
        
        # Start with base pressure
        base_p = self.mapping_params['base_pressure']
        
        # Calculate pressure adjustments for each actuator
        # User perspective: Actuator 3 is Front (0 deg).
        # Physical Actuator 1 is Robot's Back-Right (120 deg).
        # Physical Actuator 2 is Robot's Back-Left (240 deg).
        # Order in list: [Angle for Physical Actuator 1, Angle for Physical Actuator 2, Angle for Physical Actuator 3]
        actuator_angles = [math.pi, 5*math.pi/3, math.pi/3]  # Original default
        pressures = []
        
        for i, angle in enumerate(actuator_angles):
            pressure = base_p
            
            # Height effect (Z movement)
            pressure += z * self.mapping_params['height_sensitivity']
            
            # Lateral movement effect (X/Y movement)
            # To move in +X direction, reduce pressure on actuators facing +X
            lateral_effect_x = x * math.cos(angle) * self.mapping_params['lateral_sensitivity']
            lateral_effect_y = y * math.sin(angle) * self.mapping_params['lateral_sensitivity']
            pressure -= (lateral_effect_x + lateral_effect_y)
            
            # Tilt effects (roll and pitch)
            # Roll: rotation around X-axis
            roll_effect = roll * math.sin(angle) * self.mapping_params['tilt_sensitivity'] * 30  # Scale factor
            pressure -= roll_effect
            
            # Pitch: rotation around Y-axis  
            pitch_effect = pitch * math.cos(angle) * self.mapping_params['tilt_sensitivity'] * 30  # Scale factor
            pressure -= pitch_effect
            
            # Yaw doesn't directly affect individual actuator pressures in this simple model
            # (it would require coordinated movement)
            
            # Clamp pressure to valid range
            pressure = max(self.min_pressure, min(self.max_pressure, pressure))
            pressures.append(round(pressure, 2))
        
        return pressures

    def _update_gui_displays(self):
        """Update all relevant GUI displays for joystick control."""
        if self.current_control_mode == self.CONTROL_MODE_COORDINATE:
            self.update_position_display()
        self.update_pressure_display()

    def apply_virtual_limits(self):
        """Apply reasonable limits to virtual position and orientation"""
        # Position limits (arbitrary units)
        max_pos_xy = 75 
        max_pos_z = 300 # Specific limit for Z-axis

        self.virtual_position['x'] = max(-max_pos_xy, min(max_pos_xy, self.virtual_position['x']))
        self.virtual_position['y'] = max(-max_pos_xy, min(max_pos_xy, self.virtual_position['y']))
        self.virtual_position['z'] = max(-max_pos_z, min(max_pos_z, self.virtual_position['z']))
        
        # Orientation limits (degrees)
        max_angle = 45
        for key in ['roll', 'pitch', 'yaw']:
            self.virtual_orientation[key] = max(-max_angle, min(max_angle, self.virtual_orientation[key]))

    def check_pressure_threshold_and_vibrate(self):
        """Check if any actuator pressure reaches threshold and manage continuous haptic feedback"""
        if not self.haptic_enabled or not self.joystick:
            return
            
        # Check if any pressure reaches or exceeds the threshold
        pressure_exceeded = any(pressure >= self.pressure_threshold for pressure in self.target_pressures)
        
        if pressure_exceeded and not self.is_vibrating:
            # Start continuous vibration
            self.is_vibrating = True
            self._trigger_vibration()  # Immediate vibration
            self.haptic_timer.start(self.haptic_interval)  # Start continuous timer
            print(f"Haptic feedback started! Pressure threshold {self.pressure_threshold} kPa exceeded.")
            print(f"Current pressures: {[f'{p:.1f}' for p in self.target_pressures]} kPa")
        
        elif not pressure_exceeded and self.is_vibrating:
            # Stop continuous vibration if all pressures are below threshold
            self.is_vibrating = False
            self.haptic_timer.stop()
            self._stop_vibration()
            print("Haptic feedback stopped - pressures below threshold.")

    def _continuous_haptic_feedback(self):
        """Called by timer to provide continuous haptic feedback while threshold exceeded"""
        if not self.haptic_enabled or not self.joystick:
            self.haptic_timer.stop()
            self.is_vibrating = False
            return
            
        # Check if we should still be vibrating
        pressure_exceeded = any(pressure >= self.pressure_threshold for pressure in self.target_pressures)
        
        if pressure_exceeded:
            self._trigger_vibration()
            # Print current pressures every few vibrations (to avoid spam)
            if hasattr(self, '_vibration_count'):
                self._vibration_count += 1
            else:
                self._vibration_count = 1
            
            if self._vibration_count % 5 == 0:  # Print every 5th vibration (every ~1.5 seconds)
                print(f"Continuous haptic feedback - Pressures: {[f'{p:.1f}' for p in self.target_pressures]} kPa")
        else:
            # Pressures dropped below threshold, stop vibrating
            self.is_vibrating = False
            self.haptic_timer.stop()
            self._stop_vibration()
            self._vibration_count = 0
            print("Haptic feedback stopped - pressures below threshold.")

    def _trigger_vibration(self):
        """Trigger a single vibration pulse"""
        try:
            if hasattr(self.joystick, 'rumble'):
                self.joystick.rumble(
                    self.vibration_intensity, 
                    self.vibration_intensity, 
                    self.vibration_duration
                )
        except Exception as e:
            print(f"Error triggering haptic feedback: {e}")

    def _stop_vibration(self):
        """Stop any active vibration"""
        try:
            if hasattr(self.joystick, 'rumble'):
                self.joystick.rumble(0.0, 0.0, 0)
        except Exception as e:
            print(f"Error stopping haptic feedback: {e}")

    def toggle_haptic_feedback(self, enabled):
        """Enable or disable haptic feedback"""
        self.haptic_enabled = enabled
        if not enabled and self.is_vibrating:
            # Stop vibration if disabled
            self.is_vibrating = False
            self.haptic_timer.stop()
            self._stop_vibration()
        print(f"Haptic feedback {'enabled' if enabled else 'disabled'}")
        
        # Update the control mapping display
        self.mapping_label.setText(self.get_control_mapping_text())

    def update_position_display(self):
        """Update the position display"""
        x = self.virtual_position['x']
        y = self.virtual_position['y']
        z = self.virtual_position['z']
        roll = self.virtual_orientation['roll']
        pitch = self.virtual_orientation['pitch']
        yaw = self.virtual_orientation['yaw']
        
        pos_text = f"Position: X={x:.1f}, Y={y:.1f}, Z={z:.1f}\n"
        pos_text += f"Orientation: Roll={roll:.1f}°, Pitch={pitch:.1f}°, Yaw={yaw:.1f}°"
        self.position_display.setText(pos_text)

    def update_pressure_display(self):
        """Update pressure display"""
        for i in range(3):
            self.pressure_display[i].setText(f"Actuator {i+1}: {self.target_pressures[i]:.1f} kPa")

    # Calibration parameter update methods
    def update_translation_sensitivity(self, value):
        self.movement_sensitivity['translation'] = value

    def update_rotation_sensitivity(self, value):
        self.movement_sensitivity['rotation'] = value

    def update_base_pressure(self, value):
        self.mapping_params['base_pressure'] = value

    def update_tilt_sensitivity(self, value):
        self.mapping_params['tilt_sensitivity'] = value

    def update_height_sensitivity(self, value):
        self.mapping_params['height_sensitivity'] = value

    def update_lateral_sensitivity(self, value):
        self.mapping_params['lateral_sensitivity'] = value

    def update_pressure_threshold(self, value):
        """Update the pressure threshold for haptic feedback"""
        self.pressure_threshold = value
        print(f"Haptic feedback threshold updated to {value} kPa")
        
        # Update the control mapping display
        self.mapping_label.setText(self.get_control_mapping_text())

    def update_vibration_intensity(self, value):
        """Update the vibration intensity for haptic feedback"""
        self.vibration_intensity = value
        print(f"Vibration intensity updated to {value}")

    def update_haptic_interval(self, value):
        """Update the vibration interval for continuous feedback"""
        self.haptic_interval = int(value)
        print(f"Haptic feedback interval updated to {self.haptic_interval} ms")
        
        # If currently vibrating, restart the timer with new interval
        if self.is_vibrating and self.haptic_timer.isActive():
            self.haptic_timer.stop()
            self.haptic_timer.start(self.haptic_interval)

    def test_haptic_feedback(self):
        """Test the haptic feedback manually"""
        if not self.joystick:
            print("No joystick connected for haptic feedback test")
            return
            
        if not self.haptic_enabled:
            print("Haptic feedback is disabled")
            return
            
        try:
            if hasattr(self.joystick, 'rumble'):
                self.joystick.rumble(
                    self.vibration_intensity, 
                    self.vibration_intensity, 
                    500  # Test vibration for 500ms
                )
                print(f"Test vibration triggered with intensity {self.vibration_intensity}")
            else:
                print("Controller does not support rumble/haptic feedback")
        except Exception as e:
            print(f"Error testing haptic feedback: {e}")

    def reset_to_center(self):
        """Reset to center position"""
        self.virtual_position = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.virtual_orientation = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
        # Calculate base pressures for coordinate mode reset
        if self.current_control_mode == self.CONTROL_MODE_COORDINATE:
            self.target_pressures = self.geometric_mapping_to_pressures()
        else: # For individual mode, or if called before mode switch
            self.target_pressures = [self.mapping_params['base_pressure']] * 3

        # self.update_position_display() # No direct call, GUI timer will handle
        # self.update_pressure_display() # No direct call, GUI timer will handle
        self._update_gui_displays() # Update once immediately

        self.pressure_command_sent.emit(self.target_pressures)
        
        # Check haptic feedback after reset
        self.check_pressure_threshold_and_vibrate()

    def reset_pressures(self):
        """Reset all pressures to zero"""
        self.target_pressures = [self.min_pressure] * 3
        self.pressure_command_sent.emit(self.target_pressures)
        # self.update_pressure_display() # No direct call, GUI timer will handle
        self._update_gui_displays() # Update once immediately
        
        # Check haptic feedback after reset
        self.check_pressure_threshold_and_vibrate()

    def toggle_logging(self, enabled):
        self.logging_enabled = enabled
        if enabled:
            print("Data logging enabled")
        else:
            print("Data logging disabled")

    def log_control_data(self, joystick_inputs):
        """Log control data for analysis"""
        if not self.logging_enabled:
            return
            
        timestamp = datetime.now().isoformat()
        log_entry = {
            'timestamp': timestamp,
            'control_mode': self.current_control_mode,
            'virtual_position': self.virtual_position.copy(),
            'virtual_orientation': self.virtual_orientation.copy(),
            'pressures': self.target_pressures.copy(),
            'joystick_inputs': joystick_inputs,
            'movement_sensitivity': self.movement_sensitivity.copy(),
            'mapping_params': self.mapping_params.copy()
        }
        
        self.log_data.append(log_entry)
        
        # Update log display (show last few entries) - Reduced frequency
        # Update every 50 entries (approx 1.5 seconds if poll is 30ms)
        if len(self.log_data) % 50 == 0: 
            self.update_log_display()

    def update_log_display(self):
        """Update the log display with recent entries"""
        if not self.log_data:
            return
            
        recent_entries = self.log_data[-3:]  # Show last 3 entries
        display_text = ""
        
        for entry in recent_entries:
            display_text += f"{entry['timestamp'][-8:]}: Mode={entry['control_mode']}\n"
            if entry['virtual_position']:
                pos = entry['virtual_position']
                display_text += f"  Pos: X={pos['x']:.1f}, Y={pos['y']:.1f}, Z={pos['z']:.1f}\n"
            display_text += f"  Pressures: {[round(p, 1) for p in entry['pressures']]}\n\n"
        
        self.log_display.setText(display_text)

    def save_log_data(self):
        """Save logged data to file"""
        if not self.log_data:
            QMessageBox.information(self, "No Data", "No log data to save.")
            return
            
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"joystick_log_{timestamp}.json"
        
        try:
            with open(filename, 'w') as f:
                json.dump(self.log_data, f, indent=2)
            QMessageBox.information(self, "Saved", f"Log data saved to {filename}")
            print(f"Log data saved to {filename}")
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to save log data: {e}")

    def clear_log_data(self):
        """Clear logged data"""
        self.log_data.clear()
        self.log_display.clear()
        print("Log data cleared")

    def closeEvent(self, event):
        if self.joystick_timer.isActive():
            self.joystick_timer.stop()
        
        # Stop haptic feedback timer and vibration
        if self.haptic_timer.isActive():
            self.haptic_timer.stop()
        
        # Stop any active haptic feedback
        if self.joystick and self.is_vibrating:
            self.is_vibrating = False
            self._stop_vibration()
            print("Haptic feedback stopped on close")
        
        if self.joystick:
            self.joystick.quit()
        pygame.joystick.quit()
        pygame.quit()
        super().closeEvent(event)

    def update_theme_dependent_styles(self, is_dark_mode):
        """Update styles that depend on the current theme."""
        if is_dark_mode:
            self.mapping_label.setStyleSheet("""
                background-color: #353535; 
                color: #e0e0e0; 
                padding: 10px; 
                border: 1px solid #555555;
                border-radius: 4px;
            """)
        else:
            self.mapping_label.setStyleSheet("""
                background-color: #f0f0f0; 
                color: #000000; /* Default black for light theme */
                padding: 10px; 
                border: 1px solid #cccccc;
                border-radius: 4px;
            """)

# Integration example for main_window.py:
# 
# from gui.joystick_control_widget import JoystickControlWidget
#
# # In your main window __init__ or init_ui:
# self.joystick_widget = JoystickControlWidget()
# self.tabs.addTab(self.joystick_widget, "Joystick Control")
#
# # Connect signals:
# self.joystick_widget.pressure_command_sent.connect(self.handle_joystick_pressure_command)
# self.joystick_widget.stop_ml_processor_requested.connect(self.stop_ml_processor_for_joystick) 