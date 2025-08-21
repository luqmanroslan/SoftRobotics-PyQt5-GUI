import time
from PyQt5.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QTabWidget, QPushButton, QMessageBox, QApplication, QLabel
)
from PyQt5.QtCore import Qt, QTimer

from communication.serial_handler import SerialThread
from .widgets.connection_widget import ConnectionWidget
from .widgets.control_panel import ActuatorControlWidget
from .widgets.sensor_monitor import SensorMonitorWidget
from .widgets.visualization import Graphic3d
from .widgets.voice_control import VoiceControlWidget
from .joystick_control_widget import JoystickControlWidget
from .dance_control_widget import DanceControlWidget
from .styles.dark_palette import DarkPalette
from .styles.light_palette import LightPalette

class SoftRobotGUI(QMainWindow):
    def __init__(self, high_level_processor=None):
        """Initialize the main GUI window"""
        super().__init__()
        self.setWindowTitle("Soft Robot Control Interface (USB Serial)")
        self.resize(1200, 800)
        self.high_level_processor = high_level_processor
        
        self.init_ui()
        
        # Serial thread will be provided from outside - don't create one here
        self.serial_thread = None
        
        # Create status bar first (before applying any theme)
        self.status_bar = self.statusBar()
        
        # Show default status
        self.status_bar.showMessage("System initialized. Connect to ESP32 and select control mode to begin.")
        
        # Apply theme after creating status bar
        self.apply_light_theme()
        self.is_dark_mode = False
        
        # Connect command signal from high-level processor - will be connected when device is connected
        if self.high_level_processor is not None:
            # Don't automatically start any control mode - wait for user input
            
            # Connect pose updates to visualization
            # This signal was changed in ml_processor to emit (pose_list, lengths_list) - REVERTING THAT CONNECTION
            self.high_level_processor.pose_updated.connect(self.update_visualization_pose) # REVERTED to simpler handler
            print("System ready - waiting for user to select control mode")
    
    def init_ui(self):
        central_widget = QWidget()
        main_layout = QVBoxLayout(central_widget)
        main_layout.setSpacing(10)
        main_layout.setContentsMargins(10, 10, 10, 10)
        
        # Top section: Connection controls and theme toggle
        top_layout = QHBoxLayout()
        
        # Connection widget
        self.connection_widget = ConnectionWidget()
        top_layout.addWidget(self.connection_widget)
        
        # Theme toggle button
        self.theme_btn = QPushButton("🌙 Light Mode")
        self.theme_btn.setStyleSheet("""
            QPushButton {
                background-color: #2b2b2b;
                color: white;
                border: 1px solid #444;
                padding: 5px 10px;
                border-radius: 4px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #3b3b3b;
            }
        """)
        self.theme_btn.clicked.connect(self.toggle_theme)
        top_layout.addWidget(self.theme_btn)
        
        main_layout.addLayout(top_layout)
        
        # Middle section: Tabs for control, monitoring, and 3D visualization
        self.tabs = QTabWidget()
        self.tabs.setStyleSheet("""
            QTabWidget::pane {
                border: 1px solid #444;
                background: #2b2b2b;
            }
            QTabBar::tab {
                background: #3b3b3b;
                color: white;
                padding: 8px 12px;
                border: 1px solid #444;
                border-bottom: none;
            }
            QTabBar::tab:selected {
                background: #2b2b2b;
            }
        """)
        main_layout.addWidget(self.tabs)
        
        # Tab 1: Actuator Control Panel
        self.control_panel = ActuatorControlWidget()
        self.tabs.addTab(self.control_panel, "Control Panel")
        
        # Tab 2: Sensor Monitoring
        self.sensor_monitor = SensorMonitorWidget()
        self.tabs.addTab(self.sensor_monitor, "Sensor Monitoring")
        
        # Tab 3: 3D Visualization
        self.visualization_widget = Graphic3d()
        self.visualization_widget.show_neutral_pose()
        self.tabs.addTab(self.visualization_widget, "3D Visualization")
        
        # Tab 4: Voice Control
        self.voice_control = VoiceControlWidget()
        self.tabs.addTab(self.voice_control, "Voice Control")

        # Tab 5: Joystick Control (New)
        self.joystick_widget = JoystickControlWidget()
        self.tabs.addTab(self.joystick_widget, "Joystick Control")

        # Tab 6: Dance Control (New)
        self.dance_control = DanceControlWidget()
        self.tabs.addTab(self.dance_control, "🎭 Dance Control")
        
        # Connect signals to slots
        self.connection_widget.connection_request.connect(self.connect_to_esp32)
        self.connection_widget.disconnect_request.connect(self.disconnect_from_esp32)
        self.voice_control.command_sent.connect(self.send_command_to_esp32)
        self.voice_control.pressure_updated.connect(self.update_control_panel_pressure)
        self.voice_control.all_pressures_updated.connect(self.handle_all_pressures_updated_from_voice)
        self.voice_control.voice_dance_started.connect(self.stop_advanced_dance_for_voice_dance)
        
        # Add new specific signal connections from ActuatorControlWidget
        self.control_panel.pressure_command_sent.connect(self.handle_pressure_command)
        self.control_panel.length_command_sent.connect(self.handle_length_command)
        self.control_panel.emergency_stop_sent.connect(self.handle_emergency_stop)
        self.control_panel.start_sequence_sent.connect(self.handle_start_sequence)
        self.control_panel.stop_sequence_sent.connect(self.handle_stop_sequence)
        
        # Connect JoystickControlWidget signals
        self.joystick_widget.pressure_command_sent.connect(self.handle_joystick_pressure_command)
        self.joystick_widget.stop_ml_processor_requested.connect(self.stop_ml_processor_for_joystick)
        
        # Connect DanceControlWidget signals
        self.dance_control.pressure_command_sent.connect(self.handle_dance_pressure_command)
        self.dance_control.stop_ml_processor_requested.connect(self.stop_ml_processor_for_dance)
        self.dance_control.advanced_dance_started.connect(self.stop_voice_dance_for_advanced_dance)
        self.dance_control.esp32_command_sent.connect(self.send_command_to_esp32)  # New connection for ESP32 commands
        
        self.setCentralWidget(central_widget)
    
    def set_serial_thread(self, serial_thread):
        """Set the serial thread from outside and connect signals"""
        self.serial_thread = serial_thread
        if self.serial_thread:
            # Connect signals to the serial thread
            self.serial_thread.data_received.connect(self.process_sensor_data)
            self.serial_thread.data_received.connect(self.forward_data_to_processor)
            self.serial_thread.connection_error.connect(self.handle_connection_error)
            # Connect raw message signal for autonomous dance
            self.serial_thread.raw_message_received.connect(self.handle_raw_serial_message)
    
    def connect_to_esp32(self, port):
        """This function now just forwards the connection request to the external handler"""
        # Connection will be handled externally by the lambda function in ml_main.py
        pass
    
    def disconnect_from_esp32(self):
        """This function now just forwards the disconnection request to the external handler"""
        # Disconnection will be handled externally by the lambda function in ml_main.py
        pass
    
    def send_command_to_esp32(self, command_dict):
        """Send command to ESP32 through serial thread's command queue."""
        print(f"MainWin.send_command_to_esp32: Attempting to send: {command_dict}") # Log incoming command
        if self.serial_thread and self.serial_thread.running:
            self.serial_thread.send_command(command_dict)
            print(f"MainWin.send_command_to_esp32: Command {command_dict} sent to serial thread.")
        else:
            print(f"MainWin.send_command_to_esp32: Cannot send command. Serial thread available: {self.serial_thread is not None}, Serial thread running: {self.serial_thread.running if self.serial_thread else 'N/A'}")
    
    def process_sensor_data(self, data_dict):
        """Process incoming sensor data from the serial thread."""
        # Update sensor monitor with raw data
        self.sensor_monitor.update_data(data_dict)
        
        # Forward data to dance control for autonomous mode
        self.dance_control.handle_serial_data(data_dict)
        
        # Update status bar with pressure readings
        if 'pressure_1' in data_dict:
            self.status_bar.showMessage(
                f"P1: {data_dict['pressure_1']:.1f} | " +
                f"P2: {data_dict.get('pressure_2', 0):.1f} | " +
                f"P3: {data_dict.get('pressure_3', 0):.1f} | " +
                f"Ext: {data_dict.get('potentiometer_1', 0):.1f} mm"
            )
        elif 'pressure' in data_dict:
            if 'potentiometer_1' in data_dict:
                self.status_bar.showMessage(f"Pressure: {data_dict['pressure']:.2f} kPa | Extension: {data_dict['potentiometer_1']:.2f} mm")
    
    def handle_connection_error(self, error_message):
        self.connection_widget.update_connection_status(False, error_message)
        self.status_bar.showMessage(f"Connection error: {error_message}")
    
    def closeEvent(self, event):
        """Handle application close event"""
        # Cleanly disconnect the serial thread
        try:
            self.serial_thread.disconnect()
        except Exception as e:
            print(f"Error disconnecting serial thread: {e}")
        
        # Stop the high-level processor without blocking
        if self.high_level_processor is not None:
            try:
                # Just set the running flag to false without any blocking operations
                self.high_level_processor.should_run = False
            except Exception as e:
                print(f"Error stopping high-level processor: {e}")
        
        # Always accept the close event
        event.accept()

    def update_control_panel_pressure(self, actuator_num, pressure):
        """Update the control panel's slider and spinbox for the specified actuator"""
        if 0 <= actuator_num <= 2:
            try:
                # Convert to float if it's not already (for backward compatibility)
                pressure_float = float(pressure)
                # Ensure it's in the valid range
                pressure_float = max(0.0, min(13.0, pressure_float))
                # For the slider, multiply by 10 since we're using 0.1 increments
                self.control_panel.actuator_sliders[actuator_num].setValue(int(pressure_float * 10))
                # For the spinbox, use the float value directly
                self.control_panel.actuator_spinboxes[actuator_num].setValue(pressure_float)
            except Exception as e:
                print(f"Error updating control panel pressure: {e}")

    def handle_all_pressures_updated_from_voice(self, pressures):
        """Handle the all_pressures_updated signal from VoiceControlWidget."""
        if not isinstance(pressures, list) or len(pressures) != 3:
            print(f"Error: MainWin.handle_all_pressures_updated_from_voice received invalid data: {pressures}")
            return

        print(f"MainWin: Handling all_pressures_updated_from_voice: {pressures}")

        if self.high_level_processor and self.high_level_processor.should_run:
            print("MainWin: Stopping ML processor for voice global pressure set.")
            self.high_level_processor.stop_processing() # Reverted: no wait=True

        updated_pressures_for_status = []
        for i in range(3):
            try:
                pressure_float = float(pressures[i])
                pressure_float = max(0.0, min(13.0, pressure_float))
                self.control_panel.actuator_sliders[i].setValue(int(pressure_float * 10))
                self.control_panel.actuator_spinboxes[i].setValue(pressure_float)
                updated_pressures_for_status.append(f"{pressure_float:.1f}")
            except Exception as e:
                print(f"Error updating GUI for actuator {i} from voice: {e}")
                updated_pressures_for_status.append("Err")
        
        self.status_bar.showMessage(f"Voice cmd: All pressures set to P1: {updated_pressures_for_status[0]}, P2: {updated_pressures_for_status[1]}, P3: {updated_pressures_for_status[2]} kPa")

    def toggle_theme(self):
        self.is_dark_mode = not self.is_dark_mode
        if self.is_dark_mode:
            self.theme_btn.setText("🌙 Light Mode")
            self.theme_btn.setStyleSheet("""
                QPushButton {
                    background-color: #2b2b2b;
                    color: white;
                    border: 1px solid #444;
                    padding: 5px 10px;
                    border-radius: 4px;
                    font-weight: bold;
                }
                QPushButton:hover {
                    background-color: #3b3b3b;
                }
            """)
            self.apply_dark_theme()
        else:
            self.theme_btn.setText("☀️ Dark Mode")
            self.theme_btn.setStyleSheet("""
                QPushButton {
                    background-color: #f0f0f0;
                    color: black;
                    border: 1px solid #ccc;
                    padding: 5px 10px;
                    border-radius: 4px;
                    font-weight: bold;
                }
                QPushButton:hover {
                    background-color: #e0e0e0;
                }
            """)
            self.apply_light_theme()

    def apply_dark_theme(self):
        app = QApplication.instance()
        app.setPalette(DarkPalette())
        app.setStyleSheet("""
            QMainWindow {
                background-color: #000000;
            }
            QWidget {
                font-family: 'Segoe UI', Arial, sans-serif;
                background-color: #000000;
            }
            QGroupBox {
                border: 1px solid #444;
                border-radius: 5px;
                margin-top: 1em;
                padding-top: 10px;
                background: #1a1a1a;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 3px;
                color: white;
            }
            QPushButton {
                background-color: #0d47a1;
                color: white;
                border: none;
                padding: 8px 16px;
                border-radius: 4px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #1565c0;
            }
            QPushButton:pressed {
                background-color: #0a3d91;
            }
            QPushButton:disabled {
                background-color: #263238;
                color: #546e7a;
            }
            QSlider::groove:horizontal {
                border: 1px solid #444;
                height: 8px;
                background: #3b3b3b;
                margin: 2px 0;
            }
            QSlider::handle:horizontal {
                background: #0d47a1;
                border: 1px solid #444;
                width: 18px;
                margin: -2px 0;
                border-radius: 3px;
            }
            QSlider::handle:horizontal:hover {
                background: #1565c0;
            }
            QSpinBox {
                background: #3b3b3b;
                border: 1px solid #444;
                border-radius: 3px;
                padding: 3px;
                color: white;
            }
            QLabel {
                color: white;
            }
            QCheckBox {
                color: white;
            }
            QLineEdit {
                background: #3b3b3b;
                border: 1px solid #444;
                border-radius: 3px;
                padding: 5px;
                color: white;
            }
            QProgressBar {
                border: 1px solid #444;
                border-radius: 3px;
                text-align: center;
                background: #3b3b3b;
            }
            QProgressBar::chunk {
                background-color: #0d47a1;
            }
            QTabWidget::pane {
                border: 1px solid #444;
                background: #1a1a1a;
            }
            QTabBar::tab {
                background: #2b2b2b;
                color: white;
                padding: 8px 12px;
                border: 1px solid #444;
                border-bottom: none;
            }
            QTabBar::tab:selected {
                background: #1a1a1a;
            }
            QTabBar::tab:hover {
                background: #333333;
            }
            QStatusBar {
                background: #1a1a1a;
                color: white;
                padding: 5px;
            }
            QScrollBar:vertical {
                border: 1px solid #444;
                background: #1a1a1a;
                width: 10px;
                margin: 0px;
            }
            QScrollBar::handle:vertical {
                background: #444444;
                border-radius: 5px;
                min-height: 20px;
            }
            QScrollBar::handle:vertical:hover {
                background: #555555;
            }
            QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {
                height: 0px;
            }
        """)
        
        # Update specific widget styles
        self.tabs.setStyleSheet("""
            QTabWidget::pane {
                border: 1px solid #444;
                background: #1a1a1a;
            }
            QTabBar::tab {
                background: #2b2b2b;
                color: white;
                padding: 8px 12px;
                border: 1px solid #444;
                border-bottom: none;
            }
            QTabBar::tab:selected {
                background: #1a1a1a;
            }
            QTabBar::tab:hover {
                background: #333333;
            }
        """)
        
        self.status_bar.setStyleSheet("""
            QStatusBar {
                background: #1a1a1a;
                color: white;
                padding: 5px;
            }
        """)
        
        # Update voice control widget styles
        self.voice_control.status_label.setStyleSheet("""
            font-size: 14pt;
            color: #64b5f6;
            font-weight: bold;
        """)
        
        self.voice_control.command_history.setStyleSheet("""
            font-size: 12pt;
            color: #90caf9;
            padding: 10px;
            background: #1a1a1a;
            border-radius: 5px;
        """)
        
        # Update help text for dark mode
        help_text = """
        <div style='background: #1a1a1a; padding: 15px; border-radius: 8px; border: 1px solid #444;'>
            <h3 style='color: #64b5f6; margin-bottom: 10px;'>Voice Command Examples:</h3>
            <ul style='color: #90caf9;'>
                <li><b>Set Single Actuator Pressure (Actuator 1-3, Value 0-13 kPa):</b>
                    <ul>
                        <li>"Set actuator one pressure to 5"</li>
                        <li>"Actuator 2 pressure 7.5 kpa"</li>
                    </ul>
                </li>
                <li><b>Set All Actuators Pressure (Value 0-13 kPa):</b>
                    <ul>
                        <li>"Deflate all actuators" (sets all to 0 kPa)</li>
                        <li>"Inflate all actuators" (default 10 kPa, or specify value)</li>
                        <li>"Set all actuators to three kilopascals"</li>
                    </ul>
                </li>
                <li><b>Fun Modes:</b>
                    <ul>
                        <li>"Dance for me" / "Start dancing" (Simple dance patterns)</li>
                        <li>"Random mode" / "Activate random"</li>
                        <li>"Stop dancing" / "Stop random mode" / "Stop movement"</li>
                        <li><i>Note: For advanced choreographed dancing with music, use the 🎭 Dance Control tab</i></li>
                    </ul>
                </li>
                <li><b>Emergency Stop:</b>
                    <ul>
                        <li>"Emergency stop", "Stop all", "E stop"</li>
                    </ul>
                </li>
            </ul>
            <p style='color: #777; font-size: 9pt; margin-top: 10px;'>Note: Numbers can be spoken as digits (e.g., "5") or words (e.g., "five").</p>
        </div>
        """
        self.voice_control.help_label.setText(help_text)
        
        # Update sensor monitor plots
        self.sensor_monitor.pressure_plot.setBackground('#1a1a1a')
        for plot in self.sensor_monitor.pot_plots:
            plot.setBackground('#1a1a1a')
        
        # Update joystick control widget styles
        if hasattr(self, 'joystick_widget') and self.joystick_widget:
            self.joystick_widget.update_theme_dependent_styles(True)
        
        # Update matplotlib figure background
        if self.visualization_widget:
            self.visualization_widget.plot_figure.patch.set_facecolor('#1a1a1a')
            self.visualization_widget.ax.set_facecolor('#1a1a1a')
            self.visualization_widget.draw_idle()

    def apply_light_theme(self):
        app = QApplication.instance()
        app.setPalette(LightPalette())
        app.setStyleSheet("""
            QMainWindow {
                background-color: #f0f0f0;
            }
            QWidget {
                font-family: 'Segoe UI', Arial, sans-serif;
            }
            QGroupBox {
                border: 1px solid #ccc;
                border-radius: 5px;
                margin-top: 1em;
                padding-top: 10px;
                background: #ffffff;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 3px;
                color: black;
            }
            QPushButton {
                background-color: #0078d4;
                color: white;
                border: none;
                padding: 8px 16px;
                border-radius: 4px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #1084e3;
            }
            QPushButton:pressed {
                background-color: #006cbd;
            }
            QPushButton:disabled {
                background-color: #cccccc;
                color: #666666;
            }
            QSlider::groove:horizontal {
                border: 1px solid #ccc;
                height: 8px;
                background: #f0f0f0;
                margin: 2px 0;
            }
            QSlider::handle:horizontal {
                background: #0078d4;
                border: 1px solid #ccc;
                width: 18px;
                margin: -2px 0;
                border-radius: 3px;
            }
            QSlider::handle:horizontal:hover {
                background: #1084e3;
            }
            QSpinBox {
                background: #ffffff;
                border: 1px solid #ccc;
                border-radius: 3px;
                padding: 3px;
                color: black;
            }
            QLabel {
                color: black;
            }
            QCheckBox {
                color: black;
            }
            QLineEdit {
                background: #ffffff;
                border: 1px solid #ccc;
                border-radius: 3px;
                padding: 5px;
                color: black;
            }
            QProgressBar {
                border: 1px solid #ccc;
                border-radius: 3px;
                text-align: center;
                background: #f0f0f0;
            }
            QProgressBar::chunk {
                background-color: #0078d4;
            }
            QTabWidget::pane {
                border: 1px solid #ccc;
                background: #ffffff;
            }
            QTabBar::tab {
                background: #f0f0f0;
                color: black;
                padding: 8px 12px;
                border: 1px solid #ccc;
                border-bottom: none;
            }
            QTabBar::tab:selected {
                background: #ffffff;
            }
            QTabBar::tab:hover {
                background: #e0e0e0;
            }
            QStatusBar {
                background: #f0f0f0;
                color: black;
                padding: 5px;
            }
            QScrollBar:vertical {
                border: 1px solid #ccc;
                background: #f0f0f0;
                width: 10px;
                margin: 0px;
            }
            QScrollBar::handle:vertical {
                background: #cccccc;
                border-radius: 5px;
                min-height: 20px;
            }
            QScrollBar::handle:vertical:hover {
                background: #bbbbbb;
            }
            QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {
                height: 0px;
            }
        """)
        
        # Update specific widget styles
        self.tabs.setStyleSheet("""
            QTabWidget::pane {
                border: 1px solid #ccc;
                background: #ffffff;
            }
            QTabBar::tab {
                background: #f0f0f0;
                color: black;
                padding: 8px 12px;
                border: 1px solid #ccc;
                border-bottom: none;
            }
            QTabBar::tab:selected {
                background: #ffffff;
            }
            QTabBar::tab:hover {
                background: #e0e0e0;
            }
        """)
        
        self.status_bar.setStyleSheet("""
            QStatusBar {
                background: #f0f0f0;
                color: black;
                padding: 5px;
            }
        """)
        
        # Update voice control widget styles
        self.voice_control.status_label.setStyleSheet("""
            font-size: 14pt;
            color: #0078d4;
            font-weight: bold;
        """)
        
        self.voice_control.command_history.setStyleSheet("""
            font-size: 12pt;
            color: #333333;
            padding: 10px;
            background: #ffffff;
            border: 1px solid #ccc;
            border-radius: 5px;
        """)
        
        # Update help text for light mode
        help_text = """
        <div style='background: #f8f9fa; padding: 15px; border-radius: 8px; border: 1px solid #ccc;'>
            <h3 style='color: #0078d4; margin-bottom: 10px;'>Voice Command Examples:</h3>
            <ul style='color: #333333;'>
                <li><b>Set Single Actuator Pressure (Actuator 1-3, Value 0-13 kPa):</b>
                    <ul>
                        <li>"Set actuator one pressure to 5"</li>
                        <li>"Actuator 2 pressure 7.5 kpa"</li>
                    </ul>
                </li>
                <li><b>Set All Actuators Pressure (Value 0-13 kPa):</b>
                    <ul>
                        <li>"Deflate all actuators" (sets all to 0 kPa)</li>
                        <li>"Inflate all actuators" (default 10 kPa, or specify value)</li>
                        <li>"Set all actuators to three kilopascals"</li>
                    </ul>
                </li>
                <li><b>Fun Modes:</b>
                    <ul>
                        <li>"Dance for me" / "Start dancing" (Simple dance patterns)</li>
                        <li>"Random mode" / "Activate random"</li>
                        <li>"Stop dancing" / "Stop random mode" / "Stop movement"</li>
                        <li><i>Note: For advanced choreographed dancing with music, use the 🎭 Dance Control tab</i></li>
                    </ul>
                </li>
                <li><b>Emergency Stop:</b>
                    <ul>
                        <li>"Emergency stop", "Stop all", "E stop"</li>
                    </ul>
                </li>
            </ul>
            <p style='color: #555; font-size: 9pt; margin-top: 10px;'>Note: Numbers can be spoken as digits (e.g., "5") or words (e.g., "five").</p>
        </div>
        """
        self.voice_control.help_label.setText(help_text)
        
        # Update sensor monitor plots
        self.sensor_monitor.pressure_plot.setBackground('w')
        for plot in self.sensor_monitor.pot_plots:
            plot.setBackground('w')
        
        # Update joystick control widget styles
        if hasattr(self, 'joystick_widget') and self.joystick_widget:
            self.joystick_widget.update_theme_dependent_styles(False)
        
        # Update matplotlib figure background
        if self.visualization_widget:
            self.visualization_widget.plot_figure.patch.set_facecolor('#ffffff')
            self.visualization_widget.ax.set_facecolor('#ffffff')
            self.visualization_widget.draw_idle()

    def update_actuator_lengths(self, actuator_lengths):
        """
        Update the UI with current actuator lengths from the high-level processor.
        
        Args:
            actuator_lengths (list): List of 6 actuator lengths
        """
        # Skip if we don't have a sensor monitor
        if not hasattr(self, 'sensor_monitor'):
            return
            
        # Update sensor monitor or other UI elements showing actuator lengths
        try:
            for i, length in enumerate(actuator_lengths):
                pot_key = f'potentiometer_{i+1}'
                self.sensor_monitor.update_single_value(pot_key, length)
        except Exception as e:
            print(f"Error updating actuator lengths: {e}")
    
    def update_control_errors(self, control_error_data):
        """
        Update the UI with control error information from the high-level processor.
        
        Args:
            control_error_data (dict): Dictionary containing error and control signal information
        """
        # Skip if high-level processor is disconnected for debugging
        if self.high_level_processor is None:
            return
            
        # This could update a debug panel or status information
        try:
            errors = control_error_data.get('errors', [])
            control_signals = control_error_data.get('control_signals', [])
            
            # Update status bar with simplified error information
            if errors:
                avg_error = sum(abs(e) for e in errors) / len(errors)
                self.status_bar.showMessage(f"Average control error: {avg_error:.2f} mm")
        except Exception as e:
            print(f"Error updating control errors: {e}")

    # --- New Signal Handlers for ActuatorControlWidget Specific Signals --- 

    def handle_target_pose_set(self, pose_list):
        """This method is now deprecated as we've removed pose mode."""
        # Log warning in case it's still being called somewhere
        print("Warning: handle_target_pose_set called but pose mode has been removed.")
        self.status_bar.showMessage("Error: Pose mode is no longer supported.")

    def handle_pressure_command(self, pressures):
        """Handle pressure command from the control panel (sliders/spinboxes)."""
        print(f"MainWin.handle_pressure_command: Received pressures from ControlPanel: {pressures}")

        if self.high_level_processor and self.high_level_processor.should_run:
            print("MainWin: Stopping ML processor for manual pressure command.")
            self.high_level_processor.stop_processing()  # Reverted: no wait=True
            # Add a small delay to allow ML thread to complete its last cycle
            print("MainWin: Waiting for ML processor to fully stop...")
            time.sleep(0.1) # Wait for 2x ML update interval
            print("MainWin: ML processor should be stopped. Proceeding with pressure command.")
            if hasattr(self.high_level_processor, 'set_control_mode'): 
                 self.high_level_processor.set_control_mode('manual_pressure')

        command = {"command": "set_pressure", "values": pressures}
        print(f"MainWin.handle_pressure_command: Formed command: {command}")
        self.send_command_to_esp32(command)
        self.status_bar.showMessage(f"Manual pressure command sent: {pressures}")

    def handle_emergency_stop(self):
        """Handles emergency stop: stops ML, sends e-stop command, resets pressures to 0."""
        print("MainWin: EMERGENCY STOP HANDLER ACTIVATED")
        
        if self.high_level_processor and self.high_level_processor.should_run:
            print("MainWin: Stopping ML processor for E-Stop.")
            self.high_level_processor.stop_processing() # Reverted: no wait=True

        # Stop any voice-controlled fun modes
        if self.voice_control:
            print("MainWin: Telling voice control to stop movement sequences due to E-Stop.")
            self.voice_control.stop_any_movement_sequence()

        # Send E-stop command to ESP32 (ESP32 firmware should handle this by halting)
        self.send_command_to_esp32({"emergency_stop": True})
        
        # Explicitly send command to set all pressures to 0.0 kPa as a software override
        zero_pressures = [0.0, 0.0, 0.0]
        self.send_command_to_esp32({"command": "set_pressure", "values": zero_pressures})
        print(f"MainWin: Sent set_pressure {zero_pressures} to ESP32 for E-Stop.")

        # Update GUI elements to 0 REPEATEDLY to ensure it sticks, especially sliders/spinboxes
        try:
            print("MainWin: Attempting to update ControlPanel GUI to 0 for E-Stop.")
            for i in range(3):
                if self.control_panel:
                    # Set spinbox first, which should then update slider via its connected signal
                    self.control_panel.actuator_spinboxes[i].setValue(0.0)
                    # Directly set slider as well for robustness, in case signal connection has issues
                    self.control_panel.actuator_sliders[i].setValue(0) 
            print("MainWin: ControlPanel GUI elements set to 0.")
        except Exception as e:
            print(f"MainWin: Error updating ControlPanel GUI during E-Stop: {e}")
        
        self.status_bar.showMessage("Emergency Stop Activated. All actuators set to 0 kPa.")

        # Update visualization to neutral pose and zero pressure (min length)
        if self.visualization_widget:
            print("MainWin: Updating 3D visualization for E-Stop (neutral pose).")
            self.visualization_widget.show_neutral_pose() # Reverted show_neutral_pose doesn't need params
            print("MainWin: 3D visualization updated for E-Stop.")
        else:
            print("MainWin: 3D visualization not available for E-Stop update.")

    def handle_start_sequence(self):
        """Handle start auto sequence signal."""
        # This might involve the high-level processor or direct commands
        # For now, just log and send a placeholder command if needed
        print("Start sequence command received by main window.")
        command = {"command": "start_sequence"} # Placeholder
        self.send_command_to_esp32(command)
        self.status_bar.showMessage("Starting auto sequence...")

    def handle_stop_sequence(self):
        """Handle stop auto sequence signal."""
        # This might involve the high-level processor or direct commands
        print("Stop sequence command received by main window.")
        command = {"command": "stop_sequence"} # Placeholder
        self.send_command_to_esp32(command)
        self.status_bar.showMessage("Stopping auto sequence.")

    # --- End New Signal Handlers --- 

    # New method to forward data to the high-level processor
    def forward_data_to_processor(self, data_dict):
        """Forward received data to the high-level processor"""
        if self.high_level_processor is not None:
            try:
                self.high_level_processor.add_sensor_data(data_dict)
            except Exception as e:
                print(f"Error forwarding data to high-level processor: {e}")

    def handle_length_command(self, lengths):
        """Handle length command from the control panel (direct length input)."""
        if self.high_level_processor:
            # Ensure any previous pressure control mode is overridden by restarting processor
            if self.high_level_processor.should_run: # If already running, stop it first
                self.high_level_processor.stop_processing(wait=False) # Non-blocking stop
            
            # Set target lengths and start the processor
            self.high_level_processor.set_target_lengths(lengths)
            self.high_level_processor.start_processing() # This will use the new target_lengths
            print(f"MainWin: ML processor (re)started for length control with lengths: {lengths}.")
            self.status_bar.showMessage(f"Length control mode activated. Target lengths: {lengths}")
            if hasattr(self.high_level_processor, 'set_control_mode'): # Optional: inform processor
                 self.high_level_processor.set_control_mode('length_control')
        else:
            print("MainWin: High-level processor not available for length command.")
            self.status_bar.showMessage("Error: ML processor not available.")
        
        # Visualization will be updated by 'update_visualization_pose'
        # when the high_level_processor emits its 'pose_updated' signal with new lengths.

    # Renamed from update_visualization_pose and changed signature
    def update_visualization_pose(self, pose_list):
        """
        Update the 3D visualization based on pose data from the high-level processor.
        Expected pose_list: [x, y, z, roll, pitch, yaw]
        """
        if not pose_list or len(pose_list) < 6:
            print(f"Visualization Error: Expected pose_list [x,y,z,r,p,y], got {pose_list}")
            return
        
        # The new Graphic3d update_pose_from_list expects a flat list
        if self.visualization_widget:
            self.visualization_widget.update_pose_from_list(pose_list)

    def handle_joystick_pressure_command(self, pressures):
        """Handle pressure commands from the joystick widget."""
        print(f"MainWin.handle_joystick_pressure_command: Received pressures from Joystick: {pressures}")

        if self.high_level_processor and self.high_level_processor.should_run:
            print("MainWin.handle_joystick_pressure_command: Warning - ML processor is still running. Attempting to stop.")
            self.high_level_processor.stop_processing() # Reverted: no wait=True
            # Add a small delay to allow ML thread to complete its last cycle
            print("MainWin: Waiting for ML processor to fully stop (joystick command)...")
            time.sleep(0.1) # Wait for 2x ML update interval
            print("MainWin: ML processor should be stopped. Proceeding with joystick pressure command.")

        command = {"command": "set_pressure", "values": pressures}
        print(f"MainWin.handle_joystick_pressure_command: Formed command: {command}")
        self.send_command_to_esp32(command)
        
        # Update GUI elements (sliders/spinboxes on ControlPanel) to reflect joystick changes
        updated_pressures_for_status = []
        for i in range(3):
            try:
                pressure_float = float(pressures[i])
                pressure_float = max(0.0, min(13.0, pressure_float)) # Ensure range
                self.control_panel.actuator_sliders[i].setValue(int(pressure_float * 10))
                self.control_panel.actuator_spinboxes[i].setValue(pressure_float)
                updated_pressures_for_status.append(f"{pressure_float:.1f}")
            except Exception as e:
                print(f"Error updating GUI for actuator {i} from joystick: {e}")
                updated_pressures_for_status.append("Err")

        self.status_bar.showMessage(f"Joystick: Pressures P1:{updated_pressures_for_status[0]} P2:{updated_pressures_for_status[1]} P3:{updated_pressures_for_status[2]} kPa")
            
    def stop_ml_processor_for_joystick(self):
        if self.high_level_processor and self.high_level_processor.should_run:
            print("MainWin: Stopping ML processor for joystick mode.")
            self.high_level_processor.stop_processing() # Reverted: no wait=True
        else:
            print("MainWin: stop_ml_processor_for_joystick called, but ML processor was not running or available.")

    def handle_dance_pressure_command(self, pressures):
        """Handle pressure command from the dance control widget."""
        print(f"MainWin.handle_dance_pressure_command: Received pressures from Dance Control: {pressures}")

        if self.high_level_processor and self.high_level_processor.should_run:
            print("MainWin: Stopping ML processor for dance pressure command.")
            self.high_level_processor.stop_processing()
            # Add a small delay to allow ML thread to complete its last cycle
            print("MainWin: Waiting for ML processor to fully stop (dance command)...")
            time.sleep(0.1) # Wait for ML update interval
            print("MainWin: ML processor should be stopped. Proceeding with dance pressure command.")

        command = {"command": "set_pressure", "values": pressures}
        print(f"MainWin.handle_dance_pressure_command: Formed command: {command}")
        self.send_command_to_esp32(command)
        
        # Update GUI elements (sliders/spinboxes on ControlPanel) to reflect dance changes
        updated_pressures_for_status = []
        for i in range(3):
            try:
                pressure_float = float(pressures[i])
                pressure_float = max(0.0, min(13.0, pressure_float)) # Ensure range
                self.control_panel.actuator_sliders[i].setValue(int(pressure_float * 10))
                self.control_panel.actuator_spinboxes[i].setValue(pressure_float)
                updated_pressures_for_status.append(f"{pressure_float:.1f}")
            except Exception as e:
                print(f"Error updating GUI for actuator {i} from dance: {e}")
                updated_pressures_for_status.append("Err")

        self.status_bar.showMessage(f"🎭 Dance: Pressures P1:{updated_pressures_for_status[0]} P2:{updated_pressures_for_status[1]} P3:{updated_pressures_for_status[2]} kPa")

    def stop_ml_processor_for_dance(self):
        """Stop the ML processor when dance mode is activated."""
        if self.high_level_processor and self.high_level_processor.should_run:
            print("MainWin: Stopping ML processor for dance mode.")
            self.high_level_processor.stop_processing()
        else:
            print("MainWin: stop_ml_processor_for_dance called, but ML processor was not running or available.")

    def stop_voice_dance_for_advanced_dance(self):
        """Stop voice-controlled dancing when advanced dance starts."""
        if self.voice_control:
            print("MainWin: Stopping voice-controlled dancing due to advanced dance.")
            self.voice_control.stop_any_movement_sequence()

    def stop_advanced_dance_for_voice_dance(self):
        """Stop advanced dance when voice dancing starts."""
        if self.dance_control and self.dance_control.is_dancing:
            print("MainWin: Stopping advanced dance due to voice dancing.")
            self.dance_control.stop_dance()

    def handle_raw_serial_message(self, message):
        """Handle raw serial message from the serial thread."""
        # Forward to dance control for autonomous mode processing
        self.dance_control.handle_serial_raw_message(message)