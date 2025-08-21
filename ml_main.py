import sys
import os
import warnings
from PyQt5.QtWidgets import QApplication, QMessageBox

# Add workspace directories to path
script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(script_dir)
sys.path.append(os.path.join(script_dir, "NEW ML Model"))

# Regular imports
from gui.main_window import SoftRobotGUI
from gui.styles.dark_palette import DarkPalette

# Our ML-based processor
from control.ml_processor import MLHighLevelProcessor

# Communication modules
from communication.length_sender import LengthSender
from communication.serial_handler import SerialThread, list_ports

# Suppress annoying FP16 warning - we're not using it
warnings.filterwarnings("ignore", message="FP16 is not supported on CPU; using FP32 instead")

def main():
    """
    Machine Learning based control system for Stewart platform with pneumatic actuators.
    This implementation uses ML models instead of mathematical kinematics.
    """
    # Initialize Qt application
    app = QApplication(sys.argv)
    
    # Apply our dark theme
    app.setPalette(DarkPalette())
    app.setStyle('Fusion')
    
    # Add custom styling
    app.setStyleSheet("""
        QMainWindow { background-color: #2b2b2b; }
        QWidget { font-family: 'Segoe UI', Arial, sans-serif; }
        QGroupBox {
            border: 1px solid #444;
            border-radius: 5px;
            margin-top: 1em;
            padding-top: 10px;
            background: #2b2b2b;
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
        QPushButton:hover { background-color: #1565c0; }
        QPushButton:pressed { background-color: #0a3d91; }
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
        QSlider::handle:horizontal:hover { background: #1565c0; }
        QSpinBox {
            background: #3b3b3b;
            border: 1px solid #444;
            border-radius: 3px;
            padding: 3px;
            color: white;
        }
        QLabel { color: white; }
        QCheckBox { color: white; }
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
        QProgressBar::chunk { background-color: #0d47a1; }
    """)
    
    # Create our ML-based high-level processor
    ml_processor = MLHighLevelProcessor()
    
    # Try to find a reasonable COM port to use
    default_port = "COM3"
    
    # Find available ports
    ports = list_ports()
    if ports:
        # Look for a COM port that's not the default
        for port_info in ports:
            port_name = port_info['device']
            if 'COM' in port_name and port_name != default_port:
                default_port = port_name
                print(f"Found port: {default_port}")
                break
    
    print(f"Will try to connect to: {default_port}")
    
    # Create serial communication thread
    serial_handler = SerialThread()
    
    # Create length sender for ESP32 communication
    actuator_sender = LengthSender()
    
    # Connect signals between components
    ml_processor.command_ready.connect(actuator_sender.handle_length_command)
    
    # Create main GUI window
    gui = SoftRobotGUI(high_level_processor=ml_processor)
    
    # Setup serial communication in GUI
    gui.set_serial_thread(serial_handler)
    
    # Setup connection handlers
    gui.connection_widget.connection_request.connect(
        lambda port: connect_to_hardware(port, serial_handler, actuator_sender, gui, ml_processor)
    )
    gui.connection_widget.disconnect_request.connect(
        lambda: disconnect_from_hardware(serial_handler, actuator_sender, gui, ml_processor)
    )
    
    # Show the interface
    gui.show()
    
    # Show info about our system setup
    print("\n=== Stewart Platform Control System ===")
    print("• System ready - waiting for user to select control mode")
    print("=======================================\n")
    
    # Start app event loop
    exit_code = app.exec_()
    
    # Cleanup
    actuator_sender.disconnect()
    serial_handler.disconnect()
    
    # Exit with the result code from Qt
    sys.exit(exit_code)


def connect_to_hardware(port, serial_handler, actuator_sender, gui, ml_processor):
    """Connect to the hardware platform"""
    # Check if we're already connected
    if serial_handler.running:
        print(f"Already connected to hardware")
        return
    
    # Attempt to connect
    if serial_handler.connect_to_device(port):
        # Connect length sender to serial thread
        actuator_sender.connect(serial_handler)
        print(f"Connected to hardware on {port}")
        
        # Update GUI status
        gui.connection_widget.update_connection_status(True)
        gui.status_bar.showMessage(f"Connected to hardware on {port} - Select control mode to begin")
        
        # Connect processor to command handler (but don't start it yet)
        ml_processor.command_ready.connect(gui.send_command_to_esp32)
        
        # Don't automatically start the processor - wait for user to select control mode
        # ml_processor.start_processing()
        
        # Send initialization command to hardware
        gui.send_command_to_esp32({"command": "init"})
        print("Hardware connected - waiting for user to select control mode")
    else:
        # Connection failed
        print(f"Failed to connect to {port}")
        gui.connection_widget.update_connection_status(False, "Connection failed")
        gui.status_bar.showMessage(f"Failed to connect to {port}")


def disconnect_from_hardware(serial_handler, actuator_sender, gui, ml_processor):
    """Disconnect from the hardware platform"""
    # Disconnect processor first
    try:
        ml_processor.command_ready.disconnect(gui.send_command_to_esp32)
        ml_processor.stop_processing()
    except Exception as e:
        print(f"Warning during processor disconnect: {e}")
    
    # Disconnect actuator sender
    actuator_sender.disconnect()
    
    # Disconnect serial thread
    serial_handler.disconnect()
    
    print("Disconnected from hardware")
    
    # Update GUI status
    gui.connection_widget.update_connection_status(False)
    gui.status_bar.showMessage("Disconnected from hardware - Connect and select control mode to begin")
    
    # Don't automatically set any target lengths - keep system neutral
    # ml_processor.set_target_lengths([200.0, 200.0, 200.0])
    # print("Platform reset to neutral position")
    print("System returned to neutral state")


# Main entry point
if __name__ == '__main__':
    main() 