import json
import serial
import serial.tools.list_ports
import time
from PyQt5.QtCore import QThread, pyqtSignal

# Remove simulator import entirely
SIMULATION_AVAILABLE = False

class SerialThread(QThread):
    data_received = pyqtSignal(dict)
    connection_error = pyqtSignal(str)
    raw_message_received = pyqtSignal(str)  # New signal for raw messages
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.serial_port = None
        self.running = False
        self.command_queue = []
        self.port = None
        self.baud_rate = 115200
        
        # Add throttling for better performance
        self.last_update_time = 0
        self.update_interval = 150  # ms - increase this value to reduce GUI load

    def connect_to_device(self, port):
        self.port = port
        self.running = True
        
        if not self.isRunning():
            self.start()
        return True

    def disconnect(self):
        """Safely disconnect from the serial port"""
        # Set running flag to false first to stop the thread loop
        self.running = False
        
        # Clear any pending commands
        self.command_queue.clear()
        
        # Close the serial port if it exists
        if hasattr(self, 'serial_port') and self.serial_port:
            try:
                if hasattr(self.serial_port, 'is_open') and self.serial_port.is_open:
                    self.serial_port.close()
                    print("Serial port closed")
            except Exception as e:
                print(f"Error closing serial port: {e}")
                # Continue despite error

    def send_command(self, command_dict):
        # Queue the command to be sent in the serial thread
        print("Serial_handler: sending", command_dict)
        self.command_queue.append(command_dict)

    def run(self):
        try:
            # Use real serial port - no simulation option anymore
            self.serial_port = serial.Serial(
                port=self.port,
                baudrate=self.baud_rate,
                timeout=0.1
            )
        except Exception as e:
            self.connection_error.emit(f"Connection error: {str(e)}")
            return

        while self.running:
            # Try receiving messages from the ESP32
            try:
                if self.serial_port.in_waiting:
                    message = self.serial_port.readline().decode('utf-8').strip()
                    if message:
                        # Always emit raw message first for autonomous dance detection
                        self.raw_message_received.emit(message)
                        
                        try:
                            data = json.loads(message)
                            
                            # Get current time for throttling
                            current_time = int(time.time() * 1000)  # Convert to milliseconds
                            time_diff = current_time - self.last_update_time
                            
                            # Only emit data at the specified interval
                            if time_diff >= self.update_interval:
                                # Emit data for GUI components and other listeners
                                print("Serial_handler rx: ", data)
                                self.data_received.emit(data)
                                self.last_update_time = current_time
                        except json.JSONDecodeError:
                            # Not JSON, that's okay - we already emitted the raw message
                            pass
            except Exception as e:
                # It is normal to hit a timeout – ignore and continue.
                pass

            # Send any pending commands
            if self.command_queue:
                command = self.command_queue.pop(0)
                try:
                    command_str = json.dumps(command) + '\n'
                    self.serial_port.write(command_str.encode('utf-8'))
                except Exception as e:
                    self.connection_error.emit(f"Send error: {str(e)}")
            self.msleep(10)

    def currentTime(self):
        """
        Get current time in milliseconds for throttling (deprecated - using time.time() directly now)
        """
        return int(time.time() * 1000)

# Function to list available ports - no more simulator
def list_ports():
    """
    List all available serial ports.
    
    Returns:
        list: List of dictionaries with port information
    """
    # Get real hardware ports only
    real_ports = []
    for port in serial.tools.list_ports.comports():
        real_ports.append({
            'device': port.device,
            'description': port.description,
            'hwid': port.hwid
        })
    
    return real_ports 