"""
ESP32 Actuator Length Communicator

Handles sending actuator length commands to the ESP32 controller.
The ESP32 converts these lengths to pneumatic pressure values internally.
"""

import json
from PyQt5.QtCore import QObject, pyqtSignal, pyqtSlot

from communication.serial_handler import SerialThread

class LengthSender(QObject):
    """
    Stewart Platform length command sender
    
    Sends length commands to the ESP32 controller, which then
    converts these to pressure values for the pneumatic actuators.
    """
    
    # Signal for sensor data notifications
    sensor_data_received = pyqtSignal(dict)
    
    # Class constants
    DEFAULT_PORT = "COM3"
    NUM_ACTUATORS = 3
    DECIMAL_PRECISION = 2
    
    def __init__(self, port=DEFAULT_PORT):
        """Create a new length sender instance"""
        super().__init__()
        self.com_port = port
        self.serial_thread = None
        self.is_connected = False
        
        # Add some custom tracking variables
        self.packet_counter = 0
        self.last_sent_lengths = [0, 0, 0]
    
    def connect(self, serial_thread=None):
        """
        Connect to the hardware using an existing serial thread or create new one
        
        Arguments:
            serial_thread: Optional existing SerialThread instance
        
        Returns:
            True if connection successful, False otherwise
        """
        # Use provided thread or create a new one if needed
        if serial_thread is not None:
            self.serial_thread = serial_thread
        else:
            # Create a new thread and connect it to our port
            self.serial_thread = SerialThread()
            if not self.serial_thread.connect_to_device(self.com_port):
                print(f"Failed to connect to ESP32 on {self.com_port}")
                return False
        
        # Connect the serial thread's signals to our handlers
        self.serial_thread.data_received.connect(self._handle_incoming_data)
        
        # Update connection status
        self.is_connected = True
        print(f"Length sender connected on port {self.com_port}")
        return True
    
    def disconnect(self):
        """Disconnect from the hardware and clean up"""
        if self.serial_thread:
            # Disconnect our signal handler first
            try:
                self.serial_thread.data_received.disconnect(self._handle_incoming_data)
            except TypeError:
                # This is fine - signal might not be connected
                pass
                
        # Update status 
        self.is_connected = False
        self.packet_counter = 0  # Reset packet counter
        print("Length sender disconnected")
    
    def start_communication(self):
        """Start communication with hardware (no-op, managed by SerialThread)"""
        # This function exists for API compatibility
        # The SerialThread manages its own communication
        pass
    
    def stop_communication(self):
        """Stop communication with hardware (no-op, managed by SerialThread)"""
        # This function exists for API compatibility
        # The SerialThread manages its own communication
        pass
    
    def send_lengths(self, lengths):
        """
        Send actuator length values to ESP32
        
        Args:
            lengths: List of actuator lengths to send
        
        Returns:
            True if command sent successfully, False otherwise
        """
        # Check connection status
        if not self.is_connected or not self.serial_thread:
            print("Cannot send lengths: No connection to ESP32")
            return False
            
        # Validate input length
        if len(lengths) != self.NUM_ACTUATORS:
            print(f"Error: Expected {self.NUM_ACTUATORS} length values, got {len(lengths)}")
            return False
        
        # Create our command packet 
        length_command = {
            "command": "length",  # Command type
            "values": [
                round(float(lengths[0]), self.DECIMAL_PRECISION),
                round(float(lengths[1]), self.DECIMAL_PRECISION),
                round(float(lengths[2]), self.DECIMAL_PRECISION)
            ]
        }
        
        # Track what we're sending
        self.last_sent_lengths = lengths.copy()
        self.packet_counter += 1
        
        # Send command through serial thread
        self.serial_thread.send_command(length_command)
        
        # If this is our 10th packet, print debug info
        if self.packet_counter % 10 == 0:
            values_str = ", ".join(f"{l:.1f}" for l in lengths)
            print(f"Sent length packet #{self.packet_counter}: [{values_str}]")
            
        return True
    
    @pyqtSlot(dict)
    def _handle_incoming_data(self, data):
        """
        Process data received from ESP32
        
        Args:
            data: Dictionary of sensor values from ESP32
        """
        # Check for IMU data and log it if present
        has_imu = all(key in data for key in ['imu_roll', 'imu_pitch', 'imu_yaw'])
        
        if has_imu and self.packet_counter % 15 == 0:  # Only log occasionally
            roll = data['imu_roll']
            pitch = data['imu_pitch']
            yaw = data['imu_yaw']
            print(f"Platform orientation: roll={roll:.1f}°, pitch={pitch:.1f}°, yaw={yaw:.1f}°")
        
        # Forward the data to anyone listening to our signal
        self.sensor_data_received.emit(data)
    
    def handle_length_command(self, command):
        """
        Process length commands from processor
        
        Args:
            command: Command dictionary from processor
            
        Returns:
            True if successfully processed, False otherwise
        """
        # Check for supported command formats
        if 'lengths' in command:
            # Old format - simple length array
            return self.send_lengths(command['lengths'])
            
        elif 'command' in command and command['command'] == 'lengths_control':
            # New enhanced format with target and current lengths
            
            # Check connection first
            if not self.is_connected or not self.serial_thread:
                print("Cannot send control command: Not connected to ESP32")
                return False
            
            # Forward entire command to ESP32 for processing
            self.serial_thread.send_command(command)
            
            # Increment packet counter
            self.packet_counter += 1
            
            # Log occasionally for monitoring
            if self.packet_counter % 20 == 0 and 'target_lengths' in command:
                targets = command['target_lengths']
                targets_str = ", ".join(f"{l:.1f}" for l in targets)
                print(f"Sent control packet #{self.packet_counter} with targets: [{targets_str}]")
            
            return True
            
        else:
            # Unknown format
            print(f"Unsupported command format: {list(command.keys())}")
            return False 