import serial.tools.list_ports
from PyQt5.QtWidgets import QWidget, QHBoxLayout, QLabel, QPushButton, QComboBox
from PyQt5.QtCore import pyqtSignal

# Import our custom list_ports function
from communication.serial_handler import list_ports

class ConnectionWidget(QWidget):
    connection_request = pyqtSignal(str)
    disconnect_request = pyqtSignal()
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.init_ui()
    
    def init_ui(self):
        layout = QHBoxLayout(self)
        
        layout.addWidget(QLabel("Port:"))
        self.port_combo = QComboBox()
        self.refresh_ports()
        layout.addWidget(self.port_combo)
        
        refresh_btn = QPushButton("Refresh")
        refresh_btn.clicked.connect(self.refresh_ports)
        layout.addWidget(refresh_btn)
        
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self.toggle_connection)
        layout.addWidget(self.connect_btn)
        
        self.status_label = QLabel("Disconnected")
        self.status_label.setStyleSheet("color: red; font-weight: bold;")
        layout.addWidget(self.status_label)
    
    def refresh_ports(self):
        self.port_combo.clear()
        
        # Use our custom function for ports
        ports_info = list_ports()
        
        # Add available hardware ports
        for port_info in ports_info:
            self.port_combo.addItem(port_info['device'], port_info['device'])
        
        # If no ports found, show a message
        if not ports_info:
            self.port_combo.addItem("No ports available")
    
    def toggle_connection(self):
        if self.connect_btn.text() == "Connect":
            # Use the userData (actual port name) rather than display text
            port = self.port_combo.currentData()
            if not port:  # Fall back to text if data not set
                port = self.port_combo.currentText()
                
            if port and port != "No ports available":
                self.connection_request.emit(port)
        else:
            self.disconnect_request.emit()
    
    def update_connection_status(self, connected, message=""):
        if connected:
            self.connect_btn.setText("Disconnect")
            self.status_label.setText("Connected")
            self.status_label.setStyleSheet("color: green; font-weight: bold;")
            self.port_combo.setEnabled(False)
        else:
            self.connect_btn.setText("Connect")
            self.status_label.setText(f"Disconnected {message}")
            self.status_label.setStyleSheet("color: red; font-weight: bold;")
            self.port_combo.setEnabled(True) 