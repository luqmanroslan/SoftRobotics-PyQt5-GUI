"""
Stewart Platform Processor

Handles high-level control of the Stewart platform using predictive models.
This module uses predictive models for kinematics.
"""

import threading
import time
import numpy as np
from queue import Queue
from PyQt5.QtCore import QObject, pyqtSignal

from MLmodels.ml_kinematics import KinematicModel as PredictionModel # Renamed import for clarity

class MLHighLevelProcessor(QObject):
    """
    Processes sensor data and controls the Stewart platform using predictive models.
    
    This processor runs in its own thread and:
    1. Takes raw sensor readings
    2. Uses predictive models to determine platform pose
    3. Calculates needed actuator lengths
    4. Sends length commands
    """
    
    command_ready = pyqtSignal(dict)
    pose_updated = pyqtSignal(list)
    
    UPDATE_INTERVAL = 0.05
    DEFAULT_ACTUATOR_LENGTH = 200.0
    DEFAULT_POSE = [0, 0, 200, 0, 0, 0]
    DEBUG_PRINT_INTERVAL = 10
    
    def __init__(self, parent=None):
        """Setup the processor and initialize state variables"""
        super().__init__(parent)
        
        self.model = PredictionModel() # Use the renamed import
        
        self.incoming_data = Queue()
        
        self.actuator_targets = [self.DEFAULT_ACTUATOR_LENGTH] * 3
        self.sensor_values = [0, 0, 0, 200, 200, 200, 200, 200, 200]
        self.platform_pose = None
        
        self.should_run = False
        self.worker_thread = None
        self.cycle_counter = 0
        
    def start_processing(self):
        """Launch the processing thread if not already running"""
        if self.worker_thread is not None and self.should_run:
            print("Processing thread already running")
            return
            
        self.should_run = True
        self.worker_thread = threading.Thread(target=self._run_processing_loop)
        self.worker_thread.daemon = True
        self.worker_thread.start()
        print("Processing thread started")
        
    def stop_processing(self):
        """Stop the processing thread gracefully."""
        if not self.should_run:
            print("MLHighLevelProcessor: stop_processing called but should_run is already False.")
            if self.worker_thread is not None and not self.worker_thread.is_alive():
                self.worker_thread = None
            return

        print("MLHighLevelProcessor: stop_processing - setting should_run to False.")
        self.should_run = False 
        self.cycle_counter = 0

    def add_sensor_data(self, data_package):
        self.incoming_data.put(data_package)
        
    def set_target_lengths(self, target_lengths):
        if len(target_lengths) != 3:
            print(f"Error: Expected 3 length values, got {len(target_lengths)}")
            return
        self.actuator_targets = target_lengths.copy()
        
    def _run_processing_loop(self):
        """Main loop that processes sensor data and generates commands"""
        print("Starting processing loop")
        
        while self.should_run:
            if not self.incoming_data.empty():
                try:
                    sensor_package = self.incoming_data.get()
                    self._process_package(sensor_package)
                except Exception as e:
                    print(f"ERROR in processing: {e}")
            time.sleep(self.UPDATE_INTERVAL)
            
    def _process_package(self, sensor_package):
        """
        Process one package of sensor data and generate commands
        """
        readings = []
        for axis in ['imu_roll', 'imu_pitch', 'imu_yaw']:
            readings.append(sensor_package.get(axis, 0.0))
        
        for pot_idx in range(6):
            pot_name = f'potentiometer_{pot_idx+1}'
            if pot_name in sensor_package:
                readings.append(sensor_package[pot_name])
            else:
                if len(self.sensor_values) >= 9:
                    readings.append(self.sensor_values[pot_idx+3])
                else:
                    readings.append(self.DEFAULT_ACTUATOR_LENGTH)
        self.sensor_values = readings
        
        try:
            # Apply model to estimate current pose
            estimated_pose = self.model.getFK(self.DEFAULT_POSE, readings) # Using self.model
            self.platform_pose = estimated_pose
            self.pose_updated.emit(self.platform_pose.copy())
        except Exception as e:
            print(f"Forward prediction error: {e}")
            if self.platform_pose is None:
                self.platform_pose = self.DEFAULT_POSE
        
        targets = self.actuator_targets.copy()
        current_lengths = [self.DEFAULT_ACTUATOR_LENGTH] * 3
        length_differences = [0, 0, 0]
        
        try:
            # Use inverse prediction to get vectors
            actuator_vecs = self.model.getIK(self.platform_pose) # Using self.model
            current_lengths = [np.linalg.norm(vec) for vec in actuator_vecs]
            length_differences = [t - c for t, c in zip(targets, current_lengths)]
        except Exception as e:
            print(f"Length calculation error: {e}")
        
        try:
            command_package = {
                'command': 'lengths_control',
                'target_lengths': targets,
                'current_lengths': current_lengths
            }
            self.command_ready.emit(command_package)
            
            self.cycle_counter += 1
            if self.cycle_counter % self.DEBUG_PRINT_INTERVAL == 0:
                pose_rounded = [round(x, 1) for x in self.platform_pose]
                lengths_rounded = [round(x, 1) for x in current_lengths]
                targets_rounded = [round(x, 1) for x in targets]
                errors_rounded = [round(x, 1) for x in length_differences]
                
                print(f"Platform pose: {pose_rounded}")
                print(f"Current lengths: {lengths_rounded}")
                print(f"Target lengths: {targets_rounded}")
                print(f"Length differences: {errors_rounded}")
        except Exception as e:
            print(f"Command sending error: {e}") 