"""
Kinematics Model for Stewart Platform
Handles forward and inverse kinematics using trained predictive models.

This code is used to predict platform pose from sensor readings and 
calculate actuator lengths needed to achieve desired positions.
"""

import numpy as np
import os
from pathlib import Path
import tensorflow as tf
from tensorflow import keras
from joblib import load

class KinematicModel:
    MODEL_PATH_NOTE = "Models are expected to be in NEW ML Model directory relative to this file's parent."
    DEFAULT_POSE = [0, 0, 200, 0, 0, 0]
    SENSOR_COUNT = 9
    
    def __init__(self):
        models_folder = Path(__file__).parent.parent / "NEW ML Model"
        
        # Load the first model (e.g., for forward kinematics)
        self.model1_predictor = load(models_folder / "rf_pose_model.joblib")
        self.model1_normalizer = load(models_folder / "fk_scaler.joblib")
        
        # Load the second model (e.g., for inverse kinematics)
        self.model2_predictor = keras.models.load_model(models_folder / "ik_nn_model.keras")
        self.model2_normalizer = load(models_folder / "ik_scaler.joblib")
        
        print("Stewart platform predictive models loaded from disk")
    
    def getFK(self, unused_pose, sensor_values, debug_output=False):
        """
        Estimates platform position using sensor data and a predictive model.
        
        Inputs:
            unused_pose: Kept for compatibility.
            sensor_values: List of sensor readings.
            debug_output: Whether to print diagnostic info.
            
        Returns:
            Estimated 6-DOF pose.
        """
        if len(sensor_values) != self.SENSOR_COUNT:
            raise ValueError(f"Need exactly {self.SENSOR_COUNT} sensor readings, received {len(sensor_values)}")
        
        normalized_input = self.model1_normalizer.transform([sensor_values])
        predicted_pose = self.model1_predictor.predict(normalized_input)[0]
        
        if debug_output:
            print("--- Forward Prediction Debug ---")
            print(f"Input sensors: {sensor_values}")
            print(f"Output pose: {predicted_pose}")
        
        return predicted_pose
    
    def getIK(self, desired_pose):
        """
        Calculates actuator lengths needed for a desired pose using a predictive model.
        
        Input:
            desired_pose: Target position.
            
        Returns:
            actuator_vecs: 3x3 array of actuator vectors.
        """
        scaled_pose = self.model2_normalizer.transform([desired_pose])
        lengths = self.model2_predictor.predict(scaled_pose)[0]
        
        result_vectors = np.zeros((3, 3))
        for idx in range(3):
            result_vectors[idx, 2] = lengths[idx]
            noise = np.random.uniform(-0.01, 0.01, 2)
            result_vectors[idx, 0:2] = noise
        
        return result_vectors 