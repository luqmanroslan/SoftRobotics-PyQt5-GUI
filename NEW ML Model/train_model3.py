import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# ML Imports
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
from sklearn.ensemble import RandomForestRegressor
from sklearn.multioutput import MultiOutputRegressor
from sklearn.metrics import mean_squared_error

# TensorFlow/Keras Imports
import tensorflow as tf
from tensorflow import keras

def main():
    """
    This script trains models for pose prediction and control.
    1. Forward Prediction: predicts pose from sensor readings
    2. Inverse Prediction: predicts actuator settings from desired pose
    """

    # ---------------------------
    # 1. LOAD & FILTER THE DATA
    # ---------------------------
    data_path = r"C:\Users\rosla\OneDrive\Desktop\ML Model\simulated_datax.csv"
  
    # Load full dataset to include estimated pose columns for later comparison
    full_df = pd.read_csv(data_path)
    
    print("Loading data from:", data_path)
    print("Data loaded. First 5 rows:\n", full_df.head(), "\n")
    print("Full dataset shape:", full_df.shape)

    # Define output directory
    output_dir = r"C:\Users\rosla\OneDrive\Desktop\ML Model"

    # ########################################
    # PART A: FORWARD PREDICTION (SENSOR → POSE)
    # ########################################
    print("\n===== FORWARD PREDICTION MODELS =====")
    print("Training models to predict pose from sensor readings")
    
    # Columns for forward prediction model
    fk_feature_cols = [
        "IMU roll", "IMU pitch", "IMU yaw",
        "Pot 1 distance", "Pot 2 distance", "Pot 3 distance",
        "Pot 4 distance", "Pot 5 distance", "Pot 6 distance"
    ]
    
    fk_target_cols = [
        "Actual pose x", "Actual pose y", "Actual pose z",
        "Actual pose roll", "Actual pose pitch", "Actual pose yaw"
    ]
    
    estimated_cols = [
        "Estimated pose x", "Estimated pose y", "Estimated pose z",
        "Estimated pose roll", "Estimated pose pitch", "Estimated pose yaw"
    ]
    
    X_fk = full_df[fk_feature_cols].values  # sensor inputs
    y_fk = full_df[fk_target_cols].values  # pose parameters to predict
    
    # Create a mapping from original indices to keep track for comparison later
    indices = np.arange(len(full_df))

    # ---------------------------------------------
    # 2. TRAIN/TEST SPLIT + SCALING FOR FORWARD MODEL
    # ---------------------------------------------
    X_train, X_test, y_train, y_test, indices_train, indices_test = train_test_split(
        X_fk, y_fk, indices, test_size=0.2, random_state=42
    )

    # Get estimated pose values for the test set
    y_estimated = full_df[estimated_cols].values[indices_test]

    fk_scaler = StandardScaler()
    X_train_scaled = fk_scaler.fit_transform(X_train)
    X_test_scaled = fk_scaler.transform(X_test)

    print(f"Train set size: {X_train.shape[0]}, Test set size: {X_test.shape[0]}")

    # --------------------------
    # 3A. FIRST MODEL TYPE (FORWARD)
    # --------------------------
    print("\n===== FIRST MODEL TYPE (FORWARD) =====")
    # Use MultiOutputRegressor to handle multiple targets
    base_model_1 = RandomForestRegressor(n_estimators=100, random_state=42, max_depth=None, min_samples_split=2, n_jobs=None)
    fk_model_1 = MultiOutputRegressor(base_model_1)
    fk_model_1.fit(X_train_scaled, y_train)

    y_pred_fk_model_1 = fk_model_1.predict(X_test_scaled)
    
    mse_fk_model_1_avg = mean_squared_error(y_test, y_pred_fk_model_1)
    print(f"First Model Type Test MSE (Average): {mse_fk_model_1_avg}")

    # Optional: Compare predictions vs. actual visually for each parameter
    plt.figure(figsize=(20, 10))
    for i in range(6):  # Now 6 parameters (x, y, z, roll, pitch, yaw)
        plt.subplot(2, 3, i+1)
        plt.scatter(y_test[:, i], y_pred_fk_model_1[:, i], alpha=0.5, label=f"Model 1 {fk_target_cols[i]}")
        plt.plot([y_test[:, i].min(), y_test[:, i].max()], 
                [y_test[:, i].min(), y_test[:, i].max()],
                'r--', label="Ideal")
        plt.xlabel(f"True {fk_target_cols[i]}")
        plt.ylabel(f"Predicted {fk_target_cols[i]}")
        plt.title(f"First Model Type - {fk_target_cols[i]} Predictions")
        plt.legend()
    plt.tight_layout()
    plt.show()

    # --------------------------
    # 3B. SECOND MODEL TYPE (FORWARD)
    # --------------------------
    print("\n===== SECOND MODEL TYPE (FORWARD) =====")
    fk_model_2 = keras.Sequential([
        keras.layers.Dense(128, activation='relu', input_shape=(X_train_scaled.shape[1],)),
        keras.layers.Dense(64, activation='relu'),
        keras.layers.Dense(32, activation='relu'),
        keras.layers.Dense(6)  # six outputs for x, y, z, roll, pitch, yaw
    ])

    optimizer_fk_2 = keras.optimizers.Adam(learning_rate=0.001)
    fk_model_2.compile(optimizer_fk_2, loss='mean_squared_error')

    fk_history_2 = fk_model_2.fit(
        X_train_scaled, y_train,
        validation_split=0.2,
        epochs=100,
        batch_size=32,
        verbose=1,
        callbacks=[
            keras.callbacks.EarlyStopping(
                monitor='val_loss',
                patience=10,
                restore_best_weights=True
            )
        ]
    )

    test_loss_fk_model_2 = fk_model_2.evaluate(X_test_scaled, y_test, verbose=0)
    print("Second Model Type Test MSE (Average):", test_loss_fk_model_2)

    y_pred_fk_model_2 = fk_model_2.predict(X_test_scaled)
    
    # Plot learning curves
    plt.figure(figsize=(10, 4))
    plt.plot(fk_history_2.history['loss'], label='Training Loss')
    plt.plot(fk_history_2.history['val_loss'], label='Validation Loss')
    plt.xlabel('Epoch')
    plt.ylabel('Loss (MSE)')
    plt.legend()
    plt.title('Second Model Type Learning Curves (Forward Prediction)')
    plt.tight_layout()
    plt.show()

    # Visualize Model 2 predictions
    plt.figure(figsize=(20, 10))
    for i in range(6):  # Now 6 parameters
        plt.subplot(2, 3, i+1)
        plt.scatter(y_test[:, i], y_pred_fk_model_2[:, i], alpha=0.5, label=f"Model 2 {fk_target_cols[i]}")
        plt.plot([y_test[:, i].min(), y_test[:, i].max()], 
                [y_test[:, i].min(), y_test[:, i].max()],
                'r--', label="Ideal")
        plt.xlabel(f"True {fk_target_cols[i]}")
        plt.ylabel(f"Predicted {fk_target_cols[i]}")
        plt.title(f"Second Model Type - {fk_target_cols[i]} Predictions")
        plt.legend()
    plt.tight_layout()
    plt.show()
    
    # --------------------------
    # 4. COMPARISON WITH ESTIMATED POSE (FORWARD)
    # --------------------------
    print("\n===== COMPARING PREDICTION MODELS VS. EXISTING ESTIMATOR (FORWARD) =====")
    mse_estimator_fk = mean_squared_error(y_test, y_estimated, multioutput='raw_values')
    print(f"Built-in Estimator - MSE (Average): {mean_squared_error(y_test, y_estimated)}")
    
    mse_fk_model_1_individual = mean_squared_error(y_test, y_pred_fk_model_1, multioutput='raw_values')
    mse_fk_model_2_individual = mean_squared_error(y_test, y_pred_fk_model_2, multioutput='raw_values')

    # Visualize the comparison - Bar chart of MSE values
    plt.figure(figsize=(15, 8))
    x_comp_fk = np.arange(len(fk_target_cols))
    width_comp_fk = 0.25

    plt.bar(x_comp_fk - width_comp_fk, mse_estimator_fk, width_comp_fk, label='Built-in Estimator')
    plt.bar(x_comp_fk, mse_fk_model_1_individual, width_comp_fk, label='Model 1')
    plt.bar(x_comp_fk + width_comp_fk, mse_fk_model_2_individual, width_comp_fk, label='Model 2')

    plt.xticks(x_comp_fk, fk_target_cols)
    plt.ylabel('Mean Squared Error')
    plt.title('Error Comparison: Built-in Estimator vs. Prediction Models (Forward)')
    plt.legend()
    plt.yscale('log')
    plt.tight_layout()
    plt.savefig(f"{output_dir}/mse_comparison_all_params.png")
    plt.show()

    # Separate position and orientation errors for clearer visualization
    plt.figure(figsize=(15, 10))
    
    # Position errors (x, y, z)
    plt.subplot(2, 1, 1)
    x_pos_comp_fk = np.arange(3)
    plt.bar(x_pos_comp_fk - width_comp_fk, mse_estimator_fk[:3], width_comp_fk, label='Built-in Estimator')
    plt.bar(x_pos_comp_fk, mse_fk_model_1_individual[:3], width_comp_fk, label='Model 1')
    plt.bar(x_pos_comp_fk + width_comp_fk, mse_fk_model_2_individual[:3], width_comp_fk, label='Model 2')
    plt.xticks(x_pos_comp_fk, fk_target_cols[:3])
    plt.ylabel('Mean Squared Error')
    plt.title('Position Error Comparison (X, Y, Z) - Forward')
    plt.legend()
    plt.yscale('log')
    
    # Orientation errors (roll, pitch, yaw)
    plt.subplot(2, 1, 2)
    x_ori_comp_fk = np.arange(3)
    plt.bar(x_ori_comp_fk - width_comp_fk, mse_estimator_fk[3:], width_comp_fk, label='Built-in Estimator')
    plt.bar(x_ori_comp_fk, mse_fk_model_1_individual[3:], width_comp_fk, label='Model 1')
    plt.bar(x_ori_comp_fk + width_comp_fk, mse_fk_model_2_individual[3:], width_comp_fk, label='Model 2')
    plt.xticks(x_ori_comp_fk, fk_target_cols[3:])
    plt.ylabel('Mean Squared Error')
    plt.title('Orientation Error Comparison (Roll, Pitch, Yaw) - Forward')
    plt.legend()
    plt.yscale('log')
    
    plt.tight_layout()
    plt.savefig(f"{output_dir}/mse_comparison_position_orientation.png")
    plt.show()

    # Add direct comparison plots for one position parameter (e.g., x) and one orientation parameter (e.g., pitch)
    fig, axes = plt.subplots(2, 1, figsize=(12, 10))
    
    # For position (x)
    pos_idx = 0  # x position
    plot_size = min(100, len(y_test))
    x_indices = np.arange(plot_size)
    
    axes[0].plot(x_indices, y_test[:plot_size, pos_idx], 'k-', label='Actual')
    axes[0].plot(x_indices, y_estimated[:plot_size, pos_idx], 'r--', label='Built-in Estimator')
    axes[0].plot(x_indices, y_pred_fk_model_1[:plot_size, pos_idx], 'b--', label='Model 1')
    axes[0].plot(x_indices, y_pred_fk_model_2[:plot_size, pos_idx], 'g--', label='Model 2')
    axes[0].set_xlabel('Sample Index')
    axes[0].set_ylabel('X Position')
    axes[0].set_title('Comparison of X Position Predictions')
    axes[0].legend()
    axes[0].grid(True)
    
    # For orientation (pitch)
    angle_idx = 4  # pitch (index 4 after including x,y,z)
    axes[1].plot(x_indices, y_test[:plot_size, angle_idx], 'k-', label='Actual')
    axes[1].plot(x_indices, y_estimated[:plot_size, angle_idx], 'r--', label='Built-in Estimator')
    axes[1].plot(x_indices, y_pred_fk_model_1[:plot_size, angle_idx], 'b--', label='Model 1')
    axes[1].plot(x_indices, y_pred_fk_model_2[:plot_size, angle_idx], 'g--', label='Model 2')
    axes[1].set_xlabel('Sample Index')
    axes[1].set_ylabel('Pitch (degrees)')
    axes[1].set_title('Comparison of Pitch Predictions')
    axes[1].legend()
    axes[1].grid(True)
    
    plt.tight_layout()
    plt.savefig(f"{output_dir}/position_orientation_comparison.png")
    plt.show()

    # Create scatter plots comparing actual vs. predicted values for all parameters
    fig, axes = plt.subplots(2, 3, figsize=(18, 12))
    axes = axes.flatten()
    
    for i, param in enumerate(fk_target_cols):
        axes[i].scatter(y_test[:, i], y_estimated[:, i], alpha=0.5, label='Built-in Estimator')
        axes[i].scatter(y_test[:, i], y_pred_fk_model_1[:, i], alpha=0.5, label='Model 1')
        axes[i].scatter(y_test[:, i], y_pred_fk_model_2[:, i], alpha=0.5, label='Model 2')
        
        # Add ideal line
        min_val = min(y_test[:, i].min(), y_estimated[:, i].min(), y_pred_fk_model_1[:, i].min(), y_pred_fk_model_2[:, i].min())
        max_val = max(y_test[:, i].max(), y_estimated[:, i].max(), y_pred_fk_model_1[:, i].max(), y_pred_fk_model_2[:, i].max())
        axes[i].plot([min_val, max_val], [min_val, max_val], 'k--')
        
        axes[i].set_xlabel(f'Actual {param}')
        axes[i].set_ylabel(f'Predicted {param}')
        axes[i].set_title(f'{param} Predictions vs. Actual')
        axes[i].legend()
        axes[i].grid(True)

    plt.tight_layout()
    plt.savefig(f"{output_dir}/scatter_comparison_all_params.png")
    plt.show()
    
    # ########################################
    # PART B: INVERSE PREDICTION (POSE → ACTUATOR LENGTHS)
    # ########################################
    print("\n===== INVERSE PREDICTION MODEL =====")
    print("Training model to predict actuator lengths from desired pose")
    
    # Define columns for inverse prediction
    ik_feature_cols = [
        "Actual pose x", "Actual pose y", "Actual pose z", 
        "Actual pose roll", "Actual pose pitch", "Actual pose yaw"
    ]
    ik_target_cols = ["l1", "l2", "l3"]
    
    # Extract data for IK model
    X_ik = full_df[ik_feature_cols].values
    y_ik = full_df[ik_target_cols].values
    
    # Train/test split for IK model
    X_ik_train, X_ik_test, y_ik_train, y_ik_test = train_test_split(
        X_ik, y_ik, test_size=0.2, random_state=42
    )
    
    # Scale IK inputs
    ik_scaler = StandardScaler()
    X_ik_train_scaled = ik_scaler.fit_transform(X_ik_train)
    X_ik_test_scaled = ik_scaler.transform(X_ik_test)
    
    print(f"IK Train set size: {X_ik_train.shape[0]}, Test set size: {X_ik_test.shape[0]}")
    
    # --------------------------
    # INVERSE PREDICTION MODEL TYPE
    # --------------------------
    print("\n===== MODEL FOR INVERSE PREDICTION =====")
    ik_model = keras.Sequential([
        keras.layers.Dense(64, activation='relu', input_shape=(X_ik_train_scaled.shape[1],)),
        keras.layers.Dense(128, activation='relu'),
        keras.layers.Dense(64, activation='relu'),
        keras.layers.Dense(3)  # Three outputs for l1, l2, l3
    ])
    
    ik_optimizer = keras.optimizers.Adam(learning_rate=0.001)
    ik_model.compile(optimizer=ik_optimizer, loss='mean_squared_error')

    ik_history = ik_model.fit(
        X_ik_train_scaled, y_ik_train,
        validation_split=0.2,
        epochs=100,
        batch_size=32,
        verbose=1,
        callbacks=[
            keras.callbacks.EarlyStopping(
                monitor='val_loss',
                patience=10,
                restore_best_weights=True
            )
        ]
    )
    
    ik_test_loss = ik_model.evaluate(X_ik_test_scaled, y_ik_test, verbose=0)
    print(f"Inverse Prediction Model - MSE (Average): {ik_test_loss}")
    
    y_ik_pred = ik_model.predict(X_ik_test_scaled)
    
    ik_param_names = ["l1", "l2", "l3"]
    
    # Plot IK learning curves
    plt.figure(figsize=(10, 4))
    plt.plot(ik_history.history['loss'], label='Training Loss')
    plt.plot(ik_history.history['val_loss'], label='Validation Loss')
    plt.xlabel('Epoch')
    plt.ylabel('Loss (MSE)')
    plt.legend()
    plt.title('Inverse Prediction Learning Curves')
    plt.tight_layout()
    plt.show()
    
    # Visualize IK predictions
    plt.figure(figsize=(15, 5))
    for i in range(3):
        plt.subplot(1, 3, i+1)
        plt.scatter(y_ik_test[:, i], y_ik_pred[:, i], alpha=0.5)
        plt.plot([y_ik_test[:, i].min(), y_ik_test[:, i].max()], 
                [y_ik_test[:, i].min(), y_ik_test[:, i].max()],
                'r--', label="Ideal")
        plt.xlabel(f"True {ik_param_names[i]}")
        plt.ylabel(f"Predicted {ik_param_names[i]}")
        plt.title(f"Inverse Prediction - {ik_param_names[i]} Predictions")
        plt.legend()
    plt.tight_layout()
    plt.savefig(f"{output_dir}/ik_predictions.png")
    plt.show()
    
    # --------------------------
    # SAVE ALL MODELS
    # --------------------------
    # Import joblib for saving models
    import joblib
    
    # Save Forward Prediction models
    joblib.dump(fk_model_1, f"{output_dir}/rf_pose_model.joblib")
    joblib.dump(fk_scaler, f"{output_dir}/fk_scaler.joblib")
    fk_model_2.save(f"{output_dir}/fk_nn_pose_model.keras")
    
    # Save Inverse Prediction model
    joblib.dump(ik_scaler, f"{output_dir}/ik_scaler.joblib")
    ik_model.save(f"{output_dir}/ik_nn_model.keras")
    
    print("\nAll models saved successfully!")
    print("Forward prediction models: rf_pose_model.joblib, fk_nn_pose_model.keras")
    print("Inverse prediction model: ik_nn_model.keras")
    print("\nScript complete! Review the plots to see model performance.")


if __name__ == "__main__":
    main()