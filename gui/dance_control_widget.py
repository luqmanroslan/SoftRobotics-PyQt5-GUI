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

class DanceControlWidget(QWidget):
    # Signals
    pressure_command_sent = pyqtSignal(list)
    stop_ml_processor_requested = pyqtSignal()
    advanced_dance_started = pyqtSignal()  # New signal to notify other systems
    esp32_command_sent = pyqtSignal(dict)  # New signal to send commands to ESP32

    def __init__(self, parent=None):
        super().__init__(parent)
        
        pygame.mixer.init()  # Initialize audio mixer for music
        
        # Dance Mode Settings
        self.is_dancing = False
        self.dance_timer = QTimer(self)
        self.dance_timer.timeout.connect(self._execute_dance_step)
        self.dance_step_interval = 200  # milliseconds per dance step
        self.dance_step_index = 0
        self.dance_start_time = 0
        self.current_dance_sequence = []
        
        # Music synchronization
        self.music_duration_seconds = 0.0
        self.music_timer = QTimer(self)
        self.music_timer.timeout.connect(self._check_music_finished)
        self.music_check_interval = 500  # Check every 500ms if music is still playing
        
        # Music settings
        self.music_file_path = ""
        self.music_volume = 0.7

        # Target pressures for dance
        self.target_pressures = [6.5, 6.5, 6.5]  # Default neutral pressures
        self.min_pressure = 0.0  # kPa
        self.max_pressure = 13.0 # kPa

        # Mapping parameters for pressure calculations
        self.mapping_params = {
            'base_pressure': 6.5,      # Base pressure when centered (kPa)
        }

        self.init_ui()
        self._create_butterfly_dance_sequence()
        
        # Initialize dance info display after UI is fully created
        self.update_dance_info_display()

        # Initialize ESP32 dance variables
        self.esp32_passive_mode_enabled = False
        self.esp32_music_file = ""
        self.esp32_dance_active = False
        self.esp32_music_timer = QTimer(self)
        self.esp32_music_timer.timeout.connect(self.play_esp32_music)
        self.esp32_music_timer.setSingleShot(True)
        
        # Initialize status labels that may not be created yet
        self.dance_status_label = getattr(self, 'dance_status_label', None)
        self.current_step_label = getattr(self, 'current_step_label', None)

    def init_ui(self):
        layout = QVBoxLayout(self)
        
        # Title
        title_label = QLabel("🎭 Robot Dance Control System")
        title_label.setStyleSheet("font-size: 18px; font-weight: bold; color: #2196F3; margin: 10px;")
        layout.addWidget(title_label)
        
        # Important distinction note
        distinction_note = QLabel("⚠️ Note: This is the advanced dance system with music synchronization.\nFor simple voice-controlled dancing, use Voice Control tab with commands like 'dance for me'.")
        distinction_note.setStyleSheet("""
            color: #FF9800; 
            font-weight: bold; 
            background-color: rgba(255, 152, 0, 0.1); 
            padding: 8px; 
            border: 1px solid #FF9800; 
            border-radius: 4px;
            margin: 5px;
        """)
        layout.addWidget(distinction_note)
        
        # Create tab widget for different dance modes
        self.dance_tabs = QTabWidget()
        self.dance_tabs.setStyleSheet("""
            QTabWidget::pane {
                border: 1px solid #C0C0C0;
                background-color: white;
            }
            QTabBar::tab {
                background-color: #E0E0E0;
                padding: 8px 16px;
                margin-right: 2px;
                border-top-left-radius: 4px;
                border-top-right-radius: 4px;
            }
            QTabBar::tab:selected {
                background-color: #2196F3;
                color: white;
                font-weight: bold;
            }
            QTabBar::tab:hover {
                background-color: #BBDEFB;
            }
        """)
        
        # Create individual tabs
        self.create_basic_dance_tab()
        self.create_esp32_dance_tab()
        
        layout.addWidget(self.dance_tabs)

    def create_basic_dance_tab(self):
        """Create the basic dance control tab"""
        basic_tab = QWidget()
        layout = QVBoxLayout(basic_tab)
        
        # Music controls
        music_group = QGroupBox("🎵 Music Controls")
        music_layout = QVBoxLayout(music_group)
        
        # Music file selection
        file_layout = QHBoxLayout()
        self.music_file_label = QLabel("No music file loaded")
        self.load_music_btn = QPushButton("Load Music File")
        self.load_music_btn.clicked.connect(self.load_music_file)
        file_layout.addWidget(self.music_file_label)
        file_layout.addWidget(self.load_music_btn)
        music_layout.addLayout(file_layout)
        
        # Volume control
        volume_layout = QHBoxLayout()
        volume_layout.addWidget(QLabel("Volume:"))
        self.volume_slider = QSlider(Qt.Horizontal)
        self.volume_slider.setRange(0, 100)
        self.volume_slider.setValue(int(self.music_volume * 100))
        self.volume_slider.valueChanged.connect(self.update_music_volume)
        volume_layout.addWidget(self.volume_slider)
        music_layout.addLayout(volume_layout)
        
        # Music playback controls
        playback_layout = QHBoxLayout()
        self.play_music_btn = QPushButton("▶️ Play Music")
        self.play_music_btn.clicked.connect(self.play_music)
        self.stop_music_btn = QPushButton("⏹️ Stop Music")
        self.stop_music_btn.clicked.connect(self.stop_music)
        playback_layout.addWidget(self.play_music_btn)
        playback_layout.addWidget(self.stop_music_btn)
        music_layout.addLayout(playback_layout)
        
        layout.addWidget(music_group)
        
        # Dance controls
        dance_group = QGroupBox("🕺 Dance Controls")
        dance_layout = QVBoxLayout(dance_group)
        
        # Dance sequence selection
        sequence_layout = QHBoxLayout()
        sequence_layout.addWidget(QLabel("Sequence:"))
        self.dance_sequence_combo = QComboBox()
        self.dance_sequence_combo.addItems(["Butterfly Dance", "Custom Sequence"])
        sequence_layout.addWidget(self.dance_sequence_combo)
        dance_layout.addLayout(sequence_layout)
        
        # Dance duration controls
        duration_group = QGroupBox("⏱️ Duration Settings")
        duration_layout = QVBoxLayout(duration_group)
        
        # Auto-detect or manual duration
        detect_layout = QHBoxLayout()
        detect_layout.addWidget(QLabel("Music Duration (seconds):"))
        self.music_duration_spinbox = QDoubleSpinBox()
        self.music_duration_spinbox.setRange(5.0, 300.0)  # 5 seconds to 5 minutes
        self.music_duration_spinbox.setValue(15.0)  # Default to 15 seconds
        self.music_duration_spinbox.setSingleStep(1.0)
        self.music_duration_spinbox.setDecimals(1)
        detect_layout.addWidget(self.music_duration_spinbox)
        
        self.auto_adapt_checkbox = QCheckBox("Auto-adapt dance to music duration")
        self.auto_adapt_checkbox.setChecked(True)
        detect_layout.addWidget(self.auto_adapt_checkbox)
        duration_layout.addLayout(detect_layout)
        
        # Adapt button
        self.adapt_dance_btn = QPushButton("🔄 Adapt Dance to Duration")
        self.adapt_dance_btn.clicked.connect(self.adapt_dance_to_duration)
        duration_layout.addWidget(self.adapt_dance_btn)
        
        # Show current dance info
        self.dance_info_label = QLabel("Dance Info: 41 movements, calculating...")
        duration_layout.addWidget(self.dance_info_label)
        
        dance_layout.addWidget(duration_group)
        
        # Dance timing controls
        timing_layout = QHBoxLayout()
        timing_layout.addWidget(QLabel("Step Interval (ms):"))
        self.dance_timing_spinbox = QDoubleSpinBox()
        self.dance_timing_spinbox.setRange(50.0, 1000.0)
        self.dance_timing_spinbox.setValue(self.dance_step_interval)
        self.dance_timing_spinbox.setSingleStep(50.0)
        self.dance_timing_spinbox.setDecimals(0)
        self.dance_timing_spinbox.valueChanged.connect(self.update_dance_timing)
        timing_layout.addWidget(self.dance_timing_spinbox)
        dance_layout.addLayout(timing_layout)
        
        # Dance action buttons
        action_layout = QHBoxLayout()
        self.start_dance_btn = QPushButton("🎭 Start Butterfly Dance")
        self.start_dance_btn.clicked.connect(self.start_butterfly_dance)
        self.start_dance_btn.setStyleSheet("QPushButton { background-color: #4CAF50; color: white; font-weight: bold; }")
        
        self.stop_dance_btn = QPushButton("⏹️ Stop Dance")
        self.stop_dance_btn.clicked.connect(self.stop_dance)
        self.stop_dance_btn.setEnabled(False)
        self.stop_dance_btn.setStyleSheet("QPushButton { background-color: #f44336; color: white; font-weight: bold; }")
        
        action_layout.addWidget(self.start_dance_btn)
        action_layout.addWidget(self.stop_dance_btn)
        dance_layout.addLayout(action_layout)
        
        # Dance status display
        status_group = QGroupBox("📊 Dance Status")
        status_layout = QVBoxLayout(status_group)
        
        self.dance_status_label = QLabel("Dance Status: Ready")
        self.dance_status_label.setStyleSheet("color: #4CAF50; font-weight: bold; font-size: 14px;")
        status_layout.addWidget(self.dance_status_label)
        
        self.current_step_label = QLabel("Current Step: None")
        self.current_step_label.setStyleSheet("color: #666; font-style: italic;")
        status_layout.addWidget(self.current_step_label)
        
        status_group.setMaximumHeight(100)
        layout.addWidget(status_group)
        
        layout.addWidget(dance_group)
        layout.addStretch()  # Add stretch to push everything up
        
        self.dance_tabs.addTab(basic_tab, "🎭 Basic Dance")

    def create_esp32_dance_tab(self):
        """Create the ESP32 dance control tab"""
        esp32_tab = QWidget()
        layout = QVBoxLayout(esp32_tab)
        
        # ESP32 Mode Selection
        mode_group = QGroupBox("🤖 ESP32 Dance Mode")
        mode_layout = QVBoxLayout(mode_group)
        
        # Mode explanation
        mode_info = QLabel("Choose how to interact with your ESP32 for dancing:")
        mode_info.setStyleSheet("color: #666; font-style: italic; margin: 5px;")
        mode_layout.addWidget(mode_info)
        
        # Mode selection radio buttons
        mode_selection_layout = QHBoxLayout()
        
        self.esp32_mode_passive = QCheckBox("🎧 Passive Mode (Listen for ESP32 commands)")
        self.esp32_mode_passive.setChecked(False)
        self.esp32_mode_passive.stateChanged.connect(self.toggle_esp32_passive_mode)
        mode_selection_layout.addWidget(self.esp32_mode_passive)
        
        mode_layout.addLayout(mode_selection_layout)
        
        # Mode descriptions
        passive_desc = QLabel("Passive Mode: ESP32 runs its own dance sequence and sends 'PLAY SONG' when ready for music")
        passive_desc.setStyleSheet("color: #777; font-size: 11px; margin-left: 20px;")
        mode_layout.addWidget(passive_desc)
        
        layout.addWidget(mode_group)
        
        # Music Configuration
        music_group = QGroupBox("🎵 ESP32 Music Configuration")
        music_layout = QVBoxLayout(music_group)
        
        # Music file selection
        music_file_layout = QHBoxLayout()
        music_file_layout.addWidget(QLabel("Music File:"))
        self.esp32_music_file_label = QLabel("No file selected")
        self.esp32_music_file_label.setStyleSheet("color: #666; font-style: italic;")
        self.load_esp32_music_btn = QPushButton("Select Music File")
        self.load_esp32_music_btn.clicked.connect(self.load_esp32_music_file)
        music_file_layout.addWidget(self.esp32_music_file_label)
        music_file_layout.addWidget(self.load_esp32_music_btn)
        music_layout.addLayout(music_file_layout)
        
        # Timing controls
        timing_layout = QHBoxLayout()
        timing_layout.addWidget(QLabel("Music Start Delay (s):"))
        self.esp32_music_delay_spinbox = QDoubleSpinBox()
        self.esp32_music_delay_spinbox.setRange(0.0, 5.0)
        self.esp32_music_delay_spinbox.setValue(1.9)
        self.esp32_music_delay_spinbox.setSingleStep(0.1)
        self.esp32_music_delay_spinbox.setDecimals(1)
        timing_layout.addWidget(self.esp32_music_delay_spinbox)
        music_layout.addLayout(timing_layout)
        
        layout.addWidget(music_group)
        
        # Active Control Section
        control_group = QGroupBox("🎮 Active ESP32 Control")
        control_layout = QVBoxLayout(control_group)
        
        # Active mode description
        active_desc = QLabel("Send commands to ESP32 to start/stop dance sequences:")
        active_desc.setStyleSheet("color: #666; font-style: italic; margin: 5px;")
        control_layout.addWidget(active_desc)
        
        # ESP32 dance controls
        esp32_controls_layout = QHBoxLayout()
        
        self.esp32_dance_btn = QPushButton("🎭 Start ESP32 Dance")
        self.esp32_dance_btn.clicked.connect(self.start_esp32_dance)
        self.esp32_dance_btn.setStyleSheet("QPushButton { background-color: #FF9800; color: white; font-weight: bold; }")
        esp32_controls_layout.addWidget(self.esp32_dance_btn)
        
        self.esp32_stop_btn = QPushButton("⏹️ Stop ESP32 Dance")
        self.esp32_stop_btn.clicked.connect(self.stop_esp32_dance)
        self.esp32_stop_btn.setEnabled(False)
        self.esp32_stop_btn.setStyleSheet("QPushButton { background-color: #f44336; color: white; font-weight: bold; }")
        esp32_controls_layout.addWidget(self.esp32_stop_btn)
        
        control_layout.addLayout(esp32_controls_layout)
        
        layout.addWidget(control_group)
        
        # Status Display
        status_group = QGroupBox("📊 ESP32 Status")
        status_layout = QVBoxLayout(status_group)
        
        self.esp32_status_label = QLabel("ESP32 Status: Ready")
        self.esp32_status_label.setStyleSheet("color: #4CAF50; font-weight: bold;")
        status_layout.addWidget(self.esp32_status_label)
        
        self.esp32_dance_status_label = QLabel("Dance Status: Ready")
        self.esp32_dance_status_label.setStyleSheet("color: #4CAF50; font-weight: bold;")
        status_layout.addWidget(self.esp32_dance_status_label)
        
        layout.addWidget(status_group)
        
        # Instructions
        instructions_group = QGroupBox("📋 How to Use")
        instructions_layout = QVBoxLayout(instructions_group)
        
        instructions_text = QLabel("""
        Two ways to use ESP32 dancing:

        1. PASSIVE MODE: 
           • Enable "Listen for ESP32 commands"
           • Load a music file
           • ESP32 runs its own dance sequence
           • When ESP32 sends "PLAY SONG", music starts automatically

        2. ACTIVE MODE:
           • Load a music file
           • Click "Start ESP32 Dance" to send command to ESP32
           • ESP32 executes dance and sends "PLAY SONG" for music sync
           • Use "Stop ESP32 Dance" to end the sequence

        Note: Requires compatible ESP32 firmware with dance sequences.
        """)
        instructions_text.setStyleSheet("color: #555; line-height: 1.3; font-size: 11px;")
        instructions_text.setWordWrap(True)
        instructions_layout.addWidget(instructions_text)
        
        layout.addWidget(instructions_group)
        
        layout.addStretch()
        
        self.dance_tabs.addTab(esp32_tab, "🤖 ESP32 Dance")

    def load_music_file(self):
        """Load a music file for dancing"""
        from PyQt5.QtWidgets import QFileDialog
        
        file_path, _ = QFileDialog.getOpenFileName(
            self, 
            "Load Music File", 
            "", 
            "Audio Files (*.mp3 *.wav *.ogg);;All Files (*)"
        )
        
        if file_path:
            self.music_file_path = file_path
            filename = os.path.basename(file_path)
            self.music_file_label.setText(f"Loaded: {filename}")
            self.music_file_label.setStyleSheet("color: #4CAF50; font-weight: bold;")
            print(f"Music file loaded: {filename}")
        else:
            print("No music file selected")

    def play_music(self):
        """Play the loaded music file"""
        if not self.music_file_path:
            QMessageBox.warning(self, "No Music", "Please load a music file first.")
            return
            
        try:
            pygame.mixer.music.load(self.music_file_path)
            pygame.mixer.music.set_volume(self.music_volume)
            pygame.mixer.music.play()
            self.play_music_btn.setText("🔊 Playing...")
            self.play_music_btn.setEnabled(False)
            
            # Start music monitoring if dancing
            if self.is_dancing:
                self.music_timer.start(self.music_check_interval)
                print("Started music monitoring timer")
            
            print(f"Playing music: {os.path.basename(self.music_file_path)}")
        except Exception as e:
            print(f"Error playing music: {e}")
            QMessageBox.warning(self, "Music Error", f"Could not play music: {e}")

    def stop_music(self):
        """Stop the music playback"""
        try:
            pygame.mixer.music.stop()
            self.play_music_btn.setText("▶️ Play Music")
            self.play_music_btn.setEnabled(True)
            
            # Stop music monitoring timer
            if self.music_timer.isActive():
                self.music_timer.stop()
                print("Stopped music monitoring timer")
            
            print("Music stopped")
        except Exception as e:
            print(f"Error stopping music: {e}")

    def update_music_volume(self, value):
        """Update the music volume"""
        self.music_volume = value / 100.0
        pygame.mixer.music.set_volume(self.music_volume)
        print(f"Music volume updated to {self.music_volume:.2f}")

    def update_dance_timing(self, value):
        """Update the dance step timing"""
        self.dance_step_interval = int(value)
        print(f"Dance timing updated to {self.dance_step_interval} ms")
        
        # If currently dancing, restart timer with new interval
        if self.is_dancing and self.dance_timer.isActive():
            self.dance_timer.stop()
            self.dance_timer.start(self.dance_step_interval)
        
        # Update dance info display
        self.update_dance_info_display()

    def adapt_dance_to_duration(self):
        """Adapt the dance sequence to the specified duration"""
        target_duration = self.music_duration_spinbox.value()
        self.set_dance_duration(target_duration)
        self.update_dance_info_display()
        print(f"Dance adapted to {target_duration} seconds")

    def update_dance_info_display(self):
        """Update the dance information display"""
        total_movements = len(self.current_dance_sequence)
        total_steps = sum(step[1] for step in self.current_dance_sequence)
        total_duration = total_steps * (self.dance_step_interval / 1000.0)
        
        info_text = f"Dance Info: {total_movements} movements, {total_steps} steps, {total_duration:.1f}s duration"
        if hasattr(self, 'dance_info_label') and self.dance_info_label:
            self.dance_info_label.setText(info_text)
            self.dance_info_label.setStyleSheet("color: #2196F3; font-weight: bold;")

    def safe_update_dance_status(self, text, style=None):
        """Safely update dance status label if it exists"""
        if hasattr(self, 'dance_status_label') and self.dance_status_label:
            self.dance_status_label.setText(text)
            if style:
                self.dance_status_label.setStyleSheet(style)

    def safe_update_current_step(self, text, style=None):
        """Safely update current step label if it exists"""
        if hasattr(self, 'current_step_label') and self.current_step_label:
            self.current_step_label.setText(text)
            if style:
                self.current_step_label.setStyleSheet(style)

    def start_butterfly_dance(self):
        """Start the butterfly dance with music"""
        try:
            # Stop ML processor to prevent conflicts
            self.stop_ml_processor_requested.emit()
            print("Dance control enabled. ML Processor stop requested.")
            
            # Auto-adapt dance duration if enabled
            if self.auto_adapt_checkbox.isChecked():
                target_duration = self.music_duration_spinbox.value()
                self.set_dance_duration(target_duration)
                self.update_dance_info_display()
                print(f"Auto-adapted dance to {target_duration} seconds")
            
            # Store the expected music duration
            self.music_duration_seconds = self.music_duration_spinbox.value()
            
            # Reset dance sequence
            self.dance_step_index = 0
            self.dance_start_time = time.time()
            
            # Start music if loaded
            if self.music_file_path and os.path.exists(self.music_file_path):
                self.play_music()
            
            # Start dance
            self.is_dancing = True
            self.dance_timer.start(self.dance_step_interval)
            
            # Start music monitoring timer (will check if music is still playing)
            self.music_timer.start(self.music_check_interval)
            
            # Update GUI
            self.start_dance_btn.setEnabled(False)
            self.stop_dance_btn.setEnabled(True)
            
            total_steps = sum(step[1] for step in self.current_dance_sequence)
            self.safe_update_dance_status(f"🎭 Dancing - Butterfly Dance! ({total_steps} steps, {self.music_duration_seconds}s music)", "color: #FF9800; font-weight: bold; font-size: 14px;")
            
            print(f"🦋 Butterfly dance started! 🦋")
            print(f"Total steps: {total_steps}")
            print(f"Expected music duration: {self.music_duration_seconds} seconds")
            
            # Notify other systems
            self.advanced_dance_started.emit()
            
        except Exception as e:
            print(f"Error starting butterfly dance: {e}")
            QMessageBox.warning(self, "Dance Error", f"Could not start dance: {e}")

    def stop_dance(self):
        """Stop the dance mode and music"""
        self.is_dancing = False
        self.dance_timer.stop()
        self.dance_step_index = 0
        
        # Stop music monitoring timer
        if self.music_timer.isActive():
            self.music_timer.stop()
            print("Stopped music monitoring timer")
        
        # Stop music
        self.stop_music()
        
        # Reset to neutral position
        self.target_pressures = [self.mapping_params['base_pressure']] * 3
        self.pressure_command_sent.emit(self.target_pressures)
        
        # Update GUI
        self.start_dance_btn.setEnabled(True)
        self.stop_dance_btn.setEnabled(False)
        self.safe_update_dance_status("Dance Status: Stopped", "color: #f44336; font-weight: bold; font-size: 14px;")
        self.safe_update_current_step("Current Step: None")
        
        print("Dance stopped")

    def _execute_dance_step(self):
        """Execute the next dance step"""
        if not self.is_dancing or not self.current_dance_sequence:
            self.stop_dance()
            return
            
        # Get current step (loop if we've reached the end but music is still playing)
        if self.dance_step_index >= len(self.current_dance_sequence):
            # Check if music is still playing or if we haven't reached the expected duration
            elapsed_time = time.time() - self.dance_start_time
            music_still_playing = pygame.mixer.music.get_busy()
            
            if music_still_playing or elapsed_time < self.music_duration_seconds:
                # Loop the dance sequence - restart from beginning
                self.dance_step_index = 0
                print(f"Dance sequence completed, but music still playing. Looping... (elapsed: {elapsed_time:.1f}s)")
            else:
                # Music finished and we've completed at least one full sequence
                print("Dance sequence completed and music finished!")
                self.stop_dance()
                return
            
        current_step = self.current_dance_sequence[self.dance_step_index]
        pressures, duration, description = current_step
        
        # Set target pressures for this step
        self.target_pressures = pressures.copy()
        
        # Send pressure command
        self.pressure_command_sent.emit(self.target_pressures)
        
        # Update GUI displays
        elapsed_time = time.time() - self.dance_start_time
        current_loop = (self.dance_step_index // len(self.current_dance_sequence)) + 1 if len(self.current_dance_sequence) > 0 else 1
        step_in_sequence = (self.dance_step_index % len(self.current_dance_sequence)) + 1
        
        self.safe_update_current_step(f"Current Step: {description}")
        self.safe_update_dance_status(f"🎭 Dancing - Loop {current_loop}, Step {step_in_sequence}/{len(self.current_dance_sequence)} ({elapsed_time:.1f}s)")
        
        # Advance to next step
        self.dance_step_index += 1
        
        print(f"Dance step {step_in_sequence} (Loop {current_loop}): {description} - Pressures: {pressures}")

    def _check_music_finished(self):
        """Check if music has finished playing and stop dance accordingly"""
        if not self.is_dancing:
            self.music_timer.stop()
            return
            
        elapsed_time = time.time() - self.dance_start_time
        music_still_playing = pygame.mixer.music.get_busy()
        
        # Stop dance if music finished AND we've reached the expected duration
        if not music_still_playing and elapsed_time >= self.music_duration_seconds:
            print(f"Music finished after {elapsed_time:.1f}s. Stopping dance.")
            self.stop_dance()
        elif not music_still_playing:
            print(f"Music stopped but duration not reached ({elapsed_time:.1f}s < {self.music_duration_seconds}s). Continuing dance.")
        # Continue dancing if music is still playing or duration not reached

    def _create_butterfly_dance_sequence(self):
        """Create a choreographed dance sequence for 'I'm your little butterfly'"""
        # Dance sequence: each step is [pressures, duration_in_steps, description]
        # Pressures: [actuator1, actuator2, actuator3] in kPa
        
        self.butterfly_dance = [
            # Intro - gentle swaying (0-4 seconds)
            [[6.5, 6.5, 6.5], 4, "Center position"],
            [[7.0, 6.0, 6.5], 3, "Lean right gently"],
            [[6.0, 7.0, 6.5], 3, "Lean left gently"],
            [[6.5, 6.5, 7.0], 3, "Rise up slightly"],
            
            # "I'm your little" - rhythmic bouncing (4-8 seconds)
            [[8.0, 8.0, 8.0], 2, "Big bounce up"],
            [[5.0, 5.0, 5.0], 2, "Drop down"],
            [[8.0, 8.0, 8.0], 2, "Bounce up"],
            [[5.0, 5.0, 5.0], 2, "Drop down"],
            [[8.5, 8.5, 8.5], 2, "Higher bounce"],
            [[4.5, 4.5, 4.5], 2, "Lower drop"],
            
            # "Butterfly" - wing-like movements (8-12 seconds)
            [[9.0, 5.0, 7.0], 3, "Right wing up"],
            [[5.0, 9.0, 7.0], 3, "Left wing up"],
            [[9.0, 5.0, 7.0], 3, "Right wing up"],
            [[5.0, 9.0, 7.0], 3, "Left wing up"],
            [[7.0, 7.0, 9.0], 4, "Both wings up high"],
            
            # "Green, black and" - spinning motion (12-16 seconds)
            [[8.0, 6.0, 6.0], 2, "Tilt forward-right"],
            [[6.0, 8.0, 6.0], 2, "Tilt back-left"],
            [[6.0, 6.0, 8.0], 2, "Tilt up"],
            [[8.0, 6.0, 6.0], 2, "Tilt forward-right"],
            [[6.0, 8.0, 6.0], 2, "Tilt back-left"],
            [[6.0, 6.0, 8.0], 2, "Tilt up"],
            [[7.0, 7.0, 7.0], 4, "Center and rise"],
            
            # "Blue butterfly" - graceful movements (16-20 seconds)
            [[9.5, 6.5, 8.0], 4, "Elegant lean right"],
            [[6.5, 9.5, 8.0], 4, "Elegant lean left"],
            [[8.0, 8.0, 9.5], 4, "Graceful rise"],
            [[7.0, 7.0, 6.0], 4, "Gentle settle"],
            
            # "Stay with me" - swaying and pulsing (20-24 seconds)
            [[8.5, 7.5, 7.0], 3, "Sway right"],
            [[7.5, 8.5, 7.0], 3, "Sway left"],
            [[7.0, 7.0, 8.5], 3, "Pulse up"],
            [[7.0, 7.0, 6.5], 3, "Pulse down"],
            [[8.5, 7.5, 7.0], 3, "Sway right"],
            [[7.5, 8.5, 7.0], 3, "Sway left"],
            
            # "Don't say goodbye" - emotional movements (24-28 seconds)
            [[9.0, 9.0, 5.0], 4, "Reach up desperately"],
            [[5.0, 5.0, 9.0], 4, "Sudden rise (don't go)"],
            [[8.0, 6.0, 7.0], 3, "Lean in pleadingly"],
            [[6.0, 8.0, 7.0], 3, "Lean other way"],
            [[7.0, 7.0, 8.5], 4, "Final hopeful rise"],
            
            # Outro - gentle settling (28-32 seconds)
            [[7.5, 7.5, 7.5], 4, "Gentle position"],
            [[7.0, 7.0, 7.0], 4, "Settling down"],
            [[6.5, 6.5, 6.5], 6, "Return to center"],
            [[6.0, 6.0, 6.0], 8, "Final rest position"],
        ]
        
        # Calculate current dance duration
        total_steps = sum(step[1] for step in self.butterfly_dance)
        total_duration = total_steps * (self.dance_step_interval / 1000.0)
        
        self.current_dance_sequence = self.butterfly_dance.copy()
        print(f"Butterfly dance sequence created:")
        print(f"  - {len(self.butterfly_dance)} movements")
        print(f"  - {total_steps} total steps")
        print(f"  - {total_duration:.1f} seconds duration at {self.dance_step_interval}ms per step")

    def get_music_duration(self):
        """Get the duration of the loaded music file in seconds"""
        if not self.music_file_path or not os.path.exists(self.music_file_path):
            return None
            
        try:
            # Try to get music duration using pygame
            pygame.mixer.music.load(self.music_file_path)
            # Unfortunately, pygame doesn't provide duration directly
            # We'll estimate or ask user to input
            return None
        except:
            return None

    def create_adaptive_dance_sequence(self, target_duration_seconds):
        """Create a dance sequence that matches the target duration"""
        if target_duration_seconds <= 0:
            return self.butterfly_dance.copy()
            
        # Calculate how many steps we need for the target duration
        target_steps = int(target_duration_seconds * 1000 / self.dance_step_interval)
        
        # Get the original sequence
        original_sequence = self.butterfly_dance.copy()
        original_steps = sum(step[1] for step in original_sequence)
        
        # Calculate scaling factor
        scale_factor = target_steps / original_steps
        
        print(f"Adapting dance for {target_duration_seconds}s:")
        print(f"  - Target steps: {target_steps}")
        print(f"  - Original steps: {original_steps}")
        print(f"  - Scale factor: {scale_factor:.2f}")
        
        # Create adapted sequence
        adapted_sequence = []
        
        if scale_factor >= 1.0:
            # Music is longer - extend movements
            for movement in original_sequence:
                pressures, duration, description = movement
                new_duration = max(1, int(duration * scale_factor))
                adapted_sequence.append([pressures.copy(), new_duration, description])
        else:
            # Music is shorter - compress movements
            for movement in original_sequence:
                pressures, duration, description = movement
                new_duration = max(1, int(duration * scale_factor))
                adapted_sequence.append([pressures.copy(), new_duration, description])
        
        # Verify total duration
        actual_steps = sum(step[1] for step in adapted_sequence)
        actual_duration = actual_steps * (self.dance_step_interval / 1000.0)
        
        print(f"  - Actual result: {actual_steps} steps, {actual_duration:.1f}s")
        
        return adapted_sequence

    def set_dance_duration(self, duration_seconds):
        """Set the dance to match a specific duration"""
        self.current_dance_sequence = self.create_adaptive_dance_sequence(duration_seconds)
        print(f"Dance sequence adapted for {duration_seconds} seconds")

    def closeEvent(self, event):
        # Stop dance mode if active
        if self.is_dancing:
            self.stop_dance()
        
        # Stop music monitoring timer
        if self.music_timer.isActive():
            self.music_timer.stop()
        
        # Stop music
        try:
            pygame.mixer.music.stop()
        except:
            pass
        
        pygame.mixer.quit()
        super().closeEvent(event)

    def load_esp32_music_file(self):
        """Load a music file for ESP32 dance mode"""
        from PyQt5.QtWidgets import QFileDialog
        
        file_path, _ = QFileDialog.getOpenFileName(
            self, 
            "Load Music File", 
            "", 
            "Audio Files (*.mp3 *.wav *.ogg);;All Files (*)"
        )
        
        if file_path:
            self.esp32_music_file = file_path
            filename = os.path.basename(file_path)
            self.esp32_music_file_label.setText(f"Loaded: {filename}")
            self.esp32_music_file_label.setStyleSheet("color: #4CAF50; font-weight: bold;")
            print(f"ESP32 music file loaded: {filename}")
        else:
            print("No ESP32 music file selected")

    def toggle_esp32_passive_mode(self):
        """Toggle ESP32 passive mode"""
        self.esp32_passive_mode_enabled = self.esp32_mode_passive.isChecked()
        status_text = "Enabled" if self.esp32_passive_mode_enabled else "Disabled"
        self.esp32_status_label.setText(f"ESP32 Status: Passive Mode {status_text}")
        style = "color: #2196F3; font-weight: bold;" if self.esp32_passive_mode_enabled else "color: #666; font-weight: bold;"
        self.esp32_status_label.setStyleSheet(style)

    def play_esp32_music(self):
        """Play the loaded ESP32 music file"""
        if not self.esp32_music_file:
            print("DanceControl: No ESP32 music file loaded")
            return
            
        try:
            pygame.mixer.music.load(self.esp32_music_file)
            pygame.mixer.music.set_volume(self.music_volume)
            pygame.mixer.music.play()
            
            print(f"DanceControl: Playing ESP32 music: {os.path.basename(self.esp32_music_file)}")
        except Exception as e:
            print(f"Error playing ESP32 music: {e}")
            from PyQt5.QtWidgets import QMessageBox
            QMessageBox.warning(self, "Music Error", f"Could not play music: {e}")

    def start_esp32_dance(self):
        """Start ESP32 dance mode"""
        if not self.esp32_music_file:
            QMessageBox.warning(self, "No Music", "Please load a music file first.")
            return
            
        print("DanceControl: Starting ESP32 dance mode")
        
        # Stop ML processor to prevent conflicts
        self.stop_ml_processor_requested.emit()
        
        # Send start_dance command to ESP32
        command = {"command": "start_dance"}
        self.esp32_command_sent.emit(command)
        print(f"DanceControl: Sent command to ESP32: {command}")
        
        # Update status
        self.esp32_dance_status_label.setText("ESP32 Dance Status: Starting...")
        self.esp32_dance_status_label.setStyleSheet("color: #FF9800; font-weight: bold;")
        
        # Update GUI
        self.esp32_dance_btn.setEnabled(False)
        self.esp32_stop_btn.setEnabled(True)
        self.esp32_dance_active = True
        
        print("DanceControl: ESP32 dance mode started")

    def stop_esp32_dance(self):
        """Stop ESP32 dance mode"""
        print("DanceControl: Stopping ESP32 dance mode")
        
        # Send stop_dance command to ESP32
        command = {"command": "stop_dance"}
        self.esp32_command_sent.emit(command)
        print(f"DanceControl: Sent command to ESP32: {command}")
        
        # Stop music
        self.stop_music()
        
        # Update GUI
        self.esp32_dance_btn.setEnabled(True)
        self.esp32_stop_btn.setEnabled(False)
        self.esp32_dance_status_label.setText("ESP32 Dance Status: Stopped")
        self.esp32_dance_status_label.setStyleSheet("color: #f44336; font-weight: bold;")
        self.esp32_dance_active = False
        
        print("DanceControl: ESP32 dance mode stopped")

    def handle_serial_data(self, data_dict):
        """Handle incoming serial data - look for PLAY SONG commands"""
        if not self.esp32_passive_mode_enabled and not getattr(self, 'esp32_dance_active', False):
            return
            
        # Check for ESP32 dance status messages
        if isinstance(data_dict, dict):
            # Handle dance finished status
            if data_dict.get('status') == 'dance_finished':
                self.handle_esp32_dance_finished()
                return
            elif data_dict.get('status') == 'dance_mode_stopped':
                self.handle_esp32_dance_stopped()
                return
            
            # Check all string values in the data dict for "PLAY SONG"
            for key, value in data_dict.items():
                if isinstance(value, str) and "PLAY SONG" in value:
                    self.trigger_esp32_music()
                    break
        elif isinstance(data_dict, str) and "PLAY SONG" in data_dict:
            self.trigger_esp32_music()

    def handle_esp32_dance_finished(self):
        """Handle ESP32 dance sequence finished automatically"""
        print("DanceControl: ESP32 dance sequence finished automatically")
        
        if hasattr(self, 'esp32_dance_active') and self.esp32_dance_active:
            # Update ESP32 dance status
            self.esp32_dance_status_label.setText("Dance Status: Finished - Ready for other modes")
            self.esp32_dance_status_label.setStyleSheet("color: #4CAF50; font-weight: bold;")
            
            # Reset ESP32 dance state
            self.esp32_dance_active = False
            self.esp32_dance_btn.setEnabled(True)
            self.esp32_stop_btn.setEnabled(False)
            
            # Stop music if still playing
            self.stop_music()
            
            print("DanceControl: ESP32 dance mode cleaned up - ready for joystick/other control")

    def handle_esp32_dance_stopped(self):
        """Handle ESP32 dance stopped manually"""
        print("DanceControl: ESP32 dance stopped manually")
        
        if hasattr(self, 'esp32_dance_active') and self.esp32_dance_active:
            # Update ESP32 dance status
            self.esp32_dance_status_label.setText("Dance Status: Stopped - Ready for other modes")
            self.esp32_dance_status_label.setStyleSheet("color: #666; font-weight: bold;")
            
            # Reset ESP32 dance state
            self.esp32_dance_active = False
            self.esp32_dance_btn.setEnabled(True)
            self.esp32_stop_btn.setEnabled(False)
            
            # Stop music if still playing
            self.stop_music()
            
            print("DanceControl: ESP32 dance mode cleaned up - ready for joystick/other control")

    def handle_serial_raw_message(self, message):
        """Handle raw serial messages - specifically for PLAY SONG command"""
        if not self.esp32_passive_mode_enabled and not getattr(self, 'esp32_dance_active', False):
            return
            
        if isinstance(message, str) and "PLAY SONG" in message.strip():
            print(f"DanceControl: Received PLAY SONG command: {message.strip()}")
            self.trigger_esp32_music()

    def trigger_esp32_music(self):
        """Trigger ESP32 music playback"""
        if not self.esp32_music_file:
            print("DanceControl: No ESP32 music file loaded")
            return
            
        print("DanceControl: Triggering ESP32 music playback")
        
        # Update status
        self.safe_update_dance_status("Dance Status: ESP32 music triggered!", "color: #FF9800; font-weight: bold; font-size: 14px;")
        self.safe_update_current_step("Current Step: Waiting for music sync...")
        
        # Stop any existing music/dance
        self.stop_music()
        self.stop_dance()
        
        # Start music with delay
        delay_ms = int(self.esp32_music_delay_spinbox.value() * 1000)
        self.esp32_music_timer.start(delay_ms)
        
        # Signal to stop ML processor if running
        self.stop_ml_processor_requested.emit()
        
        print(f"DanceControl: Will start music in {delay_ms}ms")

# Integration example for main_window.py:
# 
# from gui.dance_control_widget import DanceControlWidget
#
# # In your main window __init__ or init_ui:
# self.dance_widget = DanceControlWidget()
# self.tabs.addTab(self.dance_widget, "🎭 Dance Control")
#
# # Connect signals:
# self.dance_widget.pressure_command_sent.connect(self.handle_dance_pressure_command)
# self.dance_widget.stop_ml_processor_requested.connect(self.stop_ml_processor_for_dance) 