import numpy as np
import sounddevice as sd
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLabel, QPushButton, QGroupBox
from PyQt5.QtCore import pyqtSignal, QThread, QObject, Qt
import re
from PyQt5.QtWidgets import QApplication
import time # For sleep in sequence worker
import random # For random mode

# Worker class for transcription
class TranscriptionWorker(QObject):
    transcription_done = pyqtSignal(str)
    transcription_error = pyqtSignal(str)

    def __init__(self, whisper_model, audio_array, sample_rate):
        super().__init__()
        self.whisper_model = whisper_model
        self.audio_array = audio_array
        self.sample_rate = sample_rate

    def run(self):
        try:
            if self.whisper_model is None:
                self.transcription_error.emit("Whisper model not loaded.")
                return
            if self.audio_array is None or len(self.audio_array) == 0:
                self.transcription_error.emit("No audio data to process.")
                return
            print(f"Transcription worker: Starting transcription with '{VoiceControlWidget.current_model_name_for_worker}' model...") # Accessing via class member
            audio_for_whisper = self.audio_array.astype(np.float32)
            max_abs_val = np.max(np.abs(audio_for_whisper))
            if max_abs_val > 1.0:
                audio_for_whisper /= max_abs_val
            initial_prompt_text = (
                "Set actuator one pressure to two kpa. "
                "Actuator two pressure to three kilopascals. "
                "Actuator three pressure to four. "
                "Actuator one needs five kpa. "
                "Put six kpa on actuator two. "
                "Deflate all actuators. Empty all. Zero all. "
                "Inflate all actuators to seven kpa. Fill all to 5. Set all actuators to three. "
                "Emergency stop. Stop all. E stop. "
                "Dance for me. Start dancing. " # New fun prompts
                "Random mode. Activate random. "  # New fun prompts
                "Stop dancing. Stop random mode. Stop movement." # New stop prompts
            )
            result = self.whisper_model.transcribe(audio_for_whisper, initial_prompt=initial_prompt_text)
            text = result["text"].strip().lower()
            print(f"Transcription worker: Recognized text: {text}")
            self.transcription_done.emit(text)
        except Exception as e:
            error_msg = f"Transcription error: {str(e)}"
            print(f"Transcription worker: {error_msg}")
            import traceback; traceback.print_exc()
            self.transcription_error.emit(error_msg)

# Worker class for movement sequences (dance, random)
class MovementSequenceWorker(QObject):
    new_pressures_to_apply = pyqtSignal(list)
    sequence_finished = pyqtSignal()

    def __init__(self, mode, voice_control_widget_ref):
        super().__init__()
        self.mode = mode
        self.voice_control_widget = voice_control_widget_ref # Reference to access flags
        self.running = True

        # Define dance sequences (list of [p1, p2, p3, duration_ms])
        self.dance_sequence_1 = [
            ([8.0, 2.0, 2.0], 500),
            ([2.0, 8.0, 2.0], 500),
            ([2.0, 2.0, 8.0], 500),
            ([5.0, 5.0, 5.0], 700),
            ([2.0, 8.0, 8.0], 500),
            ([8.0, 2.0, 8.0], 500),
            ([8.0, 8.0, 2.0], 500),
            ([3.0, 3.0, 3.0], 700),
        ]
        # Simple pulse
        self.dance_sequence_pulse = [
            ([10.0, 10.0, 10.0], 600),
            ([2.0, 2.0, 2.0], 600),
            ([10.0, 10.0, 10.0], 600),
            ([2.0, 2.0, 2.0], 600),
        ]

    def run(self):
        try:
            if self.mode == "dance":
                print("MovementSequenceWorker: Starting dance sequence.")
                sequence_to_run = self.dance_sequence_pulse # Or self.dance_sequence_1
                # Loop through the dance sequence a few times or until stopped
                for _ in range(3): # Repeat the sequence 3 times
                    if not self.voice_control_widget.is_dancing or not self.running:
                        break
                    for pressures, duration_ms in sequence_to_run:
                        if not self.voice_control_widget.is_dancing or not self.running:
                            break
                        # Ensure pressures are within bounds (should be by design, but good practice)
                        clamped_pressures = [round(max(0.0, min(13.0, p)), 1) for p in pressures]
                        self.new_pressures_to_apply.emit(clamped_pressures)
                        QThread.msleep(duration_ms)
                    if not self.voice_control_widget.is_dancing or not self.running:
                        break
                print("MovementSequenceWorker: Dance sequence completed iterations or was stopped.")

            elif self.mode == "random":
                print("MovementSequenceWorker: Starting random mode.")
                while self.voice_control_widget.is_in_random_mode and self.running:
                    pressures = [round(random.uniform(1.0, 10.0), 1) for _ in range(3)]
                    self.new_pressures_to_apply.emit(pressures)
                    random_delay_ms = random.randint(500, 1500) # Random delay between 0.5 and 1.5 seconds
                    QThread.msleep(random_delay_ms)
                print("MovementSequenceWorker: Random mode stopped.")
            
        except Exception as e:
            print(f"Error in MovementSequenceWorker: {e}")
            import traceback; traceback.print_exc()
        finally:
            self.running = False
            self.voice_control_widget.is_dancing = False # Ensure flags are reset
            self.voice_control_widget.is_in_random_mode = False
            self.sequence_finished.emit()
            print("MovementSequenceWorker: Emitted sequence_finished.")

    def stop(self):
        print("MovementSequenceWorker: stop() called.")
        self.running = False

class VoiceControlWidget(QWidget):
    command_sent = pyqtSignal(dict)
    pressure_updated = pyqtSignal(int, float)
    all_pressures_updated = pyqtSignal(list) # New signal for when all actuators are set
    voice_dance_started = pyqtSignal()  # New signal for when voice dancing starts
    # TODO: Add a signal to request stopping ML processor if MainWindow doesn't already handle it via all_pressures_updated

    current_model_name_for_worker = "small" # Class attribute for worker to access model name for print
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.recording = False
        self.audio_data = []
        self.sample_rate = 16000
        self.stream = None
        self.whisper_model = None
        self.transcription_thread = None
        self.transcription_worker = None
        self.current_model_name = "small"
        VoiceControlWidget.current_model_name_for_worker = self.current_model_name # Update class attribute
        
        # State for special movement modes
        self.is_dancing = False
        self.is_in_random_mode = False
        self.movement_sequence_thread = None
        self.movement_sequence_worker = None
        
        # Track current actuator pressures to maintain state for individual commands
        self.current_pressures = [0.0, 0.0, 0.0]
        
        self.init_ui()
        self.load_whisper_model_once()

    def init_ui(self):
        layout = QVBoxLayout(self)
        layout.setSpacing(15)
        layout.setContentsMargins(15, 15, 15, 15)
        
        voice_group = QGroupBox("Voice Control")
        voice_layout = QVBoxLayout(voice_group)
        voice_layout.setSpacing(15)
        
        self.status_label = QLabel("Loading voice recognition...")
        self.status_label.setStyleSheet("""
            font-size: 14pt;
            color: #64b5f6;
            font-weight: bold;
        """)
        voice_layout.addWidget(self.status_label)
        
        self.voice_btn = QPushButton("Start Voice Control")
        self.voice_btn.setEnabled(False)
        self.voice_btn.setMinimumHeight(60)
        self.voice_btn.setStyleSheet("""
            QPushButton {
                background-color: #0d47a1;
                color: white;
                border: none;
                padding: 15px;
                border-radius: 8px;
                font-size: 16pt;
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
        """)
        self.voice_btn.clicked.connect(self.toggle_recording)
        voice_layout.addWidget(self.voice_btn)
        
        self.command_history = QLabel("Last Command: None")
        self.command_history.setStyleSheet("""
            font-size: 12pt;
            color: #90caf9;
            padding: 10px;
            background: #1a1a1a;
            border-radius: 5px;
        """)
        voice_layout.addWidget(self.command_history)
        
        help_text = """
        <div style='background: #1a1a1a; padding: 15px; border-radius: 8px; border: 1px solid #444;'>
            <h3 style='color: #64b5f6; margin-bottom: 10px;'>Voice Command Examples:</h3>
            <ul style='color: #90caf9;'>
                <li><b>Set Single Actuator Pressure (Actuator 1-3, Value 0-13 kPa):</b>
                    <ul>
                        <li>"Set actuator one pressure to 5" (maintains other actuators)</li>
                        <li>"Actuator 2 pressure 7.5 kpa" (maintains other actuators)</li>
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
                        <li>"Dance for me" / "Start dancing"</li>
                        <li>"Random mode" / "Activate random"</li>
                        <li>"Stop dancing" / "Stop random mode" / "Stop movement"</li>
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
        self.help_label = QLabel(help_text)
        self.help_label.setStyleSheet("font-size: 12pt;")
        voice_layout.addWidget(self.help_label)
        
        layout.addWidget(voice_group)

    def load_whisper_model_once(self):
        VoiceControlWidget.current_model_name_for_worker = self.current_model_name # Ensure worker has latest if changed
        if self.whisper_model is None:
            try:
                import whisper
                print(f"\nInitializing Whisper model '{self.current_model_name}' (this may take a moment)...")
                self.status_label.setText(f"Loading '{self.current_model_name}' model...")
                self.voice_btn.setEnabled(False) # Disable button while loading
                QApplication.processEvents() # Process GUI events to show label update

                self.whisper_model = whisper.load_model(self.current_model_name)
                
                print(f"Whisper '{self.current_model_name}' model loaded successfully.")
                self.status_label.setText(f"Voice Control Ready ({self.current_model_name.capitalize()} Model)")
                self.voice_btn.setEnabled(True)
            except Exception as e:
                error_msg = f"Error loading Whisper model '{self.current_model_name}': {str(e)}"
                print(f"Error: {error_msg}")
                self.status_label.setText(f"Model Load Error: {error_msg}")
                self.voice_btn.setEnabled(False)
        else:
            if not (self.transcription_thread and self.transcription_thread.isRunning()):
                 self.voice_btn.setEnabled(True)
            self.status_label.setText(f"Voice Control Ready ({self.current_model_name.capitalize()} Model)")

    def toggle_recording(self):
        if not self.whisper_model:
            self.status_label.setText(f"Whisper '{self.current_model_name}' model not loaded. Retrying load...")
            self.load_whisper_model_once()
            return

        if self.transcription_thread and self.transcription_thread.isRunning():
            self.status_label.setText("Still processing previous command...")
            print("Transcription thread is busy.")
            return

        if not self.recording:
            self.start_recording()
        else:
            self.stop_recording_and_process()

    def start_recording(self):
        try:
            print("\nStarting audio recording...")
            self.audio_data = []
            self.recording = True
            self.voice_btn.setText("Stop Recording")
            self.voice_btn.setStyleSheet("""
                QPushButton {background-color: #f44336; color: white; border: none; padding: 10px; 
                           border-radius: 5px; font-size: 14pt; min-height: 50px;} 
            """) # Simplified for brevity
            self.status_label.setText("Recording... (Speak now)")
            device_info = sd.query_devices(kind='input')
            print(f"Using input device: {device_info['name']}")
            self.stream = sd.InputStream(
                channels=1, samplerate=self.sample_rate, callback=self.audio_callback
            )
            self.stream.start()
            print("Audio stream started")
        except Exception as e:
            error_msg = f"Recording error: {str(e)}"
            print(f"Error: {error_msg}")
            self.status_label.setText(f"Recording Error: {error_msg}")
            self.recording = False
            self.voice_btn.setText("Start Voice Control")
            self.voice_btn.setEnabled(True) # Re-enable on error
            self.voice_btn.setStyleSheet(""" 
                QPushButton {background-color: #0d47a1; color: white; border: none; padding: 15px; 
                           border-radius: 8px; font-size: 16pt; font-weight: bold;} 
            """) # Reset style

    def audio_callback(self, indata, frames, time, status):
        if status:
            print(f'Status: {status}')
        if self.recording:
            self.audio_data.extend(indata[:, 0])

    def stop_recording_and_process(self):
        if not self.recording:
            return
        print("\nStopping recording...")
        self.recording = False
        self.voice_btn.setText("Processing...") # Indicate processing
        self.voice_btn.setEnabled(False) # Disable button during processing

        try:
            if self.stream:
                self.stream.stop()
                self.stream.close()
                self.stream = None
            if not self.audio_data:
                self.status_label.setText("No audio recorded.")
                print("No audio data captured.")
                self.voice_btn.setText("Start Voice Control")
                if self.whisper_model: self.voice_btn.setEnabled(True)
                return

            current_audio_array = np.array(self.audio_data)
            self.audio_data = []
            max_amplitude = np.max(np.abs(current_audio_array))
            print(f"Max audio amplitude: {max_amplitude}")
            if max_amplitude < 0.01:
                self.status_label.setText("Audio too quiet or silent.")
                print("Audio too quiet.")
                self.voice_btn.setText("Start Voice Control")
                if self.whisper_model: self.voice_btn.setEnabled(True)
                return

            self.status_label.setText(f"Processing (using '{self.current_model_name}' model)...")
            self.transcription_thread = QThread()
            self.transcription_worker = TranscriptionWorker(self.whisper_model, current_audio_array, self.sample_rate)
            self.transcription_worker.moveToThread(self.transcription_thread)
            self.transcription_thread.started.connect(self.transcription_worker.run)
            self.transcription_worker.transcription_done.connect(self.handle_transcription_result)
            self.transcription_worker.transcription_error.connect(self.handle_transcription_error)
            self.transcription_thread.finished.connect(self.on_transcription_thread_finished) # Connect to custom slot
            self.transcription_worker.transcription_done.connect(self.transcription_thread.quit)
            self.transcription_worker.transcription_error.connect(self.transcription_thread.quit)
            self.transcription_thread.finished.connect(self.transcription_worker.deleteLater) # Worker deleted after thread quit
            self.transcription_thread.finished.connect(self.transcription_thread.deleteLater) # Thread deletes itself

            self.transcription_thread.start()
            print(f"Transcription thread started for '{self.current_model_name}' model.")
        except Exception as e:
            error_msg = f"Error in stop_recording_and_process: {str(e)}"
            print(f"Error: {error_msg}")
            import traceback
            traceback.print_exc()
            self.status_label.setText(f"Error: {error_msg}")
            self.voice_btn.setText("Start Voice Control")
            if self.whisper_model: self.voice_btn.setEnabled(True)
    
    def on_transcription_thread_finished(self):
        print("Transcription thread finished signal received.")
        if self.transcription_thread: # Check if the object still exists
            self.transcription_thread.deleteLater() # Schedule for deletion
        self.transcription_thread = None
        self.transcription_worker = None # Worker is child of thread or deleted via its own signal
        
        # This is a critical point to re-enable UI if not already handled by result/error
        if not self.recording: # Ensure we are not in a recording state
            self.voice_btn.setText("Start Voice Control")
            if self.whisper_model:
                 self.voice_btn.setEnabled(True)
            # Set a general ready status if no specific error is displayed
            current_status = self.status_label.text()
            if not any(err_keyword in current_status for err_keyword in ["Error", "Failed", "Processing"]):
                self.status_label.setText(f"Voice Control Ready ({self.current_model_name.capitalize()} Model)")

    def handle_transcription_result(self, text):
        print(f"Main thread: Received transcription: {text}")
        if text:
            self.process_command(text)
        else:
            self.status_label.setText("No speech detected.")
            self.command_history.setText("Last Command: No speech detected")
        # Button re-enabling will be handled by on_transcription_thread_finished or if another error occurs

    def handle_transcription_error(self, error_msg):
        print(f"Main thread: Received transcription error: {error_msg}")
        self.status_label.setText(f"Transcription Failed: {error_msg}")
        self.command_history.setText("Last Command: Transcription error")
        # Button re-enabling will be handled by on_transcription_thread_finished

    def process_command(self, text):
        print(f"Processing command: '{text}'")
        actuator_num = None
        # Expanded num_map for pressures up to 13, and zero
        num_map = {
            "zero": 0, "one": 1, "two": 2, "three": 3, "four": 4, "five": 5, 
            "six": 6, "seven": 7, "eight": 8, "nine": 9, "ten": 10, 
            "eleven": 11, "twelve": 12, "thirteen": 13
        }
        pressure_value_float = None
        processed_command = False
        
        # Regex for pressure part now includes text numbers via a placeholder, will refine in loop
        # General pattern structure, specific pressure part will be dynamic or handled in post-processing
        pressure_num_pattern = r"\d+(?:\.\d+)?|zero|one|two|three|four|five|six|seven|eight|nine|ten|eleven|twelve|thirteen" # Ensure single backslash for \d

        # --- Stop any ongoing special movements if a specific pressure command is given ---
        # This is a pre-check before processing specific commands.
        # We will also explicitly stop them if another command type is matched.
        # For now, we rely on individual command blocks to set is_dancing/is_in_random_mode to False.

        # --- Fun Mode Commands ---
        fun_mode_patterns = {
            r"(?:start\s+)?dance(?:\s+for\s+me)?": "start_dance",
            r"(?:activate\s+|start\s+)?random(?:\s+mode)?": "start_random_mode",
            r"stop\s+(?:dancing|dance|random(?:\s+mode)?|movement)": "stop_fun_modes"
        }

        for pattern, cmd_type in fun_mode_patterns.items():
            match = re.search(pattern, text, re.IGNORECASE)
            if match:
                if cmd_type == "start_dance":
                    print("Voice: Dance mode requested.")
                    self.stop_any_movement_sequence() 
                    self.is_dancing = True # Set flag before starting thread
                    self.is_in_random_mode = False
                    self.start_movement_sequence_thread("dance") 
                    self.command_history.setText("Starting dance sequence...")
                    self.status_label.setText("Dancing!")
                    self.voice_dance_started.emit()  # Notify other systems
                    processed_command = True; break
                elif cmd_type == "start_random_mode":
                    print("Voice: Random mode requested.")
                    self.stop_any_movement_sequence()
                    self.is_in_random_mode = True # Set flag before starting thread
                    self.is_dancing = False
                    self.start_movement_sequence_thread("random") 
                    self.command_history.setText("Starting random mode...")
                    self.status_label.setText("Random movements activated!")
                    processed_command = True; break
                elif cmd_type == "stop_fun_modes":
                    print("Voice: Stop fun modes requested.")
                    if self.is_dancing or self.is_in_random_mode:
                        self.stop_any_movement_sequence()
                        self.command_history.setText("Special movement stopped. Deflating all actuators.")
                        self.status_label.setText("Movement stopped. Deflating...")
                        # Deflate all actuators after stopping fun mode
                        deflate_pressures = [0.0, 0.0, 0.0]
                        
                        # Update our internal state
                        self.current_pressures = deflate_pressures.copy()
                        
                        command_dict = {"command": "set_pressure", "values": deflate_pressures}
                        self.command_sent.emit(command_dict)
                        self.all_pressures_updated.emit(deflate_pressures)
                        print("Deflating all actuators after stopping fun mode.")
                    else:
                        self.command_history.setText("No special movement was active.")
                    processed_command = True; break
            if processed_command: break
        if processed_command: return

        # Patterns for global commands first (ensure these stop fun modes too)
        global_command_patterns = {
            r"(deflate|empty|zero)\s+all(?:\s+actuators)?": "deflate_all",
            r"(inflate|fill|pressurize)\s+all(?:\s+actuators)?(?:\s+to\s+(?P<pressure>"+pressure_num_pattern+r"))?\s*(?:kpa|kilopascals|k\s*p\s*a)?" : "inflate_all",
            r"all\s+actuators\s+(?:pressure(?:\s+to)?\s+)?(?P<pressure>"+pressure_num_pattern+r")\s*(?:kpa|kilopascals|k\s*p\s*a)?": "set_all_to_value",
            r"set\s+all\s+actuators\s+(?:to\s+|pressure\s+to\s+)?(?P<pressure>"+pressure_num_pattern+r")\s*(?:kpa|kilopascals|k\s*p\s*a)?": "set_all_to_value"
        }

        for pattern, cmd_type in global_command_patterns.items():
            match = re.search(pattern, text, re.IGNORECASE)
            if match:
                # Stop any fun mode if a global pressure command is given
                if self.is_dancing or self.is_in_random_mode:
                    print("Stopping active fun mode due to global pressure command.")
                    self.stop_any_movement_sequence() # Ensure thread stops
                all_pressures = [0.0, 0.0, 0.0]
                target_pressure_all = 0.0
                try:
                    if cmd_type == "deflate_all":
                        target_pressure_all = 0.0
                        self.command_history.setText("Deflating all actuators to 0.0 kPa")
                    elif cmd_type == "inflate_all":
                        if match.group("pressure"):
                            pressure_str = match.group("pressure").lower()
                            if pressure_str in num_map: target_pressure_all = float(num_map[pressure_str])
                            elif re.match(r"^\d+(?:\.\d+)?$", pressure_str): target_pressure_all = float(pressure_str)
                            else: print(f"Invalid pressure string for inflate all: {pressure_str}"); continue
                        else: # No pressure specified, use default
                            target_pressure_all = 10.0 # Default for "inflate all" - CHANGED TO 10.0
                        target_pressure_all = round(max(0.0, min(13.0, target_pressure_all)), 1)
                        self.command_history.setText(f"Inflating all actuators to {target_pressure_all} kPa")
                    elif cmd_type == "set_all_to_value":
                        pressure_str = match.group("pressure").lower()
                        if pressure_str in num_map: target_pressure_all = float(num_map[pressure_str])
                        elif re.match(r"^\d+(?:\.\d+)?$", pressure_str): target_pressure_all = float(pressure_str)
                        else: print(f"Invalid pressure string for set all: {pressure_str}"); continue
                        target_pressure_all = round(max(0.0, min(13.0, target_pressure_all)), 1)
                        self.command_history.setText(f"Setting all actuators to {target_pressure_all} kPa")
                    
                    all_pressures = [target_pressure_all] * 3
                    
                    # Update our internal state
                    self.current_pressures = all_pressures.copy()
                    
                    command_dict = {"command": "set_pressure", "values": all_pressures}
                    self.command_sent.emit(command_dict)
                    self.all_pressures_updated.emit(all_pressures) # Emit new signal
                    self.status_label.setText(f"All actuators set to {target_pressure_all} kPa")
                    print(f"Global command processed: {cmd_type} to {target_pressure_all} kPa")
                    processed_command = True; break
                except Exception as e:
                    print(f"Error processing global command '{cmd_type}': {e}"); pass 
            if processed_command: break
        if processed_command: return

        # Patterns for individual actuator commands (ensure these stop fun modes too)
        set_pressure_patterns = [
            r"set\s+actuator\s+(?P<actuator>\d+|one|two|three)\s+pressure\s+(?:to\s+)?(?P<pressure>" + pressure_num_pattern + r")\s*(?:kpa|kilopascals|k\s*p\s*a)?",
            r"actuator\s+(?P<actuator>\d+|one|two|three)\s+pressure\s+(?:to\s+)?(?P<pressure>" + pressure_num_pattern + r")\s*(?:kpa|kilopascals|k\s*p\s*a)?",
            r"actuator\s+(?P<actuator>\d+|one|two|three)\s+(?:needs(?:\s+to\s+be)?|set\s+to|is(?:\s+set\s+to)?)\s+(?P<pressure>" + pressure_num_pattern + r")\s*(?:kpa|kilopascals|k\s*p\s*a)?",
            r"put\s+(?P<pressure>" + pressure_num_pattern + r")\s*(?:kpa|kilopascals|k\s*p\s*a)?\s+(?:on|in)\s+actuator\s+(?P<actuator>\d+|one|two|three)"
        ]
        for pattern in set_pressure_patterns: # This loop will only run if no global command was processed
            match = re.search(pattern, text, re.IGNORECASE)
            if match:
                # Stop any fun mode if an individual pressure command is given
                if self.is_dancing or self.is_in_random_mode:
                    print("Stopping active fun mode due to individual pressure command.")
                    self.stop_any_movement_sequence() # Ensure thread stops
                try:
                    actuator_str = match.group("actuator").lower()
                    pressure_str = match.group("pressure").lower()
                    if actuator_str in num_map and 1 <= num_map[actuator_str] <= 3: actuator_num = num_map[actuator_str] - 1
                    elif actuator_str.isdigit() and 1 <= int(actuator_str) <= 3: actuator_num = int(actuator_str) - 1
                    else: print(f"Invalid actuator string: {actuator_str}"); continue
                    if pressure_str in num_map: pressure_value_float = float(num_map[pressure_str])
                    elif re.match(r"^\d+(?:\.\d+)?$", pressure_str): pressure_value_float = float(pressure_str)
                    else: print(f"Invalid pressure string: {pressure_str}"); continue
                    pressure_value_float = round(max(0.0, min(13.0, pressure_value_float)), 1)
                    if 0 <= actuator_num <= 2:
                        # Preserve current pressures of other actuators, only modify the target actuator
                        current_values = self.current_pressures.copy()
                        current_values[actuator_num] = pressure_value_float
                        
                        # Update our internal state
                        self.current_pressures = current_values.copy()
                        
                        command = {"command": "set_pressure", "values": current_values}
                        self.command_sent.emit(command); self.pressure_updated.emit(actuator_num, pressure_value_float)
                        self.command_history.setText(f"Set Actuator {actuator_num + 1} to {pressure_value_float} kPa (others maintained)")
                        self.status_label.setText(f"Actuator {actuator_num + 1} set to {pressure_value_float} kPa")
                        print(f"Actuator command processed: Actuator {actuator_num + 1}, Pressure {pressure_value_float} kPa. Current state: {current_values}"); processed_command = True; break
                except Exception as e:
                    print(f"Error processing matched set_pressure command with pattern '{pattern}'. Text: '{text}'. Error: {e}"); pass
            if processed_command: break
        if processed_command: return

        if any(phrase in text for phrase in ["emergency stop", "stop all", "e stop"]):
            # Stop any fun mode on emergency stop
            if self.is_dancing or self.is_in_random_mode:
                print("Stopping active fun mode due to E-STOP.")
                self.stop_any_movement_sequence() # Ensure thread stops
            
            self.command_history.setText("Emergency Stop Activated. Deflating all actuators.")
            self.status_label.setText("E-STOP. Deflating...")
            print("Emergency stop command processed. Sending E-STOP and deflation to ESP32.")

            # Send the primary E-STOP command (ESP32 might have specific handling for this)
            estop_command = {"command": "emergency_stop"}
            self.command_sent.emit(estop_command)
            
            # Also explicitly command deflation
            deflate_pressures = [0.0, 0.0, 0.0]
            
            # Update our internal state
            self.current_pressures = deflate_pressures.copy()
            
            deflate_command_dict = {"command": "set_pressure", "values": deflate_pressures}
            self.command_sent.emit(deflate_command_dict)
            self.all_pressures_updated.emit(deflate_pressures) # Update GUI to show 0 pressure

            processed_command = True
        if processed_command: return

        self.command_history.setText(f"Unknown command: {text}"); self.status_label.setText(f"Unknown command: {text.capitalize()}"); print(f"Unknown command: '{text}'")

    def update_pressure_state(self, pressures):
        """Update the internal pressure state to stay synchronized with the main application."""
        if len(pressures) == 3:
            self.current_pressures = pressures.copy()
            print(f"Voice control pressure state updated: {self.current_pressures}")

    def closeEvent(self, event):
        print("VoiceControlWidget closeEvent called.")
        self.stop_any_movement_sequence() # Stop movement sequence on close
        if self.stream and self.stream.active:
            print("Stopping active audio stream.")
            try: # Add try-except for stream operations
                self.stream.stop()
                self.stream.close()
            except Exception as e:
                print(f"Error stopping/closing stream: {e}")
            self.stream = None # Ensure stream is cleared

        if self.transcription_thread and self.transcription_thread.isRunning():
            print("Waiting for transcription thread to finish...")
            self.transcription_thread.quit()
            if not self.transcription_thread.wait(3000): # If wait times out
                 print("Transcription thread did not quit gracefully, terminating.")
                 self.transcription_thread.terminate()
                 self.transcription_thread.wait() # Wait for termination to complete
        self.transcription_thread = None # Clear reference
        self.transcription_worker = None # Clear reference
        super().closeEvent(event) 

    # Add new methods for managing movement sequences
    def _handle_sequence_pressures(self, pressures):
        if not (self.is_dancing or self.is_in_random_mode):
            # If the mode was turned off externally while pressures were in flight
            print("Sequence mode off, ignoring pressure update from worker.")
            return
        
        # Update our internal state
        self.current_pressures = pressures.copy()
        
        command_dict = {"command": "set_pressure", "values": pressures}
        self.command_sent.emit(command_dict)
        self.all_pressures_updated.emit(pressures) # This updates GUI and stops ML processor via MainWindow

    def start_movement_sequence_thread(self, mode):
        self.stop_any_movement_sequence() # Stop previous one if any

        if mode == "dance":
            self.is_dancing = True
            self.is_in_random_mode = False
        elif mode == "random":
            self.is_in_random_mode = True
            self.is_dancing = False
        else:
            print(f"Unknown movement mode: {mode}")
            return

        self.movement_sequence_thread = QThread(self)
        # Pass self (VoiceControlWidget instance) to worker for flag checking
        self.movement_sequence_worker = MovementSequenceWorker(mode, self)
        self.movement_sequence_worker.moveToThread(self.movement_sequence_thread)

        self.movement_sequence_worker.new_pressures_to_apply.connect(self._handle_sequence_pressures)
        self.movement_sequence_thread.started.connect(self.movement_sequence_worker.run)
        # Use a QueuedConnection for sequence_finished to ensure stop_any_movement_sequence runs in the main thread
        self.movement_sequence_worker.sequence_finished.connect(self.stop_any_movement_sequence, Qt.QueuedConnection)
        # Also connect thread's finished signal for cleanup.
        self.movement_sequence_thread.finished.connect(self._on_movement_sequence_finished)

        print(f"Starting {mode} sequence thread...")
        self.movement_sequence_thread.start()

    def stop_any_movement_sequence(self):
        self.is_dancing = False
        self.is_in_random_mode = False

        if self.movement_sequence_worker:
            self.movement_sequence_worker.stop() # Tell worker loop to exit
        
        if self.movement_sequence_thread and self.movement_sequence_thread.isRunning():
            self.movement_sequence_thread.quit()
            if not self.movement_sequence_thread.wait(1000): # Wait up to 1 sec
                print("VoiceControl: Movement sequence thread did not quit gracefully, terminating.")
                self.movement_sequence_thread.terminate()
                self.movement_sequence_thread.wait() # Ensure termination completes
        
        # Explicitly clear worker and thread to allow for new ones
        if self.movement_sequence_worker:
            self.movement_sequence_worker.deleteLater()
        if self.movement_sequence_thread:
            self.movement_sequence_thread.deleteLater()

        self.movement_sequence_worker = None
        self.movement_sequence_thread = None

    def _on_movement_sequence_finished(self):
        # This is an additional cleanup hook. stop_any_movement_sequence should handle most of it.
        if self.movement_sequence_thread and not self.movement_sequence_thread.isRunning():
            self.movement_sequence_thread.deleteLater()
            self.movement_sequence_thread = None
            # Worker should be cleaned up by its sequence_finished signal path
            if self.movement_sequence_worker and not self.movement_sequence_worker.running:
                 self.movement_sequence_worker.deleteLater()
                 self.movement_sequence_worker = None
 

