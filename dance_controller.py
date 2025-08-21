import serial
import json
import time
import threading
import pygame
import os
import sys

class DanceController:
    def __init__(self, com_port='COM3', baud_rate=115200, music_file="ClaireDeLune.mp3"):
        self.com_port = com_port
        self.baud_rate = baud_rate
        self.music_file = music_file
        self.ser = None
        self.running = False
        
        # Check if music file exists
        if not os.path.exists(self.music_file):
            print(f"⚠️ Warning: Music file '{self.music_file}' not found!")
            print(f"📁 Please place your music file in: {os.getcwd()}")
            print(f"🔧 Or update the music_file parameter in the script")
        
        # Initialize pygame mixer
        pygame.mixer.init()
        
    def connect(self):
        """Connect to the Arduino"""
        try:
            self.ser = serial.Serial(self.com_port, self.baud_rate, timeout=1)
            print(f"Connected to {self.com_port} at {self.baud_rate} baud")
            time.sleep(2)  # Allow Arduino to initialize
            return True
        except Exception as e:
            print(f"Failed to connect: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from Arduino"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("Disconnected from Arduino")
    
    def send_command(self, command_dict):
        """Send JSON command to Arduino"""
        if not self.ser or not self.ser.is_open:
            print("Error: Not connected to Arduino")
            return False
        
        try:
            command_json = json.dumps(command_dict)
            self.ser.write((command_json + '\n').encode())
            print(f"Sent command: {command_json}")
            return True
        except Exception as e:
            print(f"Error sending command: {e}")
            return False
    
    def start_dance(self):
        """Send start_dance command to Arduino"""
        command = {"command": "start_dance"}
        return self.send_command(command)
    
    def stop_dance(self):
        """Send stop_dance command to Arduino"""
        command = {"command": "stop_dance"}
        return self.send_command(command)
    
    def emergency_stop(self):
        """Send emergency_stop command to Arduino"""
        command = {"command": "emergency_stop"}
        return self.send_command(command)
    
    def listen_for_responses(self):
        """Listen for responses from Arduino in a separate thread"""
        self.running = True
        print("Listening for Arduino responses...")
        
        while self.running:
            try:
                if self.ser and self.ser.in_waiting:
                    line = self.ser.readline().decode().strip()
                    if line:
                        print(f"Arduino: {line}")
                        
                        # Check if it's the PLAY SONG command
                        if line == "PLAY SONG":
                            self.play_music()
                        
                        # Try to parse as JSON for other responses
                        try:
                            response = json.loads(line)
                            self.handle_json_response(response)
                        except json.JSONDecodeError:
                            # Not JSON, just a regular message
                            pass
                            
                time.sleep(0.01)  # Small delay to prevent excessive CPU usage
                
            except Exception as e:
                if self.running:  # Only print error if we're still supposed to be running
                    print(f"Error reading from Arduino: {e}")
                break
    
    def handle_json_response(self, response):
        """Handle JSON responses from Arduino"""
        if "status" in response:
            status = response["status"]
            if status == "dance_mode_started":
                print("✅ Dance mode started successfully!")
            elif status == "dance_finished":
                print("🎉 Dance finished!")
            elif status == "emergency_stop_activated":
                print("⚠️ Emergency stop activated!")
                pygame.mixer.music.stop()
            elif status == "dance_mode_stopped":
                print("⏹️ Dance mode stopped")
                pygame.mixer.music.stop()
    
    def play_music(self):
        """Play the music file"""
        if not os.path.exists(self.music_file):
            print(f"❌ Cannot play music: File '{self.music_file}' not found!")
            print(f"📁 Please place the music file in: {os.getcwd()}")
            return
            
        try:
            pygame.mixer.music.load(self.music_file)
            time.sleep(1.9)  # Same delay as original Play_song.py
            pygame.mixer.music.play()
            time.sleep(1.1)  # Same delay as original Play_song.py
            print(f"🎵 Now playing: {self.music_file}")
        except Exception as e:
            print(f"Error playing music: {e}")

    def get_key_press(self):
        """Get a single key press (cross-platform)"""
        try:
            if os.name == 'nt':  # Windows
                import msvcrt
                if msvcrt.kbhit():
                    key = msvcrt.getch()
                    if key == b'\r':  # Enter key
                        return 'ENTER'
                    elif key == b' ':  # Spacebar
                        return 'SPACE'
                    elif key == b'\x1b':  # Escape key
                        return 'ESC'
                    else:
                        return key.decode('utf-8', errors='ignore').upper()
            else:  # Linux/Mac
                import termios, tty
                fd = sys.stdin.fileno()
                old_settings = termios.tcgetattr(fd)
                try:
                    tty.setcbreak(fd)
                    key = sys.stdin.read(1)
                    if key == '\r' or key == '\n':
                        return 'ENTER'
                    elif key == ' ':
                        return 'SPACE'
                    elif key == '\x1b':
                        return 'ESC'
                    else:
                        return key.upper()
                finally:
                    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        except:
            pass
        return None
    
    def run_interactive_mode(self):
        """Run interactive mode with keyboard controls"""
        if not self.connect():
            return
        
        # Start listening thread
        listener_thread = threading.Thread(target=self.listen_for_responses)
        listener_thread.daemon = True
        listener_thread.start()
        
        print("\n" + "="*60)
        print("🎭 DANCE CONTROLLER - Keyboard Control Mode")
        print("="*60)
        print("🎮 CONTROLS:")
        print("  ⏎  ENTER or SPACEBAR - Start Dance & Music")
        print("  🛑 S - Stop Dance")
        print("  🚨 E - Emergency Stop")
        print("  🚪 Q or ESC - Quit Program")
        print("="*60)
        print(f"🎵 Music file: {self.music_file}")
        print(f"📁 Current directory: {os.getcwd()}")
        print(f"🔌 Connected to: {self.com_port}")
        print("="*60)
        print("🎯 Press any key to control the dance...")
        
        try:
            while True:
                key = self.get_key_press()
                if key:
                    print(f"\n🔑 Key pressed: {key}")
                    
                    if key in ['Q', 'ESC']:
                        print("👋 Exiting...")
                        break
                    elif key in ['ENTER', 'SPACE']:
                        print("🚀 Starting dance and music...")
                        self.start_dance()
                    elif key == 'S':
                        print("⏹️ Stopping dance...")
                        self.stop_dance()
                    elif key == 'E':
                        print("🛑 EMERGENCY STOP!")
                        self.emergency_stop()
                    else:
                        print("❌ Unknown key. Use ENTER/SPACE, S, E, or Q")
                    
                    print("🎯 Press any key to control...")
                
                time.sleep(0.05)  # Small delay to prevent excessive CPU usage
        
        except KeyboardInterrupt:
            print("\n🛑 Interrupted by user (Ctrl+C)")
        
        finally:
            self.running = False
            pygame.mixer.music.stop()
            self.disconnect()
            print("👋 Goodbye!")

def main():
    # 🔧 CONFIGURATION - Update these as needed:
    controller = DanceController(
        com_port='COM5',           # ← Change to your Arduino's COM port (was COM5)
        baud_rate=115200,
        music_file="ClaireDeLune.mp3"  # ← Change to your music file name
        # music_file="my_song.mp3"     # ← Example: use a different file
        # music_file="C:/path/to/music.mp3"  # ← Example: use full path
    )
    
    # Run interactive mode
    controller.run_interactive_mode()

if __name__ == "__main__":
    main() 