import sys
import socket
import configparser
import time
from io import BytesIO
from PIL import Image
from threading import Thread
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *



class CommandsToCamera:
    START_CHARACTER = 0x55
    STOP_CHARACTER = 0xAA
    
    COMMANDS = {
        0xFF: {"name": "RESET_CAMERA", "description": "Reset camera", "parameters": {}},
        0x01: {"name": "SET_PICTURE_RESOLUTION", "description": "Set picture resolution", "parameters": {"format": None, "resolution": None}},
        0x02: {"name": "SET_VIDEO_RESOLUTION", "description": "Set video resolution", "parameters": {"resolution": None}},
        0x03: {"name": "SET_BRIGHTNESS", "description": "Set brightness", "parameters": {"brightness": None}},
        0x04: {"name": "SET_CONTRAST", "description": "Set contrast", "parameters": {"contrast": None}},
        0x05: {"name": "SET_SATURATION", "description": "Set saturation", "parameters": {"saturation": None}},
        0x06: {"name": "SET_EV", "description": "Set EV", "parameters": {"ev": None}},
        0x07: {"name": "SET_WHITEBALANCE", "description": "Set white balance", "parameters": {"white_balance": None}},
        0x08: {"name": "SET_SPECIAL_EFFECTS", "description": "Set special effects", "parameters": {"effect": None}},
        0x09: {"name": "SET_FOCUS_ENABLE", "description": "Set focus enable", "parameters": {"focus_control": None}},
        0x0A: {"name": "SET_EXPOSURE_GAIN_ENABLE", "description": "Set exposure gain enable", "parameters": {"exposure_control": None}},
        0x0C: {"name": "SET_WHITE_BALANCE_ENABLE", "description": "Set white balance enable", "parameters": {"white_balance_control": None}},
        0x0D: {"name": "SET_MANUAL_GAIN", "description": "Set manual gain", "parameters": {"manual_gain": None}},
        0x0E: {"name": "SET_MANUAL_EXPOSURE", "description": "Set manual exposure", "parameters": {"manual_exposure": None}},
        0x0F: {"name": "GET_CAMERA_INFO", "description": "Get camera info", "parameters": {}},
        0x10: {"name": "TAKE_PICTURE", "description": "Take picture", "parameters": {}},
        0x11: {"name": "SET_SHARPNESS", "description": "Set sharpness", "parameters": {"sharpness": None}},
        0x12: {"name": "DEBUG_WRITE_REGISTER", "description": "Debug write register", "parameters": {"register": None, "value": None}},
        0x21: {"name": "STOP_STREAM", "description": "Stop stream", "parameters": {}},
        0x30: {"name": "GET_FRM_VER_INFO", "description": "Get frame version info", "parameters": {}},
        0x40: {"name": "GET_SDK_VER_INFO", "description": "Get SDK version info", "parameters": {}},
        0x50: {"name": "SET_IMAGE_QUALITY", "description": "Set image quality", "parameters": {"quality": None}},
        0x60: {"name": "SET_LOWPOWER_MODE", "description": "Set low power mode", "parameters": {"mode": None}},
    }

    @staticmethod
    def get_command_name(command_id):
        return CommandsToCamera.COMMANDS[command_id]["name"]

    @staticmethod
    def get_command_description(command_id):
        return CommandsToCamera.COMMANDS[command_id]["description"]

    @staticmethod
    def get_command_parameters(command_id):
        return CommandsToCamera.COMMANDS[command_id]["parameters"]
    
    @staticmethod
    def command_get_camera_info():
        command_id = 0x0F
        command_bytes = bytes([CommandsToCamera.START_CHARACTER, command_id, CommandsToCamera.STOP_CHARACTER])
        command_str = " ".join(format(byte, "02X") for byte in command_bytes)
        return command_str
    
    @staticmethod
    def command_take_picture():
        command_id = 0x10
        command_bytes = bytes([CommandsToCamera.START_CHARACTER, command_id, CommandsToCamera.STOP_CHARACTER])
        command_str = " ".join(format(byte, "02X") for byte in command_bytes)
        return command_str

    @staticmethod
    def command_stop_stream():
        command_id = 0x21
        command_bytes = bytes([CommandsToCamera.START_CHARACTER, command_id, CommandsToCamera.STOP_CHARACTER])
        command_str = " ".join(format(byte, "02X") for byte in command_bytes)
        return command_str
    
    @staticmethod
    def command_set_picture_resolution(format_code, resolution):
        command_id = 0x01
        # Ensure format code is within the valid range
        if format_code not in (1, 2, 3):
            raise ValueError("Invalid format code. It must be 1, 2, or 3.")
        # Ensure resolution code is within the valid range
        if resolution < 0 or resolution > 13:
            raise ValueError("Invalid resolution code. It must be between 0 and 9.")
        # Combine format code and resolution into a single byte
        parameter_byte = (format_code << 4) | resolution
        command_bytes = bytes([CommandsToCamera.START_CHARACTER, command_id, parameter_byte, CommandsToCamera.STOP_CHARACTER])
        command_str = " ".join(format(byte, "02X") for byte in command_bytes)
        return command_str
    
    @staticmethod
    def command_start_streaming_mode(resolution):
        command_id = 0x02
        command_bytes = bytes([CommandsToCamera.START_CHARACTER, command_id, resolution, CommandsToCamera.STOP_CHARACTER])
        command_str = " ".join(format(byte, "02X") for byte in command_bytes)
        return command_str
    
    @staticmethod
    def command_set_brightness(brightness):
        command_id = 0x03
        # Ensure brightness code is within the valid range
        if brightness not in range(9):
            raise ValueError("Invalid brightness code. It must be between 0 and 8.")
        command_bytes = bytes([CommandsToCamera.START_CHARACTER, command_id, brightness, CommandsToCamera.STOP_CHARACTER])
        command_str = " ".join(format(byte, "02X") for byte in command_bytes)
        return command_str
    
    @staticmethod
    def command_set_contrast(contrast):
        command_id = 0x04
        # Ensure contrast code is within the valid range
        if contrast not in range(7):
            raise ValueError("Invalid contrast code. It must be between 0 and 6.")
        command_bytes = bytes([CommandsToCamera.START_CHARACTER, command_id, contrast, CommandsToCamera.STOP_CHARACTER])
        command_str = " ".join(format(byte, "02X") for byte in command_bytes)
        return command_str
    
    @staticmethod
    def command_set_saturation(saturation):
        command_id = 0x05
        # Ensure saturation code is within the valid range
        if saturation not in range(7):
            raise ValueError("Invalid saturation code. It must be between 0 and 6.")
        command_bytes = bytes([CommandsToCamera.START_CHARACTER, command_id, saturation, CommandsToCamera.STOP_CHARACTER])
        command_str = " ".join(format(byte, "02X") for byte in command_bytes)
        return command_str
    
    @staticmethod
    def command_set_ev(ev):
        command_id = 0x06
        # Ensure ev code is within the valid range
        if ev not in range(7):
            raise ValueError("Invalid EV code. It must be between 0 and 6.")
        command_bytes = bytes([CommandsToCamera.START_CHARACTER, command_id, ev, CommandsToCamera.STOP_CHARACTER])
        command_str = " ".join(format(byte, "02X") for byte in command_bytes)
        return command_str
    
    @staticmethod
    def command_set_white_balance(white_balance):
        command_id = 0x07
        # Ensure white balance code is within the valid range
        if white_balance not in range(5):
            raise ValueError("Invalid white balance code. It must be between 0 and 4.")
        command_bytes = bytes([CommandsToCamera.START_CHARACTER, command_id, white_balance, CommandsToCamera.STOP_CHARACTER])
        command_str = " ".join(format(byte, "02X") for byte in command_bytes)
        return command_str
    
    @staticmethod
    def command_set_special_effects(effect):
        command_id = 0x08
        # Ensure effect code is within the valid range
        if effect not in range(10):
            raise ValueError("Invalid special effects code. It must be between 0 and 9.")
        command_bytes = bytes([CommandsToCamera.START_CHARACTER, command_id, effect, CommandsToCamera.STOP_CHARACTER])
        command_str = " ".join(format(byte, "02X") for byte in command_bytes)
        return command_str
    
    @staticmethod
    def command_focus_control(focus_control):
        command_id = 0x09
        # Ensure focus control code is within the valid range

class ArducamMegaCameraDataProcess:
    def __init__(self):
        self.last_frame_time = None

    def process_command(self, command):
        start_marker = command[0:2]
        command_type = command[2:3]

        if start_marker == b'\xFF\xAA':
            if command_type == b'\x01':
                # Capture Video command
                bytesframe = int.from_bytes(command[3:7], byteorder='little')
                resolution = command[7:8]
                frame = command[8:8+bytesframe]
                return self.process_capture_command(bytesframe, frame)
            elif command_type == b'\x02':
                # Camera Info command
                payload_length = int.from_bytes(command[3:7], byteorder='little')
                payload = command[7:7+payload_length]
                return self.process_info_command(payload_length, payload)
            elif command_type == b'\x03':
                # Version command
                return self.process_version_command(payload)
            elif command_type == b'\x05':
                # Version 2 command
                return self.process_version2_command(payload)
            elif command_type == b'\x06':
                # Streamoff command
                return self.process_streamoff_command(payload)
            else:
                return "Unknown command type"
        elif start_marker == b'\xFF\xBB':
            # Stop command
            return self.process_stop_command(command_type, payload_length, payload)
        else:
            return "Invalid start marker"

    def process_capture_command(self, bytesframe, frame):
        current_time = time.time()

        if self.last_frame_time is None:
            self.last_frame_time = current_time
            fps = 0
        else:
            time_difference = float(current_time - self.last_frame_time)
            if time_difference > 0:
                fps = 1 / time_difference
            else:
                fps = 0  # Set FPS to 0 if time difference is zero
            self.last_frame_time = current_time

        try:
            img = Image.open(BytesIO(frame))
            img.save("img.temp", "JPEG")
        except Exception as e:
            error_message = f"Error occurred while processing image: {str(e)}"
        return f"RECV: Frame is captured!\nFrameSzie: {bytesframe}\nFPS: {fps:.2f}\nThroughPUT: {bytesframe*fps/1024:.2f}KB/SEC"

    def process_info_command(self, payload_length, payload):
        # Process info command payload
        info_payload = payload.decode('utf-8')
        return f"RECV: Connected to WiFi Camera!\n{info_payload}"

    def process_version_command(self, payload):
        # Process version command payload
        year = int.from_bytes(payload[0:1], byteorder='little')
        month = int.from_bytes(payload[1:2], byteorder='little')
        day = int.from_bytes(payload[2:3], byteorder='little')
        version = f"{(payload[3] >> 4)}.{payload[3] & 0x0F}"
        return f"Version command received. Date: {year}/{month}/{day}, Version: {version}"

    def process_version2_command(self, payload):
        # Process version 2 command payload
        year = int.from_bytes(payload[0:1], byteorder='little')
        month = int.from_bytes(payload[1:2], byteorder='little')
        day = int.from_bytes(payload[2:3], byteorder='little')
        version = f"{(payload[3] >> 4)}.{payload[3] & 0x0F}"
        return f"Version 2 command received. Date: {year}/{month}/{day}, Version: {version}"

    def process_streamoff_command(self, payload):
        # Process streamoff command payload
        return "Streamoff command received"

    def process_stop_command(self, command_type, payload_length, payload):
        # Process stop command
        return f"Stop command received. Command type: {command_type}, Payload length: {payload_length}, Payload: {payload}"


class UDPClient(QObject):
    command_receiving_signal = pyqtSignal(bytes)

    def __init__(self, server_address='192.168.1.122', server_port=60000):
        super().__init__()
        self.server_address = server_address
        self.server_port = server_port
        self.buffer_size = 1024
        self.camera = ArducamMegaCameraDataProcess()
        self.client_send_udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.client_send_udp_socket.bind(('0.0.0.0', 50000))
        self.client_recv_udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.client_recv_udp_socket.bind(('0.0.0.0', 50005))
        self.log_content_text = QTextEdit()  # Placeholder for received content text edit widget
        self.command_buffer = b''  # Buffer to store the command packets
        self.in_command = False  # Flag to indicate if currently receiving a command

    def send_command(self, command_str):
        try:
            # Convert the command string to bytes
            command_bytes = bytes.fromhex(command_str)

            # Send the command bytes to the server
            self.client_send_udp_socket.sendto(command_bytes, (self.server_address, self.server_port))

        except Exception as e:
            QMessageBox.critical(None, "Error", str(e))

    def receive_packets(self):
        try:
            while True:
                # Receive data
                data, _ = self.client_recv_udp_socket.recvfrom(self.buffer_size)
                # Print the received content as hexadecimal
                # received_hex = ' '.join(f'{byte:02x}' for byte in data)
                # print(f"Received: {received_hex}")

                # Check if the packet starts with FF AA
                if data.startswith(b'\xFF\xAA'):
                    self.command_buffer = b''
                    # Set flag to indicate receiving a command
                    self.in_command = True

                    # Append the packet to the command buffer
                    self.command_buffer += data

                # Check if the packet ends with FF BB and in a command
                elif data.endswith(b'\xFF\xBB') and self.in_command:
                    # Append the packet to the command buffer
                    self.command_buffer += data

                    # Emit signal with the complete command
                    self.command_receiving_signal.emit(self.command_buffer)

                    # Reset command buffer and flag
                    self.command_buffer = b''
                    self.in_command = False

                # Check if the packet is a command packet and in a command
                elif self.in_command:
                    # Append the packet to the command buffer
                    self.command_buffer += data

        except Exception as e:
            QMessageBox.critical(None, "Error", str(e))


class WifiCamHostGUI(QMainWindow):
    IMAGE_RESOLUTION_OPTIONS = {
        10: "96x96",
        11: "128x128",
        12: "320x320",
        # 0: "160x120",
        1: "320x240",
        2: "640x480",
        3: "800x600",
        4: "1280x720",
        5: "1280x960",
        6: "1600x1200",
        7: "1920x1080",
        8: "2048x1536",
        #9: "2592x1944"
    }

    IMAGE_FORMAT_OPTIONS = {
        1: "JPEG",
        2: "RGB",
        3: "YUV",
    }
    
    def __init__(self):
        super().__init__()
        self.resize(1200, 900)
        self.setWindowTitle("WiFi Camera Host")
        self.setWindowIcon(QIcon("icon.png"))
        self.commands=CommandsToCamera()

        self.config = configparser.ConfigParser()

        # Load config
        self.config.read('config.ini')
        target_ip = self.config['DEFAULT']['TargetIP'] 
        target_port = int(self.config['DEFAULT']['TargetPort'])
        self.target_server = (target_ip, target_port)
        # self.target_server = self.config['DEFAULT']['Target_server']
        self.client = UDPClient(*self.target_server)


        # Define pre-filled commands
        pre_filled_commands = [self.commands.command_get_camera_info(),self.commands.command_take_picture()]


        # Initialize UDPClient with pre-filled commands
        # self.client.last_commands.extend(pre_filled_commands)
        self.create_layout(pre_filled_commands)

        # Connect command receiving signal to process_received_packets slot
        self.client.command_receiving_signal.connect(self.process_received_packets)

        # Start a thread to receive UDP packets
        receive_thread = Thread(target=self.client.receive_packets)
        receive_thread.daemon = True
        receive_thread.start()

    def create_layout(self, pre_filled_commands):
        main_widget = QWidget()
        layout = QVBoxLayout()
        splitter = QSplitter(Qt.Horizontal)

        # Left side: Display received image
        self.video_frame_widget = QWidget()
        self.video_frame_widget.resize(800, 600)
        self.video_frame_widget.setStyleSheet("background-color: gray;")
        self.video_frame_widget.setWindowTitle("Video Window")

        video_frame_layout = QVBoxLayout()
        self.video_frame_label = QLabel("Video/Image will show here!")
        self.video_frame_label.setScaledContents( True )
        video_frame_layout.addWidget(self.video_frame_label)
        self.video_frame_widget.setLayout(video_frame_layout)
        splitter.addWidget(self.video_frame_widget)

        # Right side: Two parts (setting_window and log_window)
        right_side_widget = QWidget()
        right_side_layout = QVBoxLayout()

        self.connect_window(right_side_layout)
        self.capture_window(right_side_layout)  # Added Capture Window
        self.command_input_window(right_side_layout, pre_filled_commands)
        self.log_window(right_side_layout)

        right_side_widget.setLayout(right_side_layout)
        splitter.addWidget(right_side_widget)

        layout.addWidget(splitter)
        main_widget.setLayout(layout)
        self.setCentralWidget(main_widget)

    def connect_window(self, layout):
        add_widget = QWidget()
        layout.addWidget(add_widget)

        layout_add = QVBoxLayout()
        add_widget.setLayout(layout_add)

        label_add = QLabel("Target WiFi Camera Address")
        layout_add.addWidget(label_add)

        self.target_server_entry = QLineEdit(f"{self.target_server[0]}:{self.target_server[1]}")
        self.target_server_entry.setText(f"{self.target_server[0]}:{self.target_server[1]}") 
        # Connect signal to store new input 
        self.target_server_entry.textChanged.connect(self.update_target_server)
        layout_add.addWidget(self.target_server_entry)

        self.connect_button = QPushButton("Connect")
        self.connect_button.clicked.connect(self.connect_to_camera)
        layout_add.addWidget(self.connect_button)

    def capture_window(self, layout):
        capture_tab = QTabWidget()
        layout.addWidget(capture_tab)

        # Video Tab
        video_tab = QWidget()
        video_layout = QVBoxLayout()

        video_size_layout = QHBoxLayout()  # New layout for video size
        video_size_label = QLabel("Video Size:")
        video_size_layout.addWidget(video_size_label)
        self.video_size_combobox = QComboBox()
        #self.video_size_combobox.addItems(["96x96", "320x240", "320x320", "640x480"])
        # Get first 5 items from dictionary 
        video_resolution_options = list(self.IMAGE_RESOLUTION_OPTIONS.items())[:5]
        self.video_size_combobox.addItems([size for _, size in video_resolution_options])
        video_size_layout.addWidget(self.video_size_combobox)
        video_layout.addLayout(video_size_layout)  # Add the video size layout to the main video layout

        # Add Capture Video Button
        self.start_stream_button = QPushButton("Start Stream")
        self.start_stream_button.setEnabled(False)
        self.start_stream_button.clicked.connect(self.capture_video)
        video_layout.addWidget(self.start_stream_button)

        self.stop_stream_button = QPushButton("Stop Stream")
        self.stop_stream_button.setEnabled(False)
        self.stop_stream_button.clicked.connect(self.stop_stream)
        video_layout.addWidget(self.stop_stream_button)

        video_tab.setLayout(video_layout)
        capture_tab.addTab(video_tab, "Video")

        # Image Tab
        image_tab = QWidget()
        image_layout = QVBoxLayout()

        # Image Format Layout
        image_format_layout = QHBoxLayout()
        image_format_label = QLabel("Image Format:")
        image_format_layout.addWidget(image_format_label)
        self.image_format_combobox = QComboBox()
        image_format_options = list(self.IMAGE_FORMAT_OPTIONS.items())
        self.image_format_combobox.addItems([size for _, size in image_format_options])
        image_format_layout.addWidget(self.image_format_combobox)
        image_layout.addLayout(image_format_layout)

        # Image Size Layout
        image_size_layout = QHBoxLayout()
        image_size_label = QLabel("Image Size:")
        image_size_layout.addWidget(image_size_label)
        self.image_size_combobox = QComboBox()
        self.image_size_combobox.addItems([size for _, size in self.IMAGE_RESOLUTION_OPTIONS.items()])
        image_size_layout.addWidget(self.image_size_combobox)
        image_layout.addLayout(image_size_layout)

        # Add Capture Image Button
        self.capture_image_button = QPushButton("Capture Image")
        self.capture_image_button.setEnabled(False)
        self.capture_image_button.clicked.connect(self.capture_image)
        image_layout.addWidget(self.capture_image_button)

        image_tab.setLayout(image_layout)
        capture_tab.addTab(image_tab, "Image")

    def command_input_window(self, layout, pre_filled_commands):
        label = QLabel("Commands:")
        layout.addWidget(label)

        # Initialize QCompleter with pre-filled commands
        completer = QCompleter(pre_filled_commands)
        self.command_entry = QLineEdit()
        self.command_entry.setCompleter(completer)
        layout.addWidget(self.command_entry)

        send_button = QPushButton("Send Command")
        send_button.clicked.connect(lambda: self.send_command(self.command_entry.text()))
        layout.addWidget(send_button)

    def log_window(self, layout):
        label = QLabel("Log:")
        layout.addWidget(label)

        self.client.log_content_text = QTextEdit("Check WiFi Camera Device printout for the target address.\n Example: 192.168.1.100:60000")  # Set the text edit widget in client
        layout.addWidget(self.client.log_content_text)

        clear_button = QPushButton("Clear Log")
        clear_button.clicked.connect(self.clear_log)
        layout.addWidget(clear_button)

    def send_command(self, command_str):
        hex_array = command_str.split()
        if len(hex_array) > 6:
            QMessageBox.critical(None, "Error", "Maximum length of command is 6 digits.")
            return
        try:
            bytes.fromhex("".join(hex_array))
        except ValueError:
            QMessageBox.critical(None, "Error", "Invalid hex format.")
            return
        # Show the sent command in the log window
        if self.client.log_content_text:
            self.client.log_content_text.append(f"SENT: {command_str}")
        
        # Send the command
        self.client.send_command(command_str)

    def process_received_packets(self, command_buffer):
        # Process the received command buffer
        result = self.client.camera.process_command(command_buffer)

        # Append result to the received content text edit widget
        if self.client.log_content_text:
            self.client.log_content_text.append(result)
        
        # Display the received video frame
        if result.startswith("RECV: Connected to WiFi Camera!"):
            #self.connect_button.setEnabled(False)
            self.start_stream_button.setEnabled(True)
            self.stop_stream_button.setEnabled(True)
            self.capture_image_button.setEnabled(True)
            
        # Display the received video frame
        if result.startswith("RECV: Frame is captured!"):
            img_path = "img.temp"
            pixmap = QPixmap(img_path)
            self.video_frame_label.setPixmap(pixmap)
    # Update method
    def update_target_server(self, text):
         # Update target server 
        try:
            ip, port = text.split(':')
        except ValueError:
            QMessageBox.warning(self, "Invalid Input", "Please enter address as IP:Port")
            return

        self.target_server = (ip, int(port))

        # Save config
        self.config['DEFAULT'] = {
            'TargetIP': self.target_server[0], 
            'TargetPort': str(self.target_server[1])
        }
        
        with open('config.ini', 'w') as f: 
            self.config.write(f)

    def connect_to_camera(self):
        address = self.target_server_entry.text()
        if ':' not in address:
            QMessageBox.critical(None, "Error", "Invalid address format. Use IP:Port.")
            return
        ip, port = address.split(':')
        try:
            port = int(port)
        except ValueError:
            QMessageBox.critical(None, "Error", "Invalid port number.")
            return
        self.send_command(self.commands.command_get_camera_info())

    def capture_video(self):
        self.start_stream_button.setEnabled(False)
        self.stop_stream_button.setEnabled(True)
        self.client.log_content_text.clear()
        resolution_number = list(self.IMAGE_RESOLUTION_OPTIONS.keys())[list(self.IMAGE_RESOLUTION_OPTIONS.values()).index(self.video_size_combobox.currentText())]
        self.send_command(self.commands.command_start_streaming_mode(resolution_number))

    def capture_image(self):
        resolution_number = list(self.IMAGE_RESOLUTION_OPTIONS.keys())[list(self.IMAGE_RESOLUTION_OPTIONS.values()).index(self.image_size_combobox.currentText())]
        format_number = list(self.IMAGE_FORMAT_OPTIONS.keys())[list(self.IMAGE_FORMAT_OPTIONS.values()).index(self.image_format_combobox.currentText())]
        self.send_command(self.commands.command_set_picture_resolution(format_number,resolution_number))
        self.send_command(self.commands.command_take_picture())

    def stop_stream(self):
        self.start_stream_button.setEnabled(True)
        self.stop_stream_button.setEnabled(False)
        self.send_command(self.commands.command_stop_stream())

    def clear_log(self):
        if self.client.log_content_text:
            self.client.log_content_text.clear()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = WifiCamHostGUI()
    window.show()
    sys.exit(app.exec_())
