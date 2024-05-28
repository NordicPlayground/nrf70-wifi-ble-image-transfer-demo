import sys
import socket
import configparser
import time
from io import BytesIO
from PIL import Image
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
    is_res_updated = False
    def __init__(self):
        self.last_frame_time = None

    def process_command(self, command):
        start_marker = command[0:2]
        command_type = command[2:3]

        if start_marker == b'\xFF\xAA':
            if command_type == b'\x01':
                # Capture Video command
                bytesframe = int.from_bytes(command[3:7], byteorder='little')
                frame = command[7:7+bytesframe]
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
            elif command_type == b'\x07':
                new_resolution = command[7]
                # Camera change resolution
                return self.process_resolution_change_notification(new_resolution)
            elif command_type == b'\x08':
                # BLE client is connected!
                return self.process_ble_connection_established_command()
            elif command_type == b'\x09':
                # BLE client is disconnected!
                return self.process_ble_disconnect_command()
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
            img.save("img.jpg", "JPEG")
        except Exception as e:
            error_message = f"Error occurred while processing image: {str(e)}"
        return f"FrameSize: {bytesframe} Bytes\nFrames-per-second: {fps:.2f}\nThroughput: {bytesframe*8*fps/1024:.2f} kbps\n"
        
    def process_info_command(self, payload_length, payload):
        # Process info command payload
        info_payload = payload.decode('utf-8')
        if self.is_res_updated==False:
            # Update IMAGE_RESOLUTION_OPTIONS based on camera type
            if "Camera Type:3MP" in info_payload:
                # Delete max resulation for Camera Type:5MP
                self.update_resolution_options(remove_option="2592x1944")
            elif "Camera Type:5MP" in info_payload:
                # Delete max resulation for Camera Type:3MP
                self.update_resolution_options(remove_option="2048x1536")
        return f"Connected to WiFi Camera!\n{info_payload}"
    
    def update_resolution_options(self, remove_option):
       # Get the main window instance
        main_window = QApplication.instance().topLevelWidgets()[0]

        for key, value in list(main_window.IMAGE_RESOLUTION_OPTIONS.items()):
            if value == remove_option:
                del main_window.IMAGE_RESOLUTION_OPTIONS[key]
                break

        # Update the image_resolution_combobox
        image_resolution_options = list(main_window.IMAGE_RESOLUTION_OPTIONS.items())
        last_item = image_resolution_options[-1][1]
        main_window.image_resolution_combobox.addItem(last_item)
        self.is_res_updated=True


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

    def process_resolution_change_notification(self, new_resolution):
        # Camera change resolution
        resolution = new_resolution
        return f"New Resolution from BLE Client:{resolution}"
    
    def process_stop_command(self, command_type, payload_length, payload):
        # Process stop command
        return f"Stop command received. Command type: {command_type}, Payload length: {payload_length}, Payload: {payload}"
    
    def process_ble_connection_established_command(self):
        return "Camera establish connection with BLE client!"

    def process_ble_disconnect_command(self):
        return "Camera disconnect with BLE client!"

class SocketClient(QObject):
    command_receiving_signal = pyqtSignal(bytes)
    num_signal = pyqtSignal(int)
    stop_signal = pyqtSignal()

    def __init__(self, cam_address='192.168.1.1', cam_port=60010):
        #super(SocketClient, self).__init__()
        super().__init__()
        self.cam_address = cam_address
        self.cam_port = cam_port
        self.buffer_size = 4096
        self.pc_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # self.pc_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.pc_socket.bind(('0.0.0.0', 60000))
        self.pc_socket.connect((cam_address, cam_port))
        print("Connected to the Camera.")
        print("SocketClient init")
        self.command_buffer = b''  # Buffer to store the command packets
        self.in_command = False  # Flag to indicate if currently receiving a command
        self.running = True

        # Connect the stop signal to the method to handle it
        self.stop_signal.connect(self.stop)

    def __del__(self):
        print("SocketClient del")
        #self.pc_socket.close()

    def send_command(self, command_str):
        try:
            print("SocketClient send_command")
            # Convert the command string to bytes
            command_bytes = bytes.fromhex(command_str)
            # Send the command bytes to the server
            self.pc_socket.sendto(command_bytes, (self.cam_address, self.cam_port))
        except Exception as e:
            QMessageBox.critical(None, "Error", "Fail to send command")

    def run(self):
        try:
            print("SocketClient run")
            total_bytes_received=0
            while self.running:
                # Use select to check for incoming data with a timeout to allow for graceful stopping
                # ready = select.select([self.pc_socket], [], [], 1.0)
                if self.pc_socket:
                    data, _ = self.pc_socket.recvfrom(self.buffer_size) #Max 1024 for UDP
                    bytes_received = len(data)
                    total_bytes_received += bytes_received
                    # Print the received content as hexadecimal
                    received_hex = ' '.join(f'{byte:02x}' for byte in data)
                    print(f"Received({bytes_received}): {received_hex}")
                    print(f"Total bytes received: {total_bytes_received}")

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
                        total_bytes_received=0

                    # Check if the packet is a command packet and in a command
                    elif self.in_command:
                        # Append the packet to the command buffer
                        self.command_buffer += data
                    else:
                        total_bytes_received=0

        except Exception as e:
            if self.running:
                QMessageBox.critical(None, "Error", "Fail to receive packets!")

    def stop(self):
        print("SocketClient stop")
        self.running = False
        self.pc_socket.close()


class WifiCamHostGUI(QMainWindow):
    IMAGE_RESOLUTION_OPTIONS = {
        10: "96x96",
        # 11: "128x128",
        # 12: "320x320",
        # 0: "160x120",
        1: "320x240",
        2: "640x480",
        # 3: "800x600",
        4: "1280x720",
        # 5: "1280x960",
        # 6: "1600x1200",
        # 7: "1920x1080",
        8: "2048x1536",
        9: "2592x1944"
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
        self.commands = CommandsToCamera()

        self.config = configparser.ConfigParser()
        # Load config
        self.config.read('config.ini')
        target_ip = self.config['DEFAULT']['TargetIP']
        target_port = int(self.config['DEFAULT']['TargetPort'])
        self.target_server = (target_ip, target_port)
        
        # Initialize SocketClient and socket_thread
        self.client = None
        self.socket_thread = None

        self.camera = ArducamMegaCameraDataProcess()
        self.log_content_text = QTextEdit()  # Placeholder for received content text edit widget
        self.create_layout()

    def create_layout(self):
        main_widget = QWidget()
        layout = QVBoxLayout()
        splitter = QSplitter(Qt.Horizontal)

        # Left side: Display received image
        self.video_frame_widget = QWidget()
        self.video_frame_widget.setFixedSize(1600, 1200)
        self.video_frame_widget.setStyleSheet("background-color: gray;")
        self.video_frame_widget.setWindowTitle("Video Window")

        video_frame_layout = QVBoxLayout()
        self.video_frame_label = QLabel("Video/Image will show here!")
        self.video_frame_label.setScaledContents(True)
        video_frame_layout.addWidget(self.video_frame_label)
        self.video_frame_widget.setLayout(video_frame_layout)
        splitter.addWidget(self.video_frame_widget)

        # Right side: Two parts (setting_window and log_window)
        right_side_widget = QWidget()
        right_side_layout = QVBoxLayout()

        self.connect_window(right_side_layout)
        self.capture_window(right_side_layout)
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
        self.target_server_entry.textChanged.connect(self.update_target_server)
        layout_add.addWidget(self.target_server_entry)

        self.connect_button = QPushButton("Connect")
        self.connect_button.clicked.connect(self.handle_connect_button)
        layout_add.addWidget(self.connect_button)

    def capture_window(self, layout):
        capture_tab = QWidget()
        capture_layout = QVBoxLayout()

        image_resolution_layout = QHBoxLayout()
        image_resolution_label = QLabel("Image Res:")
        image_resolution_layout.addWidget(image_resolution_label)
        self.image_resolution_combobox = QComboBox()
        image_resolution_options = list(self.IMAGE_RESOLUTION_OPTIONS.items())
        self.image_resolution_combobox.addItems([size for _, size in image_resolution_options[:-2]])
        self.image_resolution_combobox.setEnabled(False)
        self.image_resolution_combobox.currentIndexChanged.connect(self.handle_resolution_change)

        image_resolution_layout.addWidget(self.image_resolution_combobox)
        capture_layout.addLayout(image_resolution_layout)

        self.stream_button = QPushButton("Start Stream")
        self.stream_button.setEnabled(False)
        self.stream_button.clicked.connect(self.toggle_stream_button)
        capture_layout.addWidget(self.stream_button)

        self.capture_image_button = QPushButton("Take Picture")
        self.capture_image_button.setEnabled(False)
        self.capture_image_button.clicked.connect(self.capture_image)
        capture_layout.addWidget(self.capture_image_button)

        capture_tab.setLayout(capture_layout)
        layout.addWidget(capture_tab)

    def toggle_stream_button(self):
        if self.stream_button.text() == "Start Stream":
            self.stream_button.setText("Stop Stream")
            self.capture_image_button.setEnabled(False)
            self.log_content_text.clear()
            resolution_number = list(self.IMAGE_RESOLUTION_OPTIONS.keys())[
                list(self.IMAGE_RESOLUTION_OPTIONS.values()).index(self.image_resolution_combobox.currentText())]
            self.send_command(self.commands.command_start_streaming_mode(resolution_number))
        else:
            self.stream_button.setText("Start Stream")
            self.capture_image_button.setEnabled(True)
            self.send_command(self.commands.command_stop_stream())

    def handle_connect_button(self):
        if self.connect_button.text() == "Connect":
            self.clear_log()
            self.stream_button.setText("Start Stream")
            self.log_content_text.append(f"Connecting to WiFi Camera at {self.target_server[0]}:{self.target_server[1]} ...")
            if self.client is not None:
                self.client.stop_signal.emit()  # Signal the client to stop
                self.client.stop()
                self.socket_thread.quit()
                self.socket_thread.wait()
            # Initialize SocketClient and socket_thread
            self.socket_thread = QThread()
            self.client = SocketClient(*self.target_server)
            self.client.moveToThread(self.socket_thread)
            self.socket_thread.started.connect(self.client.run)
            self.socket_thread.start()
            self.client.command_receiving_signal.connect(self.process_received_packets)
            self.send_command(self.commands.command_get_camera_info())
        else:
            self.send_command(self.commands.command_stop_stream())
            self.client.command_receiving_signal.disconnect(self.process_received_packets)
            self.client.stop_signal.emit()  # Signal the client to stop
            self.socket_thread.quit()
            self.connect_button.setText("Connect")
            self.stream_button.setEnabled(False)
            self.image_resolution_combobox.setEnabled(False)
            self.capture_image_button.setEnabled(False)
            self.log_content_text.append(f"Disconnected from WiFi Camera at {self.target_server[0]}:{self.target_server[1]}")
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
        if self.log_content_text:
            self.log_content_text.append(f"SENT: {command_str}")
        
        # Send the command
        self.client.send_command(command_str)
    def handle_resolution_change(self, index):
        resolution_number = list(self.IMAGE_RESOLUTION_OPTIONS.keys())[
        list(self.IMAGE_RESOLUTION_OPTIONS.values()).index(self.image_resolution_combobox.currentText())]
        self.send_command(self.commands.command_set_picture_resolution(1,resolution_number))

        # if self.stream_button.text() == "Stop Stream":
        #     # Stop the current stream
        #     self.send_command(self.commands.command_stop_stream())
        #     time.sleep(0.5)  # Adjust the delay as needed

        #     # Start the stream with the new resolution
        #     resolution_number = list(self.IMAGE_RESOLUTION_OPTIONS.keys())[
        #     list(self.IMAGE_RESOLUTION_OPTIONS.values()).index(self.image_resolution_combobox.currentText())]
        #     self.send_command(self.commands.command_start_streaming_mode(resolution_number))
        

    def log_window(self, layout):
        log_window = QGroupBox()
        log_window.setTitle("Log Content")

        log_window_layout = QVBoxLayout()
        log_window.setLayout(log_window_layout)

        log_content_layout = QVBoxLayout()
        log_window_layout.addLayout(log_content_layout)

        self.log_content_text = QTextEdit("Check WiFi Camera Device printout for the target address.\n Example: 192.168.1.100:60010")  # Set the text edit widget in client
        self.log_content_text.setReadOnly(True)
        log_content_layout.addWidget(self.log_content_text)

        layout.addWidget(log_window)  

        clear_button = QPushButton("Clear Log")
        clear_button.clicked.connect(self.clear_log)
        layout.addWidget(clear_button)
    
    def clear_log(self):
        if self.log_content_text:
            self.log_content_text.clear()

    def process_received_packets(self, command_buffer):
        self.client.command_receiving_signal.disconnect(self.process_received_packets)
        # Process the received command buffer
        result = self.camera.process_command(command_buffer)

        # Append result to the received content text edit widget
        if self.log_content_text:
            self.log_content_text.append(result)
        
        # Display the received video frame
        if result.startswith("Connected to WiFi Camera!"):
            self.capture_image_button.setEnabled(True)
            self.stream_button.setEnabled(True)
            self.image_resolution_combobox.setEnabled(True)
            self.connect_button.setText("Disconnect")

        # Display the received video frame
        if result.startswith("FrameSize"):
            img_path = "img.jpg"
            pixmap = QPixmap(img_path)
            self.video_frame_label.setPixmap(pixmap)

        # Extract the resolution from the input string
        if result.startswith("New Resolution from BLE Client"):
            new_resolution = int(result.split(":")[-1].strip())
            # Check if the new_resolution is a valid key in the IMAGE_RESOLUTION_OPTIONS dictionary
            if new_resolution in self.IMAGE_RESOLUTION_OPTIONS:
                # Set the current index of the combo box to the index corresponding to the new resolution
                self.image_resolution_combobox.setCurrentIndex(list(self.IMAGE_RESOLUTION_OPTIONS.keys()).index(new_resolution))
            else:
                print("New resolution not found in options.")
        
        if result.startswith("Camera switch to BLE mode"):
            self.send_command(self.commands.command_stop_stream())
            self.client_thread.command_receiving_signal.disconnect()
            self.capture_image_button.setEnabled(False)
            self.stream_button.setEnabled(False)
            self.image_resolution_combobox.setEnabled(False)
            self.connect_button.setText("Connect")
            self.log_content_text.clear()
            self.video_frame_label.setText("Video/Image will show here!")
            self.log_content_text.append(f"Camera switch to BLE mode!\nPlease check Android App.")    

        if result.startswith("Camera switch to Socket mode"):
            self.log_content_text.append(f"Camera switch back to Socket mode!\n Please try with connect button again.")
        self.client.command_receiving_signal.connect(self.process_received_packets)

    def update_target_server(self, text):
        if ':' in text:
            ip, port = text.split(':')
            self.target_server = (ip, int(port))

    def capture_image(self):
        self.log_content_text.append(f"Sending Take Picture command...")
        # resolution_number = list(self.IMAGE_RESOLUTION_OPTIONS.keys())[
        #         list(self.IMAGE_RESOLUTION_OPTIONS.values()).index(self.image_resolution_combobox.currentText())]
        # self.send_command(self.commands.command_set_picture_resolution(1,resolution_number))
        self.send_command(self.commands.command_take_picture())
        self.log_content_text.append(f"Take Picture command sent!")

    def closeEvent(self, event):
        # reply = QMessageBox.question(self, 'Exit', 'Are you sure you want to exit?',
        #                              QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        # if reply == QMessageBox.Yes:
        #     if self.client is not None:
        #         self.send_command(self.commands.command_stop_stream())
        #         self.client.stop_signal.emit()  # Signal the client to stop
        #         self.client.stop()
        #         self.socket_thread.quit()
        #         self.socket_thread.wait()
        #     event.accept()
        # else:
        #     event.ignore()
        
        self.send_command(self.commands.command_stop_stream())
        self.client.stop_signal.emit()  # Signal the client to stop
        self.client.stop()
        self.socket_thread.quit()
        self.socket_thread.wait()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = WifiCamHostGUI()
    window.show()
    sys.exit(app.exec())