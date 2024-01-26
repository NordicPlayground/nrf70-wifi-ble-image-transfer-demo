import sys
import socket
from threading import Thread
from collections import deque
from PyQt5.QtCore import Qt, pyqtSignal, QObject
from PyQt5.QtWidgets import (
    QApplication,
    QMainWindow,
    QVBoxLayout,
    QWidget,
    QLabel,
    QLineEdit,
    QPushButton,
    QTextEdit,
    QMessageBox,
    QCompleter,
    QHBoxLayout,
)
from PyQt5.QtGui import QPixmap
from PIL import Image
from io import BytesIO


class ArducamMegaCamera:
    def __init__(self):
        pass

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
        # Process capture command payload
        print(bytes(frame).hex())
        img = Image.open(BytesIO(frame))
        img.show()
        img.save("received_frame.jpg", "JPEG")
        return f"Capture video command received. Video length: {bytesframe}, VideoFrameData: {frame}"

    def process_info_command(self, payload_length, payload):
        # Process info command payload
        info_payload = payload.decode('utf-8')
        return f"Info command received. Payload length: {payload_length},Payload:{info_payload}"

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

    def __init__(self, server_address='192.168.1.144', server_port=50000):
        super().__init__()
        self.server_address = server_address
        self.server_port = server_port
        self.buffer_size = 1024
        self.camera = ArducamMegaCamera()
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_socket.bind(('0.0.0.0', 50006))
        self.received_content_text = None  # Placeholder for received content text edit widget
        self.command_buffer = b''  # Buffer to store the command packets
        self.in_command = False  # Flag to indicate if currently receiving a command

    def send_command(self, command_str):
        try:
            # Convert the command string to bytes
            command_bytes = bytes.fromhex(command_str)

            # Send the command bytes to the server
            self.udp_socket.sendto(command_bytes, (self.server_address, self.server_port))

        except Exception as e:
            QMessageBox.critical(None, "Error", str(e))

    def receive_packets(self):
        try:
            while True:
                # Receive data
                data, _ = self.udp_socket.recvfrom(self.buffer_size)
                # Print the received content as hexadecimal
                received_hex = ' '.join(f'{byte:02x}' for byte in data)
                print(f"Received: {received_hex}")

                # Check if the packet starts with FF AA
                if data.startswith(b'\xFF\xAA'):
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


class WIFI_CAM_GUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("WiFi Camera Host")

        # Define pre-filled commands
        pre_filled_commands = ['55 0F AA', '55 10 AA']

        self.target_server = ('192.168.1.144', 50000)
        self.client = WIFI_CAM_GUI(*self.target_server)
        # Initialize UDPClient with pre-filled commands
        # self.client.last_commands.extend(pre_filled_commands)
        self.create_target_server_input()
        self.create_send_command_window(pre_filled_commands)
        self.create_received_content_window()
        self.create_video_frame_window()

        # Connect command receiving signal to process_received_packets slot
        self.client.command_receiving_signal.connect(self.process_received_packets)

        # Start a thread to receive UDP packets
        receive_thread = Thread(target=self.client.receive_packets)
        receive_thread.daemon = True
        receive_thread.start()

    def create_target_server_input(self):
        self.target_server_widget = QWidget()
        layout = QVBoxLayout()

        label = QLabel("Target WiFi Camera Address:(e.g., 192.168.1.144:50000, which can be found in device log.):")
        layout.addWidget(label)

        self.target_server_entry = QLineEdit(f"{self.target_server[0]}:{self.target_server[1]}")
        layout.addWidget(self.target_server_entry)

        self.target_server_widget.setLayout(layout)
        self.setCentralWidget(self.target_server_widget)

    def create_send_command_window(self, pre_filled_commands):
        self.send_command_widget = QWidget()
        layout = QVBoxLayout()

        label = QLabel("Enter Command (e.g., 55 0F AA):")
        layout.addWidget(label)

        # Initialize QCompleter with pre-filled commands
        completer = QCompleter(pre_filled_commands)
        self.command_entry = QLineEdit()
        self.command_entry.setCompleter(completer)
        layout.addWidget(self.command_entry)

        send_button = QPushButton("Send Command")
        send_button.clicked.connect(self.send_command)
        layout.addWidget(send_button)

        self.send_command_widget.setLayout(layout)
        self.target_server_widget.layout().addWidget(self.send_command_widget)

    def create_received_content_window(self):
        self.received_content_widget = QWidget()
        layout = QVBoxLayout()

        label = QLabel("Received Content:")
        layout.addWidget(label)

        self.client.received_content_text = QTextEdit()  # Set the text edit widget in client
        layout.addWidget(self.client.received_content_text)

        self.received_content_widget.setLayout(layout)
        self.target_server_widget.layout().addWidget(self.received_content_widget)

    def create_video_frame_window(self):
        self.video_frame_widget = QWidget()
        layout = QVBoxLayout()

        label = QLabel("Video Frame:")
        layout.addWidget(label)

        self.video_frame_label = QLabel()
        layout.addWidget(self.video_frame_label)

        self.video_frame_widget.setLayout(layout)
        self.target_server_widget.layout().addWidget(self.video_frame_widget)

    def send_command(self):
        command_str = self.command_entry.text()
        hex_array = command_str.split()
        if len(hex_array) > 6:
            QMessageBox.critical(None, "Error", "Maximum length of command is 6 digits.")
            return
        try:
            bytes.fromhex("".join(hex_array))
        except ValueError:
            QMessageBox.critical(None, "Error", "Invalid hex format.")
            return
        self.client.send_command(command_str)

    def process_received_packets(self, command_buffer):
        # Process the received command buffer
        result = self.client.camera.process_command(command_buffer)

        # Append result to the received content text edit widget
        if self.client.received_content_text:
            self.client.received_content_text.append(result)

        # Display the received video frame
        if result.startswith("Capture video command received."):
            img_path = "received_frame.jpg"
            pixmap = QPixmap(img_path)
            self.video_frame_label.setPixmap(pixmap)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = UDPClientGUI()
    window.show()
    sys.exit(app.exec_())
