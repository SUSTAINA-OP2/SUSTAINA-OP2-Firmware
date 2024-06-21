import serial
import sys
import struct
import time
from PyQt5.QtWidgets import QApplication, QMainWindow, QTextEdit, QLineEdit, QLabel, QPushButton
from PyQt5.QtCore import Qt
from PyQt5.QtCore import QThread, pyqtSignal
from PyQt5.QtGui import QFont
# import matplotlib.pyplot as plt # グラフ作成のため

IMU_DATA_BYTE_SIZE = 44
IMU_PACKET_BYTE_SIZE = IMU_DATA_BYTE_SIZE * 10 + 2 + 3 + 2


class ImuData:
    def __init__(self):
        self.temperature = 0

        self.acc_x = 0
        self.acc_y = 0
        self.acc_z = 0

        self.gyro_x = 0
        self.gyro_y = 0
        self.gyro_z = 0

        self.quat_w = 0
        self.quat_x = 0
        self.quat_y = 0
        self.quat_z = 0
    def __str__(self):
        return f"temperature: {self.temperature}, acc_x: {self.acc_x}, acc_y: {self.acc_y}, acc_z: {self.acc_z}, gyro_x: {self.gyro_x}, gyro_y: {self.gyro_y}, gyro_z: {self.gyro_z}"

    def __repr__(self):
        return self.__str__()

    def parseFromBytes(self, data: bytes):
        self.acc_x = struct.unpack('<f', data[0:4])[0]
        self.acc_y = struct.unpack('<f', data[4:8])[0]
        self.acc_z = struct.unpack('<f', data[8:12])[0]
        self.gyro_x = struct.unpack('<f', data[12:16])[0]
        self.gyro_y = struct.unpack('<f', data[16:20])[0]
        self.gyro_z = struct.unpack('<f', data[20:24])[0]
        self.quat_w = struct.unpack('<f', data[24:28])[0]
        self.quat_x = struct.unpack('<f', data[28:32])[0]
        self.quat_y = struct.unpack('<f', data[32:36])[0]
        self.quat_z = struct.unpack('<f', data[36:40])[0]
        self.temperature = struct.unpack('<f', data[40:44])[0]

def parseImuData(data: bytes,data_size: int):
    if data_size * IMU_DATA_BYTE_SIZE != len(data):
        print("[Failed to parse]. Bytearray size {} != data_size {}".format(len(data),data_size))
        return None
    data_list = list()
    while len(data) >= IMU_DATA_BYTE_SIZE:
        imu_data = ImuData()
        imu_data.parseFromBytes(data[:IMU_DATA_BYTE_SIZE])
        if (abs(imu_data.temperature) > 0.0001) and (abs(imu_data.acc_x) > 0.0001) and (abs(imu_data.gyro_y) > 0.0001) and (abs(imu_data.quat_w) > 0.0001) :
            data_list.append(imu_data)
            # 0が詰まっているデータは無効とする
        data = data[IMU_DATA_BYTE_SIZE:]
    return data_list

class ImuCommunicater:
    def __init__(self, port: str):
        self.ser = serial.Serial(port, 2000000)
        self.ser.stopbits = serial.STOPBITS_ONE
        self.ser.parity = serial.PARITY_NONE
        print(f"Opening serial port: {port}\n Baudrate: 2000000")
        while True:
            self.ser.timeout = 1  # Set timeout to 1 second
            data = self.ser.read(64)  # Throw away initial output
            print(data)
            if not data:
                break

    def readData(self):
        if self.sendRequest() != 1:
            print("Failed to send request")
            return None
        read_size = 0
        read_data = b""
        while True:
            read_data = read_data + self.ser.read(IMU_PACKET_BYTE_SIZE - read_size)
            read_size += len(read_data)
            if read_size >= IMU_PACKET_BYTE_SIZE:
                break
        if len(read_data) != IMU_PACKET_BYTE_SIZE:
            print("[Failed to read]. Data size = {} is not correct".format(len(read_data)))
            return None
        if read_data[0] != 0xfe or read_data[1] != 0xfe:
            print("[Failed to read]. Data header is not correct. Header: {}".format(read_data[0:2]))
            return None
        return_data_size = read_data[4]
        if return_data_size != 10:
            print("return_data_size = {} is not correct".format(return_data_size))
        self.parseError(read_data[3:4])
        data_list = parseImuData(read_data[5:-2], return_data_size)
        data_str = ""
        if data_list is not None:
            for data in data_list:
                data_str += str(data)
        return data_list,read_data

    def parseError(self, data: bytes):
        if len(data) != 2:
            return None
        error_code = int.from_bytes(data[0], "little")
        if error_code != 0:
            bit_representation = bin(error_code)
            print(f"Error code[0] in binary: {bit_representation}")
        error_code = int.from_bytes(data[1], "little")
        if error_code != 0:
            bit_representation = bin(error_code)
            print(f"Error code[1] in binary: {bit_representation}")
        return

    def close(self):
        self.ser.close()
    
    def sendRequest(self):
        return self.ser.write(b"K")

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.graph_count = 0
        self.setWindowTitle("IMU Data Viewer")
        self.setGeometry(100, 100, 1000, 600)

        self.label_acc_x = QLabel("Acc X:", self)
        self.label_acc_x.setGeometry(50, 50, 100, 30)
        self.label_acc_x.setAlignment(Qt.AlignLeft)

        self.line_edit_acc_x = QLineEdit(self)
        self.line_edit_acc_x.setGeometry(100, 50, 250, 30)

        self.label_acc_y = QLabel("Acc Y:", self)
        self.label_acc_y.setGeometry(50, 100, 100, 30)
        self.label_acc_y.setAlignment(Qt.AlignLeft)

        self.line_edit_acc_y = QLineEdit(self)
        self.line_edit_acc_y.setGeometry(100, 100, 250, 30)

        self.label_acc_z = QLabel("Acc Z:", self)
        self.label_acc_z.setGeometry(50, 150, 100, 30)
        self.label_acc_z.setAlignment(Qt.AlignLeft)

        self.line_edit_acc_z = QLineEdit(self)
        self.line_edit_acc_z.setGeometry(100, 150, 250, 30)

        self.label_gyro_x = QLabel("Gyro X:", self)
        self.label_gyro_x.setGeometry(50, 200, 100, 30)
        self.label_gyro_x.setAlignment(Qt.AlignLeft)

        self.line_edit_gyro_x = QLineEdit(self)
        self.line_edit_gyro_x.setGeometry(100, 200, 250, 30)

        self.label_gyro_y = QLabel("Gyro Y:", self)
        self.label_gyro_y.setGeometry(50, 250, 100, 30)
        self.label_gyro_y.setAlignment(Qt.AlignLeft)

        self.line_edit_gyro_y = QLineEdit(self)
        self.line_edit_gyro_y.setGeometry(100, 250, 250, 30)

        self.label_gyro_z = QLabel("Gyro Z:", self)
        self.label_gyro_z.setGeometry(50, 300, 100, 30)
        self.label_gyro_z.setAlignment(Qt.AlignLeft)

        self.line_edit_gyro_z = QLineEdit(self)
        self.line_edit_gyro_z.setGeometry(100, 300, 250, 30)

        self.label_quat_w = QLabel("Quat W:", self)
        self.label_quat_w.setGeometry(50, 350, 100, 30)
        self.label_quat_w.setAlignment(Qt.AlignLeft)

        self.line_edit_quat_w = QLineEdit(self)
        self.line_edit_quat_w.setGeometry(100, 350, 250, 30)

        self.label_quat_x = QLabel("Quat X:", self)
        self.label_quat_x.setGeometry(50, 400, 100, 30)
        self.label_quat_x.setAlignment(Qt.AlignLeft)

        self.line_edit_quat_x = QLineEdit(self)
        self.line_edit_quat_x.setGeometry(100, 400, 250, 30)

        self.label_quat_y = QLabel("Quat Y:", self)
        self.label_quat_y.setGeometry(50, 450, 100, 30)
        self.label_quat_y.setAlignment(Qt.AlignLeft)

        self.line_edit_quat_y = QLineEdit(self)
        self.line_edit_quat_y.setGeometry(100, 450, 250, 30)

        self.label_quat_z = QLabel("Quat Z:", self)
        self.label_quat_z.setGeometry(50, 500, 100, 30)
        self.label_quat_z.setAlignment(Qt.AlignLeft)

        self.line_edit_quat_z = QLineEdit(self)
        self.line_edit_quat_z.setGeometry(100, 500, 250, 30)

        self.label_temperature = QLabel("Temp :", self)
        self.label_temperature.setGeometry(50, 550, 100, 30)
        self.label_temperature.setAlignment(Qt.AlignLeft)
        
        self.line_edit_temperature = QLineEdit(self)
        self.line_edit_temperature.setGeometry(100, 550, 250, 30)

        self.bytes_text_edit = QTextEdit(self)
        self.bytes_text_edit.setGeometry(400, 50, 500, 500)
        font = QFont("Courier", 6)
        self.bytes_text_edit.setFont(font)

        self.stop_button = QPushButton("Pause/Restart", self)
        self.stop_button.setGeometry(400, 560, 100, 30)
        self.stop_button.clicked.connect(self.stopUpdatingData)

        self.update_pause_flag = False
        # self.figure,ax = plt.subplots(figsize=(12.8, 7.2))
        # ax.set_xlabel('Time')
        # ax.set_ylabel('Data')
        # ax.set_title('XYZ Data')
        # self.x_data = []
        # self.y_data = []
        # self.z_data = []

        # シリアルポート
        if len(sys.argv) > 1 and sys.argv[1]:
            port = sys.argv[1]
        else:
            raise ValueError("Please specify the serial port !\n Usage: python check_imu.py [comport name]")
        self.comm = ImuCommunicater(port)
        self.worker = Worker(self.comm)
        self.worker.data_signal.connect(self.updateData)
        self.worker.start()

    def stopUpdatingData(self):
        self.update_pause_flag = not self.update_pause_flag

    def updateData(self, data):
        if self.update_pause_flag:
            return
        d = data[0][-1]
        self.line_edit_acc_x.setText(str(d.acc_x))
        self.line_edit_acc_y.setText(str(d.acc_y))
        self.line_edit_acc_z.setText(str(d.acc_z))
        self.line_edit_gyro_x.setText(str(d.gyro_x))
        self.line_edit_gyro_y.setText(str(d.gyro_y))
        self.line_edit_gyro_z.setText(str(d.gyro_z))
        self.line_edit_quat_w.setText(str(d.quat_w))
        self.line_edit_quat_x.setText(str(d.quat_x))
        self.line_edit_quat_y.setText(str(d.quat_y))
        self.line_edit_quat_z.setText(str(d.quat_z))
        self.line_edit_temperature.setText(str(d.temperature))

        index = 0
        data_str = ''
        for d in data[1]:
            if index % 16 == 0:
                data_str += '\n{:04d} :: '.format(index)
            data_str += '{:02X}'.format(d) + ' '
            index += 1
        # data_str = ' '.join([f'{byte:02X}' for byte in data[1]])
        formatted_data_str = data_str
        # formatted_data_str = '\n'.join([f'{data_str[i:i+32]}' for i in range(0, len(data_str), 16)])
        self.bytes_text_edit.setText(formatted_data_str)

        # self.graph_count += 1
        # if self.graph_count % 1 == 0:
        #     # Update graph
        #     self.x_data += [data.acc_x for data in data[0]]
        #     self.y_data += [data.acc_y for data in data[0]]
        #     self.z_data += [data.acc_z for data in data[0]]
        #     if len(self.x_data) > 100:
        #         self.x_data = self.x_data[-100:]
        #     if len(self.y_data) > 100:
        #         self.y_data = self.y_data[-100:]
        #     if len(self.z_data) > 100:
        #         self.z_data = self.z_data[-100:]
            
        #     plt.plot(self.x_data, label='X')
        #     plt.plot(self.y_data, label='Y')
        #     plt.plot(self.z_data, label='Z')
        #     plt.pause(0.004)  # Pause for a short time to update the graph
        #     print("Graph updated")


class Worker(QThread):
    data_signal = pyqtSignal(list)

    def __init__(self, comm):
        super().__init__()
        self.comm = comm

    def run(self):
        start_time = time.time()
        received_data = 0
        print("Worker started...")
        while True:
            data_list,data_packet = self.comm.readData()
            if data_list is None:
                continue
            received_data += len(data_list)
            elapsed_time = time.time() - start_time
            if elapsed_time >= 0.1:
                self.data_signal.emit([data_list,data_packet])
                frequency = received_data / elapsed_time
                print(f"Frequency: {frequency} Hz")
                start_time = time.time()
                received_data = 0


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())
