#!/usr/bin/python3
import serial
import time
import argparse

from CRC16 import CRC16 as CRC

DEFAULT_ID = 0xA1
DEFAULT_CMD = 0xA0
DEFAULT_OPT = 0x00
txData = [0b00100100, 0b00001001]

# RX packet: header + (id + length + command + option) + rxData + crc
# TX packet: header + (id + length + command(mask) + error) + txData + crc

BAUDRATE = 1000000
header = [0xFE, 0xFE]
cheackFirmwareCommand = 0xD0

ser = serial.Serial('/dev/ttyUSB0', BAUDRATE)
rxWaitTimeout = 1/10
ser.timeout = 1/10

RED = "\033[31m"
GREEN = "\033[32m"
RESET = "\033[0m"

def print_colored(color, *args, **kwargs):
    print(color + ' '.join(str(arg) for arg in args) + RESET, **kwargs)

try:
    parser = argparse.ArgumentParser(description='Command and Loop Example')
    parser.add_argument('--loop', type=int, default=None, help='Number of times to loop')
    parser.add_argument('--id', type=lambda x: int(x, 16), default=DEFAULT_ID, help='Id to execute in hexadecimal')
    parser.add_argument('--cmd', type=lambda x: int(x, 16), default=DEFAULT_CMD, help='Command to execute in hexadecimal')
    parser.add_argument('--opt', type=lambda x: int(x, 16), default=DEFAULT_OPT, help='Options to execute in hexadecimal')
    args = parser.parse_args()

    loop_count = args.loop

    id = args.id
    command = args.cmd
    option = args.opt
    
    sendCount = 0
    receivedCount = 0
    correctedCount = 0

    cheack_crc = 0

    while True:
        ser.reset_input_buffer()

        txBuf = []
        txBuf.extend(header)
        txBuf.append(id)

        tx_length = len(header) + 6 + len(txData)
        txBuf.append(tx_length)

        txBuf.append(command)
        txBuf.append(option)
        txBuf.extend(txData)

        crc = CRC.getCrc16(txBuf)
        txBuf.extend(crc.to_bytes(2, byteorder="little"))

        txBuf_hex = ['{:02X}'.format(i) for i in txBuf]

        # send packet
        ser.write(bytes(txBuf))

        sendCount += 1

        print_colored(GREEN,"<----")
        print_colored(GREEN,"id               : ", txBuf_hex[2])
        print_colored(GREEN,"length           : ", int(txBuf_hex[3],16))
        print_colored(GREEN,"command          : ", txBuf_hex[4])
        print_colored(GREEN,"option           : ", txBuf_hex[5])
        txData_ = txBuf_hex[6:-2]
        print_colored(GREEN,"data             : ", txData_, [bin(int(x, 16)) for x in txData_])
        print_colored(GREEN,"crc              : ", crc)
        print_colored(GREEN,"txBuf            : ", txBuf_hex)
        print_colored(GREEN,"<----")

        start_time = time.time()

        print_colored(RED,"---->")

        while True:

            received_header = ser.read(len(header))

            if received_header == bytes(header):

                receivedCount += 1

                rxBuf = []
                rxBuf.extend(header)
                rxBuf.extend(list(ser.read(2)))

                received_length = int(rxBuf[3])

                rxBuf.extend(list(ser.read(received_length-4)))

                rxBuf_hex = ['{:02X}'.format(i) for i in rxBuf]

                print_colored(RED,"id               : ", rxBuf_hex[2])
                print_colored(RED,"length           : ", received_length)
                print_colored(RED,"command(mask)    : ", rxBuf_hex[4])
                print_colored(RED,"error            : ", format(int(rxBuf[5]),"#010b"))

                rxData = rxBuf_hex[6:-2]
                print_colored(RED,"data             : ", rxData, [bin(int(x, 16)) for x in rxData], [chr(int(x, 16)) for x in rxData])

                crc16 = int.from_bytes(bytes(rxBuf[-2:]), byteorder="little")
                print_colored(RED,"crc              : ", crc16)

                cheack_crc = CRC.checkCrc16(rxBuf)
                print_colored(RED,"check crc        : ", cheack_crc )
                
                print_colored(RED,"rxBuf            : ", rxBuf_hex)

                if not cheack_crc:
                    break

                correctedCount += 1

                if hex(command) == hex(cheackFirmwareCommand) :
                    firmwareVersion = rxBuf_hex[6]
                    print_colored(RED,"firmware ver.    : ", firmwareVersion )
                    break

            if time.time() - start_time > rxWaitTimeout:
                break

        print_colored(RED,"---->")
        print("tx       : ", sendCount)
        print("rx       : ", receivedCount)
        # print("correct  : ", correctedCount)

        if sendCount != 0:
            print("unanswered error : ", (1.00-(correctedCount/sendCount))*100, "%")

        if receivedCount != 0:
            print("rx error         : ", (receivedCount - correctedCount)/receivedCount*100, "%")

        print("\n")

        if loop_count is not None:
            loop_count -= 1
            if loop_count <= 0:
                exit()

except KeyboardInterrupt:
    print("Program stopped.")

finally:
    ser.close()
