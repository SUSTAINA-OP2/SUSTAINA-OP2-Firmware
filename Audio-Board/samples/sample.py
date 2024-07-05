#!/usr/bin/python3
import serial
import time
import argparse

from CRC16 import CRC16 as CRC

ser = serial.Serial('/dev/ttyUSB0', 1000000)
rxWaitTimeout = 2.5
ser.timeout = 1/10

# RX packet: headder + (id + length + command + option ) + rxData + crc
# TX packet: headder + (id + length + command(mask)  + error) + txData + crc

cheackFirmwareCommand = 0xD0

header = [0xFE, 0xFE]
txData = []

try:
    parser = argparse.ArgumentParser(description='Command and Loop Example')
    parser.add_argument('--loop', type=int, default=None, help='Number of times to loop')
    parser.add_argument('--id', type=str, default='0xA3', help='Id to execute')
    parser.add_argument('--cmd', type=str, default='0xA0', help='Command to execute')
    parser.add_argument('--opt', type=str, default='0x02', help='Options to execute')
    args = parser.parse_args()

    loop_count = args.loop

    id = int(args.id,16)
    command = int(args.cmd,16)
    option = int(args.opt,16)
    
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

        print("<----")
        print("id               : ", txBuf_hex[2])        
        print("length           : ", int(txBuf_hex[3],16))
        print("command          : ", txBuf_hex[4])
        print("option           : ", txBuf_hex[5])
        print("crc              : ", crc)
        print("txBuf            : ", txBuf_hex)
        print("<----")

        start_time = time.time()

        print("---->")

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

                print("id               : ", rxBuf_hex[2])
                print("length           : ", received_length)
                print("command(mask)    : ", rxBuf_hex[4])
                print("error            : ", format(int(rxBuf[5]),"#010b"))

                crc16 = int.from_bytes(bytes(rxBuf[-2:]), byteorder="little")
                print("crc              : ", crc16)

                cheack_crc = CRC.checkCrc16(rxBuf)
                print("check crc        : ", cheack_crc )
                
                print("rxBuf            : ", rxBuf_hex)

                if not cheack_crc:
                    break

                correctedCount += 1

                if hex(command) == hex(cheackFirmwareCommand) :
                    firmwareVersion = rxBuf_hex[6]
                    print("firmware ver.    : ", firmwareVersion )
                    break

            if time.time() - start_time > rxWaitTimeout:
                break

        print("---->")
        print("tx       : ", sendCount)
        print("rx       : ", receivedCount)

        if receivedCount != 0:
            print("error    : ", (receivedCount - correctedCount)/receivedCount*100, "%\n")
        else:
            print("\n")

        if loop_count is not None:
            loop_count -= 1
            if loop_count <= 0:
                exit()

except KeyboardInterrupt:
    print("Program stopped.")

finally:
    ser.close()
