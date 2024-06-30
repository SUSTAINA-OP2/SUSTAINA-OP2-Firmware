/***********************************************************************
* SerialPortTest.cpp
* Copyright (C) 2024 Satoshi Inoue
* Copyright (C) 2024 CIT Brains
* ~~~~~~~~
* This Source Code Form is subject to the terms of the Mozilla Public
* License, v. 2.0. If a copy of the MPL was not distributed with this
* file, You can obtain one at http://mozilla.org/MPL/2.0/.
***********************************************************************/
#include "SerialPort.h"
#include "return_packet_factory_example.h"

int main(int argc, char const *argv[])
{
    using namespace citbrains;
#ifdef USE_CITBRAINS_SERIALPORT_STUB
    ReturnPacketFactory fact;
    fact.setReadPacketFile("test.txt");
    SerialPort serial_port;
    auto packet_factory = [&fact](const size_t &packet_size, uint8_t *buffers) {
        return fact.writeToReturnPacket(packet_size, buffers);
    };
    serial_port.setReturnPacketWriter(packet_factory);
#else
    SerialPort serial_port;
#endif
    return 0;
}
