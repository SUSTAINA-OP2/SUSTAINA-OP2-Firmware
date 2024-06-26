/***********************************************************************
* SerialPort.h
* Copyright (C) 2024 Satoshi Inoue
* Copyright (C) 2024 CIT Brains
* ~~~~~~~~
* This Source Code Form is subject to the terms of the Mozilla Public
* License, v. 2.0. If a copy of the MPL was not distributed with this
* file, You can obtain one at http://mozilla.org/MPL/2.0/.
***********************************************************************/

#ifndef SERIALPORT_H_
#define SERIALPORT_H_

#ifndef USE_CITBRAINS_SERIALPORT_STUB
#include "RealSerialPort.h"
namespace citbrains
{
    using SerialPort = RealSerialPort;
    static constexpr bool SERIALPORT_STUB_ENABLED = false;
}
#else
#include "SerialPortStub.h"
namespace citbrains
{
    using SerialPort = SerialPortStub;
    static constexpr bool SERIALPORT_STUB_ENABLED = true;
}
#endif

#endif // !SERIALPORT_H_