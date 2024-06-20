/***********************************************************************
* crc16.cpp
* Copyright (C) 2024 Satoshi Inoue
* ~~~~~~~~
* This Source Code Form is subject to the terms of the Mozilla Public
* License, v. 2.0. If a copy of the MPL was not distributed with this
* file, You can obtain one at http://mozilla.org/MPL/2.0/.
***********************************************************************/
#include "crc16.hpp"

uint_fast16_t bitRLReverse16(const uint_fast16_t &bit)
{
    uint_fast32_t tmp = bit;
    tmp = ((tmp & 0xaaaaaaaa) >> 1) | ((tmp & 0x55555555) << 1);
    tmp = ((tmp & 0xcccccccc) >> 2) | ((tmp & 0x33333333) << 2);
    tmp = ((tmp & 0xf0f0f0f0) >> 4) | ((tmp & 0x0f0f0f0f) << 4);
    tmp = ((tmp & 0xff00ff00) >> 8) | ((tmp & 0x00ff00ff) << 8);
    tmp = (tmp >> 16) | (tmp << 16);
    uint_fast16_t result = (tmp >> 16);
    return result;
}

uint_fast8_t bitRLReverse8(const uint_fast8_t &bit)
{
    uint_fast32_t tmp = bit;
    tmp = ((tmp & 0xaaaaaaaa) >> 1) | ((tmp & 0x55555555) << 1);
    tmp = ((tmp & 0xcccccccc) >> 2) | ((tmp & 0x33333333) << 2);
    tmp = ((tmp & 0xf0f0f0f0) >> 4) | ((tmp & 0x0f0f0f0f) << 4);
    tmp = ((tmp & 0xff00ff00) >> 8) | ((tmp & 0x00ff00ff) << 8);
    tmp = (tmp >> 16) | (tmp << 16);
    uint_fast8_t result = (tmp >> 24);
    return result;
}


uint_fast16_t calcCRC16_XMODEM(const uint8_t* packet, const size_t &calc_length)
{
    uint_fast16_t crc = 0;
    const uint8_t* itr = packet;
    size_t read_loc = 0;
    while (read_loc < calc_length)
    {
        uint_fast8_t packet_data = bitRLReverse8(*itr);
        crc = (crc >> 8) ^ crctable[0xff & (crc ^ packet_data)];
        itr++;
        read_loc++;
    }
    crc ^= 0x0000;
    return bitRLReverse16(crc);

}

uint_fast16_t calcCRC16_XMODEM(const std::vector<uint_fast8_t> &packet, const size_t &calc_length)
{
    uint_fast16_t crc = 0;
    auto itr = packet.begin();
    size_t read_loc = 0;
    while (read_loc < calc_length)
    {
        uint_fast8_t packet_data = bitRLReverse8(*itr);
        crc = (crc >> 8) ^ crctable[0xff & (crc ^ packet_data)];
        itr++;
        read_loc++;
    }
    crc ^= 0x0000;
    return bitRLReverse16(crc);
}

uint8_t getLowByte(const uint_fast16_t &data){
    return (data & 0xFF);
}

uint8_t getHighByte(const uint_fast16_t &data){
    return (data >> 8) & 0xFF;
}
