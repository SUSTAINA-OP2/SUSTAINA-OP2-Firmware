/***********************************************************************
* return_packet_factory_example.h
* Copyright (C) 2024 Satoshi Inoue
* Copyright (C) 2024 CIT Brains
* ~~~~~~~~
* This Source Code Form is subject to the terms of the Mozilla Public
* License, v. 2.0. If a copy of the MPL was not distributed with this
* file, You can obtain one at http://mozilla.org/MPL/2.0/.
***********************************************************************/

#ifndef RETURN_PACKET_FACTORY_H_
#define RETURN_PACKET_FACTORY_H_
#include <fstream>
#include <string>
#include <ranges>

namespace citbrains
{
    // デフォルトのファクトリクラス
    class ReturnPacketFactory
    {
    public:
        ReturnPacketFactory(){};
        ReturnPacketFactory(const ReturnPacketFactory& other) = default;
        ReturnPacketFactory(ReturnPacketFactory&& other) = default;
        ReturnPacketFactory& operator=(const ReturnPacketFactory& other) = default;
        ReturnPacketFactory& operator=(ReturnPacketFactory&& other) = default;
        ~ReturnPacketFactory()
        {
            if (return_packet_ifs_.is_open())
            {
                return_packet_ifs_.close();
            }
        }
        /**
         * @brief read命令において、返ってくるパケットの内容を設定する
         *
         * @param packet_size 書き込むパケットのサイズ
         * @param buffers バッファのポインタ
         */
        size_t writeToReturnPacket(const size_t &packet_size, uint8_t *buffers)
        {
            // 読み込みファイルが設定されている場合は読み込む
            if (return_packet_ifs_.is_open())
            {
                getReturnPacketFromFile(packet_size, buffers);
                return packet_size;
            }
            // 読み込みファイルが設定されていない場合は生成する
            for (size_t i = 0; i < packet_size - 1; ++i)
            {
                buffers[i] = i;
            }
            return packet_size;
        }

        /**
         * @brief readを読んだ際に返すパケットが書かれたファイルを設定する
         * @param filename ファイルパス
         */
        void setReadPacketFile(const std::string filename)
        {
            return_packet_ifs_.open(filename);
            file_name_ = filename;
            if (!return_packet_ifs_.is_open())
            {
                throw std::runtime_error("Error::setReadPacketFile failed to open file. Filename -> " + filename);
            }
            return;
        }

    private:
        void getReturnPacketFromFile(const size_t &packet_size, uint8_t *buffers)
        {
            if (!return_packet_ifs_.is_open())
            {
                throw std::runtime_error("Error:: getReturnPacketFromFile failed to open file. Filename -> " + file_name_);
            }
            std::string read_packet;
            std::getline(return_packet_ifs_, read_packet);
            size_t cnt = 0;
            for (const auto &itr : read_packet | std::ranges::views::split(','))
            {
                if (cnt >= packet_size)
                {
                    return;
                }
                buffers[cnt] = std::stoi(std::string(itr.begin(), itr.end()), nullptr, 16);
                cnt++;
            }
            return;
        }
        std::ifstream return_packet_ifs_;
        std::string file_name_;
    };

};

#endif // RETURN_PACKET_FACTORY_H_