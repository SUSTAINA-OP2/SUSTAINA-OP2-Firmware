/***********************************************************************
* SerialPortTest.cpp
* Copyright (C) 2024 Satoshi Inoue
* Copyright (C) 2024 CIT Brains
* This software is based on KSerialPort Copyright (C) 2012 Kiyoshi Irie
* ~~~~~~~~
* This Source Code Form is subject to the terms of the Mozilla Public
* License, v. 2.0. If a copy of the MPL was not distributed with this
* file, You can obtain one at http://mozilla.org/MPL/2.0/.
***********************************************************************/
#include "RealSerialPort.h"
#include <chrono>
#include <thread>

namespace citbrains
{
    RealSerialPort::RealSerialPort() : io_ctx_(), serial_port_(io_ctx_),
                                       is_terminated_(false), is_port_opened_(false)
    {
    }
    RealSerialPort::~RealSerialPort()
    {
        close();
    }

    boost::system::error_code RealSerialPort::open(const std::string portname)
    {
        boost::system::error_code err;
        serial_port_.open(portname, err);
        if (!err)
        {
            serial_port_.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
            is_port_opened_ = true;
            is_terminated_ = false;
        }
        else
        {
            is_port_opened_ = false;
        }
        return err;
    }

    void RealSerialPort::close()
    {
        if (is_port_opened_)
        {
            is_terminated_ = true;
            std::this_thread::sleep_for(std::chrono::milliseconds(50)); // 終了通知してから、他がそれを見てある程度止まる事を期待して待つ
            for (size_t i = 0; i < 5; i++)
            {
                serial_port_.cancel();
                std::this_thread::sleep_for(std::chrono::milliseconds(30));
            }
            serial_port_.close();
            io_ctx_.stop();
            is_port_opened_ = false;
        }
        if (packet_recorder_.is_open())
        {
            packet_recorder_.close();
        }
        return;
    }

    void RealSerialPort::startRecord(const std::string filename)
    {
        if (packet_recorder_.is_open())
        {
            return;
        }
        record_filename_ = filename;
        packet_recorder_.open(filename);
        return;
    }

    void RealSerialPort::setBaudRate(const int32_t &rate)
    {
        serial_port_.set_option(boost::asio::serial_port_base::baud_rate(rate));
    }

    void RealSerialPort::setStopbits(const Stopbits stopbits_setting)
    {
        using namespace boost::asio;
        if (stopbits_setting == Stopbits::Two)
        {
            serial_port_.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::two));
        }
        else
        {
            serial_port_.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));
        }
    }

    void RealSerialPort::setParity(const Parity parity_setting)
    {
        using namespace boost::asio;
        switch (parity_setting)
        {
        case Parity::None:
            serial_port_.set_option(serial_port_base::parity(serial_port_base::parity::none));
            break;
        case Parity::Even:
            serial_port_.set_option(serial_port_base::parity(serial_port_base::parity::even));
            break;
        case Parity::Odd:
            serial_port_.set_option(serial_port_base::parity(serial_port_base::parity::odd));
            break;
        }
    }

    void RealSerialPort::waitUntilIOcontextStopped(boost::asio::io_context &io_ctx, boost::asio::serial_port &port, boost::asio::high_resolution_timer &timer)
    {
        uint64_t sleep_time_ns = 20 * 1000;
        timespec tmp;
        tmp.tv_sec = 0;
        tmp.tv_nsec = sleep_time_ns;
        while (!io_ctx.stopped())
        {
            port.cancel();
            timer.cancel();
            clock_nanosleep(CLOCK_MONOTONIC, 0, &tmp, NULL);
        }
        return;
    }

    size_t RealSerialPort::read(uint8_t *read_buffer, const size_t &read_length, boost::system::error_code &ec, int32_t wait_microseconds, int64_t total_wait_timelimit_us)
    {
        size_t numread = 0;
        int64_t total_wait_time = 0;
        boost::asio::high_resolution_timer timeout(io_ctx_);
        std::atomic_uint32_t timeout_id = 0;
        uint_fast32_t num_of_executed_handler = 0; // debug用
        uint_fast32_t total_repeated = 0;          // debug用
        using namespace std::literals::chrono_literals;

        while (numread < read_length)
        {
            size_t async_numread = 0;
            if (is_terminated_)
            { // クローズされている場合は終了する
                break;
            }
            boost::posix_time::ptime now = boost::posix_time::microsec_clock::local_time();
            timeout.expires_after(std::chrono::microseconds(wait_microseconds));
            auto timer_callback = [this, &timeout_id, &ec, current_id = timeout_id.load()](const boost::system::error_code &error) mutable
            {
                if ((error != boost::asio::error::operation_aborted) && (current_id == timeout_id))
                {
                    // timeout
                    serial_port_.cancel();
                }
            };
            auto read_callback = [&timeout, &timeout_id, &async_numread, &ec](const boost::system::error_code &error, std::size_t bytes_transferred) mutable
            {
                ++timeout_id;
                timeout.cancel();
                if (!error)
                {
                    async_numread = bytes_transferred;
                }
                else
                {
                    ec = error;
                }
            };
            // ここのネストにある、非同期操作を要求する関数群はそれぞれでis_terminatedを判定した方が確実に終了できるので良い。
            if (!io_ctx_.stopped())
            { // io_context.stopped() == trueの場合は動かないので、 そうでない際にrunする。
                if (!is_terminated_)
                { // クローズされていない場合のみ操作を行う
                    timeout.async_wait(timer_callback);
                }
                if (!is_terminated_)
                {
                    boost::asio::async_read(serial_port_, boost::asio::buffer(&read_buffer[numread], read_length - numread), boost::asio::transfer_at_least(read_length - numread), read_callback);
                }
                if (!is_terminated_)
                { // クローズされていない場合のみrunする
                    num_of_executed_handler += io_ctx_.run();
                }
            }
            else
            {
                if (!is_terminated_)
                { // クローズされていない場合のみrestartする
                    io_ctx_.restart();
                }
                std::this_thread::sleep_for(10us);
            }
            total_repeated++;
            boost::posix_time::ptime end = boost::posix_time::microsec_clock::local_time();
            boost::posix_time::time_duration diff = end - now;
            total_wait_time += diff.total_microseconds();
            numread += async_numread;
            if (total_wait_time > total_wait_timelimit_us)
            {
                // 制限時間超過で終了
                break;
            }
            if (numread < read_length)
            {
                continue;
            }
        }
        timeout.cancel();
        serial_port_.cancel();
        if (!is_terminated_)
        { // クローズされている場合はrestartしてはいけない。
            io_ctx_.restart();
        }
        if (packet_recorder_.is_open())
        {
            packet_recorder_ << "[read] ";
            std::string packet(reinterpret_cast<char *>(read_buffer), numread);
            for (auto itr : packet)
            {
                uint8_t num = itr;
                packet_recorder_ << std::hex << num << " ";
            }
            packet_recorder_ << std::endl;
        }
        return numread;
    }

    size_t RealSerialPort::writeSome(const std::vector<uint8_t> &write_buffer, boost::system::error_code &ec)
    {
        if (is_terminated_)
        {
            return 0;
        }
        const size_t write_size = serial_port_.write_some(boost::asio::buffer(write_buffer, write_buffer.size()), ec);
        if (packet_recorder_.is_open())
        {
            packet_recorder_ << "[write] ";
            for (const uint8_t &num : write_buffer)
            {
                packet_recorder_ << std::hex << num << " ";
            }
            packet_recorder_ << std::endl;
        }
        return write_size;
    }

}
