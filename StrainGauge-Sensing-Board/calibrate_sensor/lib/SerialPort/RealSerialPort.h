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

#ifndef REALSERIALPORT_H_
#define REALSERIALPORT_H_

#include <boost/utility.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/asio/high_resolution_timer.hpp>
#include <boost/asio/buffer.hpp>
#include <boost/asio/async_result.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/read.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <atomic>
#include <array>
#include <fstream>
namespace citbrains
{
	class RealSerialPort
	{
	public:
		enum class Parity
		{
			None,
			Even,
			Odd
		};
		enum class Stopbits
		{
			One,
			Two
		};

		RealSerialPort();
		RealSerialPort(const RealSerialPort &) = delete;
		RealSerialPort &operator=(const RealSerialPort &) = delete;
		RealSerialPort& operator=(RealSerialPort&& other) = delete;
		RealSerialPort(RealSerialPort&& other) = default; 
		~RealSerialPort();

		/**
		 * @brief シリアルポートを開く
		 *
		 * @param portname ポート名
		 * @return boost::system::error_code エラーコード
		 */
		boost::system::error_code open(const std::string portname);

		/**
		 * @brief シリアルポートを閉じる
		 */
		void close();
		
		/**
		 * @brief シリアルポートの通信パケットを記録開始する。
		 * 
		 * @param filename 記録したパケット情報を保存するファイル
		 */
		void startRecord(const std::string filename);

		/**
		 * @brief ボーレートを指定する
		 */
		void setBaudRate(const int32_t &rate);

		/**
		 * @brief ストップビットを指定する
		 */
		void setStopbits(const Stopbits stopbits_setting);

		/**
		 * @brief パリティを指定する
		 */
		void setParity(const Parity parity_setting);

		/**
		 * @brief シリアルポートからのreadを行う。一度のreadで読み込みに失敗した場合、設定したタイムアウトの時間に達するまで、readを試みる。
		 *
		 * @param read_buffer readした結果を格納するバッファ
		 * @param read_length 読みたいバイト数
		 * @param ec エラーコードが格納される
		 * @param wait_microseconds 内部で繰り返し実行される可能性があるboost::serial_port::async_read1回に対してのタイムアウト時間
		 * @param total_wait_timelimit_us 本関数の実行時間に対するタイムアウト時間
		 * @return int readに成功したバイト数
		 */
		size_t read(uint8_t *read_buffer, const size_t &read_length, boost::system::error_code &ec, int32_t wait_microseconds = 5000, int64_t total_wait_timelimit_us = 80000);

		/**
		 * @brief シリアルポートから指定したバイト数データを読み込む
		 * 
		 * @tparam buff_size 読み込みに利用するバッファのサイズ
		 * @param read_buffer 読み込みバッファ
		 * @param read_length 読み込みバイト数
		 * @param ec エラーコード
		 * @return size_t 実際に読み込んだデータのバイト数
		 */
		template <size_t buff_size>
		size_t readSome(std::array<uint8_t, buff_size> &read_buffer, const size_t &read_length, boost::system::error_code &ec)
		{
			const size_t read_size = serial_port_.read_some(boost::asio::buffer(read_buffer, read_length), ec);
			if (packet_recorder_.is_open())
			{
				packet_recorder_ << "[read] ";
				for (const uint8_t& num  : read_buffer)
				{
					packet_recorder_ << std::hex << num << " ";
				}
				packet_recorder_ << std::endl;
			}
			return read_size;
		}

		/**
		 * @brief シリアルポートに指定したバッファのデータを書き込む
		 * 
		 * @param write_buffer 書き込みデータが入ったバッファ
		 * @param ec エラーコード
		 * @return size_t 実際に書き込んだバイト数
		 */
		size_t writeSome(const std::vector<uint8_t> &write_buffer, boost::system::error_code &ec);

		/**
		 * @brief シリアルポートに指定したバッファのデータを、指定したバイト数分書き込む
		 * 
		 * @tparam buff_size バッファのサイズ
		 * @param write_buffer 書き込み用バッファ
		 * @param len 書き込むバイト数
		 * @param ec エラーコード
		 * @return size_t 書き込み成功したバイト数
		 */
		template <size_t buff_size>
		size_t writeSome(const std::array<uint8_t, buff_size> &write_buffer, const size_t &len, boost::system::error_code &ec)
		{
			if (is_terminated_)
			{
				return 0;
			}
			const size_t write_size = serial_port_.write_some(boost::asio::buffer(write_buffer, len), ec);
			if (packet_recorder_.is_open())
			{
				packet_recorder_ << "[write] ";
				for (const uint8_t& num : write_buffer)
				{
					packet_recorder_ << std::hex << num << " ";
				}
				packet_recorder_ << std::endl;
			}
			return write_size;
		}

	private:
		void waitUntilIOcontextStopped(boost::asio::io_context &io_ctx, boost::asio::serial_port &port, boost::asio::high_resolution_timer &timer);
		boost::asio::io_context io_ctx_;
		boost::asio::serial_port serial_port_;
		volatile std::atomic_bool is_terminated_;
		bool is_port_opened_;
		std::ofstream packet_recorder_;
		std::string record_filename_;
	};
}

#endif // REALSERIALPORT_H_
