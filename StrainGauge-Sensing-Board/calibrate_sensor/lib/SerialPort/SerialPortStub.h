/***********************************************************************
* SerialPortStub.h
* Copyright (C) 2024 Satoshi Inoue
* Copyright (C) 2024 CIT Brains
* ~~~~~~~~
* This Source Code Form is subject to the terms of the Mozilla Public
* License, v. 2.0. If a copy of the MPL was not distributed with this
* file, You can obtain one at http://mozilla.org/MPL/2.0/.
***********************************************************************/
#ifndef SERIALPORTSTUB_H_
#define SERIALPORTSTUB_H_
#include <fstream>
#include <iostream>
#include <string>
#include <ranges>
#include <boost/system/error_code.hpp>
#include <functional>

#ifdef ENABLE_SERVOMOTOR_STUB_DISPOSING_OUTPUT
constexpr bool enable_file_output = false;
#else
constexpr bool enable_file_output = true;
#endif

namespace citbrains
{
	class SerialPortStub
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

		SerialPortStub()
		{
		}
		~SerialPortStub()
		{
			close();
		}

		boost::system::error_code open(const std::string port_name)
		{
			file_name_ = "packet_data_" + port_name.substr(port_name.find_last_of("/") + 1);
			file_name_ += ".txt";
			std::cout << "Writing data to " << file_name_ << std::endl;
			ofs_.open(file_name_, std::ofstream::out | std::ofstream::trunc);
			if (!ofs_.is_open())
			{
				return boost::system::errc::make_error_code(boost::system::errc::errc_t::bad_file_descriptor); // return true if open failed
			}
			return boost::system::error_code(); // エラーが無い時。operator() boolでfalseになる。
		}

		void close()
		{
			if (ofs_.is_open())
			{
				ofs_.close();
				std::cout << "Close ofstream of " << file_name_ << std::endl;
			}
			return;
		}

		void setBaudRate([[maybe_unused]] const int32_t &dummy)
		{
			return;
		}

		void setStopbits([[maybe_unused]] const Stopbits dummy)
		{
			return;
		}

		void setParity([[maybe_unused]] const Parity dummy)
		{
			return;
		}

		size_t read(uint8_t *read_buffer, const size_t &read_length, boost::system::error_code &ec, int32_t wait_microseconds = 5000, int64_t total_wait_timelimit_us = 80000)
		{
			if (!writeToReturnPacket)
			{
				throw std::runtime_error("[SerialPortStub::read()] writeToReturnPacket is not set.");
			}
			return writeToReturnPacket(read_length, read_buffer);
		}

		template <size_t buff_size>
		size_t readSome(const std::array<uint8_t, buff_size> &read_buffer, const size_t &read_length, boost::system::error_code &ec)
		{
			if (!writeToReturnPacket)
			{
				throw std::runtime_error("[SerialPortStub::read()] writeToReturnPacket is not set.");
			}
			return writeToReturnPacket(read_length, read_buffer.data());
		}

		size_t writeSome(const std::vector<uint8_t> &write_buffer, boost::system::error_code &ec)
		{
			if (!ofs_.is_open())
			{
				throw std::runtime_error("Error::write_some called before port opened");
				return -1;
			}
			if (enable_file_output)
			{
				for (auto itr : write_buffer)
				{
					uint8_t num = itr;
					ofs_ << std::hex << static_cast<uint16_t>(num) << " ";
				}
				ofs_ << std::endl;
			}
			// writeが失敗したことにするか成功した事にするか
			if (enfoce_writing_fail)
			{
				return write_buffer.size() - 1; // 失敗
			}
			else
			{
				return write_buffer.size(); // 成功
			}
		}

		template <size_t buff_size>
		size_t writeSome(const std::array<uint8_t, buff_size> &write_buffer, const size_t &len, [[maybe_unused]] boost::system::error_code &ec)
		{
			if (!ofs_.is_open())
			{
				throw std::runtime_error("Error::write_some called before port opened");
				return -1;
			}
			if (enable_file_output)
			{
				for (auto itr : write_buffer)
				{
					uint8_t num = itr;
					ofs_ << std::hex << static_cast<uint16_t>(num) << " ";
				}
				ofs_ << std::endl;
			}
			// writeが失敗したことにするか成功した事にするか
			if (enfoce_writing_fail)
			{
				return len - 1; // 失敗
			}
			else
			{
				return len; // 成功
			}
		}

		std::string getFileName()
		{
			return file_name_;
		}

		std::ofstream &getOfstreamRef()
		{
			return ofs_;
		}

		void setEnforceFailWriting(bool is_fail)
		{
			enfoce_writing_fail = is_fail;
		}

		void setReturnPacketWriter(std::function<size_t(const size_t ,uint8_t *)> writer)
		{
			writeToReturnPacket = writer;
		}

	private:
		std::ofstream ofs_;
		std::string file_name_;
		bool enfoce_writing_fail = false;
		std::function<size_t(const size_t &,uint8_t *)> writeToReturnPacket;
	};
}

#endif // !SERIALPORTSTUB_H_