#include <stdio.h>
#include <sys/time.h>
#include "./ftd2xx.h"

int main(int argc, char *argv[])
{
	FT_STATUS	ftStatus;
	FT_HANDLE	ftHandle0;
	int iport = 0;
	
	if(argc > 1) {
		sscanf(argv[1], "%d", &iport);
	}
	else {
		iport = 0;
	}
	printf("opening port %d\n", iport);
	
	FT_SetVIDPID(0x0403, 0x6011); //なんかこれやった方が良いっぽい雰囲気

	ftStatus = FT_Open(iport, &ftHandle0);
	
	if(ftStatus != FT_OK) {
		/* 
			This can fail if the ftdi_sio driver is loaded
		 	use lsmod to check this and rmmod ftdi_sio to remove
			also rmmod usbserial
		 */
		printf("FT_Open(%d) failed\n", iport);
		return 1;
	}

	printf("ftHandle0 = %p\n", ftHandle0);
	
	//書き込みデータの設定-----------------------------------------------------------
	FT_PROGRAM_DATA Write_Data = {
		0x00000000,				   // Header - must be 0x00000000
		0xFFFFFFFF,				   // Header - must be 0xffffffff
		0x00000004,				   // Header - FT_PROGRAM_DATA version
		0x0403,					   // VID
		0x6011,					   // PID
		"FTDI",					   // Manufacturer
		"FT",					   // Manufacturer ID
		"USB <-> Serial Converter", // Description
		"FT000001",				   // Serial Number     シリアルナンバーは適当で良いらしいので、全て同じこの値にした。
		100,					   // MaxPower
		1,						   // PnP      FT232Bのextensionのみに関連する設定と思われる
		0,						   // SelfPowered
		1,						   // RemoteWakeup
		1,						   // non-zero if Rev4 chip, zero otherwise FT232Bのみの設定と思われるが、取りあえずオン
		0,						   // non-zero if in endpoint is isochronous FT232Bのみの設定と思われるが、取りあえずオフ
		0,						   // non-zero if out endpoint is isochronous FT232Bのみの設定と思われるが、取りあえずオフ
		0,						   // non-zero if pull down enabled ここは無効とする
		1,						   // non-zero if serial number to be used 
		0,						   // non-zero if chip uses USBVersion ここはfalseにして良いらしい。
		0x0200,					   // BCD (0x0200 => USB2)
		//
		// FT2232C extensions (Enabled if Version = 1 or greater)
		//
		0,	 // non-zero if Rev5 chip, zero otherwise
		0,	 // non-zero if in endpoint is isochronous
		0,	 // non-zero if in endpoint is isochronous
		0,	 // non-zero if out endpoint is isochronous
		0,	 // non-zero if out endpoint is isochronous
		0,	 // non-zero if pull down enabled
		0,	 // non-zero if serial number to be used
		0,	 // non-zero if chip uses USBVersion
		0x0, // BCD (0x0200 => USB2)
		0,	 // non-zero if interface is high current
		0,	 // non-zero if interface is high current
		0,	 // non-zero if interface is 245 FIFO
		0,	 // non-zero if interface is 245 FIFO CPU target
		0,	 // non-zero if interface is Fast serial
		0,	 // non-zero if interface is to use VCP drivers
		0,	 // non-zero if interface is 245 FIFO
		0,	 // non-zero if interface is 245 FIFO CPU target
		0,	 // non-zero if interface is Fast serial
		0,	 // non-zero if interface is to use VCP drivers
		//
		// FT232R extensions (Enabled if Version = 2 or greater)
		//
		0, // Use External Oscillator
		0, // High Drive I/Os
		0, // Endpoint size
		0, // non-zero if pull down enabled
		0, // non-zero if serial number to be used
		0, // non-zero if invert TXD
		0, // non-zero if invert RXD
		0, // non-zero if invert RTS
		0, // non-zero if invert CTS
		0, // non-zero if invert DTR
		0, // non-zero if invert DSR
		0, // non-zero if invert DCD
		0, // non-zero if invert RI
		0, // Cbus Mux control
		0, // Cbus Mux control
		0, // Cbus Mux control
		0, // Cbus Mux control
		0, // Cbus Mux control
		0, // non-zero if using D2XX drivers
		//
		// Rev 7 (FT2232H) Extensions (Enabled if Version = 3 or greater)
		//
		0, // non-zero if pull down enabled
		0, // non-zero if serial number to be used
		0, // non-zero if AL pins have slow slew
		0, // non-zero if AL pins are Schmitt input
		0, // valid values are 4mA, 8mA, 12mA, 16mA
		0, // non-zero if AH pins have slow slew
		0, // non-zero if AH pins are Schmitt input
		0, // valid values are 4mA, 8mA, 12mA, 16mA
		0, // non-zero if BL pins have slow slew
		0, // non-zero if BL pins are Schmitt input
		0, // valid values are 4mA, 8mA, 12mA, 16mA
		0, // non-zero if BH pins have slow slew
		0, // non-zero if BH pins are Schmitt input
		0, // valid values are 4mA, 8mA, 12mA, 16mA
		0, // non-zero if interface is 245 FIFO
		0, // non-zero if interface is 245 FIFO CPU target
		0, // non-zero if interface is Fast serial
		0, // non-zero if interface is to use VCP drivers
		0, // non-zero if interface is 245 FIFO
		0, // non-zero if interface is 245 FIFO CPU target
		0, // non-zero if interface is Fast serial
		0, // non-zero if interface is to use VCP drivers
		0, // non-zero if using BCBUS7 to save power for self-
		// powered designs
		//
		// Rev 8 (FT4232H) Extensions (Enabled if Version = 4)
		//
		0, // non-zero if pull down enabled ここは無効にする
		1, // non-zero if serial number to be used 
		0, // non-zero if AL pins have slow slew
		0, // non-zero if AL pins are Schmitt input
		4, // valid values are 4mA, 8mA, 12mA, 16mA
		0, // non-zero if AH pins have slow slew
		0, // non-zero if AH pins are Schmitt input
		4, // valid values are 4mA, 8mA, 12mA, 16mA
		0, // non-zero if BL pins have slow slew
		0, // non-zero if BL pins are Schmitt input
		4, // valid values are 4mA, 8mA, 12mA, 16mA
		0, // non-zero if BH pins have slow slew
		0, // non-zero if BH pins are Schmitt input
		4, // valid values are 4mA, 8mA, 12mA, 16mA
		1, // non-zero if port A uses RI as RS485 TXDEN
		1, // non-zero if port B uses RI as RS485 TXDEN
		1, // non-zero if port C uses RI as RS485 TXDEN
		1, // non-zero if port D uses RI as RS485 TXDEN
		1, // non-zero if interface is to use VCP drivers
		1, // non-zero if interface is to use VCP drivers
		1, // non-zero if interface is to use VCP drivers
		1, // non-zero if interface is to use VCP drivers
		//
		// Rev 9 (FT232H) Extensions (Enabled if Version = 5)
		//
		0, // non-zero if pull down enabled
		0, // non-zero if serial number to be used
		0, // non-zero if AC pins have slow slew
		0, // non-zero if AC pins are Schmitt input
		0, // valid values are 4mA, 8mA, 12mA, 16mA
		0, // non-zero if AD pins have slow slew
		0, // non-zero if AD pins are Schmitt input
		0, // valid values are 4mA, 8mA, 12mA, 16mA
		0, // Cbus Mux control
		0, // Cbus Mux control
		0, // Cbus Mux control
		0, // Cbus Mux control
		0, // Cbus Mux control
		0, // Cbus Mux control
		0, // Cbus Mux control
		0, // Cbus Mux control
		0, // Cbus Mux control
		0, // Cbus Mux control
		0, // non-zero if interface is 245 FIFO
		0, // non-zero if interface is 245 FIFO CPU target
		0, // non-zero if interface is Fast serial
		0, // non-zero if interface is FT1248
		0, // FT1248 clock polarity - clock idle high (1) or
		// clock idle low (0)
		0, // FT1248 data is LSB (1) or MSB (0)
		0, // FT1248 flow control enable
		0, // non-zero if interface is to use VCP drivers
		0 // non-zero if using ACBUS7 to save power for
		// self-powered designs
	};

	// - -- -- - - - -- - - -- - -- - -書き込み
	// Read eeprom with data
	ftStatus = FT_EE_Program(ftHandle0, &Write_Data);

	if(ftStatus != FT_OK) {
		printf("FT_EE_Read failed (%d)\n", (int)ftStatus);
		FT_Close(ftHandle0);
		return 1;
	}
	//書いたデータの表示
	printf("--------------- Write Data is below ---------------\n");
	//基本データ
	printf("Manufacturer :: %s\n",Write_Data.Manufacturer);
	printf("ManufacturerId :: %s\n",Write_Data.ManufacturerId);
	// printf("Description :: %s\n",Write_Data.Description);
	printf("SerialNumber :: %s\n",Write_Data.SerialNumber);

	printf("MaxPower :: %d\n",Write_Data.MaxPower);
	printf("PnP :: %d\n",Write_Data.PnP);
	printf("SelfPowered :: %d\n",Write_Data.SelfPowered);
	printf("RemoteWakeup :: %d\n\n",Write_Data.RemoteWakeup);

	//FT4232H用のデータ
	printf("PullDownEnable8 :: %d\n",Write_Data.PullDownEnable8);
	printf("SerNumEnable8 :: %d\n\n",Write_Data.SerNumEnable8);

	printf("ASlowSlew :: %d\n",Write_Data.ASlowSlew);
	printf("ASchmittInput :: %d\n",Write_Data.ASchmittInput);
	printf("ADriveCurrent :: %d\n\n",Write_Data.ADriveCurrent);

	printf("BSlowSlew :: %d\n",Write_Data.BSlowSlew);
	printf("BSchmittInput :: %d\n",Write_Data.BSchmittInput);
	printf("BDriveCurrent :: %d\n\n",Write_Data.BDriveCurrent);

	printf("CSlowSlew :: %d\n",Write_Data.CSlowSlew);
	printf("CSchmittInput :: %d\n",Write_Data.CSchmittInput);
	printf("CDriveCurrent :: %d\n\n",Write_Data.CDriveCurrent);
	
	printf("DSlowSlew :: %d\n",Write_Data.DSlowSlew);
	printf("DSchmittInput :: %d\n",Write_Data.DSchmittInput);
	printf("DDriveCurrent :: %d\n\n",Write_Data.DDriveCurrent);
	
	printf("ARIIsTXDEN :: %d\n",Write_Data.ARIIsTXDEN);
	printf("BRIIsTXDEN :: %d\n",Write_Data.BRIIsTXDEN);
	printf("CRIIsTXDEN :: %d\n",Write_Data.CRIIsTXDEN);
	printf("DRIIsTXDEN :: %d\n\n",Write_Data.DRIIsTXDEN);

	printf("AIsVCP8 :: %d\n",Write_Data.AIsVCP8);
	printf("BIsVCP8 :: %d\n",Write_Data.BIsVCP8);
	printf("CIsVCP8 :: %d\n",Write_Data.CIsVCP8);
	printf("DIsVCP8 :: %d\n",Write_Data.DIsVCP8);

	FT_Close(ftHandle0);
	return 0;
}