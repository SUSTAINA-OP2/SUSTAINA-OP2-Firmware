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
	
	// FT_SetVIDPID(0x0403, 0x6011);

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
	
	FT_PROGRAM_DATA Data;
	//set data
	Data.Signature1 = 0x00000000;
	Data.Signature2 = 0xffffffff;
	Data.VendorId = 0x0403;				
	Data.ProductId = 0x6001;
	Data.Manufacturer =  "FTDI";
	Data.ManufacturerId = "FT";
	Data.Description = "USB <-> Serial";
	Data.SerialNumber = "FT_BIT-BOT";		// if fixed, or NULL
	
	Data.MaxPower = 44;
	Data.PnP = 1;
	Data.SelfPowered = 0;
	Data.RemoteWakeup = 1;
	Data.Rev4 = 1;
	Data.IsoIn = 0;
	Data.IsoOut = 0;
	Data.PullDownEnable = 1;
	Data.SerNumEnable = 1;
	Data.USBVersionEnable = 0;
	Data.USBVersion = 0x110;

	//enable TXDEN pin
	Data.ARIIsTXDEN = 1;
	Data.BRIIsTXDEN = 1;
	Data.CRIIsTXDEN = 1;
	Data.DRIIsTXDEN = 1;

	FT_PROGRAM_DATA Read_Data;
	char ManufacturerBuf[32];
	char ManufacturerIdBuf[16];
	char DescriptionBuf[64];
	char SerialNumberBuf[16];
	Read_Data.Signature1 = 0x00000000;
	Read_Data.Signature2 = 0xffffffff;
	Read_Data.Version = 0x00000004; // FT4232用のEEPROM構造 
	Read_Data.Manufacturer = ManufacturerBuf;
	Read_Data.ManufacturerId = ManufacturerIdBuf;
	Read_Data.Description = DescriptionBuf;
	Read_Data.SerialNumber = SerialNumberBuf;

	//Read eeprom with data
	ftStatus = FT_EE_Read(ftHandle0, &Read_Data);

	if(ftStatus != FT_OK) {
		printf("FT_EE_Read failed (%d)\n", (int)ftStatus);
		FT_Close(ftHandle0);
		return 1;
	}
	//読んだデータの表示
	//基本データ
	printf("Manufacturer :: %s\n",ManufacturerBuf);
	printf("ManufacturerId :: %s\n",ManufacturerIdBuf);
	printf("Description :: %s\n",DescriptionBuf);
	printf("SerialNumber :: %s\n",SerialNumberBuf);

	printf("MaxPower :: %d\n",Read_Data.MaxPower);
	printf("PnP :: %d\n",Read_Data.PnP);
	printf("SelfPowered :: %d\n",Read_Data.SelfPowered);
	printf("RemoteWakeup :: %d\n",Read_Data.RemoteWakeup);

	//FT4232H用のデータ
	printf("PullDownEnable8 :: %d\n",Read_Data.PullDownEnable8);
	printf("SerNumEnable8 :: %d\n",Read_Data.SerNumEnable8);
	printf("ASlowSlew :: %d\n",Read_Data.ASlowSlew);
	printf("ASchmittInput :: %d\n",Read_Data.ASchmittInput);
	printf("ADriveCurrent :: %d\n\n",Read_Data.ADriveCurrent);
	
	printf("BSlowSlew :: %d\n",Read_Data.BSlowSlew);
	printf("BSchmittInput :: %d\n",Read_Data.BSchmittInput);
	printf("BDriveCurrent :: %d\n\n",Read_Data.BDriveCurrent);
	
	printf("CSlowSlew :: %d\n",Read_Data.CSlowSlew);
	printf("CSchmittInput :: %d\n",Read_Data.CSchmittInput);
	printf("CDriveCurrent :: %d\n\n",Read_Data.CDriveCurrent);

	printf("DSlowSlew :: %d\n",Read_Data.DSlowSlew);
	printf("DSchmittInput :: %d\n",Read_Data.DSchmittInput);
	printf("DDriveCurrent :: %d\n\n",Read_Data.DDriveCurrent);
	printf("ARIIsTXDEN :: %d\n",Read_Data.ARIIsTXDEN);
	printf("BRIIsTXDEN :: %d\n",Read_Data.BRIIsTXDEN);
	printf("CRIIsTXDEN :: %d\n",Read_Data.CRIIsTXDEN);
	printf("DRIIsTXDEN :: %d\n",Read_Data.DRIIsTXDEN);
	printf("AIsVCP8 :: %d\n",Read_Data.AIsVCP8);
	printf("BIsVCP8 :: %d\n",Read_Data.BIsVCP8);
	printf("CIsVCP8 :: %d\n",Read_Data.CIsVCP8);
	printf("DIsVCP8 :: %d\n",Read_Data.DIsVCP8);

	FT_Close(ftHandle0);
	return 0;
}