#include <stdio.h>

union convert_float
{
    float data;
    char byte[sizeof(float)];
};

/**
 * @brief Print gyro acc, quat data to UART for hr46_b3m
 * 
 * @param gyro gyro data. 3 float values 
 * @param accel acc data. 3 float values
 * @param quat quat data. 4 float values
 */
static void printDataToHR(const float* gyro,const float* accel,const float* quat,const float temperature)
{
	union convert_float gyro_out[3];
	union convert_float acc_out[3];
	union convert_float quat_out[4];
	union convert_float temp_out;
	
	gyro_out[0].data = gyro[0];
	gyro_out[1].data = gyro[1];
	gyro_out[2].data = gyro[2];
	
	acc_out[0].data = accel[0];
	acc_out[1].data = accel[1];
	acc_out[2].data = accel[2];

	quat_out[0].data = quat[0];
	quat_out[1].data = quat[1];
	quat_out[2].data = quat[2];
	quat_out[3].data = quat[3];

	temp_out.data = temperature;
	const char crc_dummy = 0xff;
	const char header = 0xfe;
	const char dummy_command = 0xfc;
	const char send_data[28 + 2 + 3 + 2] = 
	{
		header,header, //header
		dummy_command,dummy_command,dummy_command, //command option error
		gyro_out[0].byte[0],gyro_out[0].byte[1],gyro_out[0].byte[2],gyro_out[0].byte[3], //gyro 1
		gyro_out[1].byte[4],gyro_out[1].byte[5],gyro_out[1].byte[6],gyro_out[1].byte[7], //gyro 2
		gyro_out[2].byte[8],gyro_out[2].byte[9],gyro_out[2].byte[10],gyro_out[2].byte[11],//gyro 3
		acc_out[0].byte[0],acc_out[0].byte[1],acc_out[0].byte[2],acc_out[0].byte[3], //acc 1
		acc_out[1].byte[4],acc_out[1].byte[5],acc_out[1].byte[6],acc_out[1].byte[7], //acc 2
		acc_out[2].byte[8],acc_out[2].byte[9],acc_out[2].byte[10],acc_out[2].byte[11],//acc 3
		temp_out.byte[0],temp_out.byte[1],temp_out.byte[2],temp_out.byte[3], //temp
		crc_dummy,crc_dummy //crc
	};
	printf("send_data = %.35s aaa\n", send_data);
    printf("send_data string -> hex = ");
    for(int i = 0; i < 28 + 2 + 3 + 2 + 1; i++)
    {
        printf("%x ", send_data[i]);
    }
	(void)quat_out;   // avoid werror.
	return;
}



int main(int argc, char const *argv[])
{
    union convert_float floats_data[3];
    float data = 0.123124;
    float gyro[3] = {111.000, 22.00, 33.0};
    float acc[3] = {4.0, 5.0, 6.0};
    float quat[4] = {7.0, 8.0, 9.0, 10.0};
    printf("Hello, World!\n");
    printDataToHR(gyro,acc,quat, data);
    return 0;
}
