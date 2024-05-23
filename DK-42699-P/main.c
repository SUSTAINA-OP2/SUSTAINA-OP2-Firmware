/*
 * ________________________________________________________________________________________________________
 * Copyright (c) 2017 InvenSense Inc. All rights reserved.
 *
 * This software, related documentation and any modifications thereto (collectively is subject
 * to InvenSense and its licensors' intellectual property rights under U.S. and international copyright
 * and other intellectual property rights laws.
 *
 * InvenSense and its licensors retain all intellectual property and proprietary rights in and to the Software
 * and any use, reproduction, disclosure or distribution of the Software without an express license agreement
 * from InvenSense is strictly prohibited.
 *
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, THE SOFTWARE IS
 * PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
 * TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT.
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, IN NO EVENT SHALL
 * INVENSENSE BE LIABLE FOR ANY DIRECT, SPECIAL, INDIRECT, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, OR ANY
 * DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
 * OF THE SOFTWARE.
 * ________________________________________________________________________________________________________
 */

#include "example-algo.h"

/* InvenSense utils */
#include "Invn/EmbUtils/Message.h"
#include "Invn/EmbUtils/ErrorHelper.h"
#include "Invn/EmbUtils/RingBuffer.h"

/* board driver */
#include "common.h"
#include "uart_mngr.h"
#include "delay.h"
#include "gpio.h"
#include "timer.h"
#include "rtc_timer.h"

#include "system-interface.h"

/* std */
#include <stdio.h>

/* --------------------------------------------------------------------------------------
 *  Example configuration
 * -------------------------------------------------------------------------------------- */

/*
 * Select UART port on which INV_MSG() will be printed.
 */
#define LOG_UART_ID INV_UART_SENSOR_CTRL

/* 
 * Select communication link between SmartMotion and ICM426xx 
 */
#define SERIF_TYPE ICM426XX_UI_SPI4
//#define SERIF_TYPE ICM426XX_UI_I2C

/* 
 * Define msg level 
 */
#define MSG_LEVEL INV_MSG_LEVEL_INFO

/* --------------------------------------------------------------------------------------
 *  Global variables
 * -------------------------------------------------------------------------------------- */
/* 
 * Buffer to keep track of the timestamp when icm426xx/ak09915 data ready interrupt fires.
 * The buffer can contain up to 64 items in order to store one timestamp for each packet in FIFO.
 */
RINGBUFFER_VOLATILE(timestamp_buffer_icm, 64, uint64_t);
#if USE_MAG
RINGBUFFER_VOLATILE(timestamp_buffer_mag, 64, uint64_t);
#endif

/*
 * Outptut data to print
 * Default behavior: only print accel, gyro, mag and 6-axis
 */
uint32_t data_to_print = MASK_PRINT_ACC_DATA | MASK_PRINT_GYR_DATA | MASK_PRINT_MAG_DATA |
                         MASK_PRINT_6AXIS_DATA | MASK_PRINT_9AXIS_DATA;

uint32_t data_to_print_hr = 0;  				//print or not print.  
uint32_t data_to_print_hr_1khz = 0;  			//print or not print.(1khz mode)  
uint32_t hr46_output_mode = HR46_200HZ_MODE; 	//outupt frequency mode for hr46. 200hz or 1000hz.

/*
 * Define how often traces will be printed
 */
int print_period_us = 2000; /* 2 ms */

/* --------------------------------------------------------------------------------------
 *  Static variables
 * -------------------------------------------------------------------------------------- */

/* Flag set from icm426xx device irq handler */
static volatile int irq_from_device;

#if USE_MAG
/* Flag set from irq handler of the timer used to trigger new data acquitision on Akm09915 */
static volatile int irq_from_ak09915_acquisition_timer = 0;

/* Flag set from irq handler of the timer used to trigger new data ready on Akm09915 */
static volatile int irq_from_ak09915_data_ready = 0;

/* Variable used to keep channel used for mag data ready */
static int mag_end_capture_channel = -1;

/* Variable to keep track if the mag has initialized successfully */
int mag_init_successful = 0;
#endif

/* --------------------------------------------------------------------------------------
 *  Forward declaration
 * -------------------------------------------------------------------------------------- */

static int  SetupMCUHardware(struct inv_icm426xx_serif *icm_serif,
                             struct inv_ak0991x_serif * akm_serif);
static void ext_interrupt_inv_cb(void *context, unsigned int_num);
#if USE_MAG
static void interrupt_timer_start_mag_cb(void *context);
static void interrupt_timer_data_rdy_mag_cb(void *context);
#endif
static char get_user_command_from_uart(void);
static void process_user_command(void);
static void print_help(void);
void        check_rc(int rc, const char *msg_context);
void        msg_printer(int level, const char *str, va_list ap);

/* --------------------------------------------------------------------------------------
 *  Main
 * -------------------------------------------------------------------------------------- */

int main(void)
{
	int                       rc = 0;
	struct inv_icm426xx_serif icm426xx_serif;
	struct inv_ak0991x_serif  ak09915_serif;

	/* Initialize MCU hardware */
	rc = SetupMCUHardware(&icm426xx_serif, &ak09915_serif);
	check_rc(rc, "Error while setting up MCU");

	/* Initialize ICM device */
	// INV_MSG(INV_MSG_LEVEL_INFO, "Initializing ICM device...");
	rc = SetupInvDevice(&icm426xx_serif);
	check_rc(rc, "Error while setting up ICM device");
	// INV_MSG(INV_MSG_LEVEL_INFO, "OK");

	/* Initialize algorithm */
	// INV_MSG(INV_MSG_LEVEL_INFO, "Initializing algorithm...");
	rc = InitInvAGMBiases();
	rc |= InitInvAGMAlgo();
	check_rc(rc, "Error while initializing AGM algorithm");
	// INV_MSG(INV_MSG_LEVEL_INFO, "OK");

	/* Configure ICM device */
	// INV_MSG(INV_MSG_LEVEL_INFO, "Configuring ICM device...");
	rc = ConfigureInvDevice();
	check_rc(rc, "Error while configuring ICM device");
	// INV_MSG(INV_MSG_LEVEL_INFO, "OK");

#if USE_MAG
	/* Initialize magnetomer */
	INV_MSG(INV_MSG_LEVEL_INFO, "Initializing Mag device...");
	rc = SetupMagDevice(&ak09915_serif);
	if (rc < 0) {
		INV_MSG(INV_MSG_LEVEL_WARNING,
		        "Error while setting up Mag device: not support of mag-related sensors");
		mag_init_successful = 0;
	} else {
		INV_MSG(INV_MSG_LEVEL_INFO, "OK");
		mag_init_successful = 1;
		/* Configure the timer used to trigger new magnetometer data capture */
		inv_timer_configure_callback(MAG_SAMPLING_TIMER, period_us_to_frequency(MAG_ODR_US), 0,
		                             interrupt_timer_start_mag_cb);
	}

#endif

	/* Print reminder on how to use example */
	// print_help();

	// INV_MSG(INV_MSG_LEVEL_INFO, "Start processing");
	data_to_print = 0; // Disable normal print for hr46
	do {
#if USE_MAG
		if (mag_init_successful) {
			/* Check Ak09915 IRQ */
			if (irq_from_ak09915_data_ready) {
				inv_disable_irq();
				irq_from_ak09915_data_ready = 0;
				inv_enable_irq();

				rc = GetDataFromMagDevice();
				check_rc(rc, "error while getting data from Akm09915");
			}

			/* Check IRQ from timer ruling mag acquisition */
			if (irq_from_ak09915_acquisition_timer) {
				inv_disable_irq();
				irq_from_ak09915_acquisition_timer = 0;
				inv_enable_irq();

				StartMagDeviceAcquisition();

				/* Start time for the duration of the aquisition */
				mag_end_capture_channel =
				    inv_timer_configure_callback(MAG_DATA_TIMER,
				                                 period_us_to_frequency(MAG_DATA_RDY_DELAY_US), 0,
				                                 interrupt_timer_data_rdy_mag_cb);
			}
		}
#endif

		/* Check Icm426xx IRQ */
		if (irq_from_device & TO_MASK(INV_GPIO_INT1)) {
			rc = GetDataFromInvDevice();
			// check_rc(rc, "error while getting data from Icm426xx"); //Disable this check for hr46

			inv_disable_irq();
			irq_from_device &= ~TO_MASK(INV_GPIO_INT1);
			inv_enable_irq();
		}

		process_user_command();

	} while (1);
}

/* --------------------------------------------------------------------------------------
 *  Functions definitions
 * -------------------------------------------------------------------------------------- */

static void process_user_command(void)
{
	uint8_t command_from_uart = 0;
	int     rc;

	command_from_uart = get_user_command_from_uart();

	switch (command_from_uart) {
	case 'C':
		data_to_print_hr = 1;
		hr46_output_mode = HR46_200HZ_MODE;
		data_to_print = 0;
		print_period_us = 2000; /* 2 ms */
		break;
	case 'K': /* output data to hr by 1000hz */
		data_to_print_hr = 0;
		data_to_print = 0;
		data_to_print_hr_1khz = 1;
		hr46_output_mode = HR46_1000HZ_MODE;
		break;
	case 'i':
		data_to_print ^= MASK_PRINT_INPUT_DATA;
		break; /* Print input data */
	case 'a':
		data_to_print ^= MASK_PRINT_ACC_DATA;
		break; /* Print accel data */
	case 'g':
		data_to_print ^= MASK_PRINT_GYR_DATA;
		break; /* Print gyro data */
#if USE_MAG
	case 'm':
		data_to_print ^= MASK_PRINT_MAG_DATA;
		break; /* Print mag data */
	case '9':
		data_to_print ^= MASK_PRINT_9AXIS_DATA;
		break; /* Print 9 axis data */
#endif
	case '6':
		data_to_print ^= MASK_PRINT_6AXIS_DATA;
		break; /* Print 6 axis data */

	case 'r':
		rc = ResetInvAGMBiases();
		rc |= InitInvAGMAlgo();
		check_rc(rc, "Error while initializing VR Threedof algorithms");
		break;
	case 'f': /* Toggle fast-mode (data printed every 20 ms or every 1 s) */
		print_period_us = (print_period_us == 1000000 /*1s*/) ? 20000 /*20ms*/ : 1000000 /*1s*/;
		break;
	case 'G':
		data_to_print ^= MASK_PRINT_GRAVITY_DATA;
		break; /* Print gravity data */
	case 'l':
		data_to_print ^= MASK_PRINT_LINEARACC_DATA;
		break; /* Print linear acceleration data */
	case 'h':
	case 'H':
		print_help(); /* Print helper command */
		break;
	case 0:
		break; /* No command received */
	default:
		INV_MSG(INV_MSG_LEVEL_INFO, "Unknown command : %c", command_from_uart);
		print_help();
		break;
	}
}

static void print_help(void)
{
	INV_MSG(INV_MSG_LEVEL_INFO, "##########################");
	INV_MSG(INV_MSG_LEVEL_INFO, "#   Help - Example Algo  #");
	INV_MSG(INV_MSG_LEVEL_INFO, "##########################");
	INV_MSG(INV_MSG_LEVEL_INFO, "\t'i' : print input data (raw accel, raw gyro and raw mag)");
	INV_MSG(INV_MSG_LEVEL_INFO, "\t'a' : print accel data");
	INV_MSG(INV_MSG_LEVEL_INFO, "\t'g' : print gyro data");
#if USE_MAG
	if (mag_init_successful) {
		INV_MSG(INV_MSG_LEVEL_INFO, "\t'm' : print mag data");
		INV_MSG(INV_MSG_LEVEL_INFO,
		        "\t'9' : print rv quaternion data and eulers angles (9axis fusion)");
	}
#endif
	INV_MSG(INV_MSG_LEVEL_INFO,
	        "\t'6' : print grv quaternion data and eulers angles (6axis fusion)");
	INV_MSG(INV_MSG_LEVEL_INFO, "\t'G' : print gravity estimation in sensor frame");
	INV_MSG(INV_MSG_LEVEL_INFO, "\t'l' : print linear acceleration estimation in sensor frame");
	INV_MSG(INV_MSG_LEVEL_INFO, "\t'r' : reset biases and accuracies (will also reinit algorithm)");
	INV_MSG(INV_MSG_LEVEL_INFO, "\t'f' : toggle fast-mode (data printed every 20 ms or every 1 s)");
	INV_MSG(INV_MSG_LEVEL_INFO, "\t'h' : print this helper");
}

/*
 * This function initializes MCU on which this software is running.
 * It configures:
 *   - a UART link used to print some messages
 *   - interrupt priority group and GPIOs so that MCU can receive interrupts from both
 *     ICM426xx and Akm09915
 *   - a microsecond timer requested by Icm426xx driver to compute some delay
 *   - a microsecond timer used to get some timestamps
 *   - a microsecond timer used to periodically starts magneto data acquisition
 *   - a serial link to communicate from MCU to Icm426xx
 *   - a serial link to communicate from MCU to Akm09915
 */
static int SetupMCUHardware(struct inv_icm426xx_serif *icm_serif,
                            struct inv_ak0991x_serif * akm_serif)
{
	int rc = 0;

	inv_io_hal_board_init();

	/* configure UART */
	config_uart(LOG_UART_ID);

	/* Setup message facility to see internal traces from FW */
	INV_MSG_SETUP(MSG_LEVEL, msg_printer);

	/*
	 * Configure input capture mode GPIO connected to pin PB10 (arduino connector D6).
	 * This pin is connected to Icm426xx INT1 output and thus will receive interrupts 
	 * enabled on INT1 from the device.
	 * A callback function is also passed that will be executed each time an interrupt
	 * fires.
	*/
	inv_gpio_sensor_irq_init(INV_GPIO_INT1, ext_interrupt_inv_cb, 0);

	/* Init timer peripheral for delay */
	rc |= inv_delay_init(DELAY_TIMER);

#if USE_CLK_IN
	/* Use CLKIN */
	rtc_timer_init(NULL);
	/* Output 32kHz SLCK to PA17, it is up to user to connect it or not at board level to have CLKIN capability */
	rc |= inv_gpio_output_clk_on_pin(INV_GPIO_CLKIN);
#else
	/* Configure the timer for the timebase */
	rc |= inv_timer_configure_timebase(1000000);
	inv_timer_enable(TIMEBASE_TIMER);
#endif

	/* Initialize serial inteface between MCU and Icm426xx */
	icm_serif->context    = 0; /* no need */
	icm_serif->read_reg   = inv_io_hal_read_reg;
	icm_serif->write_reg  = inv_io_hal_write_reg;
	icm_serif->max_read   = 1024 * 32; /* maximum number of bytes allowed per serial read */
	icm_serif->max_write  = 1024 * 32; /* maximum number of bytes allowed per serial write */
	icm_serif->serif_type = SERIF_TYPE;
	rc |= inv_io_hal_init(icm_serif);

#if USE_MAG
	/* Configure timer used to periodically start mag acquisition */
	if (TIMEBASE_TIMER != MAG_SAMPLING_TIMER) {
		inv_timer_enable(MAG_SAMPLING_TIMER);
	}

	/* Initialize serial inteface between MCU and Akm09911 */
	akm_serif->context   = 0; /* no need */
	akm_serif->read_reg  = akm_io_hal_read_reg;
	akm_serif->write_reg = akm_io_hal_write_reg;
	akm_serif->max_read  = 64; /* maximum number of bytes allowed per serial read */
	akm_serif->max_write = 64; /* maximum number of bytes allowed per serial write */
	akm_serif->is_spi    = 0;
	rc |= akm_io_hal_init(akm_serif);
#else
	(void)akm_serif;
#endif

	return rc;
}

/*
 * Icm426xx interrupt handler.
 * Function is executed when an Icm426xx interrupt rises on MCU.
 * This function get a timestamp and store it in a dedicated timestamp buffer.
 * Note that this function is executed in an interrupt handler and thus no protection
 * is implemented for shared variable timestamp_buffer.
 */
static void ext_interrupt_inv_cb(void *context, unsigned int_num)
{
	(void)context;

#if USE_CLK_IN
	/* Read timestamp from the RTC derived from SLCK since CLKIN is used */
	uint64_t timestamp = rtc_timer_get_time_us();
#else /* ICM42686P */
	/* Read timestamp from the timer */
	uint64_t timestamp = inv_timer_get_counter(TIMEBASE_TIMER);
#endif

	if (int_num == INV_GPIO_INT1 && !RINGBUFFER_VOLATILE_FULL(&timestamp_buffer_icm)) {
		RINGBUFFER_VOLATILE_PUSH(&timestamp_buffer_icm, &timestamp);
	}

	irq_from_device |= TO_MASK(int_num);
}

#if USE_MAG
/*
 * Interrupt handler of the timer used to trigger new magnetometer data acquisition.
 */
static void interrupt_timer_start_mag_cb(void *context)
{
	(void)context;

#if USE_CLK_IN
	/* Read timestamp from the RTC derived from SLCK since CLKIN is used */
	uint64_t timestamp = rtc_timer_get_time_us();
#else
	/* Read timestamp from the timer */
	uint64_t timestamp = inv_timer_get_counter(TIMEBASE_TIMER);
#endif

	if (!RINGBUFFER_VOLATILE_FULL(&timestamp_buffer_mag))
		RINGBUFFER_VOLATILE_PUSH(&timestamp_buffer_mag, &timestamp);

	irq_from_ak09915_acquisition_timer = 1;
}

/*
 * Interrupt handler of the timer used to trigger new magnetometer data acquisition.
 */
static void interrupt_timer_data_rdy_mag_cb(void *context)
{
	(void)context;

	irq_from_ak09915_data_ready = 1;

	if (mag_end_capture_channel >= 0)
		inv_timer_channel_stop(MAG_DATA_TIMER, (uint8_t)mag_end_capture_channel);
}

#endif /* USE_MAG */

/* Get char command on the UART */
static char get_user_command_from_uart(void)
{
	char rchar;
	char cmd = 0;

	while (inv_uart_mngr_available(LOG_UART_ID)) {
		rchar = (char)inv_uart_mngr_getc(LOG_UART_ID);
		if (rchar != '\n')
			cmd = rchar;
	}

	return cmd;
}

/*
 * Helper function to check RC value and block programm execution
 */
void check_rc(int rc, const char *msg_context)
{
	if (rc < 0) {
		INV_MSG(INV_MSG_LEVEL_ERROR, "%s: error %d (%s)\r\n", msg_context, rc, inv_error_str(rc));
		while (1)
			;
	}
}

/*
 * Printer function for message facility
 */
void msg_printer(int level, const char *str, va_list ap)
{
	static char out_str[256]; /* static to limit stack usage */
	unsigned    idx                  = 0;
	const char *s[INV_MSG_LEVEL_MAX] = {
		"", // INV_MSG_LEVEL_OFF
		"[E] ", // INV_MSG_LEVEL_ERROR
		"[W] ", // INV_MSG_LEVEL_WARNING
		"", // INV_MSG_LEVEL_INFO
		"[V] ", // INV_MSG_LEVEL_VERBOSE
		"[D] ", // INV_MSG_LEVEL_DEBUG
	};
	idx += snprintf(&out_str[idx], sizeof(out_str) - idx, "%s", s[level]);
	if (idx >= (sizeof(out_str)))
		return;
	idx += vsnprintf(&out_str[idx], sizeof(out_str) - idx, str, ap);
	if (idx >= (sizeof(out_str)))
		return;
	idx += snprintf(&out_str[idx], sizeof(out_str) - idx, "\r\n");
	if (idx >= (sizeof(out_str)))
		return;

	inv_uart_mngr_puts(LOG_UART_ID, out_str, (unsigned short)idx);
}

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
void printDataToHR(const float* gyro,const float* accel,const float* quat,const float temperature)
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
	#ifdef HR46_HUMAN_READABLE
	INV_MSG(INV_MSG_LEVEL_INFO, "Gyro: [%.3f, %.3f, %.3f]", gyro[0], gyro[1], gyro[2]);
	INV_MSG(INV_MSG_LEVEL_INFO, "Acc: [%.3f, %.3f, %.3f]", accel[0], accel[1], accel[2]);
	INV_MSG(INV_MSG_LEVEL_INFO, "Quat: [%f, %f, %f, %f]", quat[0], quat[1], quat[2], quat[3]);
	INV_MSG(INV_MSG_LEVEL_INFO, "Temp: [%f]", temp);
	#else //normal mode
	// 12 bytes for gyro, 12 bytes for acc, 16 bytes for quat, 4 bytes for temp, 2bytes for header, 3bytes for option command, 2 bytes for crc 
#define send_data_for_hr_size 28 + 16 + 2 + 3 + 2
	const char send_data[send_data_for_hr_size] = 
	{
		SEND_PACKET_HEADER,SEND_PACKET_HEADER, //header
		SEND_PACKET_DUMMY_COMMAND,SEND_PACKET_DUMMY_COMMAND,SEND_PACKET_DUMMY_COMMAND, //command option error
		gyro_out[0].byte[0],gyro_out[0].byte[1],gyro_out[0].byte[2],gyro_out[0].byte[3], //gyro 1 roll
		gyro_out[1].byte[0],gyro_out[1].byte[1],gyro_out[1].byte[2],gyro_out[1].byte[3], //gyro 2 pitch 
		gyro_out[2].byte[0],gyro_out[2].byte[1],gyro_out[2].byte[2],gyro_out[2].byte[3],//gyro 3 yaw
		acc_out[0].byte[0],acc_out[0].byte[1],acc_out[0].byte[2],acc_out[0].byte[3], //acc 1 x
		acc_out[1].byte[0],acc_out[1].byte[1],acc_out[1].byte[2],acc_out[1].byte[3], //acc 2 y
		acc_out[2].byte[0],acc_out[2].byte[1],acc_out[2].byte[2],acc_out[2].byte[3],//acc 3 z
		quat_out[0].byte[0],quat_out[0].byte[1],quat_out[0].byte[2],quat_out[0].byte[3], //quat 1 w
		quat_out[1].byte[0],quat_out[1].byte[1],quat_out[1].byte[2],quat_out[1].byte[3], //quat 2 x
		quat_out[2].byte[0],quat_out[2].byte[1],quat_out[2].byte[2],quat_out[2].byte[3], //quat 3 y
		quat_out[3].byte[0],quat_out[3].byte[1],quat_out[3].byte[2],quat_out[3].byte[3], //quat 4 z
		temp_out.byte[0],temp_out.byte[1],temp_out.byte[2],temp_out.byte[3], //temp
		SEND_PACKET_CRC_DUMMY,SEND_PACKET_CRC_DUMMY //crc
	};
	// INV_MSG(INV_MSG_LEVEL_INFO,"%.35s",send_data);
	inv_uart_mngr_puts(LOG_UART_ID,send_data,send_data_for_hr_size);
	(void)quat_out;   // avoid werror.
	#endif
	return;
}

void outputData1000hzToHR46(const char* send_buffer,const unsigned int send_size)
{
	inv_uart_mngr_puts(LOG_UART_ID,send_buffer,send_size);
}

/* --------------------------------------------------------------------------------------
 *  Extern functions definition
 * -------------------------------------------------------------------------------------- */

/*
 * Icm426xx driver needs to get time in us. Let's give its implementation here.
 */
uint64_t inv_icm426xx_get_time_us(void)
{
#if USE_CLK_IN
	return rtc_timer_get_time_us();
#else
	return inv_timer_get_counter(TIMEBASE_TIMER);
#endif
}

/*
 * Clock calibration module needs to disable IRQ. Thus inv_helper_disable_irq is
 * defined as extern symbol in clock calibration module. Let's give its implementation
 * here.
 */
void inv_helper_disable_irq(void)
{
	inv_disable_irq();
}

/*
 * Clock calibration module needs to enable IRQ. Thus inv_helper_enable_irq is
 * defined as extern symbol in clock calibration module. Let's give its implementation
 * here.
 */
void inv_helper_enable_irq(void)
{
	inv_enable_irq();
}

/*
 * Icm426xx driver needs a sleep feature from external device. Thus inv_icm426xx_sleep_us
 * is defined as extern symbol in driver. Let's give its implementation here.
 */
void inv_icm426xx_sleep_us(uint32_t us)
{
	inv_delay_us(us);
}
