--- examples/example-algo/main.c	Tue Feb 28 08:42:08 2023
+++ examples/example-algo/main.c	Thu Jun  6 15:51:38 2024
@@ -2,7 +2,7 @@
  * ________________________________________________________________________________________________________
  * Copyright (c) 2017 InvenSense Inc. All rights reserved.
  *
- * This software, related documentation and any modifications thereto (collectively “Software”) is subject
+ * This software, related documentation and any modifications thereto (collectively is subject
  * to InvenSense and its licensors' intellectual property rights under U.S. and international copyright
  * and other intellectual property rights laws.
  *
@@ -64,7 +64,6 @@
 /* --------------------------------------------------------------------------------------
  *  Global variables
  * -------------------------------------------------------------------------------------- */
-
 /* 
  * Buffer to keep track of the timestamp when icm426xx/ak09915 data ready interrupt fires.
  * The buffer can contain up to 64 items in order to store one timestamp for each packet in FIFO.
@@ -81,10 +80,14 @@
 uint32_t data_to_print = MASK_PRINT_ACC_DATA | MASK_PRINT_GYR_DATA | MASK_PRINT_MAG_DATA |
                          MASK_PRINT_6AXIS_DATA | MASK_PRINT_9AXIS_DATA;
 
+uint32_t data_to_print_hr = 0;  				//print or not print.  
+uint32_t data_to_print_hr_1khz = 0;  			//print or not print.(1khz mode)  
+uint32_t hr46_output_mode = HR46_200HZ_MODE; 	//outupt frequency mode for hr46. 200hz or 1000hz.
+
 /*
  * Define how often traces will be printed
  */
-int print_period_us = 1000000; /* 1 s */
+int print_period_us = 2000; /* 2 ms */
 
 /* --------------------------------------------------------------------------------------
  *  Static variables
@@ -139,23 +142,23 @@
 	check_rc(rc, "Error while setting up MCU");
 
 	/* Initialize ICM device */
-	INV_MSG(INV_MSG_LEVEL_INFO, "Initializing ICM device...");
+	// INV_MSG(INV_MSG_LEVEL_INFO, "Initializing ICM device...");
 	rc = SetupInvDevice(&icm426xx_serif);
 	check_rc(rc, "Error while setting up ICM device");
-	INV_MSG(INV_MSG_LEVEL_INFO, "OK");
+	// INV_MSG(INV_MSG_LEVEL_INFO, "OK");
 
 	/* Initialize algorithm */
-	INV_MSG(INV_MSG_LEVEL_INFO, "Initializing algorithm...");
+	// INV_MSG(INV_MSG_LEVEL_INFO, "Initializing algorithm...");
 	rc = InitInvAGMBiases();
 	rc |= InitInvAGMAlgo();
 	check_rc(rc, "Error while initializing AGM algorithm");
-	INV_MSG(INV_MSG_LEVEL_INFO, "OK");
+	// INV_MSG(INV_MSG_LEVEL_INFO, "OK");
 
 	/* Configure ICM device */
-	INV_MSG(INV_MSG_LEVEL_INFO, "Configuring ICM device...");
+	// INV_MSG(INV_MSG_LEVEL_INFO, "Configuring ICM device...");
 	rc = ConfigureInvDevice();
 	check_rc(rc, "Error while configuring ICM device");
-	INV_MSG(INV_MSG_LEVEL_INFO, "OK");
+	// INV_MSG(INV_MSG_LEVEL_INFO, "OK");
 
 #if USE_MAG
 	/* Initialize magnetomer */
@@ -176,10 +179,10 @@
 #endif
 
 	/* Print reminder on how to use example */
-	print_help();
-
-	INV_MSG(INV_MSG_LEVEL_INFO, "Start processing");
+	// print_help();
 
+	// INV_MSG(INV_MSG_LEVEL_INFO, "Start processing");
+	data_to_print = 0; // Disable normal print for hr46
 	do {
 #if USE_MAG
 		if (mag_init_successful) {
@@ -213,7 +216,7 @@
 		/* Check Icm426xx IRQ */
 		if (irq_from_device & TO_MASK(INV_GPIO_INT1)) {
 			rc = GetDataFromInvDevice();
-			check_rc(rc, "error while getting data from Icm426xx");
+			// check_rc(rc, "error while getting data from Icm426xx"); //Disable this check for hr46
 
 			inv_disable_irq();
 			irq_from_device &= ~TO_MASK(INV_GPIO_INT1);
@@ -237,6 +240,18 @@
 	command_from_uart = get_user_command_from_uart();
 
 	switch (command_from_uart) {
+	case 'C':
+		data_to_print_hr = 1;
+		hr46_output_mode = HR46_200HZ_MODE;
+		data_to_print = 0;
+		print_period_us = 2000; /* 2 ms */
+		break;
+	case 'K': /* output data to hr by 1000hz */
+		data_to_print_hr = 0;
+		data_to_print = 0;
+		data_to_print_hr_1khz = 1;
+		hr46_output_mode = HR46_1000HZ_MODE;
+		break;
 	case 'i':
 		data_to_print ^= MASK_PRINT_INPUT_DATA;
 		break; /* Print input data */
@@ -334,10 +349,6 @@
 	/* Setup message facility to see internal traces from FW */
 	INV_MSG_SETUP(MSG_LEVEL, msg_printer);
 
-	INV_MSG(INV_MSG_LEVEL_INFO, "###################");
-	INV_MSG(INV_MSG_LEVEL_INFO, "#   Example AGM   #");
-	INV_MSG(INV_MSG_LEVEL_INFO, "###################");
-
 	/*
 	 * Configure input capture mode GPIO connected to pin PB10 (arduino connector D6).
 	 * This pin is connected to Icm426xx INT1 output and thus will receive interrupts 
@@ -492,7 +503,7 @@
 		"", // INV_MSG_LEVEL_OFF
 		"[E] ", // INV_MSG_LEVEL_ERROR
 		"[W] ", // INV_MSG_LEVEL_WARNING
-		"[I] ", // INV_MSG_LEVEL_INFO
+		"", // INV_MSG_LEVEL_INFO
 		"[V] ", // INV_MSG_LEVEL_VERBOSE
 		"[D] ", // INV_MSG_LEVEL_DEBUG
 	};
@@ -507,6 +518,77 @@
 		return;
 
 	inv_uart_mngr_puts(LOG_UART_ID, out_str, (unsigned short)idx);
+}
+
+union convert_float
+{
+    float data;
+    char byte[sizeof(float)];
+};
+
+/**
+ * @brief Print gyro acc, quat data to UART for hr46_b3m
+ * 
+ * @param gyro gyro data. 3 float values 
+ * @param accel acc data. 3 float values
+ * @param quat quat data. 4 float values
+ */
+void printDataToHR(const float* gyro,const float* accel,const float* quat,const float temperature)
+{
+	union convert_float gyro_out[3];
+	union convert_float acc_out[3];
+	union convert_float quat_out[4];
+	union convert_float temp_out;
+	
+	gyro_out[0].data = gyro[0];
+	gyro_out[1].data = gyro[1];
+	gyro_out[2].data = gyro[2];
+	
+	acc_out[0].data = accel[0];
+	acc_out[1].data = accel[1];
+	acc_out[2].data = accel[2];
+
+	quat_out[0].data = quat[0];
+	quat_out[1].data = quat[1];
+	quat_out[2].data = quat[2];
+	quat_out[3].data = quat[3];
+
+	temp_out.data = temperature;
+	#ifdef HR46_HUMAN_READABLE
+	INV_MSG(INV_MSG_LEVEL_INFO, "Gyro: [%.3f, %.3f, %.3f]", gyro[0], gyro[1], gyro[2]);
+	INV_MSG(INV_MSG_LEVEL_INFO, "Acc: [%.3f, %.3f, %.3f]", accel[0], accel[1], accel[2]);
+	INV_MSG(INV_MSG_LEVEL_INFO, "Quat: [%f, %f, %f, %f]", quat[0], quat[1], quat[2], quat[3]);
+	INV_MSG(INV_MSG_LEVEL_INFO, "Temp: [%f]", temp);
+	#else //normal mode
+	// 12 bytes for gyro, 12 bytes for acc, 16 bytes for quat, 4 bytes for temp, 2bytes for header, 3bytes for option command, 2 bytes for crc 
+#define send_data_for_hr_size 28 + 16 + 2 + 3 + 2
+	const char send_data[send_data_for_hr_size] = 
+	{
+		SEND_PACKET_HEADER,SEND_PACKET_HEADER, //header
+		SEND_PACKET_DUMMY_COMMAND,SEND_PACKET_DUMMY_COMMAND,SEND_PACKET_DUMMY_COMMAND, //command option error
+		gyro_out[0].byte[0],gyro_out[0].byte[1],gyro_out[0].byte[2],gyro_out[0].byte[3], //gyro 1 roll
+		gyro_out[1].byte[0],gyro_out[1].byte[1],gyro_out[1].byte[2],gyro_out[1].byte[3], //gyro 2 pitch 
+		gyro_out[2].byte[0],gyro_out[2].byte[1],gyro_out[2].byte[2],gyro_out[2].byte[3],//gyro 3 yaw
+		acc_out[0].byte[0],acc_out[0].byte[1],acc_out[0].byte[2],acc_out[0].byte[3], //acc 1 x
+		acc_out[1].byte[0],acc_out[1].byte[1],acc_out[1].byte[2],acc_out[1].byte[3], //acc 2 y
+		acc_out[2].byte[0],acc_out[2].byte[1],acc_out[2].byte[2],acc_out[2].byte[3],//acc 3 z
+		quat_out[0].byte[0],quat_out[0].byte[1],quat_out[0].byte[2],quat_out[0].byte[3], //quat 1 w
+		quat_out[1].byte[0],quat_out[1].byte[1],quat_out[1].byte[2],quat_out[1].byte[3], //quat 2 x
+		quat_out[2].byte[0],quat_out[2].byte[1],quat_out[2].byte[2],quat_out[2].byte[3], //quat 3 y
+		quat_out[3].byte[0],quat_out[3].byte[1],quat_out[3].byte[2],quat_out[3].byte[3], //quat 4 z
+		temp_out.byte[0],temp_out.byte[1],temp_out.byte[2],temp_out.byte[3], //temp
+		SEND_PACKET_CRC_DUMMY,SEND_PACKET_CRC_DUMMY //crc
+	};
+	// INV_MSG(INV_MSG_LEVEL_INFO,"%.35s",send_data);
+	inv_uart_mngr_puts(LOG_UART_ID,send_data,send_data_for_hr_size);
+	(void)quat_out;   // avoid werror.
+	#endif
+	return;
+}
+
+void outputData1000hzToHR46(const char* send_buffer,const unsigned int send_size)
+{
+	inv_uart_mngr_puts(LOG_UART_ID,send_buffer,send_size);
 }
 
 /* --------------------------------------------------------------------------------------
