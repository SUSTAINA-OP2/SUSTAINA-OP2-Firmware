--- examples/example-algo/example-algo.c	Tue Feb 28 08:42:08 2023
+++ examples/example-algo/example-algo.c	Thu Jun 13 21:44:31 2024
@@ -62,13 +62,13 @@
 
 /* Full Scale Range */
 #if IS_HIGH_RES_MODE
-#if defined(ICM42686P) || defined(ICM42686V)
-static const int32_t acc_fsr = 32; /* +/- 32g */
-static const int32_t gyr_fsr = 4000; /* +/- 4000dps */
-#else
-static const int32_t acc_fsr = 16; /* +/- 16g */
-static const int32_t gyr_fsr = 2000; /* +/- 2000dps */
-#endif
+	#if defined(ICM42686P) || defined(ICM42686V)
+	static const int32_t acc_fsr = 32; /* +/- 32g */
+	static const int32_t gyr_fsr = 4000; /* +/- 4000dps */
+	#else
+	static const int32_t acc_fsr = 16; /* +/- 16g */
+	static const int32_t gyr_fsr = 2000; /* +/- 2000dps */
+	#endif
 #else
 static const int32_t acc_fsr = 4; /* +/- 4g */
 static const int32_t gyr_fsr = 2000; /* +/- 2000dps */
@@ -112,12 +112,14 @@
 
 /* Outptut data to print */
 extern uint32_t data_to_print;
+extern uint32_t data_to_print_hr_1khz;
+extern uint32_t hr46_output_mode;
 
-/* Bias previously stored in flash */
-typedef struct sensor_biases {
-	int32_t bias_q16[3];
-	uint8_t is_saved;
-} sensor_biases_t;
+/* Outptut data to print for hr46*/
+extern uint32_t data_to_print_hr;
+extern void printDataToHR(const float* gyro,const float* accel,const float* quat,const float temperature);
+extern void outputData1000hzToHR46(const char* send_buffer,const unsigned int send_size);
+static int16_t error_lists[4] = {0};  //Error type = -1 INV_ERROR, -3 INV_ERROR_TRANSPORT, -5 INV_ERROR_SIZE, -11 INV_ERROR_BAD_ARG
 
 /* --------------------------------------------------------------------------------------
  *  static function declaration
@@ -139,6 +141,155 @@
                                   const int32_t mag_bias_q16[3]);
 static void store_biases(void);
 
+
+/* ----------------------------------------------------------------------
+ * 1000hz用の各種変数
+ * ----------------------------------------------------------------------*/
+
+typedef struct {
+    float acc[3];
+    float gyro[3];
+    float quat[4];
+    float temperature;
+} HR_IMUdata;
+
+static HR_IMUdata hr_data_array[10] = {0};
+static const uint32_t hr_data_array_elem_size = sizeof(hr_data_array) / sizeof(HR_IMUdata);
+#define hr_data_array_byte_size (sizeof(hr_data_array))
+
+/* Counter for measured data. */
+static uint32_t hr_1khz_data_counter = 0;
+
+void setData(const float* gyro,const float* accel,const float* quat,const float temperature)
+{
+	const int index = hr_1khz_data_counter % hr_data_array_elem_size;
+    hr_data_array[index].gyro[0] = gyro[0];
+	hr_data_array[index].gyro[1] = gyro[1];
+	hr_data_array[index].gyro[2] = gyro[2];
+	hr_data_array[index].acc[0] = accel[0];
+	hr_data_array[index].acc[1] = accel[1];
+	hr_data_array[index].acc[2] = accel[2];
+	hr_data_array[index].quat[0] = quat[0];
+	hr_data_array[index].quat[1] = quat[1];
+	hr_data_array[index].quat[2] = quat[2];
+	hr_data_array[index].quat[3] = quat[3];
+	hr_data_array[index].temperature = temperature;
+    hr_1khz_data_counter++;
+	return;
+}
+
+
+void setErrorData(const int16_t error_status)
+{
+	const int index = hr_1khz_data_counter % hr_data_array_elem_size;
+	switch (error_status)
+	{
+	case -1:
+		error_lists[0]++;
+		break;
+	case -3:
+		error_lists[1]++;
+		break;
+	case -5:
+		error_lists[2]++;
+		break;
+	case -11:
+		error_lists[3]++;
+		break;
+	default:
+		break;
+	}
+	// Set error status to all data.
+    hr_data_array[index].gyro[0] = 0;
+	hr_data_array[index].gyro[1] = 0;
+	hr_data_array[index].gyro[2] = 0;
+	hr_data_array[index].acc[0] = 0;
+	hr_data_array[index].acc[1] = 0;
+	hr_data_array[index].acc[2] = 0;
+	hr_data_array[index].quat[0] = 0;
+	hr_data_array[index].quat[1] = 0;
+	hr_data_array[index].quat[2] = 0;
+	hr_data_array[index].quat[3] = 0;
+	hr_data_array[index].temperature = 0;
+    hr_1khz_data_counter++;
+	return;
+}
+
+void convert_and_setData(void)
+{
+	float acc_g[3];
+	float gyr_dps[3];
+	float temp;
+	float grv_quat[4];
+	/* Convert data to float before send it to the terminal */
+	fixedpoint_to_float(output.acc_cal_q16, acc_g, 16, 3);
+	fixedpoint_to_float(output.gyr_cal_q16, gyr_dps, 16, 3);
+	fixedpoint_to_float(&output.temp_degC_q16, &temp, 16, 1);
+	fixedpoint_to_float(output.grv_quat_q30, grv_quat, 30, 4);
+
+	setData(gyr_dps,acc_g,grv_quat,temp);
+}
+
+unsigned int writeDataToBuffAndSend()
+{
+	static char send_buff[512]; //buffer for send data.
+	// 2bytes for header, 3bytes for option command, 2 bytes for crc 
+	#define send_data_size_1000hz_except_imudata 2 + 3 + 2
+	#define SEND_PACKET_HEADER_SIZE (2 + 3)
+	memset(send_buff,0,sizeof(send_buff));
+	send_buff[0] = SEND_PACKET_HEADER;
+	send_buff[1] = SEND_PACKET_HEADER;
+	send_buff[2] = error_lists[0] | (error_lists[1] << 4); // INV_ERROR + INV_ERROR_TRANSPORT
+	send_buff[3] = error_lists[2] | (error_lists[3] << 4); // INV_ERROR_SIZE + INV_ERROR_BAD_ARG
+	error_lists[0] = 0;
+	error_lists[1] = 0;
+	error_lists[2] = 0;
+	error_lists[3] = 0;
+	send_buff[send_data_size_1000hz_except_imudata + hr_data_array_byte_size - 2] = SEND_PACKET_CRC_DUMMY;
+	send_buff[send_data_size_1000hz_except_imudata + hr_data_array_byte_size - 1] = SEND_PACKET_CRC_DUMMY;
+    if(hr_1khz_data_counter > hr_data_array_elem_size)
+    {
+        const int first_index = hr_1khz_data_counter % hr_data_array_elem_size;
+        const int first_size = hr_data_array_elem_size - first_index;
+        const int write_size = first_size * sizeof(HR_IMUdata);
+        memcpy(send_buff + SEND_PACKET_HEADER_SIZE, &hr_data_array[first_index], write_size);
+        memcpy(send_buff + write_size + SEND_PACKET_HEADER_SIZE, hr_data_array + SEND_PACKET_HEADER_SIZE, (hr_data_array_elem_size - first_size) * sizeof(HR_IMUdata));
+    }
+    else if(hr_1khz_data_counter == hr_data_array_elem_size)
+    {
+        memcpy(send_buff + SEND_PACKET_HEADER_SIZE, &hr_data_array, sizeof(hr_data_array));
+    }
+    else
+    {
+        memcpy(send_buff + SEND_PACKET_HEADER_SIZE, &hr_data_array, sizeof(HR_IMUdata) * hr_1khz_data_counter);
+    }
+    unsigned int ret = (hr_1khz_data_counter > hr_data_array_elem_size) ? hr_data_array_elem_size : hr_1khz_data_counter;
+	send_buff[4] = (unsigned char)ret;
+    hr_1khz_data_counter = 0;
+	unsigned int remain_send_size = send_data_size_1000hz_except_imudata + hr_data_array_byte_size;
+	unsigned int head_index = 0;
+	static const unsigned int unit_size = 128;
+	while(true)
+	{
+		if(remain_send_size < unit_size){
+			outputData1000hzToHR46(send_buff + head_index,remain_send_size);
+			break;
+		}
+		outputData1000hzToHR46(send_buff + head_index,unit_size);
+		head_index += unit_size;
+		remain_send_size -= unit_size;
+	}
+    return ret;
+}
+
+
+/* Bias previously stored in flash */
+typedef struct sensor_biases {
+	int32_t bias_q16[3];
+	uint8_t is_saved;
+} sensor_biases_t;
+
+
 /* --------------------------------------------------------------------------------------
  *  Functions definition
  * -------------------------------------------------------------------------------------- */
@@ -332,6 +483,16 @@
 
 void HandleInvDeviceFifoPacket(inv_icm426xx_sensor_event_t *event)
 {
+	if(event->error_status != 0)
+	{	
+		// Some error occured.
+		if(hr46_output_mode == HR46_1000HZ_MODE)
+		{
+			setErrorData(event->error_status);
+		}
+		return;
+	}
+
 	uint64_t irq_timestamp = 0;
 	uint64_t extended_timestamp;
 
@@ -397,9 +558,22 @@
 	/* Print data based on the gyro rate */
 	if (output.mask & INVN_ALGO_AGM_OUTPUT_MASK_GYRO_CAL) {
 		iter_algo++;
-
-		if (iter_algo % ((int)(print_period_us / odr_bitfield_to_us(GYRO_FREQ))) == 0)
-			print_algo_inputs_outputs();
+		if(hr46_output_mode == HR46_1000HZ_MODE)
+		{
+			convert_and_setData(); //setting data
+			if(data_to_print_hr_1khz > 0)
+			{
+				writeDataToBuffAndSend();
+				data_to_print_hr_1khz = 0;
+			}
+			data_to_print_hr = 0;
+			data_to_print = 0;
+		}
+		else
+		{
+			if (iter_algo % ((int)(print_period_us / odr_bitfield_to_us(GYRO_FREQ))) == 0)
+				print_algo_inputs_outputs();
+		}
 	}
 }
 
@@ -491,6 +665,7 @@
 /* --------------------------------------------------------------------------------------
  *  Static functions definition
  * -------------------------------------------------------------------------------------- */
+
 static void print_algo_inputs_outputs(void)
 {
 	float acc_g[3];
@@ -554,7 +729,7 @@
 	/* Print outputs */
 	if (data_to_print & MASK_PRINT_ACC_DATA) {
 		INV_MSG(INV_MSG_LEVEL_INFO,
-		        "%lld: OUTPUT Acc=[%.3f, %.3f, %.3f]g AccBias=[%.3f, %.3f, %.3f]mg Accuracy=%d",
+		        "%lld: OUTPUT Acc=[%.3f, %.3f, %.3f]g AccBias=[%.3f, %.3f, %.3f]mg Accuracy=%d",               //require data to print hr46
 		        input.sRimu_time_us, acc_g[0], acc_g[1], acc_g[2], acc_bias_g[0] * 1000,
 		        acc_bias_g[1] * 1000, acc_bias_g[2] * 1000, (int32_t)output.acc_accuracy_flag);
 	}
@@ -563,7 +738,7 @@
 		INV_MSG(
 		    INV_MSG_LEVEL_INFO,
 		    "%lld: OUTPUT Gyr=[%.3f, %.3f, %.3f]dps GyrBias=[%.3f, %.3f, %.3f]dps Temp=[%.2f]C Accuracy=%d",
-		    input.sRimu_time_us, gyr_dps[0], gyr_dps[1], gyr_dps[2], gyr_bias_dps[0],
+		    input.sRimu_time_us, gyr_dps[0], gyr_dps[1], gyr_dps[2], gyr_bias_dps[0],                         //require data to print hr46
 		    gyr_bias_dps[1], gyr_bias_dps[2], temp, (int32_t)output.gyr_accuracy_flag);
 	}
 
@@ -591,7 +766,7 @@
 #endif
 
 	if (data_to_print & MASK_PRINT_6AXIS_DATA) {
-		INV_MSG(INV_MSG_LEVEL_INFO, "%lld: OUTPUT 6Axis=[%f, %f, %f, %f]", input.sRimu_time_us,
+		INV_MSG(INV_MSG_LEVEL_INFO, "%lld: OUTPUT 6Axis=[%f, %f, %f, %f]", input.sRimu_time_us,                   //require data to print hr46
 		        grv_quat[0], grv_quat[1], grv_quat[2], grv_quat[3]);
 
 		INV_MSG(INV_MSG_LEVEL_INFO, "%lld: OUTPUT Euler6Axis=[%.2f, %.2f, %.2f]deg",
@@ -607,7 +782,11 @@
 		        input.sRimu_time_us, linear_acc[0], linear_acc[1], linear_acc[2],
 		        output.acc_accuracy_flag);
 	}
-
+	if(data_to_print_hr > 0)
+	{
+		printDataToHR(acc_g,gyr_dps,grv_quat,temp);
+		data_to_print_hr = 0;
+	}
 	/* Print cariage return to ease reading, only if some data are printed */
 	if (data_to_print)
 		INV_MSG(INV_MSG_LEVEL_INFO, "");
