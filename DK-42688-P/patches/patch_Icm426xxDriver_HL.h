--- Invn/Drivers/Icm426xx/Icm426xxDriver_HL.h	Tue Feb 28 08:42:10 2023
+++ Invn/Drivers/Icm426xx/Icm426xxDriver_HL.h	Mon Jun 10 20:45:55 2024
@@ -164,6 +164,7 @@
 	int16_t  temperature;
 	int8_t   accel_high_res[3];
 	int8_t   gyro_high_res[3];
+	int16_t  error_status;    // Error status of the packet
 } inv_icm426xx_sensor_event_t;
 
 /** @brief Icm426xx driver states definition 
