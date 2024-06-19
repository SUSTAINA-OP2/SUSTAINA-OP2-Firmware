--- Invn/Drivers/Icm426xx/Icm426xxDriver_HL.c	Tue Feb 28 08:42:10 2023
+++ Invn/Drivers/Icm426xx/Icm426xxDriver_HL.c	Mon Jun 10 20:56:10 2024
@@ -1002,17 +1002,28 @@
 	uint16_t packet_size = FIFO_HEADER_SIZE + FIFO_ACCEL_DATA_SIZE + FIFO_GYRO_DATA_SIZE +
 	                       FIFO_TEMP_DATA_SIZE + FIFO_TS_FSYNC_SIZE;
 	fifo_header_t *header;
+	inv_icm426xx_sensor_event_t error_event;
 
 	/* Ensure data ready status bit is set */
 	status |= inv_icm426xx_read_reg(s, MPUREG_INT_STATUS, 1, &int_status);
 	if (status)
+	{
+		error_event.error_status = status;
+		if (s->sensor_event_cb)
+			s->sensor_event_cb(&error_event);
 		return status;
+	}
 
 	if ((int_status & BIT_INT_STATUS_FIFO_THS) || (int_status & BIT_INT_STATUS_FIFO_FULL)) {
 		/* FIFO record mode configured at driver init, so we read packet number, not byte count */
 		status |= inv_icm426xx_read_reg(s, MPUREG_FIFO_COUNTH, 2, data);
 		if (status != INV_ERROR_SUCCESS)
+		{
+			error_event.error_status = status;
+			if (s->sensor_event_cb)
+				s->sensor_event_cb(&error_event);
 			return status;
+		}
 		inv_icm426xx_format_data(ICM426XX_INTF_CONFIG0_DATA_LITTLE_ENDIAN, data, &packet_count);
 
 		if (packet_count > 0) {
@@ -1038,6 +1049,9 @@
 						/* sensor data is in FIFO according to FIFO_COUNT but failed to read FIFO,
 							  reset FIFO and try next chance */
 						inv_icm426xx_reset_fifo(s);
+						error_event.error_status = status;
+						if (s->sensor_event_cb)
+							s->sensor_event_cb(&error_event);
 						return status;
 					}
 				}
@@ -1048,6 +1062,9 @@
 					/* sensor data is in FIFO according to FIFO_COUNT but failed to read FIFO,
 						  reset FIFO and try next chance */
 					inv_icm426xx_reset_fifo(s);
+					error_event.error_status = status;
+					if (s->sensor_event_cb)
+						s->sensor_event_cb(&error_event);
 					return status;
 				}
 			}
@@ -1072,6 +1089,9 @@
 				if (header->bits.msg_bit) {
 					/* MSG BIT set in FIFO header, Resetting FIFO */
 					inv_icm426xx_reset_fifo(s);
+					error_event.error_status = INV_ERROR;
+					if (s->sensor_event_cb)
+						s->sensor_event_cb(&error_event);
 					return INV_ERROR;
 				}
 
@@ -1201,7 +1221,7 @@
 
 				if (header->bits.twentybits_bit)
 					fifo_idx += FIFO_ACCEL_GYRO_HIGH_RES_SIZE;
-
+				event.error_status = 0;
 				/* call sensor event callback */
 				if (s->sensor_event_cb)
 					s->sensor_event_cb(&event);
