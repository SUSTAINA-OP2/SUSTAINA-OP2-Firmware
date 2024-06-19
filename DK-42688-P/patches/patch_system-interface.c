--- examples/system-interface/system-interface.c	Tue Feb 28 08:42:10 2023
+++ examples/system-interface/system-interface.c	Wed May 22 20:33:00 2024
@@ -2,7 +2,7 @@
  * ________________________________________________________________________________________________________
  * Copyright (c) 2016-2016 InvenSense Inc. All rights reserved.
  *
- * This software, related documentation and any modifications thereto (collectively �Software�) is subject
+ * This software, related documentation and any modifications thereto (collectively �Software�E is subject
  * to InvenSense and its licensors' intellectual property rights under U.S. and international copyright
  * and other intellectual property rights laws.
  *
@@ -58,7 +58,7 @@
 	inv_uart_mngr_init_struct_t uart_mngr_config;
 
 	uart_mngr_config.uart_num  = log_uart_id;
-	uart_mngr_config.baudrate  = 921600;
+	uart_mngr_config.baudrate  = 2000000; // original = 921600
 	uart_mngr_config.flow_ctrl = INV_UART_FLOW_CONTROL_NONE;
 	inv_uart_mngr_init(&uart_mngr_config);
 }
@@ -195,6 +195,7 @@
 int akm_io_hal_init(void *serif)
 {
 	(void)serif;
+
 
 	inv_i2c_master_init();
 	return 0;
