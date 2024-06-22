--- examples/system-interface/system-interface.c	Tue Feb 28 08:42:10 2023
+++ examples/system-interface/system-interface.c	Wed May 22 20:33:00 2024
@@ -58,7 +58,7 @@
 	inv_uart_mngr_init_struct_t uart_mngr_config;
 
 	uart_mngr_config.uart_num  = log_uart_id;
-	uart_mngr_config.baudrate  = 921600;
+	uart_mngr_config.baudrate  = 2000000; // original = 921600
 	uart_mngr_config.flow_ctrl = INV_UART_FLOW_CONTROL_NONE;
 	inv_uart_mngr_init(&uart_mngr_config);
 }