--- examples/example-algo/example-algo.h	Tue Feb 28 08:42:08 2023
+++ examples/example-algo/example-algo.h	Thu Jun 13 20:34:00 2024
@@ -2,7 +2,7 @@
  * ________________________________________________________________________________________________________
  * Copyright (c) 2016-2016 InvenSense Inc. All rights reserved.
  *
- * This software, related documentation and any modifications thereto (collectively “Software”) is subject
+ * This software, related documentation and any modifications thereto (collectively ï¿½Softwareï¿½) is subject
  * to InvenSense and its licensors' intellectual property rights under U.S. and international copyright
  * and other intellectual property rights laws.
  *
@@ -49,7 +49,7 @@
  * Set this define to 0 to disable mag support
  * Recommended value: 1
  */
-#define USE_MAG 1
+#define USE_MAG 0
 
 /*
  * Set power mode flag
@@ -94,7 +94,7 @@
  * High resolution mode : 20 bits data format
  * Warning: Enabling High Res mode will force FSR to 16g and 2000dps
  */
-#define IS_HIGH_RES_MODE 0
+#define IS_HIGH_RES_MODE 1
 
 /* 
  * Set of timers used throughout standalone applications 
@@ -113,6 +113,21 @@
 #define MASK_PRINT_9AXIS_DATA     0x20 /** 9-axis data */
 #define MASK_PRINT_GRAVITY_DATA   0x40 /** Gravity vector */
 #define MASK_PRINT_LINEARACC_DATA 0x80 /** Linear acceleration vector */
+
+/* packet data */
+#define SEND_PACKET_CRC_DUMMY  0xff
+#define SEND_PACKET_HEADER  0xfe
+#define SEND_PACKET_DUMMY_COMMAND  0xfc
+
+/* 1khz output tool for HR46   */
+void setData(const float* gyro,const float* accel,const float* quat,const float temperature);
+void setErrorData(const int16_t error_status);
+unsigned int writeDataToBuffAndSend();
+enum {
+    HR46_200HZ_MODE,
+    HR46_1000HZ_MODE,
+};
+/* -------------------------  */
 
 /**
  * \brief This function is in charge of reseting and initializing Icm426xx device. It should
