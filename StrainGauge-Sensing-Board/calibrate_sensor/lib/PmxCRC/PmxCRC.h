/** 
* @file PmxCRC.h
* @brief  PMX CRC16 math library header file
* @author Kondo Kagaku Co.,Ltd.
* @author T.Nobuhara
* @date 2023/12/13
* @version 1.0.0
* @copyright Kondo Kagaku Co.,Ltd. 2023
*/

#ifndef __Pmx_CRC_h__
#define __Pmx_CRC_h__

//#include "Arduino.h"

/// @brief PMXで使用するCRCの演算
/// @details
///  * 生成多項式：x^16+x^12+x^5+1 (CRC-16-CCITT)
///  * ビットシフト方向：左送り
///  * 初期値：0x0000
///  * 出力：非反転 （出力XOR：0x0000）
class PmxCrc16
{ 
    public:
        //PMXのCRCを計算する
        static unsigned int getCrc16(unsigned char data[], int length);


        //PMXのデータにCRCを追加する
        static void setCrc16(unsigned char data[]);

        //PMXの受信データのCRCをチェックする
        static bool checkCrc16(unsigned char data[]);
};

#endif
