
## Audio-Board

id: 0xA3

特定の音声ファイルを再生する
command: 0xA0
opt: 0x00 ~ 0x63 (再生するファイル番号)
loop: 1 (連続再生は基本不可．再生終了まで待つ必要がある)

```.bash
python3 sample.py --id 0xA3 --cmd 0xA0 --opt 0x0A --loop 1
```
## Control-Switches-Board

id: 0xA1
command: 0xA0
opt: 0x00
loop: None

sample.py内のtxDataを変えることでLEDを点灯消灯できる
例：txData = [0b00100100, 0b00001001] 4つのLEDを青色に点灯（暗）
例：txData = [0b10010010, 0b00010100] 4つのLEDを緑色に点灯（明）
例：txData = [0b00010001, 0b00011111] 4つのLEDを赤，緑，青，白色に点灯（明）

Packet structure to send to Arduino.
0~11 bits| LED state
      0~2  | 1st LED
      3~5  | 2nd LED
      6~8  | 3rd LED
      9~11 | 4th LED
12-bit | Brightness setting (1: bright / 0: dim)
13-bit | LED reset (1: reset / 0: no reset)
14-bit | not used
15-bit | not used
LED state is arranged in RGB order from the least significant bit. Each RGB has only two states: ON/OFF.
Data is in little endian format.

Packet structure returned from Arduino.
1 byte Button State
  'n': NOT_PUSHED - 0x6E
  'r': RED_PUSHED - 0x72
  'R': RED_PUSHED_OVER_3TIMES - 0x52
  'g': GREEN_PUSHED - 0x67

```.bash
python3 sample.py --id 0xA1 --cmd 0xA0
```