## Audio-Board

- id: 0xA3

### 特定の音声ファイルを再生する
- command: 0xA0
- opt: 0x00
- loop: 1 (連続再生は基本不可。再生終了まで待つ必要がある)

sample.py内のtxDataを変更することで再生するファイルを変更できる
- `txData = [1]` 001_* を再生

```bash
python3 sample.py --id 0xA3 --cmd 0xA0 --loop 1
```

## Control-Switches-Board

- id: 0xA1

### LEDの設定とボタン状態の取得
- command: 0xA0
- opt: 0x00
- loop: None

sample.py内のtxDataを変更することでLEDを点灯消灯できる。
- `txData = [0b00100100, 0b00001001]` 4つのLEDを青色に点灯（暗）
- `txData = [0b10010010, 0b00010100]` 4つのLEDを緑色に点灯（明）
- `txData = [0b00010001, 0b00011111]` 4つのLEDを赤、緑、青、白色に点灯（明）

#### Arduinoに送信するパケットの構造(2 byte):
- 0~11ビット: LED状態
  - 0~2ビット: 1つ目のLED
  - 3~5ビット: 2つ目のLED
  - 6~8ビット: 3つ目のLED
  - 9~11ビット: 4つ目のLED
- 12ビット目: 輝度設定 (1: 明 / 0: 暗)
- 13ビット目: LEDリセット (1: リセット / 0: リセットなし)
- 14ビット目: 未使用
- 15ビット目: 未使用

LED状態は最下位ビットからRGB順に配置されます。
各RGBにはON/OFFの2状態のみがあります。データはリトルエンディアン形式です。

#### Arduinoから返されるパケットの構造(1 byte):
- ボタン状態
  - 'n': NOT_PUSHED - 0x6E
  - 'r': RED_PUSHED - 0x72
  - 'R': RED_PUSHED_OVER_3TIMES - 0x52
  - 'g': GREEN_PUSHED - 0x67

```bash
python3 sample.py --id 0xA1 --cmd 0xA0
```

---