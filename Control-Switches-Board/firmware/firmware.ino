#include "src/Adafruit_NeoPixel/Adafruit_NeoPixel.h"
#include "./src/BitOperations/crc16.hpp"
#include <vector>

// PIN宣言
constexpr byte NEOPIXEL_LED_PIN = 6; // これだけはesp32の方のピン番号
constexpr byte RX_PIN = 0;
constexpr byte TX_PIN = 1;
constexpr byte TXDEN_PIN = 2;
constexpr byte GREEN_SWITCH_PIN = 4;
constexpr byte RED_SWITCH_PIN = 5;
constexpr byte BOARD_LED_RED = 14;
constexpr byte BOARD_LED_GREEN = 16;
constexpr byte BOARD_LED_BLUE = 15;

// シリアル通信用の定数
constexpr long SERIAL_BAUDRATE = 1000000;

constexpr uint8_t headerPacket[] = {0xFE, 0xFE};

//! USB-to-Quad-RS-485-Conv-Module:  0xA0 (ID: 160)
//! Control-Switches-Board:          0xA1 (ID: 161)
//! Power Logging Board:             0xA2 (ID: 162)
//! Audio Board:                     0xA3 (ID: 163)
constexpr uint8_t id = 0xA1;

constexpr uint8_t firmwareVersion = 0x01;

//! rx packet: headder + (id + length + command + option ) + data * n + crc
constexpr size_t headerPacket_length = sizeof(headerPacket);
constexpr size_t crc_length = sizeof(uint16_t);
constexpr size_t rxPacket_forward_length = headerPacket_length + 4;
constexpr size_t rxPacket_min_length = rxPacket_forward_length + crc_length;

//! tx packet: headder + (id + command + length + error) + txData + crc
constexpr size_t txPacket_min_length = headerPacket_length + 4 + crc_length;

// コマンドの定義
enum class CommandList : uint8_t
{
  GETSTATEANDSETLEDCOMMAND = 0xD0,
  CHECK_FIRMWARE_VER = 0xF0,
  UNSUPPORTED_COMMAND = 0xFF
};

constexpr uint8_t crc_errorStatus = 0b00000010;
constexpr uint8_t commandUnsupport_errorStatus = 0b00000010;
constexpr uint8_t commandProcessing_errorStatus = 0b00000100;
constexpr uint8_t return_command_mask = 0b01111111;

std::vector<uint8_t> txData;

// 100hzを基準とする。
constexpr uint8_t DELAYVAL_MS = 10;
constexpr uint8_t LOOP_HZ = 1000 / DELAYVAL_MS;

constexpr uint8_t NUM_NEOPIXEL_LEDS = 4;

Adafruit_NeoPixel pixels(NUM_NEOPIXEL_LEDS, NEOPIXEL_LED_PIN, NEO_GRB + NEO_KHZ800);

constexpr uint8_t LED_ON = 1;
constexpr uint8_t LED_OFF = 0;
constexpr uint16_t REQUEST_BUTTON_STATE = (1 << 15);

constexpr uint8_t PUSHCOUNT_RESET_THRESHOLD = LOOP_HZ / 2; // 0.5秒以上要求されなかったらリセットする。


/*
arduinoに送るパケットの構成
0~11bit LED状態
  0~2 1つめ
  3~5 2つめ
  6~8 3つめ
  9~11 4つめ
12bit 明るさの設定（1:明るい/0:暗い）
13bit ledのリセット（1:する/0:しない）
14bit 無し
15bit ボタン状態要求(有り/無し)

LED状態は下の桁から、rgbの順番で並んでいる。それぞれのRGBは、ON/OFFの2状態しかない。
送る側は、上位の1byteを先に、下位を後に送る.
*/

/*
arduinoから返ってくるパケットの構成
1byte ボタン状態
  'n': NOT_PUSHED
  'r': RED_PUSHED
  'R': RED_PUSHED_OVER_3TIMES
  'g': GREEN_PUSHED
*/
enum class ButtonStateEnum
{
  NOT_PUSHED = 'n',
  RED_PUSHED = 'r',
  RED_PUSHED_OVER_3TIMES = 'R',
  GREEN_PUSHED = 'g'
};

struct ButtonState
{
  ButtonStateEnum last_button_state_;
  volatile uint8_t red_pushed_count_ = 0;
  volatile uint8_t green_pushed_count_ = 0;
  uint8_t red_state_reset_count_ = 0;
  // ボタン状態を読み出す。これはメインループで一回呼ばれる事を想定している。
  ButtonStateEnum readButtonState()
  {
    ButtonStateEnum return_state;
    if (red_pushed_count_ > 2)
    {
      return_state = ButtonStateEnum::RED_PUSHED_OVER_3TIMES;
      red_state_reset_count_ = 0;
      red_pushed_count_ = 0;
      green_pushed_count_ = 0;
    }
    else if (red_pushed_count_ > 0)
    {
      return_state = ButtonStateEnum::RED_PUSHED;
    }
    else if (green_pushed_count_ > 0)
    {
      return_state = ButtonStateEnum::GREEN_PUSHED;
    }
    else
    {
      return_state = ButtonStateEnum::NOT_PUSHED;
    }
    green_pushed_count_ = 0;
    red_state_reset_count_++;
    if (red_state_reset_count_ > PUSHCOUNT_RESET_THRESHOLD)
    {
      red_pushed_count_ = 0;
      red_state_reset_count_ = 0;
    }
    if (return_state != ButtonStateEnum::NOT_PUSHED)
    {
      if ((last_button_state_ == ButtonStateEnum::RED_PUSHED_OVER_3TIMES) && (return_state == ButtonStateEnum::RED_PUSHED))
      {
        // 何もしない
        last_button_state_ = last_button_state_;
      }
      else
      {
        last_button_state_ = return_state;
      }
    }
    return return_state;
  }

  // メインループの周期でボタン状態を更新する
  ButtonStateEnum lastButtonState()
  {
    ButtonStateEnum tmp = last_button_state_;
    last_button_state_ = ButtonStateEnum::NOT_PUSHED;
    return tmp;
  }
};


ButtonState& getButtonState()
{
  static ButtonState button_state;
  return button_state;
}

void setLedState(const uint16_t target_state)
{
  for (int i = 0; i < NUM_NEOPIXEL_LEDS; i++)
  {
    pixels.setPixelColor(i, pixels.Color((target_state & (1 << (i * 3 + 0))) ? 125 : 0, (target_state & (1 << (i * 3 + 1))) ? 125 : 0, (target_state & (1 << (i * 3 + 2))) ? 125 : 0));
    // pixels.setPixelColor(i / 3, pixels.Color(0, 0, 0));
  }
  if ((target_state & (1 << 12)) == 0)
  {
    pixels.setBrightness(50);
  }
  else
  {
    pixels.setBrightness(255);
  }
  if ((target_state & (1 << 13)) != 0)
  {
    pixels.clear();
  }
  pixels.show(); // これを1度しか呼ばなくて良いように工夫するべき。
}

void red_pushed(void)
{
  static unsigned long red_prev_timer = 0;
  unsigned long now = millis();
  if (now - red_prev_timer > 20)
  {
    getButtonState().red_pushed_count_++;
    red_prev_timer = now;
  }
}

void green_pushed(void)
{
  static unsigned long green_prev_timer = 0;
  unsigned long now = millis();
  if (now - green_prev_timer > 20)
  {
    getButtonState().green_pushed_count_++;
    green_prev_timer = now;
  }
}

void setup()
{
  pinMode(GREEN_SWITCH_PIN, INPUT_PULLUP);
  pinMode(RED_SWITCH_PIN, INPUT_PULLUP);
  pinMode(BOARD_LED_RED, OUTPUT);
  pinMode(BOARD_LED_GREEN, OUTPUT);
  pinMode(BOARD_LED_BLUE, OUTPUT);

  attachInterrupt(GREEN_SWITCH_PIN, green_pushed, FALLING);
  attachInterrupt(RED_SWITCH_PIN, red_pushed, FALLING);

  digitalWrite(BOARD_LED_RED, HIGH);   // Red ON
  digitalWrite(BOARD_LED_GREEN, HIGH); // Green OFF
  digitalWrite(BOARD_LED_BLUE, LOW);   // Blue ON

  Serial1.begin(SERIAL_BAUDRATE, SERIAL_8N1, RX_PIN, TX_PIN); // 0がusbのやつらしい
  pinMode(TXDEN_PIN, OUTPUT);

  pixels.begin();
  pixels.clear();
  pixels.show(); // 最初はledをリセットする
}

void loop()
{
  digitalWrite(TXDEN_PIN, LOW);    // enable receiving
  delay(DELAYVAL_MS - 2); // 他で送れる事があるので、少し早くする
  txData.clear();
  auto current_state = getButtonState().readButtonState();
  if (Serial1.available() > 1) // 2byte以上来たら読み込む
  {
    uint16_t receive_data = 0;
    if (Serial1.available() >= rxPacket_min_length)
    {
      uint8_t rxPacket_forward[rxPacket_forward_length] = {};
      uint8_t tx_errorStatus = 0b00000000;

      if (checkHeader(headerPacket, headerPacket_length, rxPacket_forward))
      {
        for (int i = headerPacket_length; i < rxPacket_forward_length; i++)
        {
          rxPacket_forward[i] = Serial1.read();
        }

        uint8_t rxBoardType = rxPacket_forward[headerPacket_length];
        size_t rxPacket_length = rxPacket_forward[headerPacket_length + 1];
        uint8_t rxCommand = rxPacket_forward[headerPacket_length + 2];
        uint8_t rxOption = rxPacket_forward[headerPacket_length + 3];


        //! make rxPaket
        uint8_t rxPacket[rxPacket_length] = {};
        for (int i = 0; i < rxPacket_length; i++)
        {
          if (i < rxPacket_forward_length)
          {
            rxPacket[i] = rxPacket_forward[i];
          }
          else
          {
            rxPacket[i] = Serial1.read();
          }
        }

        if (calcCRC16_XMODEM(rxPacket, rxPacket_length - crc_length) == (uint16_t)(rxPacket[rxPacket_length - crc_length] << 8) | (uint16_t)(rxPacket[rxPacket_length - crc_length + 1]))
        {
          auto command_type = processCommand(rxCommand, &tx_errorStatus, rxPacket);
          if(command_type == CommandList::GETSTATEANDSETLEDCOMMAND)
          {
            uint8_t receive_data_1 = rxPacket[rxPacket_forward_length];
            uint8_t receive_data_2 = rxPacket[rxPacket_forward_length + 1];
            receive_data = (receive_data_2 << 8) | receive_data_1;
          }
        }
        else
        {
          tx_errorStatus |= crc_errorStatus;
        }

        //! tx packet: headder + (command + length + error) + txData + crc
        //! data: (address + voldtage + cureent) * n
        size_t txPacket_length = txPacket_min_length + txData.size();

        //! make txPacket
        uint8_t txPacket[txPacket_length] = {};
        size_t packetIndex = 0;

        //! add forward txPacket
        memcpy(txPacket, headerPacket, headerPacket_length);
        packetIndex += headerPacket_length;

        txPacket[packetIndex++] = id;
        txPacket[packetIndex++] = (uint8_t)txPacket_length;
        txPacket[packetIndex++] = rxCommand & return_command_mask; //! command
        txPacket[packetIndex++] = tx_errorStatus; //! error

        //! add txData to txPacket
        if (!txData.empty())
        {
          memcpy(txPacket + packetIndex, txData.data(), txData.size());
          packetIndex += txData.size();
        }

        //! add CRC to txPacket
        uint16_t txCrc = calcCRC16_XMODEM(txPacket, txPacket_length - crc_length);
        txPacket[packetIndex++] = lowByte(txCrc);
        txPacket[packetIndex++] = highByte(txCrc);

        // Serial.write(txPacket, txPacket_length);
        serial1SendData(txPacket, packetIndex);
      }
    }
    setLedState(receive_data);
  }

  // デバッグ用に、ボードのledを光らせる処理
  if (current_state == ButtonStateEnum::GREEN_PUSHED)
  {
    digitalWrite(BOARD_LED_RED, HIGH);  // Red OFF
    digitalWrite(BOARD_LED_GREEN, LOW); // Green ON
    digitalWrite(BOARD_LED_BLUE, HIGH); // Blue OFF
  }
  else if (current_state == ButtonStateEnum::RED_PUSHED_OVER_3TIMES)
  {
    digitalWrite(BOARD_LED_RED, HIGH);   // Red OFF
    digitalWrite(BOARD_LED_GREEN, HIGH); // Green OFF
    digitalWrite(BOARD_LED_BLUE, LOW);   // Blue ON
  }
  else if (current_state == ButtonStateEnum::RED_PUSHED)
  {
    digitalWrite(BOARD_LED_RED, LOW);    // Red ON
    digitalWrite(BOARD_LED_GREEN, HIGH); // Green OFF
    digitalWrite(BOARD_LED_BLUE, HIGH);  // Blue OFF
  }
}

CommandList processCommand(const uint8_t &command, uint8_t *error, const uint8_t txPacket[])
{
  switch (static_cast<CommandList>(command))
  {
    case CommandList::GETSTATEANDSETLEDCOMMAND:
    {
      auto last_state = getButtonState().lastButtonState();
      char send_buf = static_cast<char>(last_state);
      txData.push_back(send_buf);
      return CommandList::GETSTATEANDSETLEDCOMMAND;
    }
    case CommandList::CHECK_FIRMWARE_VER:
    {
      txData.push_back(firmwareVersion);
      return CommandList::CHECK_FIRMWARE_VER;
    }
    default:
    {
      *error |= commandUnsupport_errorStatus;
      return CommandList::UNSUPPORTED_COMMAND;
    }
  }
}

void serial1SendData(uint8_t *txPacket, const size_t &packet_num)
{
  digitalWrite(TXDEN_PIN, HIGH);
  Serial1.write(txPacket, packet_num);
  Serial1.flush();
  digitalWrite(TXDEN_PIN, LOW);
}

bool checkHeader(const uint8_t header[], const size_t length, uint8_t packet[])
{
  for (int i = 0; i < length; i++)
  {
    if (Serial1.read() != header[i])
    {
      return false;
    }
    packet[i] = header[i];
  }
  return true;
}
