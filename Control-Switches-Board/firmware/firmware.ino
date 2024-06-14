#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h>
#endif

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
  ButtonStateEnum last_button_state;
  volatile uint8_t red_pushed_count = 0;
  volatile uint8_t green_pushed_count = 0;
  uint8_t red_state_reset_count = 0;
  // ボタン状態を読み出す。これはメインループで一回呼ばれる事を想定している。
  ButtonStateEnum readButtonState()
  {
    ButtonStateEnum return_state;
    if (red_pushed_count > 2)
    {
      return_state = ButtonStateEnum::RED_PUSHED_OVER_3TIMES;
      red_state_reset_count = 0;
      red_pushed_count = 0;
      green_pushed_count = 0;
    }
    else if (red_pushed_count > 0)
    {
      return_state = ButtonStateEnum::RED_PUSHED;
    }
    else if (green_pushed_count > 0)
    {
      return_state = ButtonStateEnum::GREEN_PUSHED;
    }
    else
    {
      return_state = ButtonStateEnum::NOT_PUSHED;
    }
    green_pushed_count = 0;
    red_state_reset_count++;
    if (red_state_reset_count > PUSHCOUNT_RESET_THRESHOLD)
    {
      red_pushed_count = 0;
      red_state_reset_count = 0;
    }
    if (return_state != ButtonStateEnum::NOT_PUSHED)
    {
      if ((last_button_state == ButtonStateEnum::RED_PUSHED_OVER_3TIMES) && (return_state == ButtonStateEnum::RED_PUSHED))
      {
        // 何もしない
        last_button_state = last_button_state;
      }
      else
      {
        last_button_state = return_state;
      }
    }
    return return_state;
  }

  // メインループの周期でボタン状態を更新する
  ButtonStateEnum lastButtonState()
  {
    ButtonStateEnum tmp = last_button_state;
    last_button_state = ButtonStateEnum::NOT_PUSHED;
    return tmp;
  }
};

static ButtonState button_state;

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
    button_state.red_pushed_count++;
    red_prev_timer = now;
  }
}

void green_pushed(void)
{
  static unsigned long green_prev_timer = 0;
  unsigned long now = millis();
  if (now - green_prev_timer > 20)
  {
    button_state.green_pushed_count++;
    green_prev_timer = now;
  }
}

void setup()
{
#if defined(__AVR_ATtiny85__) && (F_CPU == 16000000)
  clock_prescale_set(clock_div_1);
#endif
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

  Serial1.begin(1000000, SERIAL_8N1, RX_PIN, TX_PIN); // 0がusbのやつらしい
  pinMode(TXDEN_PIN, OUTPUT);

  pixels.begin();
  pixels.clear();
  pixels.show(); // 最初はledをリセットする
}

void loop()
{
  digitalWrite(TXDEN_PIN, LOW);    // 受信可能にする
  delay(DELAYVAL_MS - 2); // 他で送れる事があるので、少し早くする
  auto current_state = button_state.readButtonState();
  if (Serial1.available() > 1) // 2byte以上来たら読み込む
  {
    uint16_t receive_data = 0;
    uint8_t read_count = 0;
    while (Serial1.available())
    {
      uint8_t tmp = Serial1.read();
      if (read_count == 0)
        receive_data = tmp;
      else
        receive_data = (receive_data << 8) | tmp;
      read_count++;
    }
    if ((receive_data & REQUEST_BUTTON_STATE) > 0)
    {
      auto last_state = button_state.lastButtonState();
      char send_buf = static_cast<char>(last_state);
      digitalWrite(TXDEN_PIN, HIGH); // 送信可能にする
      Serial1.write(send_buf);
      Serial1.flush();
      // debug用出力を要求された時
      // if ((receive_data & (1 << 14)) != 0)
      // {
      //   if (last_state == ButtonStateEnum::RED_PUSHED_OVER_3TIMES)
      //   {
      //     Serial.write("Red pushed over 3 times");
      //   }
      //   else if (last_state == ButtonStateEnum::RED_PUSHED)
      //   {
      //     Serial.write("Red pushed");
      //   }
      //   else if (last_state == ButtonStateEnum::GREEN_PUSHED)
      //   {
      //     Serial.write("Green pushed");
      //   }
      //   else
      //   {
      //     Serial.write("Not pushed");
      //   }
      // }
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
