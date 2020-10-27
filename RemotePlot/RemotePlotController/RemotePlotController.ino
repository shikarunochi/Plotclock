//PlotClock Controller for M5Stack Core2
#include <M5Core2.h>
#include <esp_now.h>
#include <WiFi.h>

const uint8_t peer_addr[6]
  = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; //M5ATOM MacAddress
esp_now_peer_info_t m5atom;
uint8_t data[3];

#define INIT 0
#define MOVE_AND_LIFT_DOWN 1
#define MOVE 2
#define LIFT_UP 3
#define SWEEP 4

boolean lift_up_flag = false;
boolean isSending = false;
int waitMillSecond = 0;
unsigned long sendTime;

int pre_x = -1;
int pre_y = -1;

void OnDataSent(const uint8_t* mac, esp_now_send_status_t status) {
  isSending = false;
}

void setup() {
  M5.begin(true, true, true, true);

  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextColor(BLACK);
  M5.Lcd.setTextSize(2);
  memset(&m5atom, 0, sizeof(m5atom));
  memcpy(m5atom.peer_addr,  peer_addr, 6);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_now_init();
  esp_now_register_send_cb(OnDataSent);
  esp_now_add_peer((const esp_now_peer_info_t*)&m5atom);

  data[0] = INIT;
  data[1] = 0;
  data[2] = 0;
  esp_now_send((const uint8_t*)m5atom.peer_addr, data, 3);
}

void loop() {
  TouchPoint_t pos = M5.Touch.getPressPoint();
  //Serial.printf("x:%d, y:%d\n", pos.x, pos.y);
  if (pos.x > -1) { //タッチされてたら
    //Cボタンの位置を押されていたら→クリーナー
    if (pos.y > 240) {
      if (pos.x > 218) {
        data[0] = SWEEP;
        data[1] = 0;
        data[2] = 0;
        esp_now_send((const uint8_t*)m5atom.peer_addr, data, 3);
        lift_up_flag = true;
        M5.Lcd.fillScreen(BLACK);
        delay(1000);
      }
    } else {
      if (lift_up_flag == true) {
        //ペンが上がってたら→タッチ位置まで移動して、ペンを下げる
        data[0] = MOVE_AND_LIFT_DOWN;
        data[1] = pos.x;
        data[2] = pos.y;
        pre_x = pos.x;
        pre_y = pos.y;
        lift_up_flag = false;
        esp_now_send((const uint8_t*)m5atom.peer_addr, data, 3);
        delay(10);
      } else {
        if (pre_x != pos.x || pre_y != pos.y) {
          data[0] = MOVE;
          data[1] = pos.x;
          data[2] = pos.y;
          //ペンが下がってたら→タッチ位置に移動
          pre_x = pos.x;
          pre_y = pos.y;
          esp_now_send((const uint8_t*)m5atom.peer_addr, data, 3);
          delay(10);
        }
      }
      //画面の位置に円を描く
      M5.Lcd.fillCircle(pos.x, pos.y, 5, WHITE);
    }
  } else {
    if (lift_up_flag == false) {
      //ペンが下がってたら→ペンを上げる
      data[0] = LIFT_UP;
      data[1] = pre_x;
      data[2] = pre_y;
      esp_now_send((const uint8_t*)m5atom.peer_addr, data, 3);
      lift_up_flag = true;
      delay(10);
    }
  }

  delay(10);
}
