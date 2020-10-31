#include "M5Atom.h"
#include "fontData.h"

// Plotclock
// cc - by Johannes Heberlein 2014
// v 1.02
// thingiverse.com/joo   wiki.fablab-nuernberg.de
// units: mm; microseconds; radians
// origin: bottom left of drawing surface
// time library see http://playground.arduino.cc/Code/time 
// RTC  library see http://playground.arduino.cc/Code/time 
//               or http://www.pjrc.com/teensy/td_libs_DS1307RTC.html  
// Change log:
// 1.01  Release by joo at https://github.com/9a/plotclock
// 1.02  Additional features implemented by Dave (https://github.com/Dave1001/):
//       - added ability to calibrate servofaktor seperately for left and right servos
//       - added code to support DS1307, DS1337 and DS3231 real time clock chips
//       - see http://www.pjrc.com/teensy/td_libs_DS1307RTC.html for how to hook up the real time clock 
// 1.03  Fixed the length bug at the servo2 angle calculation, other fixups

// PlotFont modified by @shikarunochi

//http://blog.robotakao.jp/blog-entry-387.html
const uint8_t Srv1 = 23, Srv2 = 19, Srv3 = 22;//GPIO No. //1:lift 2:left 3:right
const uint8_t srv_CH1 = 1, srv_CH2 = 2, srv_CH3 = 3; //チャンネル
const double PWM_Hz = 50;   //PWM周波数
const uint8_t PWM_level = 16; //PWM 16bit(0～65535)
#define WISHY 3 // Offset of the Y coordinats of the plate-wisher
// When in calibration mode, adjust the following factors until the servos move exactly 90 degrees
//#define SERVOFAKTORLEFT 600
//#define SERVOFAKTORRIGHT 600

//大きくするほど広い幅の動き
#define SERVOFAKTORLEFT 590
#define SERVOFAKTORRIGHT 580

// Zero-position of left and right servo
// When in calibration mode, adjust the NULL-values so that the servo arms are at all times parallel
// either to the X or Y axis
//#define SERVOLEFTNULL 1950
//#define SERVORIGHTNULL 815
#define SERVOLEFTNULL 1700
//#define SERVORIGHTNULL 790
//#define SERVORIGHTNULL 850
//大きくするほど左
#define SERVORIGHTNULL 1000

#define SERVOPINLIFT  2
#define SERVOPINLEFT  3
#define SERVOPINRIGHT 4

//#define ZOFF 90
#define ZOFF 200
// lift positions of lifting servo
//#define LIFT0 1110+ZOFF // on drawing surface
#define LIFT0 1170+ZOFF // on drawing surface
//#define LIFT1 995+ZOFF  // between numbers
#define LIFT1 975+ZOFF  // between numbers
//#define LIFT2 735+ZOFF  // going towards sweeper
#define LIFT2 725+ZOFF  // going towards sweeper

// speed of liftimg arm, higher is slower
#define LIFTSPEED 2000

// length of arms
//#define L1 35
//#define L2 55.1
//#define L3 13.2
//#define L4 45
#define L1 34.5
#define L2 55.3
#define L3 13.9
#define L4 44.6

// origin points of left and right servo
#define O1X 24//22
#define O1Y -25
#define O2X 49//47
#define O2Y -25

int servoLift = 1500;

volatile double lastX = 75;
volatile double lastY = 47.5;

void setup()
{

  M5.begin();

  pinMode(Srv1, OUTPUT);
  pinMode(Srv2, OUTPUT);
  pinMode(Srv3, OUTPUT);

  //モータのPWMのチャンネル、周波数の設定
  ledcSetup(srv_CH1, PWM_Hz, PWM_level);
  ledcSetup(srv_CH2, PWM_Hz, PWM_level);
  ledcSetup(srv_CH3, PWM_Hz, PWM_level);

  //モータのピンとチャンネルの設定
  ledcAttachPin(Srv1, srv_CH1);
  ledcAttachPin(Srv2, srv_CH2);
  ledcAttachPin(Srv3, srv_CH3);

  //drawTo(10, 50);
  lift(2);

  delay(5000);
  drawText(15, 30, "鬼滅", 1);
  drawText(20, 15, "の刃", 1 );

  lift(2);
}

void loop()
{
}

void drawText(float x, float y, char * plotText, float scale) {
  float xPos = x;
  float yPos = y;
  Serial.print("plotText:");
  Serial.printf("%s", plotText);
  while ( *plotText != 0x00 ) {
    // 改行処理
    if ( *plotText == '\n' ) {
      xPos = 0;
      yPos = yPos + 10 * scale;
      continue;
    }
    // フォント取得
    uint16_t strUTF16;
    plotText = efontUFT8toUTF16( &strUTF16, plotText );
    Serial.println(strUTF16, HEX);
    for (int index = 0; index < sizeof(fontIndex) / sizeof(fontIndex[0]); index++) {
      if (strUTF16 == fontIndex[index]) {
        drawFont(xPos, yPos, fontDataIndex[index], scale);
        if (strUTF16 < 256) {
          xPos = xPos + 8 * scale;
        } else {
          xPos = xPos + 16 * scale;
        }
        break;
      }
    }
  }
}

char* efontUFT8toUTF16( uint16_t *pUTF16, char *pUTF8 ) {

  // 1Byte Code
  if ( pUTF8[0] < 0x80 ) {
    // < 0x80 : ASCII Code
    *pUTF16 = pUTF8[0];
    return pUTF8 + 1;
  }

  // 2Byte Code
  if ( pUTF8[0] < 0xE0 )  {
    *pUTF16 = ( ( pUTF8[0] & 0x1F ) << 6 ) + ( pUTF8[1] & 0x3F );
    return pUTF8 + 2;
  }

  // 3Byte Code
  if ( pUTF8[0] < 0xF0 ) {
    *pUTF16 = ( ( pUTF8[0] & 0x0F ) << 12 ) + ( ( pUTF8[1] & 0x3F ) << 6 ) + ( pUTF8[2] & 0x3F );
    return pUTF8 + 3;
  } else {
    // 4byte Code
    *pUTF16 = 0;
    return pUTF8 + 4;
  }
}

//X,Y = (5,5)-(67,50)
void drawFont(float x, float y, const int *fontData, float scale) {
  if (fontData == NULL) {
    return;
  }
  float baseX = x;
  float baseY = y;

  int index = 0;
  float plotX = 0;
  float plotY = 0;
  while (fontData[index] != 0x20) {
    int fontByte = fontData[index];
    Serial.println();
    //****    20(Hex)     : Terminator
    //****    21-26,28-3F : Move to X=0--29
    if (fontByte >= 0x21 && fontByte <= 0x3F) {
      if (fontByte <= 0x26) {
        plotX = (float)(fontByte - 0x21);
      } else {
        plotX = (float)(fontByte - 0x28 + (0x26 - 0x21 + 1));
      }
      //ペンを上げてから、X位置をplotXに。
      lift(1);
      moveArm(baseX, baseY, plotX, plotY, scale);
      //****    40-5B,5E-5F : Draw to X=0--29
    } else if (fontByte >= 0x40 && fontByte <= 0x5F) {
      if (fontByte <= 0x5B) {
        plotX = (float)(fontByte - 0x40);
      } else {
        plotX = (float)(fontByte - 0x5E + (0x5B - 0x40 + 1));
      }
      //ペンを下げてから、X位置を plotXに
      lift(0);
      moveArm(baseX, baseY, plotX, plotY, scale);
      //****    60-7D       : Next X to 0--29
    } else if (fontByte >= 0x60 && fontByte <= 0x7D) {
      plotX = fontByte - 0x60;
      //****    7E,A1-BF    : Move to Y=0--31
    } else if (fontByte >= 0x7E && fontByte <= 0xBF) {
      if (fontByte <= 0x7E) {
        plotY = (float)(fontByte - 0x7E);
      } else {
        plotY = (float)(fontByte - 0xA1 + (1));
      }
      //ペンを上げてから、Y位置を plotYに
      lift(1);
      moveArm(baseX, baseY, plotX, plotY, scale);
      //****    C0-DF       : Draw to Y=0--31
    } else if (fontByte >= 0xC0 && fontByte <= 0xDF) {
      plotY = (float)(fontByte - 0xC0);
      //ペンを下げてから、Y位置を plotYに
      lift(0);
      moveArm(baseX, baseY, plotX, plotY, scale);
    }
    //****    27,5C,5D    : Reserved(else=Term);Init=(0,0)
    delay(100);
    index++;
  }

  delay(10);
}

void moveArm(float baseX, float baseY, float plotX, float plotY, float scale) {
  float x = ( plotX / 2.0 ) * scale + baseX;
  float y = ( plotY / 2.0 ) * scale + baseY;
  if(x < 5){
    x = 5;
  }
  if(x > 60){
    x = 60;
  }
  if(y < 0){
    y = 0;
  }
  if(y > 55){
    y = 55;
  }
  drawTo( x, y );
}

void sweep() {
  lift(2);
  drawTo(71.0, 52);
  lift(1);
  delay(500);

  lift(0);

  drawTo(65 - WISHY, 52);
  drawTo(65 - WISHY, 50);
  drawTo(65 - WISHY, 43);

  drawTo(65 - WISHY, 46);
  drawTo(5, 49);
  drawTo(5, 46);
  drawTo(63 - WISHY, 46);
  drawTo(63 - WISHY, 42);

  drawTo(5, 42);
  drawTo(5, 38);
  drawTo(63 - WISHY, 38);
  drawTo(63 - WISHY, 34);

  drawTo(5, 34);
  drawTo(5, 29);
  drawTo(6, 29);
  drawTo(65 - WISHY, 29);
  drawTo(65 - WISHY, 25);

  drawTo(5, 25);
  drawTo(5, 20);
  drawTo(65 - WISHY, 20);
  drawTo(65 - WISHY, 16);

  drawTo(5, 16);
  drawTo(5, 12);
  drawTo(65 - WISHY, 12);
  drawTo(65 - WISHY, 8);
  drawTo(5, 8);

  drawTo(5, 26);
  drawTo(60 - WISHY, 40);
  drawTo(60 - WISHY, 45);
  drawTo(65 - WISHY, 50);
  drawTo(71.0, 50);

  lift(2);
  drawTo(5, 52);
}

void lift(char lift) {
  switch (lift) {
    // room to optimize  !

    case 0: //850

      if (servoLift >= LIFT0) {
        while (servoLift >= LIFT0)
        {
          servoLift--;
          //servo1.writeMicroseconds(servoLift);
          ledcWrite(srv_CH1, (int)((float)servoLift * 3.28));
          delayMicroseconds(LIFTSPEED);
        }
      }
      else {
        while (servoLift <= LIFT0) {
          servoLift++;
          //servo1.writeMicroseconds(servoLift);
          ledcWrite(srv_CH1, (int)((float)servoLift * 3.28));
          delayMicroseconds(LIFTSPEED);

        }

      }

      break;

    case 1: //150

      if (servoLift >= LIFT1) {
        while (servoLift >= LIFT1) {
          servoLift--;
          //servo1.writeMicroseconds(servoLift);
          ledcWrite(srv_CH1, (int)((float)servoLift * 3.28));
          delayMicroseconds(LIFTSPEED);

        }
      }
      else {
        while (servoLift <= LIFT1) {
          servoLift++;
          //servo1.writeMicroseconds(servoLift);
          ledcWrite(srv_CH1, (int)((float)servoLift * 3.28));
          delayMicroseconds(LIFTSPEED);
        }

      }

      break;

    case 2:

      if (servoLift >= LIFT2) {
        while (servoLift >= LIFT2) {
          servoLift--;
          //servo1.writeMicroseconds(servoLift);
          ledcWrite(srv_CH1, (int)((float)servoLift * 3.28));
          delayMicroseconds(LIFTSPEED);
        }
      }
      else {
        while (servoLift <= LIFT2) {
          servoLift++;
          //servo1.writeMicroseconds(servoLift);
          ledcWrite(srv_CH1, (int)((float)servoLift * 3.28));
          delayMicroseconds(LIFTSPEED);
        }
      }
      break;
  }
}


void bogenUZS(float bx, float by, float radius, int start, int ende, float sqee) {
  float inkr = -0.05;
  float count = 0;

  do {
    drawTo(sqee * radius * cos(start + count) + bx,
           radius * sin(start + count) + by);
    count += inkr;
  }
  while ((start + count) > ende);

}

void bogenGZS(float bx, float by, float radius, int start, int ende, float sqee) {
  float inkr = 0.05;
  float count = 0;

  do {
    drawTo(sqee * radius * cos(start + count) + bx,
           radius * sin(start + count) + by);
    count += inkr;
  }
  while ((start + count) <= ende);
}


void drawTo(double pX, double pY) {
  double dx, dy, c;
  int i;

  // dx dy of new point
  dx = pX - lastX;
  dy = pY - lastY;
  //path lenght in mm, times 4 equals 4 steps per mm
  c = floor(7 * sqrt(dx * dx + dy * dy));

  if (c < 1) c = 1;

  for (i = 0; i <= c; i++) {
    // draw line point by point
    set_XY(lastX + (i * dx / c), lastY + (i * dy / c));

  }

  lastX = pX;
  lastY = pY;
}

double return_angle(double a, double b, double c) {
  // cosine rule for angle between c and a
  return acos((a * a + c * c - b * b) / (2 * a * c));
}

void set_XY(double Tx, double Ty)
{
  delay(1);
  double dx, dy, c, a1, a2, Hx, Hy;

  // calculate triangle between pen, servoLeft and arm joint
  // cartesian dx/dy
  dx = Tx - O1X;
  dy = Ty - O1Y;

  // polar lemgth (c) and angle (a1)
  c = sqrt(dx * dx + dy * dy); //
  a1 = atan2(dy, dx); //
  a2 = return_angle(L1, L2, c);

  //servo2.writeMicroseconds(floor(((a2 + a1 - M_PI) * SERVOFAKTORLEFT) + SERVOLEFTNULL));
  ledcWrite(srv_CH2, (int)((float)floor(((a2 + a1 - M_PI) * SERVOFAKTORLEFT) + SERVOLEFTNULL) * 3.28));

  // calculate joinr arm point for triangle of the right servo arm
  a2 = return_angle(L2, L1, c);
  Hx = Tx + L3 * cos((a1 - a2 + 0.621) + M_PI); //36,5°
  Hy = Ty + L3 * sin((a1 - a2 + 0.621) + M_PI);

  // calculate triangle between pen joint, servoRight and arm joint
  dx = Hx - O2X;
  dy = Hy - O2Y;

  c = sqrt(dx * dx + dy * dy);
  a1 = atan2(dy, dx);
  a2 = return_angle(L1, L4, c);

  //servo3.writeMicroseconds(floor(((a1 - a2) * SERVOFAKTORRIGHT) + SERVORIGHTNULL));
  ledcWrite(srv_CH3, (int)((float)floor(((a1 - a2) * SERVOFAKTORRIGHT) + SERVORIGHTNULL) * 3.28));


}
