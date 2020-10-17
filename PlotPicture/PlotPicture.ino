#include <Time.h>
#include <TimeLib.h> //http://nekomemo2.site/?eid=1319
#include "pictureData.h"

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

// PlotPicture modified by @shikarunochi

#include <Servo.h>

// delete or mark the next line as comment if you don't need these
//#define CALIBRATION      // enable calibration mode
//#define REALTIMECLOCK    // enable real time clock

#define WISHY 3 // Offset of the Y coordinats of the plate-wisher

// When in calibration mode, adjust the following factors until the servos move exactly 90 degrees
//#define SERVOFAKTORLEFT 600
//#define SERVOFAKTORRIGHT 600
#define SERVOFAKTORLEFT 570
#define SERVOFAKTORRIGHT 550

// Zero-position of left and right servo
// When in calibration mode, adjust the NULL-values so that the servo arms are at all times parallel
// either to the X or Y axis
//#define SERVOLEFTNULL 1950
//#define SERVORIGHTNULL 815
#define SERVOLEFTNULL 1700
//#define SERVORIGHTNULL 790
#define SERVORIGHTNULL 850

#define SERVOPINLIFT  2
#define SERVOPINLEFT  3
#define SERVOPINRIGHT 4

//#define ZOFF 90
#define ZOFF -70
// lift positions of lifting servo
//#define LIFT0 1110+ZOFF // on drawing surface
#define LIFT0 1150+ZOFF // on drawing surface
#define LIFT1 995+ZOFF  // between numbers
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
#define L2 56
#define L3 15.9
#define L4 45.5

// origin points of left and right servo 
#define O1X 24//22
#define O1Y -25
#define O2X 49//47
#define O2Y -25

int servoLift = 1500;

Servo servo1;  // 
Servo servo2;  // 
Servo servo3;  // 

volatile double lastX = 75;
volatile double lastY = 47.5;

int last_min = 0;

void setup() 
{ 
  drawTo(75.2, 47);
  lift(0);
  servo1.attach(SERVOPINLIFT);  //  lifting servo
  servo2.attach(SERVOPINLEFT);  //  left servo
  servo3.attach(SERVOPINRIGHT);  //  right servo
  delay(1000);
  lift(1);
  delay(5000);
  //drawPicture();
  drawTest();
  lift(1);
  drawTo(75.2, 47);
  servo1.detach();
  servo2.detach();
  servo3.detach();

} 

void loop() 
{ 

} 


void drawPicture(){
  int x0 = -1;
  int y0 = -1;
  int x1 = -1;
  int y1 = -1;

  //-1が出たら次の線
  int dataSize = sizeof(pictureData)/ sizeof(*pictureData);
  int index = 0;
  while(index < dataSize){
    if(x0 == -1){ //書き始め
      if(pgm_read_word_near(&pictureData[index])==-1){
        break;
      }
      index++;  //カラー読み飛ばし
      x0=pgm_read_word_near(&pictureData[index++]);
      y0=pgm_read_word_near(&pictureData[index++]);
      //ドット打つ
      draw(x0, y0);
      Serial.print("x0:");
      Serial.print(x0);
      Serial.print("Y0:");
      Serial.print(y0);
      Serial.print("\n");
      
      lift(0);
    }
    if(pgm_read_word_near(&pictureData[index])==-1){
      x0 = -1;
      y0 = -1;
      index++;
      lift(1);
      continue;
    }
    x1=pgm_read_word_near(&pictureData[index++]);
    y1=pgm_read_word_near(&pictureData[index++]);

    //距離が短すぎるなら書かない
    int dx = x1 - x0;
    int dy = y1 - y0;
    double c = sqrt(dx * dx + dy * dy); 
    if(c >= 10){
      draw(x1, y1);
        Serial.print("x1:");
        Serial.print(x1);
        Serial.print("Y1:");
        Serial.print(y1);
      x0 = x1;
      y0 = y1;
    }
  }
}

void draw(int x, int y){
  float fx = (float)x * 67 / 640;
  float fy =  (float)y * 50 / 480;
      Serial.print(": f x:");
      Serial.print(fx);
      Serial.print("f y:");
      Serial.print(fy);
      Serial.print("\n");

  if(55 - fy <= 0){
    fy = 55;
  }
  
  drawTo( fx - 5, 55 - fy);
  
  delay(10);
}

void drawTest(){

  delay(1000);
  drawTo(0,20);
  lift(0);
  delay(1000);
  drawTo(60,20);
  delay(1000);
  drawTo(60,50);
  delay(1000);
  drawTo(0,50);
  delay(1000);
  drawTo(5,20);
  delay(1000);
  drawTo(60,50);
  lift(1);
  delay(1000);
  drawTo(0,50);
  lift(0);
  delay(1000);
  drawTo(60,20);
  lift(1);
}



void lift(char lift) {
  switch (lift) {
    // room to optimize  !

  case 0: //850

      if (servoLift >= LIFT0) {
      while (servoLift >= LIFT0) 
      {
        servoLift--;
        servo1.writeMicroseconds(servoLift);				
        delayMicroseconds(LIFTSPEED);
      }
    } 
    else {
      while (servoLift <= LIFT0) {
        servoLift++;
        servo1.writeMicroseconds(servoLift);
        delayMicroseconds(LIFTSPEED);

      }

    }

    break;

  case 1: //150

    if (servoLift >= LIFT1) {
      while (servoLift >= LIFT1) {
        servoLift--;
        servo1.writeMicroseconds(servoLift);
        delayMicroseconds(LIFTSPEED);

      }
    } 
    else {
      while (servoLift <= LIFT1) {
        servoLift++;
        servo1.writeMicroseconds(servoLift);
        delayMicroseconds(LIFTSPEED);
      }

    }

    break;

  case 2:

    if (servoLift >= LIFT2) {
      while (servoLift >= LIFT2) {
        servoLift--;
        servo1.writeMicroseconds(servoLift);
        delayMicroseconds(LIFTSPEED);
      }
    } 
    else {
      while (servoLift <= LIFT2) {
        servoLift++;
        servo1.writeMicroseconds(servoLift);				
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

  servo2.writeMicroseconds(floor(((a2 + a1 - M_PI) * SERVOFAKTORLEFT) + SERVOLEFTNULL));

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

  servo3.writeMicroseconds(floor(((a1 - a2) * SERVOFAKTORRIGHT) + SERVORIGHTNULL));

}
