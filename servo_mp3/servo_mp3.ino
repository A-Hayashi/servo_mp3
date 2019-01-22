#include <Arduino.h>
/* ノイズ除去のための3回同じパルス幅が入ったら表示するようにすること */

#include "PinChangeInterrupt.h"
#include "AltSoftSerial.h"
#include "DFPlayer_Mini_Mp3.h"

typedef void(*VoidFuncPtr)(void);

class PulseMeasure {
  public:
    PulseMeasure(byte _meas_pin, VoidFuncPtr change_dispatch) {
      meas_pin = _meas_pin;
      pinMode(meas_pin, INPUT);

      level = LOW;
      level_old = LOW;
      start_time = 0;
      width = 0;

      attachPinChangeInterrupt(digitalPinToPCINT(meas_pin), change_dispatch, CHANGE);
    }

    unsigned long GetWidth() {
      return  width;
    }

    void change() {
      level = digitalRead(meas_pin);

      if (level_old != level) {
        if (level == HIGH) {
          start_time = micros();
        } else {
          width = micros() - start_time;
        }
      }
      level_old = level;
    }

  private:
    byte meas_pin;
    bool level;
    bool level_old;
    unsigned long  start_time;
    unsigned long  width;
};

void change0(void);
void change1(void);
void change2(void);
void change3(void);
void change4(void);
void change5(void);
void change6(void);
void change7(void);

PulseMeasure meas[] = {{4, change0}, {5, change1}, {6, change2}, {7, change3}};
AltSoftSerial mySerial; // RX, TX

void change0(void)
{
  meas[0].change();
}

void change1(void)
{
  meas[1].change();
}

void change2(void)
{
  meas[2].change();
}

void change3(void)
{
  meas[3].change();
}

void setup() {
  Serial.begin (9600);
  mySerial.begin (9600);
  mp3_set_serial (mySerial);  //set softwareSerial for DFPlayer-mini mp3 module
  delay(100);  //wait 1ms for mp3 module to set volume
  mp3_set_volume (15);
}

void loop() {
  static int width[8];

  //  for (int i = 0; i < sizeof(meas) / sizeof(meas[0]); i++) {
  for (int i = 0; i < 1; i++) {
    width[i] = meas[i].GetWidth();
    Serial.print(width[i]);
    Serial.print("\t");

    PlayMp3(width[0]);
  }
  Serial.println("");
}

#define CHECK_NUM 5
int  CheckNum(int pulse) {
  static bool initialized = false;
  static int nums[CHECK_NUM];

  if (initialized == false) {
    for (int i = 0; i < CHECK_NUM; i++) {
      nums[i] = i;
    }
    initialized = true;
  }

  {
    static int idx = 0;
    nums[idx] = Pulse2Digit(pulse);
    idx++;
    if (idx > CHECK_NUM - 1) idx = 0;
  }

  int i;
  for (i = 1; i < CHECK_NUM; i++) {
    if (nums[i] != nums[0]) break;
  }

  if (i == CHECK_NUM) {
    return nums[0];
  } else {
    return 0xff;
  }
}

void PlayMp3 (int pulse) {
  static int num_old = 0xff;
  int num = CheckNum(pulse);

  if (num != 0xff && num_old != num) {
    Serial.print("mp3_play (");
    Serial.print(num);
    Serial.print(");");
    mp3_play (num);
  }
  num_old = num;
}

typedef struct {
  int pmin;
  int pmax;
  int digit;
} p2d_t;

/* パルス幅と7SEGに表示する数字の関係 */
p2d_t p2d[] =
{
  {0, 595, 0},
  {596, 698, 1},
  {699, 801, 2},
  {802, 904, 3},
  {905, 1008, 4},
  {1009, 1111, 5},
  {1112, 1214, 6},
  {1215, 1317, 7},
  {1318, 1420, 8},
  {1421, 1523, 9},
  {1524, 1626, 10},
  {1627, 1729, 11},
  {1730, 1832, 12},
  {1833, 1936, 13},
  {1937, 2039, 14},
  {2040, 2142, 15},
  {2143, 2245, 16},
  {2246, 2348, 17},
  {2349, 2400, 18},
};

byte Pulse2Digit(int pulse)
{
  byte digit = 0xff;
  for (int i = 0; i < sizeof(p2d) / sizeof(p2d[0]); i++) {
    if ( p2d[i].pmin <= pulse && pulse <= p2d[i].pmax) {
      digit = p2d[i].digit;
      break;
    }
  }
  return digit;
}

/*
   mp3_play ();   //start play
   mp3_play (5);  //play "mp3/0005.mp3"
   mp3_next ();   //play next
   mp3_prev ();   //play previous
   mp3_set_volume (uint16_t volume);  //0~30
   mp3_set_EQ (); //0~5
   mp3_pause ();
   mp3_stop ();
   void mp3_get_state ();   //send get state command
   void mp3_get_volume ();
   void mp3_get_u_sum ();
   void mp3_get_tf_sum ();
   void mp3_get_flash_sum ();
   void mp3_get_tf_current ();
   void mp3_get_u_current ();
   void mp3_get_flash_current ();
   void mp3_single_loop (boolean state);  //set single loop
   void mp3_DAC (boolean state);
   void mp3_random_play ();
*/
