#include <Arduino.h>
/* ノイズ除去のための3回同じパルス幅が入ったら表示するようにすること */

#include "PinChangeInterrupt.h"

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

//void setup() {
//  Serial.begin(9600);
//}

//void loop() {
//  static int width[8];
//
//  for (int i = 0; i < sizeof(meas) / sizeof(meas[0]); i++) {
//    width[i] = meas[i].GetWidth();
//    Serial.print(width[i]);
//    Serial.print("\t");
//  }
//  Serial.println("");
//}

#include "AltSoftSerial.h"
#include "DFPlayer_Mini_Mp3.h"

AltSoftSerial mySerial; // RX, TX

void setup () {
  Serial.begin (9600);
  mySerial.begin (9600);
  mp3_set_serial (mySerial);  //set softwareSerial for DFPlayer-mini mp3 module
  delay(100);  //wait 1ms for mp3 module to set volume
  mp3_set_volume (15);
}

void loop () {
  Serial.println("mp3_play (1);");
  mp3_play (1);
  delay (6000);
  for (;;) {
    Serial.println("mp3_next ();");
    mp3_next ();
    delay (6000);
  }
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


