#include <cassert>
#include <cstdio>
#include "Arduino.h"
int mockRaw[40] = {};
uint32_t mockMicros = 0;
MockSerial Serial;
#include "../../src/main.cpp"

void reading(int left,int centre,int right) {
  mockRaw[32]=left; mockRaw[35]=centre; mockRaw[34]=right;
  readSensors();
}
void fresh() {
  for(int i=0;i<3;++i) { white[i]=0; black[i]=2000; }
  calibrated=true; arm(0); drive(0,0);
}
int main() {
  setup();
  assert(!calibrated && mode==Mode::Stopped);
  arm(0); assert(mode==Mode::Stopped);
  for (int side : {-1,1}) {
    fresh(); reading(side<0?800:0,1600,side>0?800:0);
    control(5,0.005f);
    assert(side*(commandedLeft-commandedRight)>0); // steer toward the line
    assert(mode==Mode::Tracking);
    fresh(); reading(side<0?1800:0,1800,side>0?1800:0);
    control(5,0.005f); control(100,0.005f);
    assert(mode==Mode::Tracking); // centre present: ordinary arc, not pivot
    fresh(); reading(side<0?1800:0,0,side>0?1800:0);
    control(5,0.005f); control(55,0.005f);
    assert(mode==Mode::Corner && side*commandedLeft>0 && side*commandedRight<0);
    reading(0,1800,0); control(60,0.005f);
    assert(mode==Mode::Corner);
    control(95,0.005f); assert(mode==Mode::Tracking);
    fresh(); reading(side<0?1800:0,0,side>0?1800:0);
    control(5,0.005f); control(55,0.005f);
    reading(0,0,0); control(55+CORNER_TIMEOUT_MS,0.005f);
    assert(mode==Mode::Stopped && commandedLeft==0 && commandedRight==0);
  }
  fresh(); reading(0,1800,0); control(5,0.005f);
  reading(0,0,0); control(10,0.005f); assert(mode==Mode::Gap);
  control(10+GAP_HOLD_MS,0.005f); assert(mode==Mode::Searching);
  assert(commandedLeft*commandedRight<0); // zero history still searches
  control(10+GAP_HOLD_MS+SEARCH_TIMEOUT_MS,0.005f);
  assert(mode==Mode::Stopped && commandedLeft==0 && commandedRight==0);
  reading(0,1800,0); control(2000,0.005f); assert(mode==Mode::Stopped); // stop latches
  fresh(); reading(0,0,0); control(10,0.005f);
  reading(0,1800,0); control(15,0.005f); assert(mode==Mode::Tracking);
  fresh(); reading(0,0,0); control(10,0.005f); control(10+GAP_HOLD_MS,0.005f);
  reading(0,1800,0); control(180,0.005f); assert(mode==Mode::Searching);
  control(215,0.005f); assert(mode==Mode::Tracking);
  fresh(); reading(1800,1800,1800); control(5,0.005f);
  assert(mode==Mode::Tracking && commandedLeft<commandedRight); // explicit left policy
  fresh(); black[0]=0; white[0]=2000; reading(0,0,0);
  assert(sample.b[0]==1); // calibration supports inverted electrical polarity
  fresh(); startCalibration(0); reading(1000,1000,1000); control(CALIBRATION_MS,0.005f);
  assert(!calibrated && mode==Mode::Stopped && commandedLeft==0 && commandedRight==0);
  fresh(); startCalibration(0); reading(100,100,100); control(5,0.005f);
  reading(1900,1900,1900); control(CALIBRATION_MS,0.005f);
  assert(calibrated && white[0]==100 && black[0]==1900 && mode==Mode::Stopped);
  puts("Controller tests passed.");
}
