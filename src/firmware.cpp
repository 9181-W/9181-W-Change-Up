#include "okapi/api.hpp"
using namespace okapi;

// #define OPTICAL_SHAFT_ENCODER_LEFT_TOP 'G'
// #define OPTICAL_SHAFT_ENCODER_LEFT_BOTTOM 'H'
// #define OPTICAL_SHAFT_ENCODER_RIGHT_TOP 'E'
// #define OPTICAL_SHAFT_ENCODER_RIGHT_BOTTOM 'F'
// #define OPTICAL_SHAFT_ENCODER_MIDDLE_TOP 'C'
// #define OPTICAL_SHAFT_ENCODER_MIDDLE_BOTTOM 'D'

#define OPTICAL_SHAFT_ENCODER_LEFT_TOP 'C'
#define OPTICAL_SHAFT_ENCODER_LEFT_BOTTOM 'D'
#define OPTICAL_SHAFT_ENCODER_RIGHT_TOP 'A'
#define OPTICAL_SHAFT_ENCODER_RIGHT_BOTTOM 'B'
#define OPTICAL_SHAFT_ENCODER_MIDDLE_TOP 'E'
#define OPTICAL_SHAFT_ENCODER_MIDDLE_BOTTOM 'F'

ADIEncoder* shaft_enc_l = NULL;
ADIEncoder* shaft_enc_r = NULL;
ADIEncoder* shaft_enc_m = NULL;

void encoder_initialize()
{
  // shaft_enc_l = new ADIEncoder(OPTICAL_SHAFT_ENCODER_LEFT_TOP, OPTICAL_SHAFT_ENCODER_LEFT_BOTTOM, false);
  // shaft_enc_r = new ADIEncoder(OPTICAL_SHAFT_ENCODER_RIGHT_TOP, OPTICAL_SHAFT_ENCODER_RIGHT_BOTTOM, true);
  // shaft_enc_m = new ADIEncoder(OPTICAL_SHAFT_ENCODER_MIDDLE_TOP, OPTICAL_SHAFT_ENCODER_MIDDLE_BOTTOM, true);

  shaft_enc_l = new ADIEncoder(OPTICAL_SHAFT_ENCODER_LEFT_TOP, OPTICAL_SHAFT_ENCODER_LEFT_BOTTOM, true);
  shaft_enc_r = new ADIEncoder(OPTICAL_SHAFT_ENCODER_RIGHT_TOP, OPTICAL_SHAFT_ENCODER_RIGHT_BOTTOM, false);
  shaft_enc_m = new ADIEncoder(OPTICAL_SHAFT_ENCODER_MIDDLE_TOP, OPTICAL_SHAFT_ENCODER_MIDDLE_BOTTOM, false);
}