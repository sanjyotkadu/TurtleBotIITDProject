#include "kinematics.h"
#include <math.h>

static const float SQRT3_OVER_2 = 0.8660254f;

// Largest magnitude of the three inputs, floored at 1.0 so it is only
// ever used to scale *down* (never amplify) when a wheel saturates.
static float findAbsoluteMax(float a, float b, float c) {
  float m = fabs(a);
  if (fabs(b) > m) m = fabs(b);
  if (fabs(c) > m) m = fabs(c);
  return (m < 1.0f) ? 1.0f : m;
}

WheelPowers kiwiMix(float Vx, float Vy, float Omega) {
  WheelPowers wp;

  wp.front_C     = -Vx + Omega;
  wp.backLeft_A  = (Vx * 0.5f) - (Vy * SQRT3_OVER_2) + Omega;
  wp.backRight_B = (Vx * 0.5f) + (Vy * SQRT3_OVER_2) + Omega;

  // Preserve the direction/ratio of the command while keeping every
  // wheel within +/-1.0 when the combined command would saturate.
  float maxPower = findAbsoluteMax(wp.front_C, wp.backLeft_A, wp.backRight_B);
  if (maxPower > 1.0f) {
    wp.front_C     /= maxPower;
    wp.backLeft_A  /= maxPower;
    wp.backRight_B /= maxPower;
  }

  return wp;
}
