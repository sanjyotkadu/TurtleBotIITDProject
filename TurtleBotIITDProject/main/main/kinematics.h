#ifndef KINEMATICS_H
#define KINEMATICS_H

// ============================================================
// kinematics.h — 3-wheel Kiwi (holonomic) inverse kinematics
//
// Pure math, no hardware. Converts a body-frame velocity command
// (Vx strafe, Vy forward, Omega rotation) into three normalized
// wheel powers, scaled so the largest never exceeds +/-1.0.
//
// Wheel layout (top view), matching robot_config.h:
//         FRONT
//           C (90 deg)
//          /  \
//        A      B
//    (210 deg) (330 deg)
//
//   Motor C (front)      = -Vx + Omega
//   Motor A (back-left)  =  0.5*Vx - (sqrt(3)/2)*Vy + Omega
//   Motor B (back-right) =  0.5*Vx + (sqrt(3)/2)*Vy + Omega
//
// These are the *pre-direction-correction* powers. The caller is
// responsible for applying robot_config.h's DIR_x_CW wiring signs
// before sending them to the motors.
// ============================================================

// Normalized wheel powers, each in the range -1.0 .. +1.0.
struct WheelPowers {
  float front_C;      // Motor C
  float backLeft_A;   // Motor A
  float backRight_B;  // Motor B
};

// Compute wheel powers for a kiwi drive from a normalized velocity
// command. Each input is expected in -1.0 .. +1.0.
WheelPowers kiwiMix(float Vx, float Vy, float Omega);

#endif // KINEMATICS_H
