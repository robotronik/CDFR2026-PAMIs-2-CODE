#pragma once

namespace robot_constants {
    // --- Math helpers ---
    constexpr float PI = 3.14159265359f; 
    constexpr float DEG_TO_RAD = PI / 180.0f; 
    constexpr float RAD_TO_DEG = 180.0f / PI; 

    // --- Encoder & Odometry ---
    constexpr int ENCODER_RES = 7; // ticks par tour moteur
    constexpr int REDUCTOR_RATIO = 20;
    constexpr int TICK_PER_REVOLUTION = ENCODER_RES * REDUCTOR_RATIO; 
    constexpr float WHEEL_RADIUS = 30.0f; // mm
    constexpr float DISTANCE_PER_REVOLUTION = 2.0f * PI * WHEEL_RADIUS; 
    constexpr float DEG_PER_TICK = 360.0f / TICK_PER_REVOLUTION; 
    constexpr float RAD_PER_TICK = (2.0f * PI) / TICK_PER_REVOLUTION; 

    // --- Motor control ---
    constexpr float WHEEL_DIST = 80.0f; // mm 
    constexpr float ANGLE_ERROR_MARGIN = 0.1f; // radians
    constexpr float DISTANCE_ERROR_MARGIN = 2.0f; // mm
    constexpr float SPEED_STEP = 0.1f;  // mm/s  TODO: calculate
    constexpr float SLOWDOWN_DISTANCE_PERCENTAGE = 0.2f;
    constexpr float MIN_SPEED_PERCENTAGE = 0.2f; 
}
