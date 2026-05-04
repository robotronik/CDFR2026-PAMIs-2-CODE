#pragma once
#include <string>

inline float normalize_angle_deg(float angle_deg) {
    while (angle_deg > 180.0f) {
        angle_deg -= 360.0f;
    }
    while (angle_deg < -180.0f) {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

inline float clamp(float value, float min_value, float max_value) {
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

struct coords_t {
    float x;
    float y;
    float angle;

    void updateAngle(float newAngle) {
        angle = newAngle;
        if (angle < 0) {
            angle += 360.0f;
        }
    }
};



