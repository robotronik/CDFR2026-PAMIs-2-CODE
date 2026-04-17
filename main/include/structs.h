#pragma once

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

struct map_object_t {
    int id;
    const char* name;
    coords_t coords;

    bool operator==(const map_object_t& other) const {
        return id == other.id;
    }
};
