#pragma once
#include "main.h"

#ifndef NINJA 
enum class PamiAction {
    BEGIN,
    WAIT,
    NEXT_STEP,
    MOVING,
    DANCE,
    END
};
#else
enum class PamiAction {
    BEGIN,
    NEXT_STEP,
    MOVING, 
    CALIBRATE_X,
    CALIBRATE_Y,
    DANCE,
    END
};
#endif

bool action_state();


