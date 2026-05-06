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
    TAKE,
    RELEASE,
    DANCE,
    END
};
#endif

bool action_state();


