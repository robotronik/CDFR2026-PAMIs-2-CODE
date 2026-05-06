#pragma once
#include "main.h"

#ifndef NINJA 
enum class PamiAction {
    BEGIN,
    WAIT,
    NEXT_STEP,
    MOVING,
    END
};
#else
enum class PamiAction {
    BEGIN,
    NEXT_STEP,
    MOVING,
    TAKE,
    RELEASE,
    END
};
#endif

bool action_state();


