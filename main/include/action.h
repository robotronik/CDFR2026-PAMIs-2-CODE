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

void take_stock();
void release_stock();
#endif

bool action_state();


