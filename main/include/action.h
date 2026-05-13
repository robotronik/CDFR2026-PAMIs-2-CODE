#pragma once
#include "main.h"

#ifndef NINJA 
enum class PamiAction {
    BEGIN,
    WAIT,
    SETUP,
    MOVING,
    DANCE
};
#else
enum class PamiAction {
    BEGIN,
    MOVING, 
    TAKE,
    RELEASE,
    DANCE
};
#endif

bool action_state();


