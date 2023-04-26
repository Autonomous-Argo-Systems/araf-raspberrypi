#ifndef STATES_H
#define STATES_H

#include "state.h"
#include "headers/manualControlState.h"
#include "headers/autonomousControlState.h"

static ManualControlState* manualControlState = new ManualControlState();
static AutonomousControlState* autonomousControlState = new AutonomousControlState();

#endif