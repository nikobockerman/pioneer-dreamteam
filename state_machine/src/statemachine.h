#ifndef STATEMACHINE_H
#define STATEMACHINE_H

#include "pioneer-dreamteam/stateMessage.h"

namespace state {
enum State {
  Startup = 0,
  Explore = 1,
  Approach = 2
};

inline unsigned int stateToUInt (const State& state) {
  switch (state)
  {
    case Startup: return 0; break;
    case Explore: return 1; break;
    case Approach: return 2; break;
  }
  return 0;
}

inline State uintToState (const unsigned int& stateNro) {
  switch (stateNro)
  {
    case 0: return Startup; break;
    case 1: return Explore; break;
    case 2: return Explore; break;
  }
  return Startup;
}
}
/*class StateMachine
{

public:
StateMachine();
};*/

#endif // STATEMACHINE_H
