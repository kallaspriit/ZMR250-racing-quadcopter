#ifndef STATE_H
#define STATE_H

// state machine states
enum State {
  INITIALIZING,
  LOADING,
  DISCONNECTED,
  CONNECTED,
  MONITORING
};

/**
 * Returns state enum name as string.
 *
 * @param state State enum value
 * @return State name
 */
String getStateName(State state) {
  switch (state) {
    case State::INITIALIZING:
      return "INITIALIZING";
      
    case State::LOADING:
      return "LOADING";
      
    case State::DISCONNECTED:
      return "DISCONNECTED";
      
    case State::CONNECTED:
      return "CONNECTED";
      
    case State::MONITORING:
      return "MONITORING";
      
    default:
      return "UNKNOWN";
  }
}

#endif
