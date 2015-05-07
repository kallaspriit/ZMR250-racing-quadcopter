#ifndef STATE_H
#define STATE_H

// state machine states
enum State {
  LOADING,
  DISCONNECTED,
  CONNECTED,
  MONITORING
};

/*const int STATE_LOADING = 1;
const int STATE_DISCONNECTED = 2;
const int STATE_CONNECTED = 3;
const int STATE_MONITORING = 4;*/

/**
 * Returns state enum name as string.
 *
 * @param state State enum value
 * @return State name
 */
String getStateName(int state) {
  switch (state) {
    case LOADING:
      return "LOADING";
      
    case DISCONNECTED:
      return "DISCONNECTED";
      
    case CONNECTED:
      return "CONNECTED";
      
    case MONITORING:
      return "MONITORING";
      
    default:
      return "UNKNOWN";
  }
}

#endif
