#include "Display.h"
#include "UI.h"
#include "State.h"
#include "Device.h"

#include <mavlink.h>
#include <SPI.h>

// hardware config
const int DISPLAY_DC_PIN = 2;
const int DISPLAY_CS_PIN = 4;
const int DISPLAY_RST_PIN = 3;
const int DISPLAY_MOSI_PIN = 11;
const int DISPLAY_SCLK_PIN = 13;
const int SERIAL2_TX_PIN = 10;
const int BAT_SENSE_PIN = 16;
const int BAT_CHARGER_STATE_PIN = 15;
const int BT_STATUS_PIN = 17;

// application config
const int LOCAL_SERIAL_BAUDRATE = 115200; // 115200
const int REMOTE_SERIAL_BAUDRATE = 57600; // 115200
const unsigned long batteryReadingInterval = 1000;
const unsigned long chargerReadingInterval = 1000;

// runtime information
int state = LOADING;
bool isChargingBattery = true;
boolean isFirstHeartbeat = true;
float localBatteryVoltage = 4.2f;

// timing book-keeping
unsigned long lastLoopMicrotime = 0;
unsigned long lastHeartbeatTime = 0;
unsigned long lastMessageReceiveTime = 0;
unsigned long lastDataReceiveTime = 0;
unsigned long lastStartFeedsTime = 0;
unsigned long lastBatterySenseTime = 0;
unsigned long lastChargerSenseTime = 0;
unsigned long lastUserInterfaceUpdateTime = 0;

// bluetooth state tracking
volatile unsigned long lastBluetoothStateChangeTime = 0;
volatile int lastBluetoothState = LOW;

// class instances
Display *display = NULL;
UI *ui = NULL;
Device *device = NULL;

// serial pointers
usb_serial_class *localSerial = &Serial;
HardwareSerial2 *remoteSerial = &Serial2;

//Serial_ *localSerial = &Serial;
//HardwareSerial *remoteSerial = &Serial1;

//void setState(int state);

/**
 * Sets up the application resources.
 */
void setup() {
  setupSerials();
  setupIO();
  setupInterrupts();
  setupDisplay();
  setupUserInterface();
  setupDevice();
  setupSensors();
  setupStateMachine();
}

/**
 * Main application update loop.
 */
void loop() {
  unsigned long microtime = micros();
  unsigned long currentTime = millis();
  unsigned long dt = microtime - lastLoopMicrotime;
  
  lastLoopMicrotime = microtime;
  
  stepReadMavlink(currentTime, dt);
  stepBatteryMonitor(currentTime, dt);
  stepChargerMonitor(currentTime, dt);
  stepFeedMonitor(currentTime, dt);
  stepStateMachine(currentTime, dt);
  stepUserInterface(currentTime, dt);
}

/**
 * Sets up the serial links.
 */
void setupSerials() {
  localSerial->begin(LOCAL_SERIAL_BAUDRATE);
  remoteSerial->begin(REMOTE_SERIAL_BAUDRATE);
}

/**
 * Sets up input-output pin-modes.
 */
void setupIO() {
  // make the tx pin not drive it not to affect bluetooth-radio communication
  pinMode(SERIAL2_TX_PIN, INPUT);
  
  // battery voltage sensing uses ADC
  pinMode(BAT_SENSE_PIN, INPUT);
  
  // sense battery charging state
  pinMode(BAT_CHARGER_STATE_PIN, INPUT);
  
  // listen for bluetooth status changes using interrupts
  pinMode(BT_STATUS_PIN, INPUT);
}

/**
 * Attaches interrupts.
 */
void setupInterrupts() {
  // listen for bluetooth status changes (blinks on-off if not connected, constantly high when connected)
  attachInterrupt(BT_STATUS_PIN, onBluetoothStatusChange, CHANGE);
}

/**
 * Sets up the display.
 */
void setupDisplay() {
  display = new Display(
    DISPLAY_DC_PIN,
    DISPLAY_CS_PIN,
    DISPLAY_RST_PIN,
    DISPLAY_MOSI_PIN,
    DISPLAY_SCLK_PIN
  );
  
  display->init();
}

/**
 * Sets up the user interface.
 */
void setupUserInterface() {
  ui = new UI(display);
}

/**
 * Sets up the device.
 */
void setupDevice() {
  device = new Device();
}

/**
 * Sets up sensors, taking first readings.
 */
void setupSensors() {
  // trigger initial bluetooth status reading
  onBluetoothStatusChange();  
}

/**
 * Sets up the state machine.
 */
void setupStateMachine() {
  setState(LOADING);
}

/**
 * Reads and parses mavlink messages.
 *
 * @param currentTime Current step time in milliseconds
 * @param dt Time since last step in microseconds
 */
void stepReadMavlink(unsigned long currentTime, unsigned long dt) {
  mavlink_message_t msg;
  mavlink_status_t status;
  
  // set true for any mavlink message received
  boolean receivedMessage = false;
  
  // set true for requested data messages received
  boolean receivedData = false;
  
  while(remoteSerial->available() > 0)  {
    uint8_t c = remoteSerial->read();
    
    if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      receivedMessage = true;
      
      switch(msg.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT:
          handleMavlinkHeartbeat(&msg);
        break;
        
        case MAVLINK_MSG_ID_SYS_STATUS:
          handleMavlinkSystemStatus(&msg);
        break;
        
        case MAVLINK_MSG_ID_ATTITUDE:
          handleMavlinkAttitude(&msg);
          
          receivedData = true;
        break;
        
        // also getting 36, 35, 1, 42, 24, 62, 74, 27, 29, 33, 34
        default:
          /*localSerial->print("Got unhandled message with id: ");
          localSerial->println(msg.msgid);*/
        break;
      }
    }
  }
  
  if (receivedMessage) {
    lastMessageReceiveTime = millis(); 
  }
  
  if (receivedData) {
    lastDataReceiveTime = millis(); 
  }
}

/**
 * Periodically reads battery voltage.
 *
 * @param currentTime Current step time in milliseconds
 * @param dt Time since last step in microseconds
 */
void stepBatteryMonitor(unsigned long currentTime, unsigned long dt) {
  if (currentTime - lastBatterySenseTime < batteryReadingInterval) {
    return;  
  }
  
  int rawReading = analogRead(BAT_SENSE_PIN);
  localBatteryVoltage = (float)max(min(map(rawReading, 0, 970, 0, 420), 420), 0) / 100.0f;
  
  /*localSerial->print("Battery reading: ");
  localSerial->print(rawReading);
  localSerial->print(", voltage: ");
  localSerial->println(localBatteryVoltage);*/
  
  lastBatterySenseTime = currentTime;
}

/**
 * Periodically checks battery charger state.
 *
 * @param currentTime Current step time in milliseconds
 * @param dt Time since last step in microseconds
 */
void stepChargerMonitor(unsigned long currentTime, unsigned long dt) {
  if (currentTime - lastChargerSenseTime < chargerReadingInterval) {
    return;  
  }
  
  int state = digitalRead(BAT_CHARGER_STATE_PIN);
  
  isChargingBattery = state == LOW;
  
  /*localSerial->print("Battery charger state: ");
  localSerial->println(rawReading);*/
  
  lastChargerSenseTime = currentTime;
}

/**
 * Requests for starting mavlink feeds if there hasn't been any messages for some time.
 *
 * @param currentTime Current step time in milliseconds
 * @param dt Time since last step in microseconds
 */
void stepFeedMonitor(unsigned long currentTime, unsigned long dt) {
  if (
    (currentTime - lastDataReceiveTime > 3000 || lastDataReceiveTime == 0)
    && currentTime - lastHeartbeatTime < 1000
    && currentTime - lastStartFeedsTime > 10000
  ) {
    requestMavlinkFeeds(); 
  }
}

/**
 * Updates the application state machine.
 *
 * @param currentTime Current step time in milliseconds
 * @param dt Time since last step in microseconds
 */
void stepStateMachine(unsigned long currentTime, unsigned long dt) {
  unsigned long timeSinceDataReceived = currentTime - lastDataReceiveTime;
  unsigned long timeSinceHeartbeatReceived = currentTime - lastHeartbeatTime;
  
  if (lastHeartbeatTime != 0) {
    if (timeSinceDataReceived < 5000) {
      setState(MONITORING);
    } else if (lastMessageReceiveTime < 5000) {
      setState(CONNECTED);
    } else {
      setState(DISCONNECTED);
    }
  } else {
    setState(DISCONNECTED);
  }
}

/**
 * Updates the user interface.
 *
 * @param currentTime Current step time in milliseconds
 * @param dt Time since last step in microseconds
 */
void stepUserInterface(unsigned long currentTime, unsigned long dt) {
  unsigned long timeSinceLastUpdate = currentTime - lastUserInterfaceUpdateTime;
  
  if (timeSinceLastUpdate < 1000) {
    return;  
  }
  
  //localSerial->println("Render");
  
  ui->renderHeader(isBluetoothConnected(), localBatteryVoltage, isChargingBattery);
  ui->renderFooter(getCurrentStateName());
  
  switch (state) {
    case LOADING:
      ui->renderLoadingView("LOADING", UI::LARGE);
    break;
    
    case DISCONNECTED:
      ui->renderLoadingView("Waiting for uplink", UI::SMALL);
    break;
    
    case CONNECTED:
      ui->renderLoadingView("Waiting for heartbeat", UI::SMALL);
    break;
    
    case MONITORING:
      // TODO Make it possible to switch between different views with a button
      ui->renderDetailsView(device);
    break;
  }
  
  lastUserInterfaceUpdateTime = currentTime;
}

/**
 * Updates the application state machine.
 *
 * @param currentTime Current step time in milliseconds
 * @param dt Time since last step in microseconds
 */
void setState(int newState) {
  // ignore request if state does not change
  if (newState == state) {
    return;  
  }
  
  int oldState = state;
  state = newState;
  
  // clear UI of existing views on state change
  ui->clear();

  localSerial->print("Transition from state ");
  localSerial->print(getStateName(oldState));
  localSerial->print(" to ");
  localSerial->println(getStateName(newState));
}

/**
 * Returns currently active state name.
 *
 * @return Active state name
 */
String getCurrentStateName() {
  return getStateName(state);
}

/**
 * Returns whether bluetooth is currently paired and connected to a remote device.
 *
 * @return Is bluetooth connected
 */
boolean isBluetoothConnected() {
  unsigned long timeSinceBluetootStateChange = millis() - lastBluetoothStateChangeTime;
  
  return lastBluetoothState == HIGH && timeSinceBluetootStateChange > 500;
}

/**
 * Called on bluetooth status pin state change interrupt.
 */
void onBluetoothStatusChange() {
  int state = digitalRead(BT_STATUS_PIN);
  
  /*if (state == HIGH) {
    localSerial->println("BT high");
  } else {
    localSerial->println("BT low");
  }*/
  
  lastBluetoothStateChangeTime = millis();
  lastBluetoothState = state;
}

/**
 * Attempts to start mavlink feeds.
 */
void requestMavlinkFeeds() {
  localSerial->println("Requesting mavlink feeds");
  
  mavlink_message_t msg;
  
  // first disable all
  mavlink_msg_request_data_stream_pack(127, 0, &msg, device->systemId, device->componentId, MAV_DATA_STREAM_ALL, 0, 0);
  sendMavlinkMessage(&msg);
  
  // then request all
  mavlink_msg_request_data_stream_pack(127, 0, &msg, device->systemId, device->componentId, MAV_DATA_STREAM_ALL, 1, 1);
  sendMavlinkMessage(&msg);
  
  lastStartFeedsTime = millis();
}

void sendMavlinkMessage(mavlink_message_t* msg) {
  remoteSerial->begin(REMOTE_SERIAL_BAUDRATE);
  
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buf, msg);
  
  remoteSerial->write(buf, len);
  
  // restore the remote serial transmit pin to high-z state
  delay(10);
  pinMode(SERIAL2_TX_PIN, INPUT);
  delay(10);
}

// https://pixhawk.ethz.ch/mavlink/#HEARTBEAT
void handleMavlinkHeartbeat(mavlink_message_t *msg) {
  mavlink_heartbeat_t packet;
  mavlink_msg_heartbeat_decode(msg, &packet);
  
  // do not process mission planner heartbeats if we have two receiver xbees
  if ((*msg).sysid == 0xff) {
    return;
  }
 
  // store device system info
  device->systemId = (*msg).sysid; // save the sysid and compid of the received heartbeat for use in sending new messages
  device->componentId = (*msg).compid;
  device->baseMode = packet.base_mode;
  device->customMode = packet.custom_mode;
  
  if (isFirstHeartbeat) {
    localSerial->println("Got first heartbeat");
    
    isFirstHeartbeat = false;
  } else {
    localSerial->println("Got heartbeat");
  }
  
  lastHeartbeatTime = millis();
}

// https://pixhawk.ethz.ch/mavlink/#SYS_STATUS
void handleMavlinkSystemStatus(mavlink_message_t *msg) {
  mavlink_sys_status_t packet;
  
  mavlink_msg_sys_status_decode(msg, &packet);
  
  // converts millivolts to volts
  device->voltage = (float)packet.voltage_battery / 1000.0f;
  device->current = (float)packet.current_battery / 100.0f;
}

// https://pixhawk.ethz.ch/mavlink/#ATTITUDE
void handleMavlinkAttitude(mavlink_message_t *msg) {
  mavlink_attitude_t packet;
  
  mavlink_msg_attitude_decode(msg, &packet);
          
  device->pitch = radToDeg(packet.pitch);
  device->yaw = radToDeg(packet.yaw);
  device->roll = radToDeg(packet.roll);
  
  //localSerial->println("got attitude");
}

float radToDeg(float radians) {
  return (radians * 4068) / 71; 
}

