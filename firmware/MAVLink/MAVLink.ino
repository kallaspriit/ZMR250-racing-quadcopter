#include <mavlink.h>

usb_serial_class *localSerial = &Serial;
HardwareSerial2 *remoteSerial = &Serial2;

boolean isFirstHeartbeat = true;
boolean isFeedsStarted = false;

int remoteSystemId = 0;
int remoteComponentId = 0;
int baseMode = 0;
int customMode = 0;

unsigned long lastHeartbeatTime = 0;
unsigned long lastDataReceiveTime = 0;
unsigned long lastStartFeedsTime = 0;

void setup() {
  localSerial->begin(115200);
  remoteSerial->begin(115200);
  
  //pinMode(9, INPUT);
  pinMode(10, INPUT);
}

void loop() {
  unsigned long currentTime = millis();
  
  receiveMessage();
  
  if (
    (currentTime - lastDataReceiveTime > 3000 || lastDataReceiveTime == 0)
    && currentTime - lastHeartbeatTime < 1000
    && currentTime - lastStartFeedsTime > 5000
  ) {
    startFeeds(); 
  }
}

void startFeeds() {
  remoteSerial->begin(115200);
  
  localSerial->print("!!!!!! Starting feeds for system id: ");
  localSerial->print(remoteSystemId);
  localSerial->print(", component id: ");
  localSerial->println(remoteComponentId);
  
  mavlink_message_t msg;
  
  // first disable all
  mavlink_msg_request_data_stream_pack(127, 0, &msg, remoteSystemId, remoteComponentId, MAV_DATA_STREAM_ALL, 0, 0);
  sendMessage(&msg);
  delay(10);
  
  /*mavlink_msg_request_data_stream_pack(127, 0, &msg, remoteSystemId, remoteComponentId, MAV_DATA_STREAM_RAW_SENSORS, MAV_DATA_STREAM_RAW_SENSORS_RATE, MAV_DATA_STREAM_RAW_SENSORS_ACTIVE);
  send_message(&msg);
  delay(10);*/
  
  /*mavlink_msg_request_data_stream_pack(127, 0, &msg, remoteSystemId, remoteComponentId, MAV_DATA_STREAM_EXTRA1, MAV_DATA_STREAM_EXTRA1_RATE, MAV_DATA_STREAM_EXTRA1_ACTIVE);
  send_message(&msg);
  delay(10);*/
  
  /*mavlink_msg_request_data_stream_pack(127, 0, &msg, remoteSystemId, remoteComponentId, MAV_DATA_STREAM_EXTRA2, MAV_DATA_STREAM_EXTRA2_RATE, MAV_DATA_STREAM_EXTRA2_ACTIVE);
  send_message(&msg);
  delay(10);*/
  
  /*mavlink_msg_request_data_stream_pack(127, 0, &msg, remoteSystemId, remoteComponentId, MAV_DATA_STREAM_EXTENDED_STATUS, MAV_DATA_STREAM_EXTENDED_STATUS_RATE, MAV_DATA_STREAM_EXTENDED_STATUS_ACTIVE);
  send_message(&msg);
  delay(10);*/
  
  mavlink_msg_request_data_stream_pack(127, 0, &msg, remoteSystemId, remoteComponentId, MAV_DATA_STREAM::MAV_DATA_STREAM_ALL, 1, 1);
  sendMessage(&msg);
  delay(10);
  
  /*mavlink_msg_request_data_stream_pack(127, 0, &msg, remoteSystemId, remoteComponentId, MAV_DATA_STREAM_POSITION, MAV_DATA_STREAM_POSITION_RATE, MAV_DATA_STREAM_POSITION_ACTIVE);
  send_message(&msg);
  delay(10);*/
  
  /*mavlink_msg_request_data_stream_pack(127, 0, &msg, remoteSystemId, remoteComponentId, MAV_DATA_STREAM_ALL, 1, 1);
  sendMessage(&msg);
  delay(10);*/
  
  isFeedsStarted = true;
  
  pinMode(10, INPUT);
  
  lastStartFeedsTime = millis();
}

void sendMessage(mavlink_message_t* msg) {
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buf, msg);
  
  remoteSerial->write(buf, len);
}

void receiveMessage() {
  mavlink_message_t msg;
  mavlink_status_t status;
  boolean receivedData = true;
  
  while(remoteSerial->available() > 0)  {
    uint8_t c = remoteSerial->read();
    
    if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      switch(msg.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT:
          handleMavlinkHeartbeat(&msg);
          
          receivedData = false;
        break;
        
        case MAVLINK_MSG_ID_ATTITUDE:
          handleMavlinkAttitude(&msg);
        break;
        
        // also getting 36, 35, 1, 42, 24, 62, 74, 27, 29, 33, 34
        default:
          localSerial->print("Got message with id: ");
          localSerial->println(msg.msgid);
          
          receivedData = false;
        break;
      }
    }
  }
  
  if (receivedData) {
    lastDataReceiveTime = millis(); 
  }
}

void handleMavlinkHeartbeat(mavlink_message_t *msg) {
  mavlink_heartbeat_t packet;
  mavlink_msg_heartbeat_decode(msg, &packet);
 
  if ((*msg).sysid != 0xff) { // do not process mission planner heartbeats if we have two receiver xbees
      remoteSystemId = (*msg).sysid; // save the sysid and compid of the received heartbeat for use in sending new messages
      remoteComponentId = (*msg).compid;
      baseMode = packet.base_mode;
      customMode = packet.custom_mode;
      
      localSerial->print("MAVLINK_MSG_ID_HEARTBEAT heartbeat base mode: ");
      localSerial->print(baseMode);
      localSerial->print(", custom mode: ");
      localSerial->print(customMode);
      localSerial->print(", remote system id: ");
      localSerial->print(remoteSystemId);
      localSerial->print(", remote component id: ");
      localSerial->println(remoteComponentId);
    }

  if (isFirstHeartbeat) {
    /*if (!isFeedsStarted) {
      startFeeds();
    }*/
    
    isFirstHeartbeat = false;
  }
  
  lastHeartbeatTime = millis();
}

void handleMavlinkAttitude(mavlink_message_t *msg) {
  mavlink_attitude_t packet;
  float pitch, yaw, roll;
  
  mavlink_msg_attitude_decode(msg, &packet);
          
  pitch = radToDeg(packet.pitch);
  yaw = radToDeg(packet.yaw);
  roll = radToDeg(packet.roll);
  
  localSerial->print("MAVLINK_MSG_ID_ATTITUDE pitch: ");
  localSerial->print(pitch);
  localSerial->print("deg, yaw: ");
  localSerial->print(yaw);
  localSerial->print("deg, roll ");
  localSerial->print(roll);
  localSerial->println("deg ");
}

float radToDeg(float radians) {
  return (radians * 4068) / 71; 
}

