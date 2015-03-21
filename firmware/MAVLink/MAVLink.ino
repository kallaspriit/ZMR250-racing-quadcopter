#include <mavlink.h>

usb_serial_class *localSerial = &Serial;
HardwareSerial2 *remoteSerial = &Serial2;

boolean isFirstHeartbeat = true;
boolean isFeedsStarted = false;

int received_sysid = 0;
int received_compid = 0;
int baseMode = 0;
int customMode = 0;

void setup() {
  localSerial->begin(115200);
  remoteSerial->begin(115200);
}

void loop() {
  receiveMessage();
}

void startFeeds() {
  localSerial->println("Starting feeds");
  
  mavlink_message_t msg;
  /*mavlink_msg_request_data_stream_pack(127, 0, &msg, received_sysid, received_compid, MAV_DATA_STREAM_RAW_SENSORS, MAV_DATA_STREAM_RAW_SENSORS_RATE, MAV_DATA_STREAM_RAW_SENSORS_ACTIVE);
  send_message(&msg);
  delay(10);*/
  /*mavlink_msg_request_data_stream_pack(127, 0, &msg, received_sysid, received_compid, MAV_DATA_STREAM_EXTRA1, MAV_DATA_STREAM_EXTRA1_RATE, MAV_DATA_STREAM_EXTRA1_ACTIVE);
  send_message(&msg);
  delay(10);*/
  /*mavlink_msg_request_data_stream_pack(127, 0, &msg, received_sysid, received_compid, MAV_DATA_STREAM_EXTRA2, MAV_DATA_STREAM_EXTRA2_RATE, MAV_DATA_STREAM_EXTRA2_ACTIVE);
  send_message(&msg);
  delay(10);*/
  /*mavlink_msg_request_data_stream_pack(127, 0, &msg, received_sysid, received_compid, MAV_DATA_STREAM_EXTENDED_STATUS, MAV_DATA_STREAM_EXTENDED_STATUS_RATE, MAV_DATA_STREAM_EXTENDED_STATUS_ACTIVE);
  send_message(&msg);
  delay(10);*/
  /*mavlink_msg_request_data_stream_pack(127, 0, &msg, received_sysid, received_compid, MAV_DATA_STREAM_RAW_CONTROLLER, MAV_DATA_STREAM_RAW_CONTROLLER_RATE, MAV_DATA_STREAM_RAW_CONTROLLER_ACTIVE);
  send_message(&msg);
  delay(10);*/
  /*mavlink_msg_request_data_stream_pack(127, 0, &msg, received_sysid, received_compid, MAV_DATA_STREAM_POSITION, MAV_DATA_STREAM_POSITION_RATE, MAV_DATA_STREAM_POSITION_ACTIVE);
  send_message(&msg);
  delay(10);*/
  mavlink_msg_request_data_stream_pack(127, 0, &msg, received_sysid, received_compid, MAV_DATA_STREAM_ALL, 0, 0);
  
  sendMessage(&msg);
  
  isFeedsStarted = true;
}

void sendMessage(mavlink_message_t* msg) {
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buf, msg);
  
  remoteSerial->write(buf, len);
}

void receiveMessage() {
  mavlink_message_t msg;
  mavlink_status_t status;
  
  while(remoteSerial->available() > 0)  {
    uint8_t c = remoteSerial->read();
    
    if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      switch(msg.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT:
          handleMavlinkHeartbeat(&msg);
        break;
        
        case MAVLINK_MSG_ID_ATTITUDE:
          handleMavlinkAttitude(&msg);
        break;
        
        // also getting 36, 35, 1, 42, 24, 62, 74, 27, 29, 33, 34
        default:
          localSerial->print("Got message with id: ");
          localSerial->println(msg.msgid);
        break;
      }
    }
  }
}

void handleMavlinkHeartbeat(mavlink_message_t *msg) {
  mavlink_heartbeat_t packet;
  mavlink_msg_heartbeat_decode(msg, &packet);
 
  if ((*msg).sysid != 0xff) { // do not process mission planner heartbeats if we have two receiver xbees
      received_sysid = (*msg).sysid; // save the sysid and compid of the received heartbeat for use in sending new messages
      received_compid = (*msg).compid;
      baseMode = packet.base_mode;
      customMode = packet.custom_mode;
      
      localSerial->print("MAVLINK_MSG_ID_HEARTBEAT heartbeat base mode: ");
      localSerial->print(baseMode);
      localSerial->print(", custom mode: ");
      localSerial->print(customMode);
      localSerial->print(", sysid: ");
      localSerial->print(received_sysid);
      localSerial->print(", compid: ");
      localSerial->println(received_compid);
    }

  if (isFirstHeartbeat) {
    /*if (!isFeedsStarted) {
      startFeeds();
    }*/
    
    isFirstHeartbeat = false;
  } 
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

