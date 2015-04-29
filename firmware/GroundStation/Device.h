#ifndef DEVICE_H
#define DEVICE_H

class Device {
  
public:
  /*Device() :
    systemId(0),
    componentId(0),
  
    pitch(0),
    yaw(0),
    roll(0),
    
    voltage(0.0f)
  {}*/
  
  // protocol
  int systemId;
  int componentId;
  int baseMode;
  int customMode;
  
  // attitude
  float pitch;
  float yaw;
  float roll;
  
  // system
  float voltage;
  float current;
};

#endif
