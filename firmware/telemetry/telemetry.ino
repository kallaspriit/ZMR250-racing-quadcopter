#include "Commander.h"

//Serial_ *serial = &Serial;
//HardwareSerial *serial = &Serial;
usb_serial_class *serial = &Serial;
HardwareSerial2 *serial2 = &Serial2;

Commander commander(serial);
//Commander commander2(serial2);

void setup() {
  serial->begin(9600);
  serial2->begin(57600);
}

void loop() {
  while (commander.gotCommand()) {
    handleCommand("Serial1", commander.command, commander.parameters, commander.parameterCount);
  }
  
  /*while (commander2.gotCommand()) {
    handleCommand("Serial2", commander2.command, commander2.parameters, commander2.parameterCount);
  }*/
  
  /*while (serial->available() > 0) {
    char character = serial->read();
    
    serial->print(character);
  }*/
  
  while (serial2->available() > 0) {
    char character = serial2->read();
    
    serial->print(character);
  }
}

void handleCommand(String sourceName, String command, String parameters[], int parameterCount) {
  serial->print(sourceName);
  serial->print(": ");
  
  if (command == "sum" && parameterCount == 2) {
     int a = parameters[0].toInt();
     int b = parameters[1].toInt();
     
     serial->print("Sum ");
     serial->print(a);
     serial->print("+");
     serial->print(b);
     serial->print("=");
     serial->println(a+b);
  } else {
    serial->print("Got command '");
    serial->print(command);
    serial->print("' with ");
    serial->print(parameterCount);
    serial->println(" parameters: ");
    
    for (int i = 0; i < parameterCount; i++) {
      serial->print("  > ");
      serial->print(i);
      serial->print(": ");
      serial->println(parameters[i]);
    }
  }
}
