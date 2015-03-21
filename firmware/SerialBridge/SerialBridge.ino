usb_serial_class *local = &Serial;
HardwareSerial2 *remote = &Serial2;

void setup() {
  pinMode(9, INPUT);
  
  local->begin(57600);
  remote->begin(57600);
}

void loop() {
  while (local->available() > 0) {
    char character = local->read();
    
    remote->print(character);
  }
  
  while (remote->available() > 0) {
    char character = remote->read();
    
    local->print(character);
  }
}
