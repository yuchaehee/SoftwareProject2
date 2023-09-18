# SoftwareProject2
# 05_practice_2.ino
# 과제코드 : 05P12

#define PIN_LED 7
unsigned int count, toggle;

void setup() {
  pinMode(PIN_LED, OUTPUT);
  Serial.begin(115200); //Initialize serial port
  while(!Serial) {
    ; // wait for serial port to connect
  }
  count=0;
  toggle=0;
  digitalWrite(PIN_LED, toggle); // turn on LED.
  delay(1000);

}

void loop() {
  toggle=toggle_state(toggle);
  count++;
  if (count==10) {
    digitalWrite(PIN_LED, toggle);
    toggle=toggle_state(toggle);
    delay(100);
    digitalWrite(PIN_LED, toggle);
    while(1) {}
  }
  digitalWrite(PIN_LED, toggle);
  delay(100);
}

int toggle_state(int toggle) {
  if (toggle==0) {
    toggle=1;
  }
  else if (toggle==1) {
    toggle=0;
  }
  
  return toggle;
}
