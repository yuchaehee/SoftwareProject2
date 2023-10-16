#include <Servo.h>

#define PIN_LED   9   // LED active-low
#define PIN_TRIG  12  // sonar sensor TRIGGER
#define PIN_ECHO  13  // sonar sensor ECHO
#define PIN_SERVO 10  // servo motor

#define INTERVAL 25      // sampling interval (unit: msec)
#define PULSE_DURATION 10 // ultra-sound Pulse Duration (unit: usec)

// configurable parameters for sonar
#define SND_VEL 346.0     // sound velocity at 24 celsius degree (unit: m/sec)
#define _DIST_MIN 180.0   // minimum distance to be measured (unit: mm)
#define _DIST_MAX 360.0   // maximum distance to be measured (unit: mm)

#define TIMEOUT ((INTERVAL / 2) * 1000.0) // maximum echo waiting time (unit: usec)
#define SCALE (0.001 * 0.5 * SND_VEL) // coefficient to convert duration to distance

#define _EMA_ALPHA 0.5    // EMA weight of new sample (range: 0 to 1)
                          // Setting EMA to 1 effectively disables EMA filter.

// Target Distance
#define _TARGET_LOW  180.0
#define _TARGET_HIGH 360.0

// Servo motor duty duration
#define _DUTY_MIN 1000 // servo full clockwise position (0 degree)
#define _DUTY_NEU 1500 // servo neutral position (90 degree)
#define _DUTY_MAX 2000 // servo full counterclockwise position (180 degree)

// global variables
float dist_ema, dist_prev = _DIST_MAX; // unit: mm
unsigned long last_sampling_time;       // unit: ms
Servo myservo;

void setup() {
  // Initialize GPIO pins
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);    // sonar TRIGGER
  pinMode(PIN_ECHO, INPUT);     // sonar ECHO
  digitalWrite(PIN_TRIG, LOW);  // turn-off Sonar 

  myservo.attach(PIN_SERVO); 
  myservo.writeMicroseconds(_DUTY_NEU);

  // Initialize USS related variables
  dist_prev = _DIST_MIN; // raw distance output from USS (unit: mm)

  // Initialize serial port
  Serial.begin(57600);
}

void loop() {
  float dist_raw;
  
  // Wait until the next sampling time. 
  if (millis() < (last_sampling_time + INTERVAL))
    return;

  dist_raw = USS_measure(PIN_TRIG, PIN_ECHO); // Read distance

  if (dist_raw < _DIST_MIN || dist_raw > _DIST_MAX) {
    // Cut below minimum or above maximum
    dist_raw = dist_prev;
    digitalWrite(PIN_LED, 1);
  } else {
    digitalWrite(PIN_LED, 0);
  }

  // Apply EMA filter here  
  dist_ema = _EMA_ALPHA * dist_raw + (1 - _EMA_ALPHA) * dist_prev;

  // Calculate servo angle based on distance using proportion
  int servo_angle = ((_DUTY_MAX - _DUTY_MIN) * (dist_ema - _TARGET_LOW) / (_TARGET_HIGH - _TARGET_LOW)) + _DUTY_MIN;
  myservo.writeMicroseconds(servo_angle);

  // Output the distance to the serial port
  Serial.print("Min: ");    Serial.print(_DIST_MIN);
  Serial.print(", Low: ");   Serial.print(_TARGET_LOW);
  Serial.print(", dist: ");  Serial.print(dist_raw);
  Serial.print(", Servo: "); Serial.print(servo_angle);  
  Serial.print(", High: ");  Serial.print(_TARGET_HIGH);
  Serial.print(", Max: ");   Serial.print(_DIST_MAX);
  Serial.println("");

  // Update last sampling time and dist_prev
  last_sampling_time = millis();
  dist_prev = dist_ema;
}

// Get a distance reading from USS. Return value is in millimeters.
float USS_measure(int TRIG, int ECHO) {
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(PULSE_DURATION);
  digitalWrite(TRIG, LOW);
  
  return pulseIn(ECHO, HIGH, TIMEOUT) * SCALE; // unit: mm
}
