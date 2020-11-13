#include <Servo.h>
#define PIN_IR A0
#define PIN_SERVO 10

Servo myservo;
int a, b; // unit: mm
int rail_hori = 1420;

void setup() {
  myservo.attach(PIN_SERVO);
  myservo.write(rail_hori);
  delay(3000);
  Serial.begin(57600);
  Serial.println(myservo.read());
  
// initialize serial port
  Serial.begin(57600);
  a = 67;
  b = 261;
}

float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}

void loop() {
  float raw_dist = ir_distance();
  float dist_cali = 100 + 300.0 / (b - a) * (raw_dist - a);
  Serial.print("min:0,max:500,dist:");
  Serial.print(raw_dist);
  Serial.print(",dist_cali:");
  Serial.println(dist_cali);
  if(dist_cali < 255) {
    myservo.write(rail_hori + 200);
  } else if (dist_cali > 255) {
    myservo.write(rail_hori - 200);
  }
  delay(20);
}
