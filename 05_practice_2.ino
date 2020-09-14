#define PIN_LED 7
unsigned int toggle;

void setup() {
  pinMode(PIN_LED, OUTPUT);
  toggle = 0;
  digitalWrite(PIN_LED, toggle); // turn on LED.
  delay(1000);
  for(int i = 0; i < 10; i++) {
    toggle = toggle_state(toggle); // toggle LED value.
    digitalWrite(PIN_LED, toggle); // update LED status.
    delay(200); // wait for 200 milliseconds
  }
}

void loop() {
  digitalWrite(PIN_LED, 1);
}

int toggle_state(int toggle) {
  return !toggle;
}
