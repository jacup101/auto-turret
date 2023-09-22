bool spin;
void setup() {
  // put your setup code here, to run once:
  pinMode(8, OUTPUT);
  pinMode(7, OUTPUT);
  spin = false;
  Serial.begin(9600);
}

void pulse(int pulse_dur) {
  if (!spin) {
    return;
  }
  digitalWrite(8, HIGH);
  delay(pulse_dur);
  digitalWrite(8, LOW);
  delay(pulse_dur);
}

void loop() {
  // put your main code here, to run repeatedly:
    if (Serial.available()) {
      int dir = Serial.read();
      if (dir == 'l') {
        digitalWrite(7, HIGH);
        spin = true;
      } else if (dir == 'r') {
        digitalWrite(7, LOW);
        spin = true;
      } else {
        spin = false;
      }
    }
    pulse(5);
}
