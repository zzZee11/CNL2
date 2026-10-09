// Motor A Connections
const int pinIN1 = 5;   // Connected to GPIO5
const int pinIN2 = 6;   // Connected to GPIO6
const int pinEnA = 7;   // Connected to GPIO7 (PWM)

// Motor B Connections
const int pinIN3 = 10;  // Connected to GPIO10
const int pinIN4 = 12;  // Connected to GPIO12
const int pinEnB = 11;  // Connected to GPIO11 (PWM)

void setup() {
  pinMode(pinIN1, OUTPUT);
  pinMode(pinIN2, OUTPUT);
  pinMode(pinEnA, OUTPUT);
  pinMode(pinIN3, OUTPUT);
  pinMode(pinIN4, OUTPUT);
  pinMode(pinEnB, OUTPUT);
}

void loop() {
  // Move Forward at 70% speed (PWM value ~180 out of 255)
  analogWrite(pinEnA, 180);
  analogWrite(pinEnB, 180);
  
  digitalWrite(pinIN1, HIGH);
  digitalWrite(pinIN2, LOW);
  digitalWrite(pinIN3, HIGH);
  digitalWrite(pinIN4, LOW);
  delay(2000);

  // Stop
  digitalWrite(pinIN1, LOW);
  digitalWrite(pinIN2, LOW);
  digitalWrite(pinIN3, LOW);
  digitalWrite(pinIN4, LOW);
  delay(1000);

  // Move Backward
  digitalWrite(pinIN1, LOW);
  digitalWrite(pinIN2, HIGH);
  digitalWrite(pinIN3, LOW);
  digitalWrite(pinIN4, HIGH);
  delay(2000);

  // Stop
  digitalWrite(pinIN1, LOW);
  digitalWrite(pinIN2, LOW);
  digitalWrite(pinIN3, LOW);
  digitalWrite(pinIN4, LOW);
  delay(2000);
}