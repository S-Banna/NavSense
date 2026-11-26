// L298N motor pins
const int IN1 = 5;   // Left motor forward (PWM)
const int IN2 = 6;   // Left motor reverse
const int IN3 = 9;   // Right motor forward (PWM)
const int IN4 = 10;  // Right motor reverse

// Encoder pins
const int ENC_PIN_RIGHT = 2;
const int ENC_PIN_LEFT  = 3;

// Encoder reporting
const unsigned long ENC_PRINT_MS = 100;
const unsigned long ENC_MIN_PULSE_US = 200;

// Speed settings
const int SPEED = 120;       // 0-255
const int KICK_MS = 0;      // Full-power startup burst; 0 disables it

bool moving = false;

// Encoder counters
volatile unsigned long rightCount = 0;
volatile unsigned long leftCount = 0;

volatile unsigned long lastRightUs = 0;
volatile unsigned long lastLeftUs = 0;

unsigned long lastPrintMs = 0;

// Right encoder interrupt
void onRightPulse() {
  unsigned long now = micros();

  if (now - lastRightUs >= ENC_MIN_PULSE_US) {
    rightCount++;
    lastRightUs = now;
  }
}

// Left encoder interrupt
void onLeftPulse() {
  unsigned long now = micros();

  if (now - lastLeftUs >= ENC_MIN_PULSE_US) {
    leftCount++;
    lastLeftUs = now;
  }
}

// Drive motors using PWM
void drive(int l1, int r1) {
  if (!moving && KICK_MS > 0) {
    // Full-power startup burst
    analogWrite(IN1, l1 ? 255 : 0);
    analogWrite(IN3, r1 ? 255 : 0);

    digitalWrite(IN2, LOW);
    digitalWrite(IN4, LOW);

    delay(KICK_MS);
  }

  // Drop to requested PWM speed
  analogWrite(IN1, l1);
  digitalWrite(IN2, LOW);

  analogWrite(IN3, r1);
  digitalWrite(IN4, LOW);

  moving = true;
}

void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(ENC_PIN_RIGHT, INPUT_PULLUP);
  pinMode(ENC_PIN_LEFT, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_PIN_RIGHT),
                  onRightPulse, RISING);

  attachInterrupt(digitalPinToInterrupt(ENC_PIN_LEFT),
                  onLeftPulse, RISING);

  Serial.begin(9600);

  stopAll();
}

// Main control loop
void loop() {
  if (Serial.available() > 0) {
    char c = Serial.read();

    if (c == 'a' || c == 'A') {
      drive(SPEED, 0);
      Serial.println("Left side: FORWARD");
    }
    else if (c == 'd' || c == 'D') {
      drive(0, SPEED);
      Serial.println("Right side: FORWARD");
    }
    else if (c == 'w' || c == 'W') {
      drive(SPEED, SPEED);
      Serial.println("Both sides: FORWARD");
    }
    else {
      stopAll();
      Serial.println("All stopped");
    }
  }

  printEncoders();
}

// Stop both motors
void stopAll() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  moving = false;
}

// Print encoder counts every 100 ms
void printEncoders() {
  unsigned long now = millis();

  if (now - lastPrintMs >= ENC_PRINT_MS) {
    lastPrintMs = now;

    unsigned long left, right;

    noInterrupts();
    left = leftCount;
    right = rightCount;
    interrupts();

    Serial.print("ENC,");
    Serial.print(now);
    Serial.print(",");
    Serial.print(left);
    Serial.print(",");
    Serial.println(right);
  }
}