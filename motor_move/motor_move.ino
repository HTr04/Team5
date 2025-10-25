// Motor 1 (Right motor?)
const int M1_RPWM = 3;
const int M1_LPWM = 5;

// Motor 2 (Left motor?)
const int M2_RPWM = 6;
const int M2_LPWM = 9;

void setup() {
  Serial.begin(9600);

  pinMode(M1_RPWM, OUTPUT);
  pinMode(M1_LPWM, OUTPUT);
  pinMode(M2_RPWM, OUTPUT);
  pinMode(M2_LPWM, OUTPUT);

  Serial.println("Enter motor values as: motor1,motor2");
  Serial.println("Example: 120,-100");
  Serial.println("Test 9");
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');  // read line
    input.trim();

    int commaIndex = input.indexOf(',');
    if (commaIndex == -1) {
      // malformed input, ignore
      return;
    }

    // Parse values
    int m1Val = input.substring(0, commaIndex).toInt();
    int m2Val = input.substring(commaIndex + 1).toInt();

    // Set both motors simultaneously (allow forward + turning)
    setBTS7960(M1_RPWM, M1_LPWM, m1Val);
    setBTS7960(M2_RPWM, M2_LPWM, m2Val);

    // Rate-limit prints to avoid USB backpressure
    static unsigned long lastPrintMs = 0;
    const unsigned long PRINT_PERIOD_MS = 500;
    unsigned long now = millis();
    if (now - lastPrintMs >= PRINT_PERIOD_MS) {
      Serial.print("M1: "); Serial.print(m1Val);
      Serial.print(" | M2: "); Serial.println(m2Val);
      lastPrintMs = now;
    }
  }
}

void setBTS7960(int RPWM, int LPWM, int value) {
  int pwm = abs(value);
  if (pwm > 255) pwm = 255;  // clamp to PWM range

  if (value > 0) {
    analogWrite(RPWM, pwm);
    analogWrite(LPWM, 0);
  } else if (value < 0) {
    analogWrite(RPWM, 0);
    analogWrite(LPWM, pwm);
  } else {
    analogWrite(RPWM, 0);
    analogWrite(LPWM, 0);
  }
}
