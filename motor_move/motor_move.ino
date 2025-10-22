// Motor 1 (Right motor?)
const int M1_RPWM = 3;
const int M1_LPWM = 5;

// Motor 2 (Left motor?)
const int M2_RPWM = 6;
const int M2_LPWM = 9;

void setup() {
  Serial.begin(115200);

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
      Serial.println("Invalid format. Use: motor1,motor2");
      return;
    }

    // Parse values
    int m1Val = input.substring(0, commaIndex).toInt();
    int m2Val = input.substring(commaIndex + 1).toInt();

    // Enforce mutual exclusion: run only the motor with larger absolute value
    if (abs(m1Val) >= abs(m2Val) && m1Val != 0) {
      setBTS7960(M1_RPWM, M1_LPWM, m1Val);
      setBTS7960(M2_RPWM, M2_LPWM, 0);  // stop the other motor
    } else if (m2Val != 0) {
      setBTS7960(M2_RPWM, M2_LPWM, m2Val);
      setBTS7960(M1_RPWM, M1_LPWM, 0);  // stop the other motor
    } else {
      // If both are zero, stop both
      setBTS7960(M1_RPWM, M1_LPWM, 0);
      setBTS7960(M2_RPWM, M2_LPWM, 0);
    }

    Serial.print("M1: "); Serial.print(m1Val);
    Serial.print(" | M2: "); Serial.println(m2Val);
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
