#define DIR_PIN 2
#define STEP_PIN 3
#define EN_PIN 4

const int stepsPerRevolution = 1600;
float maxSpeed = 200.0;
int currentPosition = 0; 

void setup() {
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);  // Enable motor

  Serial.begin(9600);
  Serial.println("Send 'spd <value>' or 'pos <0–360>'");
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.startsWith("spd")) {
      maxSpeed = input.substring(4).toFloat();
      Serial.print("Max speed set to: ");
      Serial.println(maxSpeed);
    } else if (input.startsWith("pos")) {
      float degree = input.substring(4).toFloat();
      degree = constrain(degree, 0, 360);
      int targetStep = mapFloat(degree, 0, 360, 0, stepsPerRevolution);
      moveToPosition(targetStep);
    } else {
      Serial.println("Unknown command.");
    }
  }
}

// --- Map float like Arduino's map()
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// --- Smooth stepper movement using sine profile
void moveToPosition(int targetStep) {
  int stepsToMove = targetStep - currentPosition;
  int direction = stepsToMove >= 0 ? 1 : -1;
  stepsToMove = abs(stepsToMove);
  
  digitalWrite(DIR_PIN, direction > 0 ? HIGH : LOW);

  for (int i = 0; i < stepsToMove; i++) {
    float progress = (float)i / stepsToMove;  // 0.0 to 1.0
    float speedFactor = sin(progress * PI);   // Smooth accel/decel
    float delayTime = 1e6 / (maxSpeed * speedFactor); // µs per step

    if (delayTime > 1e6) delayTime = 1e6; // Avoid overflow when speedFactor ~0

    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds((int)delayTime / 2);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds((int)delayTime / 2);
  }

  currentPosition = targetStep;
}
