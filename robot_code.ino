// Define motor pins
#define ENA 5
#define IN1 8
#define IN2 9
#define ENB 6
#define IN3 10
#define IN4 11

// Relay pins
#define MOP_RELAY 27
#define PUMP_RELAY 28

// Ultrasonic Sensor Pins
#define TRIG_PIN 22
#define ECHO_PIN 23

// IR Sensors Pins
#define IR_LEFT 26
#define IR_CENTER 25
#define IR_RIGHT 24

// BLDC ESC control
#include <Servo.h>
#define ESC_PIN 2
Servo esc;

// Bluetooth
#define bluetooth Serial1

// Variables
int mode = 0; // 0 = Manual, 1 = Automatic
unsigned long lastCommandTime = 0;
unsigned long commandTimeout = 20000;
bool isBluetoothConnected = true;
int motorSpeed = 150;

bool manualPumpState = false;
bool autoPumpState = false;
unsigned long autoPumpLastToggleTime = 0;
unsigned long autoPumpOnDuration = 3000;
unsigned long autoPumpOffDuration = 10000;

bool mopStateManual = false;
bool vacuumStateManual = false;

void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(MOP_RELAY, OUTPUT);
  pinMode(PUMP_RELAY, OUTPUT);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(IR_LEFT, INPUT);
  pinMode(IR_CENTER, INPUT);
  pinMode(IR_RIGHT, INPUT);

  Serial.begin(9600);
  bluetooth.begin(9600);

  esc.attach(ESC_PIN);
  esc.writeMicroseconds(1000); // Initially off

  stopMotors();
  digitalWrite(MOP_RELAY, LOW);
  digitalWrite(PUMP_RELAY, LOW);

  // --- Sensor Calibration Output ---
  Serial.println("=== SENSOR CALIBRATION ===");

  int leftIR = digitalRead(IR_LEFT);
  int centerIR = digitalRead(IR_CENTER);
  int rightIR = digitalRead(IR_RIGHT);

  Serial.print("IR LEFT: ");
  Serial.print(leftIR == LOW ? "EDGE " : "SURFACE ");
  Serial.print(" | IR CENTER: ");
  Serial.print(centerIR == LOW ? "EDGE " : "SURFACE ");
  Serial.print(" | IR RIGHT: ");
  Serial.println(rightIR == LOW ? "EDGE" : "SURFACE");

  int dist = getDistance();
  Serial.print("Ultrasonic Distance: ");
  Serial.print(dist);
  Serial.println(" cm");

  Serial.println("===========================");
  delay(2000); // Wait for user to read
}

void loop() {
  while (bluetooth.available()) {
    char data = bluetooth.read();
    lastCommandTime = millis();
    isBluetoothConnected = true;
    processCommand(data);
  }

  if (millis() - lastCommandTime > commandTimeout && isBluetoothConnected) {
    isBluetoothConnected = false;
    mode = 1;
    stopMotors();
  }

  // Pump control
  if (mode == 0) {
    digitalWrite(PUMP_RELAY, manualPumpState ? HIGH : LOW);
  }

  if (mode == 1) {
    automaticMovement();
    if (autoPumpState) {
      if (millis() - autoPumpLastToggleTime >= (digitalRead(PUMP_RELAY) == LOW ? autoPumpOffDuration : autoPumpOnDuration)) {
        digitalWrite(PUMP_RELAY, !digitalRead(PUMP_RELAY));
        autoPumpLastToggleTime = millis();
      }
    } else {
      digitalWrite(PUMP_RELAY, LOW);
    }
  }

  delay(50);
}

void processCommand(char cmd) {
  switch (cmd) {
    case 'F': moveForward(); break;
    case 'B': moveBackward(); break;
    case 'L': turnLeft(); break;
    case 'R': turnRight(); break;
    case 'S': stopMotors(); break;
    case 'V': esc.writeMicroseconds(1300); vacuumStateManual = true; break;
    case 'v': esc.writeMicroseconds(1000); vacuumStateManual = false; break;
    case 'M': digitalWrite(MOP_RELAY, HIGH); mopStateManual = true; break;
    case 'm': digitalWrite(MOP_RELAY, LOW); mopStateManual = false; break;
    case 'P': manualPumpState = true; break;
    case 'p': manualPumpState = false; digitalWrite(PUMP_RELAY, LOW); break;
    case 'A': mode = 1; stopMotors(); break;
    case 'a': mode = 0; stopMotors(); break;
  }
}

void moveForward() {
  analogWrite(ENA, motorSpeed);
  analogWrite(ENB, motorSpeed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  autoPumpState = true;
  if (mode == 1) {
    esc.writeMicroseconds(1300);
    digitalWrite(MOP_RELAY, HIGH);
  }
}

void moveBackward() {
  analogWrite(ENA, motorSpeed);
  analogWrite(ENB, motorSpeed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  autoPumpState = false;
  if (mode == 1) {
    esc.writeMicroseconds(1000);
    digitalWrite(MOP_RELAY, LOW);
  }
}

void turnLeft() {
  analogWrite(ENA, motorSpeed);
  analogWrite(ENB, motorSpeed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  autoPumpState = false;
  if (mode == 1) {
    esc.writeMicroseconds(1000);
    digitalWrite(MOP_RELAY, LOW);
  }
}

void turnRight() {
  analogWrite(ENA, motorSpeed);
  analogWrite(ENB, motorSpeed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  autoPumpState = false;
  if (mode == 1) {
    esc.writeMicroseconds(1000);
    digitalWrite(MOP_RELAY, LOW);
  }
}

void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  autoPumpState = false;
  if (mode == 1) {
    esc.writeMicroseconds(1000);
    digitalWrite(MOP_RELAY, LOW);
  }
}

void automaticMovement() {
  int distance = getDistance();
  int leftIR = digitalRead(IR_LEFT);
  int centerIR = digitalRead(IR_CENTER);
  int rightIR = digitalRead(IR_RIGHT);

  static unsigned long recoveryStartTime = 0;
  static int recoveryState = 0;

  if (leftIR == LOW && rightIR == LOW && centerIR == LOW) {
    stopMotors(); recoveryStartTime = millis(); recoveryState = 1;
  } else if (leftIR == LOW && centerIR == LOW) {
    stopMotors(); recoveryStartTime = millis(); recoveryState = 3;
  } else if (rightIR == LOW && centerIR == LOW) {
    stopMotors(); recoveryStartTime = millis(); recoveryState = 4;
  } else if (leftIR == LOW) {
    stopMotors(); recoveryStartTime = millis(); recoveryState = 5;
  } else if (rightIR == LOW) {
    stopMotors(); recoveryStartTime = millis(); recoveryState = 6;
  } else if (distance < 15) {
    stopMotors(); recoveryStartTime = millis(); recoveryState = 1;
  }

  if (recoveryState == 1) {
    moveBackward();
    if (millis() - recoveryStartTime > 600) {
      stopMotors(); recoveryStartTime = millis(); recoveryState = 2;
    }
  } else if (recoveryState == 2) {
    turnLeft();
    if (millis() - recoveryStartTime > 700) {
      stopMotors(); recoveryState = 0;
    }
  } else if (recoveryState == 3) {
    turnRight();
    if (millis() - recoveryStartTime > 700) {
      stopMotors(); recoveryState = 0;
    }
  } else if (recoveryState == 4) {
    turnLeft();
    if (millis() - recoveryStartTime > 700) {
      stopMotors(); recoveryState = 0;
    }
  } else if (recoveryState == 5) {
    turnRight();
    if (millis() - recoveryStartTime > 400) {
      stopMotors(); recoveryState = 0;
    }
  } else if (recoveryState == 6) {
    turnLeft();
    if (millis() - recoveryStartTime > 400) {
      stopMotors(); recoveryState = 0;
    }
  } else {
    moveForward();
  }
}

int getDistance() {
  long sum = 0;
  int samples = 5;
  for (int i = 0; i < samples; i++) {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    long duration = pulseIn(ECHO_PIN, HIGH, 30000);
    if (duration == 0) duration = 30000;
    sum += (duration * 0.034 / 2);
    delay(5);
  }
  return sum/samples;
}

