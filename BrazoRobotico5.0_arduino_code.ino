#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Constantes
const int TOTAL_SERVOS = 7;
const int MIN_SERVO_VALUE = 0;
const int MAX_SERVO_VALUE = 180;
const int MIN_SPEED_VALUE = 0;
const int MAX_SPEED_VALUE = 100;
const int NO_SMOOTH_THRESHOLD = 50;

// Mapeo de servos a canales del shield
const int servoChannels[TOTAL_SERVOS] = {0, 1, 2, 3, 4, 5, 7}; // Garra, Muñeca1, Muñeca2, BrazoSup, BrazoInf, Antebrazo, Base
const int OPPOSITE_SERVO_CHANNEL = 6;

// Rangos de pulso para los servos (500-2500µs)
const int SERVO_MIN_PULSE = 500;
const int SERVO_MAX_PULSE = 2500;
const int PULSE_FREQ = 50;

// Variables de control
float currentPositions[TOTAL_SERVOS];
float targetPositions[TOTAL_SERVOS];
int currentSpeed = 50;
bool autoMode = false;
bool isMoving = false;
int receivedPositions = 0;
String inputBuffer = "";
bool inputComplete = false;

// Valores ajustables para control de velocidad
const float MAX_SMOOTH_FACTOR = 0.05;
const int MIN_DELAY = 10;
const int MAX_DELAY = 50;
const float THRESHOLD = 0.5;

int angleToPulse(float angle) {
  int pulse = map(round(angle), 0, 180, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
  return map(pulse, 0, 20000, 0, 4096);
}

float getSmoothFactor() {
  // Si la velocidad es menor o igual a 50, no hay suavizado
  if (currentSpeed <= NO_SMOOTH_THRESHOLD) {
    return 1.0; // Factor que elimina el suavizado
  }
  // Para velocidades mayores a 50, el suavizado es proporcional a la velocidad
  // Cuanto menor sea la velocidad, más rápido será el suavizado
  float speedRatio = (currentSpeed - NO_SMOOTH_THRESHOLD) / (float)(MAX_SPEED_VALUE - NO_SMOOTH_THRESHOLD);
  float adjustedSmoothFactor = MAX_SMOOTH_FACTOR * (1 + (1 - speedRatio));
  return adjustedSmoothFactor;
}

int getMovementDelay() {
  return map(currentSpeed, 0, 100, MAX_DELAY, MIN_DELAY);
}

void setup() {
  Serial.begin(9600);
  inputBuffer.reserve(200);

  pwm.begin();
  pwm.setPWMFreq(PULSE_FREQ);
  Wire.setClock(400000);

  // Inicializar posiciones
  for (int i = 0; i < TOTAL_SERVOS; i++) {
    currentPositions[i] = 90;
    targetPositions[i] = 90;
    pwm.setPWM(servoChannels[i], 0, angleToPulse(90));
  }
  pwm.setPWM(OPPOSITE_SERVO_CHANNEL, 0, angleToPulse(90));
}

void loop() {
  if (inputComplete) {
    processSerialCommand(inputBuffer);
    inputBuffer = "";
    inputComplete = false;
  }

  if (autoMode && isMoving) {
    bool allReached = true;
    float smoothFactor = getSmoothFactor();

    for (int i = 0; i < TOTAL_SERVOS; i++) {
      float diff = targetPositions[i] - currentPositions[i];
      
      if (abs(diff) > THRESHOLD) {
        allReached = false;
        
        if (currentSpeed <= NO_SMOOTH_THRESHOLD) {
          // Movimiento seco para velocidades <= 50
          float step = (diff > 0) ? min(diff, 1.0) : max(diff, -1.0);
          currentPositions[i] += step;
        } else {
          // Suavizado ajustado según la velocidad
          float step = diff * smoothFactor;
          float maxStep = 2.0 * (currentSpeed / 100.0);
          if (abs(step) > maxStep) {
            step = (step > 0) ? maxStep : -maxStep;
          }
          currentPositions[i] += step;
        }
        
        // Aseguramos que no nos pasemos del objetivo
        if ((diff > 0 && currentPositions[i] > targetPositions[i]) ||
            (diff < 0 && currentPositions[i] < targetPositions[i])) {
          currentPositions[i] = targetPositions[i];
        }
        
        if (i == 5) { // Servo del antebrazo
          float oppositePosition = 180 - currentPositions[i];
          pwm.setPWM(OPPOSITE_SERVO_CHANNEL, 0, angleToPulse(oppositePosition));
        }
        
        pwm.setPWM(servoChannels[i], 0, angleToPulse(currentPositions[i]));
      }
    }

    if (allReached) {
      isMoving = false;
      Serial.println("DONE");
    }

    delay(getMovementDelay());
  }
}

void processSerialCommand(String command) {
  command.trim();

  if (command.startsWith("MODE")) {
    autoMode = command.substring(5).toInt() == 1;
    if (autoMode) {
      receivedPositions = 0;
      isMoving = false;
    }
    return;
  }

  if (command.startsWith("SPD")) {
    int speed = command.substring(4).toInt();
    if (speed >= MIN_SPEED_VALUE && speed <= MAX_SPEED_VALUE) {
      currentSpeed = speed;
    }
    return;
  }

  int spaceIndex = command.indexOf(' ');
  if (spaceIndex != -1) {
    int servoIndex = command.substring(0, spaceIndex).toInt() - 1;
    float position = command.substring(spaceIndex + 1).toFloat();

    if (servoIndex >= 0 && servoIndex < TOTAL_SERVOS &&
        position >= MIN_SERVO_VALUE && position <= MAX_SERVO_VALUE) {
      
      targetPositions[servoIndex] = position;
      
      if (!autoMode) {
        currentPositions[servoIndex] = position;
        
        if (servoIndex == 5) {
          float oppositePosition = 180 - position;
          pwm.setPWM(OPPOSITE_SERVO_CHANNEL, 0, angleToPulse(oppositePosition));
        }
        
        pwm.setPWM(servoChannels[servoIndex], 0, angleToPulse(position));
      } else {
        receivedPositions++;
        if (receivedPositions == TOTAL_SERVOS) {
          isMoving = true;
          receivedPositions = 0;
        }
      }
    }
  }
}

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      inputComplete = true;
    } else {
      inputBuffer += inChar;
    }
  }
}

