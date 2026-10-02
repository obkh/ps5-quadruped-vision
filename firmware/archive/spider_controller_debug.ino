#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <ps5Controller.h>
#include <math.h>
String lastDetection = "No detection yet";

// UART to ESP32-CAM
// ESP32-CAM TX(GPIO14) -> ESP32 RX(GPIO16)
// ESP32-CAM RX(GPIO15) -> ESP32 TX(GPIO17)
#define CAM_RX 34
#define CAM_TX 35
#define HIP_CENTER   90


// AI mode toggle
bool aiModeOn = false;
bool lastCrossState = false;
#define LED_PIN 27

#define I2C_SDA 21
#define I2C_SCL 22
#define PCA9685_ADDR 0x40

#define SERVOMIN 120
#define SERVOMAX 600
#define SERVO_FREQ 50

#define DEADZONE 20

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDR, Wire);

const int hip[4]  = {0, 2, 4, 6};
const int knee[4] = {1, 3, 5, 7};

// --------------------------------------------------
// Crab stance: each side parallel
// --------------------------------------------------
#define KNEE_DOWN    30

const int hipCenter[4] = { 50, 110, 50, 110 };

// Tuning
#define STEP_LEN     15
#define LIFT_HEIGHT   25
#define TURN_LEN     12

int hipPos[4]  = {90, 90, 90, 90};
int kneePos[4] = {90, 90, 90, 90};

enum RobotCommand { CMD_STOP, CMD_FORWARD, CMD_BACKWARD, CMD_LEFT, CMD_RIGHT, CMD_SPIN_LEFT, CMD_SPIN_RIGHT, CMD_READY, CMD_WAVE };
volatile RobotCommand currentCommand = CMD_STOP;
RobotCommand activeCommand = CMD_STOP;

// --------------------------------------------------
void setServoRaw(int ch, int angle) {
  angle = constrain(angle, 0, 180);
  pwm.setPWM(ch, 0, map(angle, 0, 180, SERVOMIN, SERVOMAX));
}

#define STEP_DELAY   300
#define WALK_DELAY   60

// --------------------------------------------------
void setAllLegs(int h[4], int k[4]) {
  for (int i = 0; i < 4; i++) {
    setServoRaw(hip[i], h[i]);
    setServoRaw(knee[i], k[i]);
    hipPos[i] = h[i];
    kneePos[i] = k[i];
  }
}

// --------------------------------------------------
bool moveAllSmooth(int h0, int k0, int h1, int k1,
                   int h2, int k2, int h3, int k3) {
  int tH[4] = {h0, h1, h2, h3};
  int tK[4] = {k0, k1, k2, k3};
  int startH[4], startK[4], diffH[4], diffK[4];
  int steps = 1;

  for (int i = 0; i < 4; i++) {
    startH[i] = hipPos[i];  startK[i] = kneePos[i];
    diffH[i] = tH[i] - startH[i];
    diffK[i] = tK[i] - startK[i];
    if (abs(diffH[i]) > steps) steps = abs(diffH[i]);
    if (abs(diffK[i]) > steps) steps = abs(diffK[i]);
  }

  for (int s = 1; s <= steps; s++) {
    int h[4], k[4];
    for (int i = 0; i < 4; i++) {
      h[i] = startH[i] + (diffH[i] * s) / steps;
      k[i] = startK[i] + (diffK[i] * s) / steps;
    }
    setAllLegs(h, k);
    delay(1);

    if ((s & 3) == 0 && currentCommand != activeCommand) {
      return false;
    }
  }
  return true;
}

#define MOVE_OR_BAIL(h0,k0,h1,k1,h2,k2,h3,k3) \
  if (!moveAllSmooth(h0,k0,h1,k1,h2,k2,h3,k3)) return;

// --------------------------------------------------
void stand() {
  moveAllSmooth(hipCenter[0], KNEE_DOWN, hipCenter[1], KNEE_DOWN,
                hipCenter[2], KNEE_DOWN, hipCenter[3], KNEE_DOWN);
}

// Bye-bye wave
void waveBye() {
  activeCommand = CMD_WAVE;

  MOVE_OR_BAIL(HIP_CENTER-10, KNEE_DOWN, HIP_CENTER, kneePos[1],
               HIP_CENTER-10, KNEE_DOWN, HIP_CENTER+10, KNEE_DOWN);

  MOVE_OR_BAIL(hipPos[0],kneePos[0],   HIP_CENTER, 120,
               hipPos[2],kneePos[2],   hipPos[3],kneePos[3]);

  for (int i = 0; i < 3; i++) {
    if (currentCommand != activeCommand) return;
    MOVE_OR_BAIL(hipPos[0],kneePos[0],   HIP_CENTER+15, 120,
                 hipPos[2],kneePos[2],   hipPos[3],kneePos[3]);
    MOVE_OR_BAIL(hipPos[0],kneePos[0],   HIP_CENTER-15, 120,
                 hipPos[2],kneePos[2],   hipPos[3],kneePos[3]);
  }

  MOVE_OR_BAIL(hipPos[0],kneePos[0],   HIP_CENTER, 120,
               hipPos[2],kneePos[2],   hipPos[3],kneePos[3]);

  MOVE_OR_BAIL(hipPos[0],kneePos[0],   HIP_CENTER, KNEE_DOWN,
               hipPos[2],kneePos[2],   hipPos[3],kneePos[3]);

  MOVE_OR_BAIL(HIP_CENTER, KNEE_DOWN,  HIP_CENTER, KNEE_DOWN,
               HIP_CENTER, KNEE_DOWN,  HIP_CENTER, KNEE_DOWN);
}
// --------------------------------------------------
// walkCycle — 6-phase diagonal trot (NO recenter to keep
// same-side legs 30° apart and avoid collision)
// --------------------------------------------------
void walkCycle(int dir[4], int amp[4], RobotCommand cmd) {
  activeCommand = cmd;

  int d[4];
  for (int i = 0; i < 4; i++) d[i] = dir[i] * amp[i];

  // === PHASE 1: Lift diagonal pair 0,3 ===
  MOVE_OR_BAIL(hipPos[0], KNEE_DOWN + LIFT_HEIGHT,  hipPos[1], KNEE_DOWN,
               hipPos[2], KNEE_DOWN,                hipPos[3], KNEE_DOWN + LIFT_HEIGHT);
  delay(WALK_DELAY);
  yield();
  if (currentCommand != activeCommand) return;

  // === PHASE 2: Swing 0,3 + push 1,2 ===
  MOVE_OR_BAIL(hipCenter[0] + d[0], KNEE_DOWN + LIFT_HEIGHT,
               hipCenter[1] - d[1], KNEE_DOWN,
               hipCenter[2] - d[2], KNEE_DOWN,
               hipCenter[3] + d[3], KNEE_DOWN + LIFT_HEIGHT);
  delay(WALK_DELAY);
  yield();
  if (currentCommand != activeCommand) return;

  // === PHASE 3: Plant 0,3 ===
  MOVE_OR_BAIL(hipCenter[0] + d[0], KNEE_DOWN,
               hipCenter[1] - d[1], KNEE_DOWN,
               hipCenter[2] - d[2], KNEE_DOWN,
               hipCenter[3] + d[3], KNEE_DOWN);
  delay(WALK_DELAY / 2);
  yield();
  if (currentCommand != activeCommand) return;

  // === PHASE 4: Lift diagonal pair 1,2 ===
  MOVE_OR_BAIL(hipCenter[0] + d[0], KNEE_DOWN,
               hipCenter[1] - d[1], KNEE_DOWN + LIFT_HEIGHT,
               hipCenter[2] - d[2], KNEE_DOWN + LIFT_HEIGHT,
               hipCenter[3] + d[3], KNEE_DOWN);
  delay(WALK_DELAY);
  yield();
  if (currentCommand != activeCommand) return;

  // === PHASE 5: Swing 1,2 + push 0,3 ===
  MOVE_OR_BAIL(hipCenter[0] - d[0], KNEE_DOWN,
               hipCenter[1] + d[1], KNEE_DOWN + LIFT_HEIGHT,
               hipCenter[2] + d[2], KNEE_DOWN + LIFT_HEIGHT,
               hipCenter[3] - d[3], KNEE_DOWN);
  delay(WALK_DELAY);
  yield();
  if (currentCommand != activeCommand) return;

  // === PHASE 6: Plant 1,2 ===
  MOVE_OR_BAIL(hipCenter[0] - d[0], KNEE_DOWN,
               hipCenter[1] + d[1], KNEE_DOWN,
               hipCenter[2] + d[2], KNEE_DOWN,
               hipCenter[3] - d[3], KNEE_DOWN);
  delay(WALK_DELAY / 2);
  yield();
}

// --------------------------------------------------
// FORWARD/BACKWARD — same walkCycle as left/right, all legs
// --------------------------------------------------
void forwardCycle() {
  int dir[4] = {1, -1, 1, -1};
  int amp[4] = {STEP_LEN, STEP_LEN, STEP_LEN, STEP_LEN};
  walkCycle(dir, amp, CMD_FORWARD);
}

void backwardCycle() {
  int dir[4] = {-1, 1, -1, 1};
  int amp[4] = {STEP_LEN, STEP_LEN, STEP_LEN, STEP_LEN};
  walkCycle(dir, amp, CMD_BACKWARD);
}

// --------------------------------------------------
// SPIN — change POV, rotate body in place
// --------------------------------------------------
#define SPIN_ANGLE  45

void spinLeftCycle() {
  activeCommand = CMD_SPIN_LEFT;
  setServoRaw(hip[0], 90); setServoRaw(hip[1], 90);setServoRaw(hip[2], 90);setServoRaw(hip[3], 90);
  // Set hips to 90: h1 and h3 together, then h0 and h2
  delay(STEP_DELAY);
  setServoRaw(knee[0], 130); setServoRaw(knee[2], 130);
  delay(STEP_DELAY);
  setServoRaw(hip[0], 120); setServoRaw(hip[2], 120);
  delay(STEP_DELAY);
  setServoRaw(knee[0], 30); setServoRaw(knee[2], 30);
  delay(STEP_DELAY);
  setServoRaw(knee[1], 130); setServoRaw(knee[3], 130);
  delay(STEP_DELAY);
  setServoRaw(hip[1], 120); setServoRaw(hip[3], 120);
  delay(STEP_DELAY);
  setServoRaw(knee[1], 30); setServoRaw(knee[3], 30);


  // turnLeft
  setServoRaw(knee[2], KNEE_DOWN + LIFT_HEIGHT); setServoRaw(knee[1], KNEE_DOWN + LIFT_HEIGHT);
  delay(STEP_DELAY);
  setServoRaw(hip[2], 120); setServoRaw(hip[1], 120);
  delay(STEP_DELAY);
  setServoRaw(knee[2], KNEE_DOWN); setServoRaw(knee[1], KNEE_DOWN);

  setServoRaw(knee[3], KNEE_DOWN + LIFT_HEIGHT); setServoRaw(knee[0], KNEE_DOWN + LIFT_HEIGHT);
  delay(STEP_DELAY);
  setServoRaw(hip[3], 120); setServoRaw(hip[0], 120);
  delay(STEP_DELAY);
  setServoRaw(knee[3], KNEE_DOWN); setServoRaw(knee[0], KNEE_DOWN);

  delay(STEP_DELAY);
  hipPos[0] = 120; hipPos[1] = 120; hipPos[2] = 120; hipPos[3] = 120;
  kneePos[0] = KNEE_DOWN; kneePos[1] = KNEE_DOWN; kneePos[2] = KNEE_DOWN; kneePos[3] = KNEE_DOWN;
  stand();
}

void spinRightCycle() {
  activeCommand = CMD_SPIN_RIGHT;
  setServoRaw(hip[0], 90); setServoRaw(hip[1], 90);setServoRaw(hip[2], 90);setServoRaw(hip[3], 90);
  // Set hips to 90: h1 and h3 together, then h0 and h2
  delay(STEP_DELAY);
  setServoRaw(knee[0], 130); setServoRaw(knee[2], 130);
  delay(STEP_DELAY);
  setServoRaw(hip[0], 60); setServoRaw(hip[2], 60);
  delay(STEP_DELAY);
  setServoRaw(knee[0], 30); setServoRaw(knee[2], 30);
  delay(STEP_DELAY);
  setServoRaw(knee[1], 130); setServoRaw(knee[3], 130);
  delay(STEP_DELAY);
  setServoRaw(hip[1], 60); setServoRaw(hip[3], 60);
  delay(STEP_DELAY);
  setServoRaw(knee[1], 30); setServoRaw(knee[3], 30);

  // turnRight
  setServoRaw(knee[0], KNEE_DOWN + LIFT_HEIGHT); setServoRaw(knee[3], KNEE_DOWN + LIFT_HEIGHT);
  delay(STEP_DELAY);
  setServoRaw(hip[0], 60); setServoRaw(hip[3], 60);
  delay(STEP_DELAY);
  setServoRaw(knee[0], KNEE_DOWN); setServoRaw(knee[3], KNEE_DOWN);

  setServoRaw(knee[1], KNEE_DOWN + LIFT_HEIGHT); setServoRaw(knee[2], KNEE_DOWN + LIFT_HEIGHT);
  delay(STEP_DELAY);
  setServoRaw(hip[1], 60); setServoRaw(hip[2], 60);
  delay(STEP_DELAY);
  setServoRaw(knee[1], KNEE_DOWN); setServoRaw(knee[2], KNEE_DOWN);

  delay(STEP_DELAY);
  hipPos[0] = 60; hipPos[1] = 60; hipPos[2] = 60; hipPos[3] = 60;
  kneePos[0] = KNEE_DOWN; kneePos[1] = KNEE_DOWN; kneePos[2] = KNEE_DOWN; kneePos[3] = KNEE_DOWN;
  stand();
}

void leftCycle() {
  int dir[4] = {1, 1, 1, 1};
  int amp[4] = {TURN_LEN, TURN_LEN, TURN_LEN, TURN_LEN};
  walkCycle(dir, amp, CMD_LEFT);
}

void rightCycle() {
  int dir[4] = {-1, -1, -1, -1};
  int amp[4] = {TURN_LEN, TURN_LEN, TURN_LEN, TURN_LEN};
  walkCycle(dir, amp, CMD_RIGHT);
}

// --------------------------------------------------
// PS5 callbacks
// --------------------------------------------------
void notify() {
  // X button — toggle AI mode
  bool crossNow = ps5.Cross();
  if (crossNow && !lastCrossState) {
    aiModeOn = !aiModeOn;
    if (aiModeOn) {
      Serial2.println("START_AI");
      digitalWrite(LED_PIN, HIGH);
      Serial.println("[DEBUG] Sent START_AI to CAM via Serial2");
    } else {
      Serial2.println("STOP_AI");
      digitalWrite(LED_PIN, LOW);
      Serial.println("[DEBUG] Sent STOP_AI to CAM via Serial2");
    }
  }
  lastCrossState = crossNow;

  // AI mode active — disable manual control
  if (aiModeOn) {
    currentCommand = CMD_STOP;
    return;
  }

  if (ps5.Circle()) { currentCommand = CMD_WAVE;  return; }

  // Right stick — spin in place
  int rx = ps5.RStickX();
  if (rx < -DEADZONE)     { currentCommand = CMD_SPIN_LEFT;  return; }
  if (rx > DEADZONE)      { currentCommand = CMD_SPIN_RIGHT; return; }

  // D-pad — movement
  if (ps5.Up())          currentCommand = CMD_FORWARD;
  else if (ps5.Down())   currentCommand = CMD_BACKWARD;
  else if (ps5.Left())   currentCommand = CMD_LEFT;
  else if (ps5.Right())  currentCommand = CMD_RIGHT;
  else                    currentCommand = CMD_STOP;
}

void onConnect() {
  Serial.println("PS5 connected!");
}

void onDisconnect() {
  Serial.println("PS5 disconnected!");
  currentCommand = CMD_STOP;
}

// --------------------------------------------------
// UART from ESP32-CAM + web routes
// --------------------------------------------------
void readCamUART() {
  if (Serial2.available()) {
    Serial.printf("[DEBUG] Serial2 has %d bytes\n", Serial2.available());
  }
  while (Serial2.available()) {
    String line = Serial2.readStringUntil('\n');
    line.trim();
    Serial.println("[CAM RAW] " + line);
    if (line.startsWith("DET:")) {
      lastDetection = line.substring(4);
      lastDetection.trim();
      Serial.println("[DETECTION] " + lastDetection);
    }
  }
}

// --------------------------------------------------
void setup() {
  Serial.begin(115200);
  Serial.println("PS5 Spider Controller — Crab FK");

  // LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // UART to ESP32-CAM
  Serial2.begin(115200, SERIAL_8N1, CAM_RX, CAM_TX);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);
  delay(500);

  for (int i = 0; i < 4; i++) {
    setServoRaw(hip[i],  hipPos[i]);
    setServoRaw(knee[i], kneePos[i]);
  }
  delay(300);

  stand();

  ps5.attach(notify);
  ps5.attachOnConnect(onConnect);
  ps5.attachOnDisconnect(onDisconnect);
  ps5.begin("00:00:00:00:00:00"); // Replace with your ESP32 Bluetooth host address before use.

  Serial.println("Waiting for PS5 controller...");
  while (!ps5.isConnected()) {
    Serial.println("PS5 controller not found");
    delay(300);
  }
  Serial.println("Ready!");
}

void loop() {
  readCamUART();

  switch (currentCommand) {
    case CMD_FORWARD:  forwardCycle();  break;
    case CMD_BACKWARD: backwardCycle(); break;
    case CMD_LEFT:     leftCycle();     break;
    case CMD_RIGHT:    rightCycle();    break;
    case CMD_SPIN_LEFT:  spinLeftCycle();  break;
    case CMD_SPIN_RIGHT: spinRightCycle(); break;
    case CMD_WAVE:     waveBye(); currentCommand = CMD_STOP; break;
    case CMD_STOP:
    default:
      stand();
      delay(10);
      break;
  }
}
