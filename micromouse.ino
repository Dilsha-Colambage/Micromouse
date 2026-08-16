#include <Wire.h>
#include <VL53L0X.h>
#include "driver/pcnt.h"

// ======================== PIN DEFINITIONS ========================
// --- Motors (TB6612FNG) ---
#define AIN1 13
#define AIN2 12
#define PWMA 33
#define BIN1 15
#define BIN2 14
#define PWMB 32
#define STBY 21

// --- Encoders (input-only pins; may need external pull-ups) ---
#define L_ENC_A 35
#define L_ENC_B 34
#define R_ENC_A 36
#define R_ENC_B 39

// --- Time of Flight (I2C shared bus + XSHUT) ---
#define I2C_SDA 23
#define I2C_SCL 19
#define XSHUT_F 5
#define XSHUT_L 4
#define XSHUT_R 2

// ToF runtime I2C addresses (reassigned sequentially at boot)
#define ADDR_FRONT 0x30
#define ADDR_LEFT  0x31
#define ADDR_RIGHT 0x32

// ======================== ROBOT CONFIG ========================
const int ENCODER_CPR = 714;      // PCNT counts both edges (~2x the 356 PPR single-edge figure)
const int CELL_SIZE_MM = 180;
const int WHEEL_CIRC_MM = 89;

// Velocity Profile
const int MIN_PWM = 65;
const int MAX_PWM = 115;
const int RAMP_TICKS = 300;

// Sensing: distance below this (mm) counts as a wall. Tune against the 180mm cell.
const int WALL_THRESHOLD_MM = 110;

// Maze Logic
#define MAZE_SIZE 6
#define NORTH 0
#define EAST  1
#define SOUTH 2
#define WEST  3

int maze[MAZE_SIZE][MAZE_SIZE];
bool nWalls[MAZE_SIZE][MAZE_SIZE], eWalls[MAZE_SIZE][MAZE_SIZE], sWalls[MAZE_SIZE][MAZE_SIZE], wWalls[MAZE_SIZE][MAZE_SIZE];

int robotX = 0, robotY = 0, robotDir = NORTH;
int robotState = 0; // 0=Explore, 1=Return, 2=Wait, 3=SpeedRun
bool v_left, v_front, v_right;

pcnt_unit_t pcntL = PCNT_UNIT_0;
pcnt_unit_t pcntR = PCNT_UNIT_1;

// ======================== TOF SENSORS ========================
VL53L0X sensorFront, sensorLeft, sensorRight;
bool okFront = false, okLeft = false, okRight = false; // init status; fail-safe = treat as wall

// ======================== MOTOR HELPERS ========================
void setMotorLeft(int pwm) {
  pwm = constrain(pwm, -255, 255);
  if (pwm > 0) { digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW); analogWrite(PWMA, pwm); }
  else if (pwm < 0) { digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH); analogWrite(PWMA, -pwm); }
  else { digitalWrite(AIN1, HIGH); digitalWrite(AIN2, HIGH); analogWrite(PWMA, 255); }
}

void setMotorRight(int pwm) {
  pwm = constrain(pwm, -255, 255);
  if (pwm > 0) { digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW); analogWrite(PWMB, pwm); }
  else if (pwm < 0) { digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH); analogWrite(PWMB, -pwm); }
  else { digitalWrite(BIN1, HIGH); digitalWrite(BIN2, HIGH); analogWrite(PWMB, 255); }
}

void stopMotors() { setMotorLeft(0); setMotorRight(0); }

// ======================== REAL SENSORS (replaces virtual UDP) ========================
// Reads one sensor; returns true if a wall is present (or on failure/timeout, fail-safe).
bool wallFromSensor(VL53L0X &s, bool ok) {
  if (!ok) return true; // sensor never initialised -> assume wall
  uint16_t d = s.readRangeContinuousMillimeters();
  if (s.timeoutOccurred()) return true; // no reading -> assume wall
  return (d < WALL_THRESHOLD_MM);
}

void updateVirtualSensors() {
  v_left  = wallFromSensor(sensorLeft,  okLeft);
  v_front = wallFromSensor(sensorFront, okFront);
  v_right = wallFromSensor(sensorRight, okRight);
  Serial.printf("Scanned (%d,%d) -> L:%d F:%d R:%d\n", robotX, robotY, v_left, v_front, v_right);
}

// ======================== NAVIGATION & FLOODFILL ========================
void addWall(int x, int y, int dir) {
  if (dir == NORTH && y < MAZE_SIZE - 1) { nWalls[x][y] = 1; sWalls[x][y+1] = 1; }
  if (dir == EAST  && x < MAZE_SIZE - 1) { eWalls[x][y] = 1; wWalls[x+1][y] = 1; }
  if (dir == SOUTH && y > 0)            { sWalls[x][y] = 1; nWalls[x][y-1] = 1; }
  if (dir == WEST  && x > 0)            { wWalls[x][y] = 1; eWalls[x-1][y] = 1; }
}

void updateMazeWalls() {
  updateVirtualSensors();
  if (v_front) addWall(robotX, robotY, robotDir);
  if (v_left)  addWall(robotX, robotY, (robotDir + 3) % 4);
  if (v_right) addWall(robotX, robotY, (robotDir + 1) % 4);
}

void setGoal(int targetMode) {
  for (int x=0; x<MAZE_SIZE; x++) for (int y=0; y<MAZE_SIZE; y++) maze[x][y] = 255;
  if (targetMode == 0) {
    // Target is Center (Explore & Speed Run)
    maze[2][2]=0; maze[2][3]=0; maze[3][2]=0; maze[3][3]=0;
  } else {
    // Target is Start (Return Run)
    maze[0][0]=0;
  }
}

void floodFill() {
  bool changed = true;
  int safeCap = 255;
  while (changed && safeCap > 0) {
    changed = false;
    safeCap--;
    for (int x = 0; x < MAZE_SIZE; x++) {
      for (int y = 0; y < MAZE_SIZE; y++) {
        if (maze[x][y] == 0) continue;

        int minDist = 255;
        if (!nWalls[x][y] && y < MAZE_SIZE-1) minDist = min(minDist, maze[x][y+1]);
        if (!eWalls[x][y] && x < MAZE_SIZE-1) minDist = min(minDist, maze[x+1][y]);
        if (!sWalls[x][y] && y > 0)           minDist = min(minDist, maze[x][y-1]);
        if (!wWalls[x][y] && x > 0)           minDist = min(minDist, maze[x-1][y]);

        if (minDist < 255 && maze[x][y] != minDist + 1) {
            maze[x][y] = minDist + 1;
            changed = true;
        }
      }
    }
  }
}

int getNextDirection() {
  int bestDir = -1;
  int bestDist = 999;
  int dirs[] = {NORTH, EAST, SOUTH, WEST};

  for (int d : dirs) {
    bool wall = false;
    if (d == NORTH) wall = nWalls[robotX][robotY];
    else if (d == EAST)  wall = eWalls[robotX][robotY];
    else if (d == SOUTH) wall = sWalls[robotX][robotY];
    else if (d == WEST)  wall = wWalls[robotX][robotY];

    if (!wall) {
      int nx = robotX, ny = robotY;
      if (d == NORTH) ny++; else if (d == EAST) nx++; else if (d == SOUTH) ny--; else if (d == WEST) nx--;
      if (nx >= 0 && nx < MAZE_SIZE && ny >= 0 && ny < MAZE_SIZE) {
        if (maze[nx][ny] < bestDist) { bestDist = maze[nx][ny]; bestDir = d; }
      }
    }
  }
  if (bestDir == -1) return (robotDir + 2) % 4;
  return bestDir;
}

// Looks ahead to see how many cells we can drive in a straight line safely
int getStraightRunCount() {
  int count = 0;
  int simX = robotX;
  int simY = robotY;
  int currentDist = maze[simX][simY];

  while (true) {
    bool wallInFront = false;
    if (robotDir == NORTH) wallInFront = nWalls[simX][simY];
    else if (robotDir == EAST) wallInFront = eWalls[simX][simY];
    else if (robotDir == SOUTH) wallInFront = sWalls[simX][simY];
    else if (robotDir == WEST) wallInFront = wWalls[simX][simY];

    if (wallInFront) break;

    int nextX = simX, nextY = simY;
    if (robotDir == NORTH) nextY++; else if (robotDir == EAST) nextX++;
    else if (robotDir == SOUTH) nextY--; else if (robotDir == WEST) nextX--;

    // If the next step doesn't get us closer to the goal, break the combo
    if (maze[nextX][nextY] >= currentDist) break;

    count++;
    simX = nextX; simY = nextY;
    currentDist = maze[simX][simY];

    if (currentDist == 0) break; // Reached goal
  }
  return count;
}

// ======================== PHYSICAL MOVEMENT ========================
// Can move 1 cell (stop-and-go) or N cells (speed run) continuously.
void moveCells(int numCells) {
  pcnt_counter_clear(pcntL); pcnt_counter_clear(pcntR);

  long targetTicks = ((long)CELL_SIZE_MM * numCells * ENCODER_CPR) / WHEEL_CIRC_MM;
  long currentTicks = 0;

  while (currentTicks < targetTicks) {
    int16_t lT, rT;
    pcnt_get_counter_value(pcntL, &lT);
    pcnt_get_counter_value(pcntR, &rT);

    currentTicks = (abs(lT) + abs(rT)) / 2;

    int baseSpeed;
    if (currentTicks < RAMP_TICKS)
        baseSpeed = map(currentTicks, 0, RAMP_TICKS, MIN_PWM, MAX_PWM);
    else if (currentTicks > (targetTicks - RAMP_TICKS))
        baseSpeed = map(currentTicks, targetTicks - RAMP_TICKS, targetTicks, MAX_PWM, MIN_PWM);
    else
        baseSpeed = MAX_PWM;

    setMotorLeft(baseSpeed);
    setMotorRight(baseSpeed);
    delay(1);
  }
  stopMotors();
  delay(150);

  // Update logical position based on how far we traveled
  if (robotDir == NORTH) robotY += numCells; else if (robotDir == EAST) robotX += numCells;
  else if (robotDir == SOUTH) robotY -= numCells; else if (robotDir == WEST) robotX -= numCells;
}

void executeTurn(float targetRotations, int leftSpeed, int rightSpeed) {
  pcnt_counter_clear(pcntL); pcnt_counter_clear(pcntR);
  long targetTicks = targetRotations * ENCODER_CPR;
  setMotorLeft(leftSpeed); setMotorRight(rightSpeed);

  while (true) {
    int16_t lT, rT;
    pcnt_get_counter_value(pcntL, &lT); pcnt_get_counter_value(pcntR, &rT);

    bool lDone = (abs(lT) >= targetTicks) || (leftSpeed == 0);
    bool rDone = (abs(rT) >= targetTicks) || (rightSpeed == 0);

    if (lDone) setMotorLeft(0);
    if (rDone) setMotorRight(0);
    if (lDone && rDone) break;
    delay(1);
  }
  stopMotors();
  delay(200);
}

void setupQuadrature(pcnt_unit_t unit, int pinA, int pinB) {
  pcnt_config_t config = {};
  config.pulse_gpio_num = pinA; config.ctrl_gpio_num = pinB;
  config.unit = unit; config.channel = PCNT_CHANNEL_0;
  config.pos_mode = PCNT_COUNT_INC; config.neg_mode = PCNT_COUNT_DEC;
  config.lctrl_mode = PCNT_MODE_REVERSE; config.hctrl_mode = PCNT_MODE_KEEP;
  config.counter_h_lim = 32767; config.counter_l_lim = -32768;
  pcnt_unit_config(&config);
  pcnt_counter_pause(unit); pcnt_counter_clear(unit); pcnt_counter_resume(unit);
}

// ======================== TOF INIT ========================
void setupToF() {
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000); // 400kHz Fast I2C

  // Hold all sensors in reset, then bring each up one at a time to reassign addresses.
  pinMode(XSHUT_F, OUTPUT); pinMode(XSHUT_L, OUTPUT); pinMode(XSHUT_R, OUTPUT);
  digitalWrite(XSHUT_F, LOW); digitalWrite(XSHUT_L, LOW); digitalWrite(XSHUT_R, LOW);
  delay(10);

  // Boot Front
  digitalWrite(XSHUT_F, HIGH); delay(10);
  okFront = sensorFront.init();
  if (okFront) { sensorFront.setAddress(ADDR_FRONT); sensorFront.setTimeout(500); }
  else Serial.println("WARN: Front ToF init failed");

  // Boot Left
  digitalWrite(XSHUT_L, HIGH); delay(10);
  okLeft = sensorLeft.init();
  if (okLeft) { sensorLeft.setAddress(ADDR_LEFT); sensorLeft.setTimeout(500); }
  else Serial.println("WARN: Left ToF init failed");

  // Boot Right
  digitalWrite(XSHUT_R, HIGH); delay(10);
  okRight = sensorRight.init();
  if (okRight) { sensorRight.setAddress(ADDR_RIGHT); sensorRight.setTimeout(500); }
  else Serial.println("WARN: Right ToF init failed");

  if (okFront) sensorFront.startContinuous(20);
  if (okLeft)  sensorLeft.startContinuous(20);
  if (okRight) sensorRight.startContinuous(20);
}

// ======================== MAIN SETUP & LOOP ========================
void setup() {
  Serial.begin(115200);
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT); pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT); pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT); digitalWrite(STBY, HIGH);

  setupQuadrature(pcntL, L_ENC_A, L_ENC_B);
  setupQuadrature(pcntR, R_ENC_A, R_ENC_B);

  setupToF();

  // Initialize Arrays & Borders
  for (int x=0; x<MAZE_SIZE; x++) {
      for (int y=0; y<MAZE_SIZE; y++) {
          maze[x][y] = 255;
          nWalls[x][y] = eWalls[x][y] = sWalls[x][y] = wWalls[x][y] = 0;
      }
  }
  for(int i=0; i<MAZE_SIZE; i++) {
      sWalls[i][0] = 1; nWalls[i][MAZE_SIZE-1] = 1; wWalls[0][i] = 1; eWalls[MAZE_SIZE-1][i] = 1;
  }

  Serial.println("======================================");
  Serial.println("Send 'g' to start EXPLORATION!");
  Serial.println("======================================");
  while(true) {
      if (Serial.available()) {
          char c = Serial.read();
          if (c == 'g' || c == 'G') break;
      }
  }
}

void loop() {

  // ---------------- STATE 0: EXPLORE ----------------
  if (robotState == 0) {
    updateMazeWalls();
    setGoal(0); // Set goal to Center
    floodFill();

    if (maze[robotX][robotY] == 0) {
      Serial.println("CENTER FOUND! Preparing to return...");
      robotState = 1;
      delay(1000);
      return;
    }

    int nextDir = getNextDirection();
    if (nextDir != robotDir) {
      int diff = (nextDir - robotDir + 4) % 4;
      if (diff == 1) executeTurn(1.35, 70, 0); else if (diff == 2) executeTurn(1.30, 70, -70); else if (diff == 3) executeTurn(1.35, 0, 70);
      robotDir = nextDir;
    }
    moveCells(1); // Stop-and-Go exploration
  }

  // ---------------- STATE 1: RETURN TO START ----------------
  else if (robotState == 1) {
    updateMazeWalls(); // Keep scanning to find faster return paths
    setGoal(1); // Set goal to Start (0,0)
    floodFill();

    if (maze[robotX][robotY] == 0) {
      Serial.println("RETURNED TO START!");
      Serial.println("Press 's' to initiate SPEED RUN.");
      stopMotors();
      robotState = 2;
      return;
    }

    int nextDir = getNextDirection();
    if (nextDir != robotDir) {
      int diff = (nextDir - robotDir + 4) % 4;
      if (diff == 1) executeTurn(1.35, 70, 0); else if (diff == 2) executeTurn(1.30, 70, -70); else if (diff == 3) executeTurn(1.35, 0, 70);
      robotDir = nextDir;
    }
    moveCells(1); // Stop-and-Go return
  }

  // ---------------- STATE 2: WAITING ----------------
  else if (robotState == 2) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c == 's' || c == 'S') {
        Serial.println("SPEED RUN INITIATED!");
        robotState = 3;
      }
    }
    delay(50);
  }

  // ---------------- STATE 3: SPEED RUN ----------------
  else if (robotState == 3) {
    updateMazeWalls(); // Refresh walls from real sensors as we go
    setGoal(0); // Goal is Center again
    floodFill(); // Calculate the fastest path

    if (maze[robotX][robotY] == 0) {
      Serial.println("SPEED RUN COMPLETE!");
      stopMotors();
      while(1) delay(1000); // Done
    }

    int nextDir = getNextDirection();
    if (nextDir != robotDir) {
      int diff = (nextDir - robotDir + 4) % 4;
      if (diff == 1) executeTurn(1.35, 70, 0); else if (diff == 2) executeTurn(1.30, 70, -70); else if (diff == 3) executeTurn(1.35, 0, 70);
      robotDir = nextDir;
    }

    // Look ahead to see how many straight cells we can blast through
    int continuousCells = getStraightRunCount();
    if (continuousCells < 1) continuousCells = 1;

    Serial.printf("Speed run: Surging %d cells forward!\n", continuousCells);
    moveCells(continuousCells);
  }
}
