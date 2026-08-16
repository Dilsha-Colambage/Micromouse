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
// STBY is tied directly to 3.3V on the board (not driven by the ESP32).
// Pin 21 is used for I2C SDA (see below).

// --- Encoders (input-only pins; may need external pull-ups) ---
#define L_ENC_A 35
#define L_ENC_B 34
#define R_ENC_A 36
#define R_ENC_B 39

// --- Time of Flight (I2C shared bus + XSHUT) ---
#define I2C_SDA 21
#define I2C_SCL 22
#define XSHUT_F 4
#define XSHUT_L 5
#define XSHUT_R 18

// --- Onboard BOOT button (reused as a start switch after boot) ---
#define BTN_BOOT 0   // GPIO0, active-low (pressed = LOW), has internal pull-up

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

// Front-wall ranging: if a wall ahead gets this close (mm) while driving, stop early
// instead of finishing the fixed tick count. Prevents nosing into the front wall and
// re-references position each time we face a wall. Tune to your desired standoff.
const int FRONT_STOP_MM = 70;

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

// ======================== BOOT BUTTON ========================
// Returns true on a debounced press. Uses the pattern verified working on this board.
bool bootPressed() {
  if (digitalRead(BTN_BOOT) == LOW) {   // LOW means pressed
    delay(50);                          // software debounce filter
    if (digitalRead(BTN_BOOT) == LOW) { // double check it is still held down
      return true;
    }
  }
  return false;
}

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
// Last filtered distances in mm. 9999 = out of range / open corridor / glitch read.
int dLeft = 9999, dFront = 9999, dRight = 9999;

// Sentinel returned for an open corridor or an unusable reading.
const int TOF_OPEN = 9999;

// Reads one sensor, filtering out the VL53L0X's junk values (0, 65535, timeouts).
int readDistance(VL53L0X &s, bool ok) {
  if (!ok) return -1; // sensor never initialised -> caller treats as a wall
  uint16_t d = s.readRangeContinuousMillimeters();
  if (s.timeoutOccurred() || d == 0 || d == 65535) return TOF_OPEN; // glitch -> open
  return (int)d;
}

// A wall is present if the sensor is dead (-1) or the reading is under threshold.
bool distIsWall(int d) { return (d < 0) || (d < WALL_THRESHOLD_MM); }

// Refresh the distances only (used for live telemetry while driving).
void readToF() {
  dLeft  = readDistance(sensorLeft,  okLeft);
  dFront = readDistance(sensorFront, okFront);
  dRight = readDistance(sensorRight, okRight);
}

void printToF(const char *tag) {
  Serial.printf("%s ToF[mm] L:%4d F:%4d R:%4d  (thr=%d)\n", tag, dLeft, dFront, dRight, WALL_THRESHOLD_MM);
}

void updateVirtualSensors() {
  readToF();
  v_left  = distIsWall(dLeft);
  v_front = distIsWall(dFront);
  v_right = distIsWall(dRight);
  Serial.printf("Scanned (%d,%d) -> L:%d F:%d R:%d | mm L:%d F:%d R:%d\n",
                robotX, robotY, v_left, v_front, v_right, dLeft, dFront, dRight);
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
  unsigned long lastTele = 0;
  unsigned long lastFront = 0;

  while (currentTicks < targetTicks) {
    int16_t lT, rT;
    pcnt_get_counter_value(pcntL, &lT);
    pcnt_get_counter_value(pcntR, &rT);

    currentTicks = (abs(lT) + abs(rT)) / 2;

    // Front-wall ranging (~30 Hz): stop early if we've reached the standoff from a
    // wall ahead, so a tick-count overshoot never noses us into the front wall.
    if (okFront && millis() - lastFront >= 30) {
      dFront = readDistance(sensorFront, okFront);
      lastFront = millis();
      if (dFront != TOF_OPEN && dFront <= FRONT_STOP_MM) {
        Serial.printf("[front-stop] wall at %d mm, halting early\n", dFront);
        break;
      }
    }

    // Live ToF telemetry while driving (throttled to ~10 Hz so it never stalls motion)
    if (millis() - lastTele >= 100) {
      readToF();
      printToF("[drive]");
      lastTele = millis();
    }

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

// Turn parameters (spin in place: both wheels drive opposite directions).
// Per-wheel rotations for a 90 deg spin is ~half the old single-wheel pivot value.
const float TURN_90_ROT  = 0.675; // was 1.35 rot on one wheel when pivoting
const float TURN_180_ROT = 1.30;  // unchanged (already a spin)
const int   TURN_SPEED   = 70;

// Rotate in place by the shortest amount to face nextDir, given current robotDir.
// diff: 1 = turn right 90, 2 = 180, 3 = turn left 90.
void turnByDiff(int diff) {
  if (diff == 1)      executeTurn(TURN_90_ROT,  TURN_SPEED, -TURN_SPEED); // spin right
  else if (diff == 2) executeTurn(TURN_180_ROT, TURN_SPEED, -TURN_SPEED); // spin 180
  else if (diff == 3) executeTurn(TURN_90_ROT, -TURN_SPEED,  TURN_SPEED); // spin left
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
// Brings one sensor out of reset and initialises it, retrying a few times so a
// slow-rising XSHUT line or a busy bus doesn't leave a good sensor offline.
bool bootSensor(VL53L0X &s, int xshutPin, uint8_t addr, const char *name) {
  digitalWrite(xshutPin, HIGH);
  delay(50); // settle time (longer than before for reliable power-up)

  for (int attempt = 1; attempt <= 3; attempt++) {
    s.setTimeout(500);
    if (s.init()) {
      s.setAddress(addr);
      s.startContinuous();
      Serial.printf("[OK] %s ToF at 0x%02X (attempt %d)\n", name, addr, attempt);
      return true;
    }
    Serial.printf("[..] %s ToF init failed, retry %d\n", name, attempt);
    delay(30);
  }
  Serial.printf("[ERROR] %s ToF failed to initialize!\n", name);
  return false;
}

void setupToF() {
  Wire.begin(I2C_SDA, I2C_SCL); // default 100kHz (matches the verified 3-sensor sketch)

  // Hold all sensors in reset simultaneously, then bring each up one at a time.
  pinMode(XSHUT_F, OUTPUT); digitalWrite(XSHUT_F, LOW);
  pinMode(XSHUT_L, OUTPUT); digitalWrite(XSHUT_L, LOW);
  pinMode(XSHUT_R, OUTPUT); digitalWrite(XSHUT_R, LOW);
  delay(100); // full power-down settle (verified value)

  okFront = bootSensor(sensorFront, XSHUT_F, ADDR_FRONT, "Front");
  okLeft  = bootSensor(sensorLeft,  XSHUT_L, ADDR_LEFT,  "Left");
  okRight = bootSensor(sensorRight, XSHUT_R, ADDR_RIGHT, "Right");
}

// ======================== MAIN SETUP & LOOP ========================
void setup() {
  Serial.begin(115200);
  delay(2000); // let USB-serial settle and give GPIO0 time to release after boot

  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT); pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT); pinMode(PWMB, OUTPUT);
  // STBY tied to 3.3V in hardware; no GPIO setup needed.

  pinMode(BTN_BOOT, INPUT_PULLUP); // BOOT button as start switch

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
  Serial.println("Press BOOT button (or send 'g') to start EXPLORATION!");
  Serial.println("======================================");
  while(true) {
      if (bootPressed()) break;
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
      turnByDiff(diff);
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
      turnByDiff(diff);
      robotDir = nextDir;
    }
    moveCells(1); // Stop-and-Go return
  }

  // ---------------- STATE 2: WAITING ----------------
  else if (robotState == 2) {
    if (bootPressed()) {
      Serial.println("SPEED RUN INITIATED!");
      robotState = 3;
    } else if (Serial.available()) {
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
      turnByDiff(diff);
      robotDir = nextDir;
    }

    // Look ahead to see how many straight cells we can blast through
    int continuousCells = getStraightRunCount();
    if (continuousCells < 1) continuousCells = 1;

    Serial.printf("Speed run: Surging %d cells forward!\n", continuousCells);
    moveCells(continuousCells);
  }
}
