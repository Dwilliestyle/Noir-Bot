// Noirbot motor firmware: closed-loop (PID) wheel velocity control.
//
// Serial protocol (unchanged from the open-loop version):
//   host -> Arduino : "V <left_rad_s> <right_rad_s>\n"
//   Arduino -> host : "<left_ticks> <right_ticks>\n"  (cumulative, signed, ~30 Hz)
//
// Requires the "PID" library by Brett Beauregard (PID_v1.h), the same one
// Bumperbot's robot_control.ino uses.

#include <PID_v1.h>

// ============================================================
// PINS
// ============================================================
// Encoder pairing fixed: each wheel's A (interrupt) pin is now paired with
// its OWN B pin, matching Bumperbot's robot_control.ino
// (left A=2/B=4, right A=3/B=5). Previously A=3 was paired with B=4 and
// A=2 with B=5, so each ISR read the other wheel's B channel for direction.
const int LEFT_ENCODER_A_PIN  = 2;   // interrupt pin
const int LEFT_ENCODER_B_PIN  = 4;
const int RIGHT_ENCODER_A_PIN = 3;   // interrupt pin
const int RIGHT_ENCODER_B_PIN = 5;

const int RIGHT_MOTOR_PWM_PIN = 9;   // L298N enA
const int RIGHT_MOTOR_IN1_PIN = 12;
const int RIGHT_MOTOR_IN2_PIN = 13;

const int LEFT_MOTOR_PWM_PIN  = 11;  // L298N enB
const int LEFT_MOTOR_IN3_PIN  = 7;
const int LEFT_MOTOR_IN4_PIN  = 8;

// Flip these if a wheel counts negative when it turns forward.
const int LEFT_TICK_SIGN  = 1;
const int RIGHT_TICK_SIGN = 1;

// ============================================================
// CONSTANTS
// ============================================================
// Ticks per wheel revolution, counting RISING edges on phase A only.
// 385 is what Bumperbot's firmware uses for the same motors. Verify by
// turning a wheel exactly one revolution by hand and reading the tick
// count, and make ticks_per_rev in the xacro match this number.
const double TICKS_PER_REV = 385.0;

const unsigned long PID_INTERVAL_MS    = 100;  // same as Bumperbot
const unsigned long REPORT_INTERVAL_MS = 33;   // ~30 Hz, matches loop_rate_hz
const unsigned long CMD_TIMEOUT_MS     = 500;  // stop if no command received

// PID gains: Bumperbot's values as a starting point.
double Kp_r = 11.5, Ki_r = 7.5, Kd_r = 0.1;
double Kp_l = 12.8, Ki_l = 8.3, Kd_l = 0.1;

// ============================================================
// STATE
// ============================================================
volatile long left_ticks = 0;
volatile long right_ticks = 0;

long prev_left_ticks = 0;
long prev_right_ticks = 0;

// Signed commands from the host (rad/s)
double cmd_left_rad_s = 0.0;
double cmd_right_rad_s = 0.0;

// PID works on speed magnitudes; direction comes from the command sign.
double left_setpoint = 0.0,  left_measured = 0.0,  left_pwm = 0.0;
double right_setpoint = 0.0, right_measured = 0.0, right_pwm = 0.0;

PID leftPID(&left_measured, &left_pwm, &left_setpoint, Kp_l, Ki_l, Kd_l, DIRECT);
PID rightPID(&right_measured, &right_pwm, &right_setpoint, Kp_r, Ki_r, Kd_r, DIRECT);

unsigned long last_cmd_ms = 0;
unsigned long last_pid_ms = 0;
unsigned long last_report_ms = 0;

// ============================================================
// ENCODER ISRs
// ============================================================
void leftEncoderISR()
{
  bool b = digitalRead(LEFT_ENCODER_B_PIN);
  left_ticks += (b ? -1 : 1) * LEFT_TICK_SIGN;
}

void rightEncoderISR()
{
  bool b = digitalRead(RIGHT_ENCODER_B_PIN);
  right_ticks += (b ? 1 : -1) * RIGHT_TICK_SIGN;  // motors face opposite ways
}

// ============================================================
// MOTOR OUTPUT (L298N: two direction pins + one PWM pin per motor)
// ============================================================
void driveMotor(int pwm_pin, int in_pin_a, int in_pin_b, bool forward, double pwm)
{
  if (forward) {
    digitalWrite(in_pin_a, HIGH);
    digitalWrite(in_pin_b, LOW);
  } else {
    digitalWrite(in_pin_a, LOW);
    digitalWrite(in_pin_b, HIGH);
  }
  analogWrite(pwm_pin, constrain(static_cast<int>(pwm), 0, 255));
}

void stopMotors()
{
  analogWrite(LEFT_MOTOR_PWM_PIN, 0);
  analogWrite(RIGHT_MOTOR_PWM_PIN, 0);
}

// Runs one PID step for a wheel. With a zero setpoint the PID is switched
// to MANUAL and its output forced to 0; switching back to AUTOMATIC on the
// next non-zero command re-initialises it, so no integral is left over
// from the previous move.
void updateWheel(PID & pid, double setpoint, double & pwm)
{
  if (setpoint == 0.0) {
    pid.SetMode(MANUAL);
    pwm = 0.0;
  } else {
    pid.SetMode(AUTOMATIC);
    pid.Compute();
  }
}

// ============================================================
// SERIAL PROTOCOL
// ============================================================
// Reads one line if available. Returns true and fills `line` when a
// full '\n'- or '\r'-terminated line has arrived; non-blocking otherwise.
bool readLine(String & line)
{
  static String buffer;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (buffer.length() == 0) {
        continue;  // skip a lone \r or \n (e.g. the other half of \r\n)
      }
      line = buffer;
      buffer = "";
      return true;
    }
    buffer += c;
  }
  return false;
}

// Parses "V <left_rad_s> <right_rad_s>". Returns false on a malformed line.
bool parseVelocityCommand(const String & line, double & left, double & right)
{
  if (line.length() < 2 || line.charAt(0) != 'V') {
    return false;
  }
  int firstSpace = line.indexOf(' ', 2);
  if (firstSpace < 0) {
    return false;
  }
  left = line.substring(2, firstSpace).toDouble();
  right = line.substring(firstSpace + 1).toDouble();
  return true;
}

// ============================================================
// SETUP / LOOP
// ============================================================
void setup()
{
  Serial.begin(115200);  // must match baud_rate in noirbot_ros2_control.xacro

  pinMode(LEFT_ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(LEFT_ENCODER_B_PIN, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_B_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_A_PIN), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_A_PIN), rightEncoderISR, RISING);

  pinMode(RIGHT_MOTOR_PWM_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_IN1_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_IN2_PIN, OUTPUT);
  pinMode(LEFT_MOTOR_PWM_PIN, OUTPUT);
  pinMode(LEFT_MOTOR_IN3_PIN, OUTPUT);
  pinMode(LEFT_MOTOR_IN4_PIN, OUTPUT);

  leftPID.SetSampleTime(PID_INTERVAL_MS);
  rightPID.SetSampleTime(PID_INTERVAL_MS);
  leftPID.SetOutputLimits(0, 255);
  rightPID.SetOutputLimits(0, 255);

  stopMotors();
}

void loop()
{
  // --- Receive commands ---
  String line;
  if (readLine(line)) {
    double l, r;
    if (parseVelocityCommand(line, l, r)) {
      cmd_left_rad_s = l;
      cmd_right_rad_s = r;
      last_cmd_ms = millis();
    }
  }

  unsigned long now = millis();

  // --- Safety stop if the host goes quiet (USB unplugged, node crashed) ---
  if (now - last_cmd_ms > CMD_TIMEOUT_MS) {
    cmd_left_rad_s = 0.0;
    cmd_right_rad_s = 0.0;
  }

  // --- PID step ---
  if (now - last_pid_ms >= PID_INTERVAL_MS) {
    double dt = (now - last_pid_ms) / 1000.0;
    last_pid_ms = now;

    noInterrupts();
    long lt = left_ticks;
    long rt = right_ticks;
    interrupts();

    // Measured wheel speed (magnitude) in rad/s
    left_measured  = fabs((lt - prev_left_ticks)  / TICKS_PER_REV * TWO_PI / dt);
    right_measured = fabs((rt - prev_right_ticks) / TICKS_PER_REV * TWO_PI / dt);
    prev_left_ticks = lt;
    prev_right_ticks = rt;

    left_setpoint  = fabs(cmd_left_rad_s);
    right_setpoint = fabs(cmd_right_rad_s);

    updateWheel(leftPID, left_setpoint, left_pwm);
    updateWheel(rightPID, right_setpoint, right_pwm);

    driveMotor(LEFT_MOTOR_PWM_PIN, LEFT_MOTOR_IN3_PIN, LEFT_MOTOR_IN4_PIN,
               cmd_left_rad_s >= 0.0, left_pwm);
    driveMotor(RIGHT_MOTOR_PWM_PIN, RIGHT_MOTOR_IN1_PIN, RIGHT_MOTOR_IN2_PIN,
               cmd_right_rad_s >= 0.0, right_pwm);
  }

  // --- Report cumulative ticks to the host ---
  if (now - last_report_ms >= REPORT_INTERVAL_MS) {
    last_report_ms = now;
    noInterrupts();
    long lt = left_ticks;
    long rt = right_ticks;
    interrupts();
    Serial.print(lt);
    Serial.print(' ');
    Serial.println(rt);
  }
}