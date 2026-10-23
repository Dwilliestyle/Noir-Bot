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

const int LEFT_MOTOR_PWM_PIN  = 9;   // L298N enA
const int LEFT_MOTOR_IN3_PIN  = 12;
const int LEFT_MOTOR_IN4_PIN  = 13;

const int RIGHT_MOTOR_PWM_PIN = 11;  // L298N enB
const int RIGHT_MOTOR_IN1_PIN = 7;
const int RIGHT_MOTOR_IN2_PIN = 8;

// Flip these if a wheel counts negative when it turns forward.
const int LEFT_TICK_SIGN  = -1;
const int RIGHT_TICK_SIGN = -1;

// ============================================================
// CONSTANTS
// ============================================================
// Measured on Noirbot: 3 hand turns = 1871 / 1869
const double TICKS_PER_REV = 623.0;

const unsigned long PID_INTERVAL_MS    = 100;  // same as Bumperbot
const unsigned long REPORT_INTERVAL_MS = 33;   // ~30 Hz, matches loop_rate_hz
const unsigned long CMD_TIMEOUT_MS     = 500;  // stop if no command received

// PID gains: Kp lowered for the measured-curve feedforward (same as Indigobot).
double Kp_r = 3.0, Ki_r = 7.5, Kd_r = 0.0;   // FF: Kp was 6.0, Kd was 0.1
double Kp_l = 3.0, Ki_l = 8.3, Kd_l = 0.0;   // FF: Kp was 6.0, Kd was 0.1

// FF: measured open-loop curve, wheels lifted, average of both wheels.
// FF_SPEED[k] is the speed reached at PWM = 15 + 15*k  (15, 30, 45 ... 255)
const int FF_N = 17;
const double FF_SPEED[FF_N] = {
   0.00,  1.59,  6.57,  9.67, 11.79, 13.26, 14.30, 15.05, 15.61,
  16.05, 16.39, 16.69, 16.92, 17.14, 17.30, 17.45, 17.84
};

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
double left_setpoint = 0.0,  left_measured = 0.0,  left_pid_out = 0.0,  left_pwm = 0.0;
double right_setpoint = 0.0, right_measured = 0.0, right_pid_out = 0.0, right_pwm = 0.0;

PID leftPID(&left_measured, &left_pid_out, &left_setpoint, Kp_l, Ki_l, Kd_l, DIRECT);
PID rightPID(&right_measured, &right_pid_out, &right_setpoint, Kp_r, Ki_r, Kd_r, DIRECT);
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

// BRAKE: set direction and power, or short the motor terminals to brake.
void driveMotor(int pwm_pin, int in_pin_a, int in_pin_b, bool forward, double pwm, bool brake)
{
  if (brake) {
    digitalWrite(in_pin_a, LOW);
    digitalWrite(in_pin_b, LOW);
    analogWrite(pwm_pin, 255);
    return;
  }
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

// FF: PWM for a target wheel speed, interpolated from the measured curve
double feedforward(double target)
{
  if (target <= 0.0) return 0.0;
  if (target >= FF_SPEED[FF_N - 1]) return 255.0;
  for (int i = 1; i < FF_N; i++) {
    if (target <= FF_SPEED[i]) {
      double frac = (target - FF_SPEED[i - 1]) / (FF_SPEED[i] - FF_SPEED[i - 1]);
      return 15.0 * i + 15.0 * frac;   // PWM of point i-1 is 15*i
    }
  }
  return 255.0;
}

// Feedforward sets the base PWM from the target speed; the PID adds or
// subtracts a correction. A zero setpoint stops the wheel and resets the PID.
void updateWheel(PID & pid, double setpoint, double & pid_out, double & pwm)
{
  if (setpoint == 0.0) {
    pid.SetMode(MANUAL);
    pid_out = 0.0;
    pwm = 0.0;
  } else {
    pid.SetMode(AUTOMATIC);
    pid.Compute();
    double ff = feedforward(setpoint);   // FF: was FF_MIN_PWM + FF_PWM_PER_RAD_S * setpoint
    pwm = constrain(ff + pid_out, 0.0, 255.0);
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
  leftPID.SetOutputLimits(-255, 255);
  rightPID.SetOutputLimits(-255, 255);

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

    updateWheel(leftPID, left_setpoint, left_pid_out, left_pwm);
    updateWheel(rightPID, right_setpoint, right_pid_out, right_pwm);

    driveMotor(LEFT_MOTOR_PWM_PIN, LEFT_MOTOR_IN3_PIN, LEFT_MOTOR_IN4_PIN,
               cmd_left_rad_s >= 0.0, left_pwm, left_setpoint == 0.0);     // BRAKE
    driveMotor(RIGHT_MOTOR_PWM_PIN, RIGHT_MOTOR_IN1_PIN, RIGHT_MOTOR_IN2_PIN,
               cmd_right_rad_s >= 0.0, right_pwm, right_setpoint == 0.0);  // BRAKE
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
