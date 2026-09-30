// Noirbot motor/encoder firmware — matches noirbot_firmware's NoirInterface.
// Motor driver: L298N (two direction pins + one PWM enable pin per motor),
// pins confirmed from Bumperbot's working robot_control.ino.
//
// Protocol (must match noir_interface.cpp exactly):
//   RX (host -> board): "V <left_rad_s> <right_rad_s>\n"
//   TX (board -> host):  "<left_ticks> <right_ticks>\n"   sent every loop
//
// ============================================================
// CONFIGURATION — confirmed against Bumperbot's L298N wiring.
// ============================================================
const int LEFT_ENCODER_A_PIN  = 3;   // interrupt pin
const int LEFT_ENCODER_B_PIN  = 4;
const int RIGHT_ENCODER_A_PIN = 2;   // interrupt pin
const int RIGHT_ENCODER_B_PIN = 5;

const int RIGHT_MOTOR_PWM_PIN = 9;   // L298N enA
const int RIGHT_MOTOR_IN1_PIN = 12;
const int RIGHT_MOTOR_IN2_PIN = 13;

const int LEFT_MOTOR_PWM_PIN  = 11;  // L298N enB
const int LEFT_MOTOR_IN3_PIN  = 7;
const int LEFT_MOTOR_IN4_PIN  = 8;

const double TICKS_PER_REV = 2240.0;   // TODO: match ticks_per_rev in the xacro
const double WHEEL_RADIUS_M = 0.033;   // TODO: match noirbot_controllers.yaml

// Open-loop rad/s -> PWM scale. This is a placeholder: it maps commanded
// velocity linearly onto the PWM range with no feedback. Bumperbot's own
// firmware runs a PID loop here instead (see robot_control.ino) — worth
// porting over once this open-loop version is confirmed to move at all.
const double MAX_RAD_S = 6.0;    // rad/s that maps to full PWM (255)
const int MAX_PWM = 255;

// ============================================================
// STATE
// ============================================================
volatile long left_ticks = 0;
volatile long right_ticks = 0;

double cmd_left_rad_s = 0.0;
double cmd_right_rad_s = 0.0;

unsigned long last_cmd_ms = 0;
const unsigned long CMD_TIMEOUT_MS = 500;  // stop if no command received

unsigned long last_report_ms = 0;
const unsigned long REPORT_INTERVAL_MS = 33;  // ~30 Hz, matches loop_rate_hz

// ============================================================
// ENCODER ISRs
// ============================================================
void leftEncoderISR()
{
  bool b = digitalRead(LEFT_ENCODER_B_PIN);
  left_ticks += b ? -1 : 1;
}

void rightEncoderISR()
{
  bool b = digitalRead(RIGHT_ENCODER_B_PIN);
  right_ticks += b ? 1 : -1;  // reversed vs. left: motors face opposite ways
}

// ============================================================
// MOTOR OUTPUT — L298N: two direction pins + one PWM pin per motor.
// ============================================================
void setMotor(int pwm_pin, int in_pin_a, int in_pin_b, double rad_s)
{
  double clamped = constrain(rad_s, -MAX_RAD_S, MAX_RAD_S);
  int pwm = static_cast<int>(abs(clamped) / MAX_RAD_S * MAX_PWM);
  pwm = constrain(pwm, 0, MAX_PWM);

  if (clamped >= 0) {
    digitalWrite(in_pin_a, HIGH);
    digitalWrite(in_pin_b, LOW);
  } else {
    digitalWrite(in_pin_a, LOW);
    digitalWrite(in_pin_b, HIGH);
  }
  analogWrite(pwm_pin, pwm);
}

void stopMotors()
{
  analogWrite(LEFT_MOTOR_PWM_PIN, 0);
  analogWrite(RIGHT_MOTOR_PWM_PIN, 0);
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

  // Default direction, matching Bumperbot's setup() — harmless since
  // stopMotors() below zeroes PWM regardless of direction pin state.
  digitalWrite(RIGHT_MOTOR_IN1_PIN, HIGH);
  digitalWrite(RIGHT_MOTOR_IN2_PIN, LOW);
  digitalWrite(LEFT_MOTOR_IN3_PIN, HIGH);
  digitalWrite(LEFT_MOTOR_IN4_PIN, LOW);

  stopMotors();
}

void loop()
{
  String line;
  if (readLine(line)) {
    double l, r;
    if (parseVelocityCommand(line, l, r)) {
      cmd_left_rad_s = l;
      cmd_right_rad_s = r;
      last_cmd_ms = millis();
    }
  }

  // Safety stop if the host goes quiet (USB unplugged, node crashed, etc.)
  if (millis() - last_cmd_ms > CMD_TIMEOUT_MS) {
    cmd_left_rad_s = 0.0;
    cmd_right_rad_s = 0.0;
  }

  setMotor(LEFT_MOTOR_PWM_PIN, LEFT_MOTOR_IN3_PIN, LEFT_MOTOR_IN4_PIN, cmd_left_rad_s);
  setMotor(RIGHT_MOTOR_PWM_PIN, RIGHT_MOTOR_IN1_PIN, RIGHT_MOTOR_IN2_PIN, cmd_right_rad_s);

  if (millis() - last_report_ms >= REPORT_INTERVAL_MS) {
    last_report_ms = millis();
    noInterrupts();
    long lt = left_ticks;
    long rt = right_ticks;
    interrupts();
    Serial.print(lt);
    Serial.print(' ');
    Serial.println(rt);
  }
}