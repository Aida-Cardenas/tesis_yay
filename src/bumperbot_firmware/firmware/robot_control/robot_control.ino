#include <PID_v1.h>

// Serial protocol at 115200 baud, one line every 100 ms:
//   to the PC   "rp4.55,ln0.12,rc1234,lc65000,"
//               r/l wheel, p/n sign and speed in rad/s, then rc/lc the wheel's
//               encoder counter (0-65535, wraps around; the PC uses differences)
//   from the PC "rp05.20,ln03.10," wheel, sign and speed setpoint in rad/s

// Encoder resolution: both edges of phase A are counted, so this is twice the
// number of phase A pulses per wheel revolution (385).
const double counts_per_rev = 770.0;
const double rad_per_count = 2.0 * PI / counts_per_rev;

// Control period and safety timeout for commands from the PC (ROS crashed, cable
// unplugged, launch closed: both wheels stop).
const unsigned long interval = 100;
const unsigned long command_timeout = 500;

struct Wheel {
  uint8_t en, in1, in2;      // L298N: PWM and direction pins
  uint8_t enc_a, enc_b;      // encoder phases (phase A on an interrupt pin)
  int8_t enc_sign;           // +1 if A == B on an edge means positive rotation
  char name;                 // 'r' or 'l' in the serial protocol
  volatile uint16_t count;   // written by the interrupt
  uint16_t last_count;
  bool forward;              // commanded direction
  bool flip_hold;            // PWM held at 0 for one cycle after a direction change
  double setpoint;           // rad/s, magnitude
  double measured;           // rad/s, positive in the commanded direction
  double output;             // PWM 0-255
};

Wheel right = {9, 12, 13, 3, 5, +1, 'r', 0, 0, true, false, 0.0, 0.0, 0.0};
Wheel left = {11, 7, 8, 2, 4, -1, 'l', 0, 0, true, false, 0.0, 0.0, 0.0};

// Gains per second; see setup() for the sample time
PID rightPid(&right.measured, &right.output, &right.setpoint, 11.5, 7.5, 0.1, DIRECT);
PID leftPid(&left.measured, &left.output, &left.setpoint, 12.8, 8.3, 0.1, DIRECT);

Wheel *cmd_wheel = nullptr;  // wheel of the command being received
char value[] = "00.00";
uint8_t value_idx = 0;
unsigned long last_millis = 0;
unsigned long last_micros = 0;
unsigned long last_cmd_millis = 0;

void setDirection(Wheel &w, bool forward);
void updateMode(PID &pid, Wheel &w);
void measure(Wheel &w, double signed_speed);
void report(const Wheel &w, int16_t delta, double elapsed_s);
void readSerial();

void countEdge(Wheel &w) {
  w.count += (digitalRead(w.enc_a) == digitalRead(w.enc_b)) ? w.enc_sign : -w.enc_sign;
}
void rightEncoderCallback() { countEdge(right); }
void leftEncoderCallback() { countEdge(left); }

void setupWheel(Wheel &w, PID &pid, void (*isr)()) {
  pinMode(w.en, OUTPUT);
  pinMode(w.in1, OUTPUT);
  pinMode(w.in2, OUTPUT);
  digitalWrite(w.in1, HIGH);
  digitalWrite(w.in2, LOW);
  pinMode(w.enc_a, INPUT);
  pinMode(w.enc_b, INPUT);
  attachInterrupt(digitalPinToInterrupt(w.enc_a), isr, CHANGE);

  // The control block runs every 100 ms by millis(). PID_v1 keeps its own timer
  // and skips Compute() if it sees less than SampleTime, which happened when
  // millis() ticked in between. With 90 ms it never skips; Ki and Kd are
  // rescaled so the controller behaves as with the original 100 ms.
  const double sample_ratio = 100.0 / 90.0;
  pid.SetSampleTime(90);
  pid.SetTunings(pid.GetKp(), pid.GetKi() * sample_ratio, pid.GetKd() / sample_ratio);
  // MANUAL until a non-zero speed is requested (see updateMode)
  pid.SetMode(MANUAL);
}

void setup() {
  setupWheel(right, rightPid, rightEncoderCallback);
  setupWheel(left, leftPid, leftEncoderCallback);
  Serial.begin(115200);
}

void loop() {
  readSerial();

  unsigned long current_millis = millis();
  if (current_millis - last_cmd_millis > command_timeout) {
    right.setpoint = 0.0;
    left.setpoint = 0.0;
  }
  if (current_millis - last_millis < interval) {
    return;
  }
  last_millis = current_millis;

  noInterrupts();
  unsigned long current_micros = micros();
  uint16_t right_count = right.count;
  uint16_t left_count = left.count;
  interrupts();

  // Speed from the real elapsed time, which is never exactly 100 ms
  double elapsed_s = (current_micros - last_micros) * 1e-6;
  if (last_micros == 0 || elapsed_s <= 0.0) {
    elapsed_s = interval * 1e-3;
  }
  last_micros = current_micros;

  int16_t right_delta = (int16_t)(right_count - right.last_count);
  int16_t left_delta = (int16_t)(left_count - left.last_count);
  right.last_count = right_count;
  left.last_count = left_count;

  measure(right, right_delta * rad_per_count / elapsed_s);
  measure(left, left_delta * rad_per_count / elapsed_s);
  updateMode(rightPid, right);
  updateMode(leftPid, left);
  rightPid.Compute();
  leftPid.Compute();

  report(right, right_delta, elapsed_s);
  Serial.print(',');
  report(left, left_delta, elapsed_s);
  Serial.print(",rc");
  Serial.print(right_count);
  Serial.print(",lc");
  Serial.print(left_count);
  Serial.println(',');

  analogWrite(right.en, (int)right.output);
  analogWrite(left.en, (int)left.output);
}

// The PID works with the speed in the commanded direction: a wheel turning the
// other way counts as negative, so the PID pushes harder instead of seeing it as
// already moving.
void measure(Wheel &w, double signed_speed) {
  w.measured = w.forward ? signed_speed : -signed_speed;
}

void report(const Wheel &w, int16_t delta, double elapsed_s) {
  Serial.print(w.name);
  Serial.print(delta < 0 ? 'n' : 'p');
  Serial.print(abs(delta) * rad_per_count / elapsed_s, 2);
}

// Reads at most one character per loop, so the control block is never delayed.
void readSerial() {
  if (!Serial.available()) {
    return;
  }
  char chr = Serial.read();
  if (chr == 'r' || chr == 'l') {
    cmd_wheel = (chr == 'r') ? &right : &left;
    value_idx = 0;
  } else if (chr == 'p' || chr == 'n') {
    if (cmd_wheel != nullptr) {
      setDirection(*cmd_wheel, chr == 'p');
    }
  } else if (chr == ',') {
    if (cmd_wheel != nullptr) {
      cmd_wheel->setpoint = atof(value);
      if (cmd_wheel == &left) {
        last_cmd_millis = millis();
      }
    }
    strcpy(value, "00.00");
    value_idx = 0;
  } else if (chr == '\n' || chr == '\r' || chr == ' ') {
    // line endings and spaces (e.g. from the Arduino IDE serial monitor)
  } else if (value_idx < 5) {
    value[value_idx++] = chr;
  }
}

// A change of direction switches the H-bridge with the motor off and holds the
// PWM at 0 until the next control cycle.
void setDirection(Wheel &w, bool forward) {
  if (w.forward == forward) {
    return;
  }
  analogWrite(w.en, 0);
  digitalWrite(w.in1, forward ? HIGH : LOW);
  digitalWrite(w.in2, forward ? LOW : HIGH);
  w.forward = forward;
  w.flip_hold = true;
}

// A zero setpoint (from ROS or from the watchdog) or a change of direction puts
// the PID in MANUAL with the PWM at 0. With the wheel stopped the error is 0 - 0,
// so in AUTOMATIC the integral would keep the last speed's value and the next
// start would begin at the old PWM. Going back to AUTOMATIC re-initialises the
// PID from the current output, which is 0.
void updateMode(PID &pid, Wheel &w) {
  if (w.setpoint == 0.0 || w.flip_hold) {
    pid.SetMode(MANUAL);
    w.output = 0.0;
    w.flip_hold = false;
  } else if (pid.GetMode() == MANUAL) {
    w.output = 0.0;
    pid.SetMode(AUTOMATIC);
  }
}
