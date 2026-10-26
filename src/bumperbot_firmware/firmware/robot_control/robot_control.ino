#include <PID_v1.h>

// L298N H-Bridge Connection PINs
#define L298N_enA 9  // PWM
#define L298N_enB 11  // PWM
#define L298N_in4 8  // Dir Motor B
#define L298N_in3 7  // Dir Motor B
#define L298N_in2 13  // Dir Motor A
#define L298N_in1 12  // Dir Motor A

// Wheel Encoders Connection PINs
#define right_encoder_phaseA 3  // Interrupt 
#define right_encoder_phaseB 5  
#define left_encoder_phaseA 2   // Interrupt
#define left_encoder_phaseB 4

// Encoder resolution: pulses of phase A per wheel revolution
const double encoder_ppr = 385.0;

// Encoders (written inside the interrupts, so volatile and read with interrupts off)
volatile unsigned int right_encoder_counter = 0;
volatile unsigned int left_encoder_counter = 0;
volatile char right_wheel_sign = 'p';  // 'p' = positive, 'n' = negative
volatile char left_wheel_sign = 'p';  // 'p' = positive, 'n' = negative
unsigned long last_millis = 0;
unsigned long last_micros = 0;
const unsigned long interval = 100;

// Safety: if no complete command arrives for this long (ROS crashed, cable
// unplugged, launch closed), both wheels are stopped.
const unsigned long command_timeout = 500;
unsigned long last_cmd_millis = 0;

// Interpret Serial Messages
bool is_right_wheel_cmd = false;
bool is_left_wheel_cmd = false;
bool is_right_wheel_forward = true;
bool is_left_wheel_forward = true;
char value[] = "00.00";
uint8_t value_idx = 0;

// After a change of direction the PWM is held at 0 for one control cycle and
// the PID starts again from zero, so the motor is never reversed at full duty.
bool right_flip_hold = false;
bool left_flip_hold = false;

// PID
// Setpoint - Desired
double right_wheel_cmd_vel = 0.0;     // rad/s
double left_wheel_cmd_vel = 0.0;      // rad/s
// Input - Measurement
double right_wheel_meas_vel = 0.0;    // rad/s
double left_wheel_meas_vel = 0.0;     // rad/s
// Output - Command
double right_wheel_cmd = 0.0;             // 0-255
double left_wheel_cmd = 0.0;              // 0-255
// Tuning (gains per second; see setup() for the sample time)
double Kp_r = 11.5;
double Ki_r = 7.5;
double Kd_r = 0.1;
double Kp_l = 12.8;
double Ki_l = 8.3;
double Kd_l = 0.1;
// Controller
PID rightMotor(&right_wheel_meas_vel, &right_wheel_cmd, &right_wheel_cmd_vel, Kp_r, Ki_r, Kd_r, DIRECT);
PID leftMotor(&left_wheel_meas_vel, &left_wheel_cmd, &left_wheel_cmd_vel, Kp_l, Ki_l, Kd_l, DIRECT);

void setup() {
  // Init L298N H-Bridge Connection PINs
  pinMode(L298N_enA, OUTPUT);
  pinMode(L298N_enB, OUTPUT);
  pinMode(L298N_in1, OUTPUT);
  pinMode(L298N_in2, OUTPUT);
  pinMode(L298N_in3, OUTPUT);
  pinMode(L298N_in4, OUTPUT);

  // Set Motor Rotation Direction
  digitalWrite(L298N_in1, HIGH);
  digitalWrite(L298N_in2, LOW);
  digitalWrite(L298N_in3, HIGH);
  digitalWrite(L298N_in4, LOW);

  // The control block below runs every 100 ms by millis(). PID_v1 keeps its own
  // timer and skips Compute() if it sees less than SampleTime, which happened when
  // millis() ticked in between. With 90 ms it never skips; Ki and Kd are rescaled
  // so the controller behaves as with the original 100 ms.
  const double sample_ratio = 100.0 / 90.0;
  rightMotor.SetSampleTime(90);
  leftMotor.SetSampleTime(90);
  rightMotor.SetTunings(Kp_r, Ki_r * sample_ratio, Kd_r / sample_ratio);
  leftMotor.SetTunings(Kp_l, Ki_l * sample_ratio, Kd_l / sample_ratio);
  // MANUAL until a non-zero speed is requested (see updateMode)
  rightMotor.SetMode(MANUAL);
  leftMotor.SetMode(MANUAL);
  Serial.begin(115200);

  // Init encoders
  pinMode(right_encoder_phaseB, INPUT);
  pinMode(left_encoder_phaseB, INPUT);
  // Set Callback for Wheel Encoders Pulse
  attachInterrupt(digitalPinToInterrupt(right_encoder_phaseA), rightEncoderCallback, RISING);
  attachInterrupt(digitalPinToInterrupt(left_encoder_phaseA), leftEncoderCallback, RISING);
}

void loop() {
  // Read and Interpret Wheel Velocity Commands
  if (Serial.available())
  {
    char chr = Serial.read();
    // Right Wheel Motor
    if(chr == 'r')
    {
      is_right_wheel_cmd = true;
      is_left_wheel_cmd = false;
      value_idx = 0;
    }
    // Left Wheel Mo tor
    else if(chr == 'l')
    {
      is_right_wheel_cmd = false;
      is_left_wheel_cmd = true;
      value_idx = 0;
    }
    // Positive direction
    else if(chr == 'p')
    {
      if(is_right_wheel_cmd && !is_right_wheel_forward)
      {
        // change the direction of the rotation, with the motor off
        analogWrite(L298N_enA, 0);
        digitalWrite(L298N_in1, HIGH - digitalRead(L298N_in1));
        digitalWrite(L298N_in2, HIGH - digitalRead(L298N_in2));
        is_right_wheel_forward = true;
        right_flip_hold = true;
      }
      else if(is_left_wheel_cmd && !is_left_wheel_forward)
      {
        // change the direction of the rotation, with the motor off
        analogWrite(L298N_enB, 0);
        digitalWrite(L298N_in3, HIGH - digitalRead(L298N_in3));
        digitalWrite(L298N_in4, HIGH - digitalRead(L298N_in4));
        is_left_wheel_forward = true;
        left_flip_hold = true;
      }
    }
    // Negative direction
    else if(chr == 'n')
    {
      if(is_right_wheel_cmd && is_right_wheel_forward)
      {
        // change the direction of the rotation, with the motor off
        analogWrite(L298N_enA, 0);
        digitalWrite(L298N_in1, HIGH - digitalRead(L298N_in1));
        digitalWrite(L298N_in2, HIGH - digitalRead(L298N_in2));
        is_right_wheel_forward = false;
        right_flip_hold = true;
      }
      else if(is_left_wheel_cmd && is_left_wheel_forward)
      {
        // change the direction of the rotation, with the motor off
        analogWrite(L298N_enB, 0);
        digitalWrite(L298N_in3, HIGH - digitalRead(L298N_in3));
        digitalWrite(L298N_in4, HIGH - digitalRead(L298N_in4));
        is_left_wheel_forward = false;
        left_flip_hold = true;
      }
    }
    // Separator
    else if(chr == ',')
    {
      if(is_right_wheel_cmd)
      {
        right_wheel_cmd_vel = atof(value);
      }
      else if(is_left_wheel_cmd)
      {
        left_wheel_cmd_vel = atof(value);
        last_cmd_millis = millis();
      }
      // Reset for next command
      value_idx = 0;
      value[0] = '0';
      value[1] = '0';
      value[2] = '.';
      value[3] = '0';
      value[4] = '0';
      value[5] = '\0';
    }
    // Line endings and spaces (e.g. from the Arduino IDE serial monitor) are ignored
    else if(chr == '\n' || chr == '\r' || chr == ' ')
    {
    }
    // Command Value
    else
    {
      if(value_idx < 5)
      {
        value[value_idx] = chr;
        value_idx++;
      }
    }
  }

  // Encoder
  unsigned long current_millis = millis();
  if(current_millis - last_cmd_millis > command_timeout)
  {
    right_wheel_cmd_vel = 0.0;
    left_wheel_cmd_vel = 0.0;
  }
  if(current_millis - last_millis >= interval)
  {
    noInterrupts();
    unsigned long current_micros = micros();
    unsigned int right_count = right_encoder_counter;
    unsigned int left_count = left_encoder_counter;
    char right_sign = right_wheel_sign;
    char left_sign = left_wheel_sign;
    right_encoder_counter = 0;
    left_encoder_counter = 0;
    interrupts();

    // Speed from the real elapsed time, which is never exactly 100 ms
    double elapsed_s = (current_micros - last_micros) * 1e-6;
    if(last_micros == 0 || elapsed_s <= 0.0)
    {
      elapsed_s = interval * 1e-3;
    }
    last_micros = current_micros;
    const double rad_per_pulse = 2.0 * PI / encoder_ppr;
    right_wheel_meas_vel = right_count * rad_per_pulse / elapsed_s;
    left_wheel_meas_vel = left_count * rad_per_pulse / elapsed_s;

    updateMode(rightMotor, right_wheel_cmd_vel, right_wheel_cmd, right_flip_hold);
    updateMode(leftMotor, left_wheel_cmd_vel, left_wheel_cmd, left_flip_hold);
    rightMotor.Compute();
    leftMotor.Compute();

    Serial.print('r');
    Serial.print(right_sign);
    Serial.print(right_wheel_meas_vel, 2);
    Serial.print(",l");
    Serial.print(left_sign);
    Serial.print(left_wheel_meas_vel, 2);
    Serial.println(',');
    last_millis = current_millis;

    analogWrite(L298N_enA, right_wheel_cmd);
    analogWrite(L298N_enB, left_wheel_cmd);
  }
}

// A zero setpoint (from ROS or from the watchdog) or a change of direction puts
// the PID in MANUAL with the PWM at 0. With the wheel stopped the error is 0 - 0,
// so in AUTOMATIC the integral would keep the last speed's value and the next
// start would begin at the old PWM. Going back to AUTOMATIC re-initialises the
// PID from the current output, which is 0.
void updateMode(PID &pid, double setpoint, double &output, bool &flip_hold)
{
  if(setpoint == 0.0 || flip_hold)
  {
    pid.SetMode(MANUAL);
    output = 0.0;
    flip_hold = false;
  }
  else if(pid.GetMode() == MANUAL)
  {
    output = 0.0;
    pid.SetMode(AUTOMATIC);
  }
}

// New pulse from Right Wheel Encoder
void rightEncoderCallback()
{
  if(digitalRead(right_encoder_phaseB) == HIGH)
  {
    right_wheel_sign = 'p';
  }
  else
  {
    right_wheel_sign = 'n';
  }
  right_encoder_counter++;
}

// New pulse from Left Wheel Encoder
void leftEncoderCallback()
{
  if(digitalRead(left_encoder_phaseB) == HIGH)
  {
    left_wheel_sign = 'n';
  }
  else
  {
    left_wheel_sign = 'p';
  }
  left_encoder_counter++;
}
