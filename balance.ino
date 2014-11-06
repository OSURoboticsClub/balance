#include <I2C.h>
#include <math.h>

#define DEBUG

#define DT 0.005

#define MAX_TILT 2
#define PITCH_TRIM 6

#define POS_KP 1.0
#define POS_KI 0
#define POS_KD 0.9

#define ATT_KP 100
#define ATT_KI 0.1
#define ATT_KD 0.9

#define MPU_I2C_ADDR 0x68

#define PIN_SONAR_TRIG 7
#define PIN_SONAR_ECHO 2

/**
 * A single reading from the MPU. Values are full range.
 */
struct mpu_update_t {
  int16_t accel_xout;
  int16_t accel_yout;
  int16_t accel_zout;

  int16_t temp_out;

  int16_t gyro_xout;
  int16_t gyro_yout;
  int16_t gyro_zout;
};

struct pid_data_t {
  float curr_err;
  float accum_error;
  float prev_err;

  float p;
  float i;
  float d;
};

struct state_t {
  float pitch;

  long sonar_read_deadline;
  float sonar_dist;

  struct pid_data_t pos_pid;
  struct pid_data_t att_pid;
  bool last_dir;

  int print_count;
};

struct state_t state;

/**
 * Called once by the Arduino library. Initializes the MPU and program state.
 */
void setup() {
  memset(&state, 0, sizeof(struct state_t));

  pinMode(PIN_SONAR_TRIG, OUTPUT);
  pinMode(PIN_SONAR_ECHO, INPUT);

  I2c.begin();
  I2c.timeOut(2);
  I2c.write(MPU_I2C_ADDR, 0x6B, 0x00); // Wake the MPU up

#ifdef DEBUG
  Serial.begin(57600);
#endif
}

int16_t read_int16() {
  int16_t read = I2c.receive() << 8;
  read |= I2c.receive();

  return read;
}

bool read_mpu(struct mpu_update_t *update) {
  int err = I2c.read(MPU_I2C_ADDR, 0x3B, 14);

  if(err) {
    return false;
  }

  update->accel_xout = read_int16();
  update->accel_yout = read_int16();
  update->accel_zout = read_int16();
  update->temp_out = read_int16();
  update->gyro_xout = read_int16();
  update->gyro_yout = read_int16();
  update->gyro_zout = read_int16();
}

/**
 * Update the pitch estimate using the latest accelerometer and gyroscope
 * readings.
 */
float read_pitch(struct state_t *state, struct mpu_update_t *update) {
  // Integrate the rotational velocity to get change in position.
  float gyr = ((float) update->gyro_xout) * DT / 131.0f * 2;
  float acc = atan2f(update->accel_yout, update->accel_zout) * 180.0f / M_PI + 90.0f + PITCH_TRIM;

  float acc_mag = sqrt(powf(update->accel_xout / 16384.0f, 2)
                       + powf(update->accel_yout / 16384.0f, 2)
                       + powf(update->accel_zout / 16384.0f, 2));

  if(acc_mag > 0.8 && acc_mag < 1.2) {
    // Combine accelerometer and gyroscope measurements with a complementary
    // filter.
    state->pitch = 0.98 * (gyr + state->pitch) + 0.02 * acc;
  } else {
    // Just use the gyroscope reading.
    state->pitch = gyr + state->pitch;
  }
}

/**
 * Read from the sonar sensor.
 */
float read_sonar_dist(struct state_t *state) {
  // Tell the sensor we want to take a measurement.
  digitalWrite(PIN_SONAR_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_SONAR_TRIG, LOW);

  // Sensor outputs a high pulse proportional to the distance.
  long pulse_width = pulseIn(PIN_SONAR_ECHO, HIGH, 5000);
  float dist = pulse_width / 58.0; // to cm

  // Compensate for the angular offset of the robot.
  float comp = dist * cosf(state->pitch / 180.0 * M_PI);

  return comp;
}

/**
 * Run a PID controller for the given setpoint and process variable. This gives
 * the robot a fast and accurate response to imbalance.
 *
 * Resource: https://en.wikipedia.org/wiki/PID_controller
 */
float update_pid(float sp, float pv, struct pid_data_t *pid, float kp, float ki, float kd) {
  pid->prev_err = pid->curr_err;
  pid->curr_err = sp - pv;
  pid->accum_error += pid->curr_err;

  pid->p = kp * pid->curr_err;
  pid->i = ki * pid->accum_error;
  pid->d = kd * (pid->curr_err - pid->prev_err) / DT;

  return pid->p + pid->i + pid->d;
}

#ifdef DEBUG
void print_debug(struct state_t *state, uint8_t motor_power) {
  if(state->print_count++ == 100) {
    /*
    Serial.print("p: "); Serial.print(state->pid.p);
    Serial.print(" | i: "); Serial.print(state->pid.i);
    Serial.print(" | d: "); Serial.print(state->pid.d);
    */
    Serial.print(" | pow: "); Serial.print(motor_power);
    Serial.print(" | pitch: "); Serial.print(state->pitch);
    Serial.print(" | dist: "); Serial.println(state->sonar_dist);
    state->print_count = 0;
  }
}
#endif

void drive(bool dir, uint8_t motor_power) {
  if(dir) {
    analogWrite(5, motor_power);
    analogWrite(6, 0);

    analogWrite(3, motor_power);
    analogWrite(4, 0);
  } else {
    analogWrite(5, 0);
    analogWrite(6, motor_power);

    analogWrite(3, 0);
    analogWrite(4, motor_power);
  }
}

/**
 * Called repeatedly by the Arduino library. This is our main program logic.
 */
void loop() {
  // The loop is executing at a fixed time interval to simplify velocity
  // integration, so record when the next loop should start.
  unsigned long deadline = micros() + 5000; // 5ms

  struct mpu_update_t update;
  read_mpu(&update);

  state.pitch = read_pitch(&state, &update);

  if(micros() > state.sonar_read_deadline) {
    state.sonar_dist = read_sonar_dist(&state);
    state.sonar_read_deadline = micros() + 100000; // 60ms
  }

  float att_sp = update_pid(30, state.sonar_dist, &state.pos_pid, POS_KP, POS_KI, POS_KD);
  att_sp = min(MAX_TILT, att_sp);

  float motor_sp = update_pid(-att_sp, state.pitch, &state.att_pid, ATT_KP, ATT_KI, ATT_KD);
  uint8_t motor_power = abs(constrain(motor_sp, -255.0, 255.0));

#ifdef DEBUG
  print_debug(&state, motor_power);
#endif

  // The direction of the motors depends on which side the robot is leaning
  // toward.
  bool dir = motor_sp < 0.0 ? true : false;
  drive(dir, motor_power);

  if(dir != state.last_dir) {
    // TODO state.pos_pid.accum_error = 0.0;
    state.att_pid.accum_error = 0.0;
  }

  state.last_dir = dir;

  // Busy-wait until it is time to start the next loop.
  while(micros() < deadline) {
  }
}
