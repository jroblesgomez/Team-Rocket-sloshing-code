
// Juan R. Robles Gómez. 2019.
// Team Rocket Sloshing. Control System 

// Rocket control system for Arduino micro
// This code controls one IMU: MPU 6050 (accelermeter + gyroscope), two servomotors, 1 green led and 1 red led.
// More information at README.md


// Libraries
#include <Wire.h>
#include <MPU6050.h>
#include <I2Cdev.h>
#include <Servo.h>


// Objetcts & variables
MPU6050 IMU;
Servo port;
Servo starboard;

int flap_angle = 5;

const int portpin = 5;
const int starboardpin = 6;
const int valvepin = 4;
const int redled = 8;
const int greenled = 9;
const int button = 7;

const int countdown = 10*1000; //ms
const int raising_time = 3*1000; //ms

long flight_time;
long liftoff_time;


int ax, ay, az;
int gx, gy, gz; //Raw values

int pitch, yaw; // Orientation
int prevpitch, prevyaw; //Complementary filter

long prevtime = 0; //Complementary filter
float dt = 0; //Complementary filter

// Setup ------------------------------

void setup() {
  // Initialize
  Serial.begin(57600);
  Wire.begin();

  IMU.initialize();
  //IMU.setFullScaleAccelRange(MPU6050_ACCEL_FS_8) //Range

  port.attach(portpin);
  starboard.attach(starboardpin);

  pinMode(redled, OUTPUT);
  pinMode(greenled, OUTPUT);
  pinMode(portpin, OUTPUT);
  pinMode(starboardpin, OUTPUT);
  pinMode(valvepin, OUTPUT);
  pinMode(button, INPUT);

  // IMU Check

  if (IMU.testConnection()) {
    Serial.println("IMU initialized. Status: OK");
    digitalWrite(greenled, HIGH);
  }
  else {
    Serial.println("Error. IMU connection failed");
    digitalWrite(redled, HIGH);
  }

  // Calibration
  //Calibration();

  // Servomotors
  port.write(0);
  starboard.write(0);

  digitalWrite(greenled, LOW);
  // Waiting for button...
  while (true) {
    if (digitalRead(button) == true) {
      break;
    }
  }

  // Countdown
  Countdown();

  // LIFTOFF
  digitalWrite(valvepin, HIGH);
  liftoff_time = millis();
}

// Loop ------------------------------

void loop() {
  flight_time = millis()-liftoff_time;

  //Orientation();

  if (flight_time > raising_time) {
    port.write(flap_angle);
    starboard.write(flap_angle);
  }
}


//Functions ------------------------------

// IMU calibration
void Calibration() {
  int counter = 0;
  // Filter (filtered measures: p)
  long f_ax, f_ay, f_az;
  int p_ax, p_ay, p_az;
  long f_gx, f_gy, f_gz;
  int p_gx, p_gy, p_gz;
  // Offsets
  int ax_o, ay_o, az_o;
  int gx_o, gy_o, gz_o;

  // Previal offsets
  ax_o = IMU.getXAccelOffset();
  ay_o = IMU.getYAccelOffset();
  az_o = IMU.getZAccelOffset();
  gx_o = IMU.getXGyroOffset();
  gy_o = IMU.getYGyroOffset();
  gz_o = IMU.getZGyroOffset();

  Serial.println("Offsets:");
  Serial.print(ax_o); Serial.print("\t");
  Serial.print(ay_o); Serial.print("\t");
  Serial.print(az_o); Serial.print("\t");
  Serial.print(gx_o); Serial.print("\t");
  Serial.print(gy_o); Serial.print("\t");
  Serial.print(gz_o); Serial.print("\t");
  Serial.println("Press any button to start calibration");

  // Waiting...
  while (true) {
    if (Serial.available()) {
      break;
    }
  }

  // Start calibration ----------
  Serial.println("Calibrating... Do not move the IMU...");
  Serial.println("-----   AX    AY    AZ    GX    GY    GZ");
  Serial.println("NOTE: Reset Arduino when values approach: 0 0 +16384 0 0 0");
  Serial.print("3");
  delay(1000);
  Serial.print(", 2");
  delay(1000);
  Serial.print(", 1...");
  delay(1000);

  while (true) {
    // Read acceleration and rotation
    IMU.getAcceleration(&ax, &ay, &az);
    IMU.getRotation(&gx, &gy, &gz);

    // Filter the measures
    f_ax = f_ax - (f_ax >> 5) + ax;
    p_ax = f_ax >> 5;

    f_ay = f_ay - (f_ay >> 5) + ay;
    p_ay = f_ay >> 5;

    f_az = f_az - (f_az >> 5) + az;
    p_az = f_az >> 5;

    f_gx = f_gx - (f_gx >> 3) + gx;
    p_gx = f_gx >> 3;

    f_gy = f_gy - (f_gy >> 3) + gy;
    p_gy = f_gy >> 3;

    f_gz = f_gz - (f_gz >> 3) + gz;
    p_gz = f_gz >> 3;

    // Correcting the offset every 100 measures
    if (counter == 100) {
      Serial.println("-----     AX    AY    AZ    GX    GY    GZ");
      Serial.print("Average: "); Serial.print("\t");
      Serial.print(p_ax); Serial.print("\t");
      Serial.print(p_ay); Serial.print("\t");
      Serial.print(p_az); Serial.print("\t");
      Serial.print(p_gx); Serial.print("\t");
      Serial.print(p_gy); Serial.print("\t");
      Serial.println(p_gz);

      // Accelerations
      // 0 in X axis
      if (p_ax > 0) {
        ax_o--;
      }
      else {
        ax_o++;
      }
      // -1g(16384) in Y axis
      if (p_ay - 16384 > 0) {
        ay_o--;
      }
      else {
        ay_o++;
      }
      // 0 in Z axis
      if (p_az > 0) {
        az_o--;
      }
      else {
        az_o++;
      }
      // Apply new offsets
      IMU.setXAccelOffset(ax_o);
      IMU.setYAccelOffset(ay_o);
      IMU.setZAccelOffset(az_o);

      // Angles
      // 0º/s in X axis
      if (p_gx > 0) {
        gx_o--;
      }
      else {
        gx_o++;
      }
      // 0º/s in Y axis
      if (p_gy > 0) {
        gy_o--;
      }
      else {
        gy_o++;
      }
      // 0º/s in Z axis
      if (p_gz > 0) {
        gz_o--;
      }
      else {
        gz_o++;
      }
      // Apply new offsets
      IMU.setXGyroOffset(gx_o);
      IMU.setYGyroOffset(gy_o);
      IMU.setZGyroOffset(gz_o);

      counter = 0;
    }
    counter++;
  }
}

// Countdown
void Countdown() {
  long time_count = millis();
  long time_cur = millis()-time_count;

  while (time_cur < countdown)
  {
    digitalWrite(greenled, HIGH);
    
    if (time_cur > (countdown * 0.5)) {
      digitalWrite(redled, HIGH);
    }
    time_cur = millis()-time_count;

  }
  // LIFTOFF
  digitalWrite(redled, LOW);
  digitalWrite(greenled, LOW);
}

// Orientation
void Orientation() {
  // Using complementary filter
  dt = (millis() - prevtime) / 1000.0;
  prevtime = millis();

  //Angles with accelerometer
  float accel_pitch = atan(ay / sqrt(pow(ax, 2) + pow(az, 2))) * (180.0 / 3.14);
  float accel_yaw = atan(-ax / sqrt(pow(ay, 2) + pow(az, 2))) * (180.0 / 3.14);

  //Angles with filter
  pitch = 0.98 * (prevpitch + (gx / 131) * dt) + 0.02 * accel_pitch;
  yaw = 0.98 * (prevyaw + (gy / 131) * dt) + 0.02 * accel_yaw;

  prevpitch = pitch;
  prevyaw = yaw;
}


//// Debug
//void Debug() {
//  IMU.getAcceleration(&ax, &ay, &az);
//  IMU.getRotation(&gx, &gy, &gz);
//
//  //Raw values
//  Serial.print("a[x y z] g[x y z]:\t");
//  Serial.print(ax); Serial.print("\t");
//  Serial.print(ay); Serial.print("\t");
//  Serial.print(az); Serial.print("\t");
//  Serial.print(gx); Serial.print("\t");
//  Serial.print(gy); Serial.print("\t");
//  Serial.println(gz);
//
//  delay(2000);
//
//  starboard.write(90);
//  port.write(90);
//}
