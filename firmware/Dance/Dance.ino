#include <Arduino.h>
#include <Adafruit_BNO08x.h>
#include <Wire.h>
#include <math.h>
#include <cstdlib>
// Pins
const int inflateValvePin_1 = 17;     // Inflate valve
const int deflateValvePin_1 = 25;     // Deflate valve
const int pressureSensorPin_1 = 12;   // Pressure sensor feedback

const int inflateValvePin_2 = 14;     // Inflate valve
const int deflateValvePin_2 = 16;     // Deflate valve
const int pressureSensorPin_2 =   13  ;   // Pressure sensor feedback

const int inflateValvePin_3 = 27;     // Inflate valve
const int deflateValvePin_3 = 26;     // Deflate valve
const int pressureSensorPin_3 = 4;   // Pressure sensor feedback

const int l1Pin = 36;
const int l2Pin = 39;
const int l3Pin = 34;
const int l4Pin = 35;
const int l5Pin = 32;
const int l6Pin = 33;

int t0 = micros();
int tm0 = millis();


// PID constants
float starting_Kp = 0;
float Kp_step = 400;
float final_Kp = 12000;
//float Kp = starting_Kp;
float starting_c = 4100;
float c_step = 400;
float final_c = starting_c + 2900;
//float c = starting_c;

float Kp = 5600;
float c = 2900;
//float Ki = 0;
//float Kd = 0;

// PID variables
float setpoint_1 = 0;   // Desired pressure 
float setpoint_2 = 0;   // Desired pressure 
float setpoint_3 = 0;   // Desired pressure 


float filteredPressure_1 = -100;
float filteredPressure_2 = -100;
float filteredPressure_3 = -100;
float pressure_offset_1 = 0;
float pressure_offset_2 = 0;
float pressure_offset_3 = 0;

float x = 0.8;
float max_p = 13.0;
float min_p = 1.0;

double path[][4] = {
{0, 0, 0, 0},
{3000, 0, 0, 0},
{4100, 0.8, 0.8, 0.8},

{6500, 0.8, 0.8, 0.8},
{8500, 0.8, 0.8, 0.3},
{9200, 0.6, 0.2, 0.2},
{9700, 0.6, 0.6, 0.6},
{10700, 0, 0, 0},

{13200, 0, 0, 0},
//{14200, 0.2, 0.4, 0.5},
//{14900, 0.8, 0.1, 0.1},
{15300, 0.2, 0.8, 0.2},

//{16100, 0.3, 0.9, 0.9},
//{17000, 0.7, 0.4, 0.4},
//{17800, 0, 0.6, 0},
//{18500, 0.1, 0.4, 0.8},
{19200, 0.1, 0.4, 0.8},

{20100, 0.1, 0.1, 0.1},
{22300, 0.4, 0.55, 0.8},
//{23300, 0, 0.3, 0},
{23700, 0.7, 0.2, 0.2},
{24400, 0.8, 0, 0},
//{25100, 0.5, 0.5, 0.5},
//{25600, 0.5, 0.5, 0.5},
{26100, 0.0, 0.8, 0},
//{27200, 0, 0, 0.3},
{27800, 0, 0, 0.8},
//{28300, 0.3, 0.3, 0.0},
//{28900, 0.2, 0.2, 0.4},
{29800, 0.8, 0.8, 0.8},


{32000, 0.8, 0.8, 0.8},
{32100, 1, 1, 1}, //ghost

{34100, 1, 1, 1},
{34200, 0, 0, 0},//ghost


{35100, 0.8, 0, 0},
{35500, 0, 0.65, 0.65},
{36100, 0.8, 0, 0},
{36700, 0, 0.65, 0.65},
{37200, 0.8, 0, 0},
{37900, 0, 0.8, 0.8},//chord
//{38700, 0.5, 0.5, 0.5},
//{39500, 0.5, 0.5, 0.5},
{39900, 0.2, 0, 0.2},//chord
//{40500, 0.5, 0.5, 0.5},
{41100, 0.5, 0.5, 0.5},

//{45100, 1, 1, 1},
{46300, 1, 1, 1},
{47100, 0, 0, 0},
{47700, 0, 0, 0},

{48700, 0, 0, 0.5},
{50900, 0, 0.5, 0.5},
{52900, 0.5, 0.5, 0.5},

//{53900, 0.5, 0.5, 0.5},
{54600, 0.8, 0.3, 0.5},
//{55900, 0.5, 0.5, 0.5},
{58300, 0, 0, 0.2},
{59100, 0, 0.4, 0},
{59700, 0.6, 0, 0},
{60900, 1.0, 1.0, 1.0},
{63100, 0.5, 0.5, 0.5},
{63200, 0, 0, 0},


{1000000000, 0, 0, 0}
} ;

// IMU DEFs --------------------------------------------

struct euler_t {
  float yaw;
  float pitch;
  float roll;
} ypr;

Adafruit_BNO08x  bno08x;
sh2_SensorValue_t sensorValue;
int new_imu_flag = 0;

//#define FAST_MODE
#ifdef FAST_MODE
  // Top frequency is reported to be 1000Hz (but freq is somewhat variable)
  sh2_SensorId_t reportType = SH2_GYRO_INTEGRATED_RV;
  long reportIntervalUs = 2000;
#else
  // Top frequency is about 250Hz but this report is more accurate
  sh2_SensorId_t reportType = SH2_ARVR_STABILIZED_RV;
  long reportIntervalUs = 10000; //5000
#endif


// Control Functions -----------------------------------

float adc_to_voltage(float adc_count) { //Converts ADC Count to voltage (v)
  if (adc_count < 2690) {
    return 0.00084*adc_count + 0.1427;
  }
  return -0.0000001821*pow(adc_count, 2) + 0.001766*(adc_count) - 1.031;
}

float voltage_to_pressure(double voltage) { //Converts voltage (V) to absolute pressure (kPa)
  return (voltage - 0.00842*5)/(0.002421*5);
}


float readFilteredPressure(const int pressurePin, float filteredPressure, float pressure_offset) {
  int sensorValue = analogRead(pressurePin);
  float pressure = voltage_to_pressure(adc_to_voltage(sensorValue)) - pressure_offset;
  if (filteredPressure == -100) {
    filteredPressure = pressure; //overwrite filter if first value
  } else {
    filteredPressure = 0.15*pressure + 0.85*filteredPressure;
  }
  return filteredPressure;
}

int performPIDControl3(float setpoint, float currentPressure, float prop_gain, float c_offset, const int inflateValvePin, const int deflateValvePin) {
  float error = setpoint - currentPressure;
  float output = constrain(error * prop_gain, -100000, 100000); //100ms

  if (output < 0) {
    digitalWrite(inflateValvePin, LOW);
    digitalWrite(deflateValvePin, HIGH);

  } else if (output > 0) { 
    digitalWrite(inflateValvePin, HIGH);
    digitalWrite(deflateValvePin, LOW);
  }
  return abs(output) + c_offset;
}

// IMU FUNCTIONS
void setReports(sh2_SensorId_t reportType, long report_interval) {
  Serial.println("Setting desired reports");
  if (! bno08x.enableReport(reportType, report_interval)) {
    Serial.println("Could not enable stabilized remote vector");
  }
}

void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees = false) {

    float sqr = sq(qr);
    float sqi = sq(qi);
    float sqj = sq(qj);
    float sqk = sq(qk);

    ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
    ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
    ypr->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

    if (degrees) {
      ypr->yaw *= RAD_TO_DEG;
      ypr->pitch *= RAD_TO_DEG;
      ypr->roll *= RAD_TO_DEG;
    }
}

void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr, bool degrees = false) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

void quaternionToEulerGI(sh2_GyroIntegratedRV_t* rotational_vector, euler_t* ypr, bool degrees = false) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

void setup() {
  Serial.begin(115200);
  // Set valve pins as outputs
  pinMode(inflateValvePin_1, OUTPUT);
  pinMode(deflateValvePin_1, OUTPUT);
  pinMode(inflateValvePin_2, OUTPUT);
  pinMode(deflateValvePin_2, OUTPUT);
  pinMode(inflateValvePin_3, OUTPUT);
  pinMode(deflateValvePin_3, OUTPUT);

  if (!bno08x.begin_I2C()) {
    Serial.println("Failed to find BNO08x chip");
    while (1) { delay(10); }
  }
  Serial.println("BNO08x Found!");
  setReports(reportType, reportIntervalUs);


  digitalWrite(deflateValvePin_1, HIGH);
  digitalWrite(deflateValvePin_2, HIGH);
  digitalWrite(deflateValvePin_3, HIGH);
  delay(1000);
  for (int i = 0; i<100; i++) {
    pressure_offset_1 += readFilteredPressure(pressureSensorPin_1, -100, 0)/100.0;
    pressure_offset_2 += readFilteredPressure(pressureSensorPin_2, -100, 0)/100.0;
    pressure_offset_3 += readFilteredPressure(pressureSensorPin_3, -100, 0)/100.0;
    delay(10);
  }
  digitalWrite(deflateValvePin_1, LOW);
  digitalWrite(deflateValvePin_2, LOW);
  digitalWrite(deflateValvePin_3, LOW);

  delay(1000);
  Serial.println("PLAY SONG");
  t0 = micros();
  tm0 = millis();

  /*
  int i = 0;
  while (path[i][0] < 100000000) {
    path[i][1] = random(0,10000)/10000.0;
    path[i][2] = random(0,10000)/10000.0;
    path[i][3] = random(0,10000)/10000.0;
    i++;
  }
  Serial.print(path[0][0]);
  Serial.print(", ");
  Serial.print(path[0][1]);
  Serial.print(", ");
  Serial.print(path[0][2]);
  Serial.print(", ");
  Serial.print(path[0][3]);
  Serial.println(", ");
  */
}

void loop() {

  int loop_start = micros();
  int t = micros() - t0;
  int tm = millis() - tm0;
  int new_imu_flag = 0;
  int next_exp_flag = 0;

  if (bno08x.wasReset()) {
    Serial.print("sensor was reset ");
    setReports(reportType, reportIntervalUs);
  }
  
  if (bno08x.getSensorEvent(&sensorValue)) {
    // in this demo only one report type will be received depending on FAST_MODE define (above)
    switch (sensorValue.sensorId) {
      case SH2_ARVR_STABILIZED_RV:
        quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true);
      case SH2_GYRO_INTEGRATED_RV:
        // faster (more noise?)
        quaternionToEulerGI(&sensorValue.un.gyroIntegratedRV, &ypr, true);
        break;
    }
    static long last = 0;
    long now = micros();
    last = now;

    new_imu_flag = 1;
  }

  setpoint_1 = min_p; //GENERATES TEST SETPOINTS
  setpoint_2 = min_p;
  setpoint_3 = min_p;
  int i_after = 0;
  while (path[i_after][0] < tm) {
    i_after ++;
  }
  float t_before = path[i_after-1][0];
  float t_after = path[i_after][0];
  float x = (tm - t_before) / (t_after-t_before); 

  setpoint_1 = path[i_after-1][1] + (path[i_after][1]-path[i_after-1][1])*x;
  setpoint_2 = path[i_after-1][2] + (path[i_after][2]-path[i_after-1][2])*x;
  setpoint_3 = path[i_after-1][3] + (path[i_after][3]-path[i_after-1][3])*x;

  setpoint_1 = min_p + setpoint_1 * (max_p - min_p);
  setpoint_2 = min_p + setpoint_2 * (max_p - min_p);
  setpoint_3 = min_p + setpoint_3 * (max_p - min_p);

  Serial.print(t);
  Serial.print(", ");
  Serial.print(i_after-1);
  Serial.print(", ");
  Serial.print(x);
  Serial.print(", ");
  Serial.print(Kp);
  Serial.print(", ");
  Serial.print(c);
  Serial.print(", ");

  Serial.print(setpoint_1);
  Serial.print(", ");
  Serial.print(setpoint_2);
  Serial.print(", ");
  Serial.print(setpoint_3);
  Serial.print(", ");


  // Read and filter the pressure from the sensor
  filteredPressure_1 = readFilteredPressure(pressureSensorPin_1, filteredPressure_1, pressure_offset_1);
  filteredPressure_2 = readFilteredPressure(pressureSensorPin_2, filteredPressure_2, pressure_offset_2);
  filteredPressure_3 = readFilteredPressure(pressureSensorPin_3, filteredPressure_3, pressure_offset_3);

  Serial.print(filteredPressure_1);
  Serial.print(", ");
  Serial.print(filteredPressure_2);
  Serial.print(", ");
  Serial.print(filteredPressure_3);
  Serial.print(", ");

  Serial.print(""); //error print placeholder
  Serial.print(", ");
  Serial.print("");
  Serial.print(", ");
  Serial.print("");
  Serial.print(", ");

  Serial.print(new_imu_flag);
  Serial.print(", ");
  Serial.print(sensorValue.status);  // This is accuracy in the range of 0 to 3
  Serial.print(", ");
  Serial.print(ypr.yaw);
  Serial.print(", ");
  Serial.print(ypr.pitch);
  Serial.print(", ");
  Serial.print(ypr.roll);
  Serial.print(", ");
  Serial.print(analogRead(l1Pin));
  Serial.print(", ");
  Serial.print(analogRead(l2Pin));
  Serial.print(", ");
  Serial.print(analogRead(l3Pin));
  Serial.print(", ");
  Serial.print(analogRead(l4Pin));
  Serial.print(", ");
  Serial.print(analogRead(l5Pin));
  Serial.print(", ");
  Serial.print(analogRead(l6Pin));
  Serial.print(", ");
  Serial.print(micros() - loop_start);
  Serial.println(", ");
  
  // Perform PID control based on the filtered pressure
  int error_1 = performPIDControl3(setpoint_1, filteredPressure_1, Kp, c, inflateValvePin_1, deflateValvePin_1);
  int error_2 = performPIDControl3(setpoint_2, filteredPressure_2, Kp, c, inflateValvePin_2, deflateValvePin_2);
  int error_3 = performPIDControl3(setpoint_3, filteredPressure_3, Kp, c, inflateValvePin_3, deflateValvePin_3);
  int valve_on_time = micros();

  // DO NOT ADD CODE HERE

  while (micros() - valve_on_time <  20 * 1000){ //50hz //start valve timer instead of loop_start
    int v_time = micros() - valve_on_time;
    if (v_time > error_1) {
          digitalWrite(inflateValvePin_1, LOW);
          digitalWrite(deflateValvePin_1, LOW);
    } if (v_time > error_2) {
          digitalWrite(inflateValvePin_2, LOW);
          digitalWrite(deflateValvePin_2, LOW);      
    } if (v_time > error_3) {
          digitalWrite(inflateValvePin_3, LOW);
          digitalWrite(deflateValvePin_3, LOW);
    }
  }
  if (next_exp_flag) {
    int dwell_timer = micros();
    digitalWrite(inflateValvePin_1, LOW);
    digitalWrite(inflateValvePin_2, LOW);
    digitalWrite(inflateValvePin_3, LOW);
    digitalWrite(deflateValvePin_1, HIGH);
    digitalWrite(deflateValvePin_2, HIGH);
    digitalWrite(deflateValvePin_3, HIGH);
    delay(4000);
    pressure_offset_1 = 0;
    pressure_offset_2 = 0;
    pressure_offset_3 = 0;
    for (int i = 0; i<100; i++) {
      pressure_offset_1 += readFilteredPressure(pressureSensorPin_1, -100, 0)/100.0;
      pressure_offset_2 += readFilteredPressure(pressureSensorPin_2, -100, 0)/100.0;
      pressure_offset_3 += readFilteredPressure(pressureSensorPin_3, -100, 0)/100.0;
      delay(10);
    } 
    delay(100);
    digitalWrite(inflateValvePin_1, LOW);
    digitalWrite(inflateValvePin_2, LOW);
    digitalWrite(inflateValvePin_3, LOW);
    digitalWrite(deflateValvePin_1, LOW);
    digitalWrite(deflateValvePin_2, LOW);
    digitalWrite(deflateValvePin_3, LOW);
    while(micros() - dwell_timer < 16000000) { //16s
      ;
    }
    t0 = micros();
  }

}
