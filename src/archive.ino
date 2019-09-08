#include <HCSR04.h>
#include <DHT.h>

#define A_SSR_Pin 4
#define r_SSR_Pin 3
#define c_SSR_Pin 2
#define A_Temp_Pin 7
#define r_Temp_Pin 6
#define c_Temp_Pin 5
#define US_Sensor_1_Trig 8
#define US_Sensor_1_Echo 9
#define US_Sensor_2_Trig 10
#define US_Sensor_2_Echo 11
#define US_Sensor_3_Trig 12
#define US_Sensor_3_Echo 13

DHT A_Temp_Sensor(7, DHT22);
DHT r_Temp_Sensor(6, DHT22);
DHT c_Temp_Sensor(5, DHT22);

UltraSonicDistanceSensor distanceSensor1(US_Sensor_1_Trig, US_Sensor_1_Echo);
UltraSonicDistanceSensor distanceSensor2(US_Sensor_2_Trig, US_Sensor_2_Echo);
UltraSonicDistanceSensor distanceSensor3(US_Sensor_3_Trig, US_Sensor_3_Echo);

float temps[3];
float old_temps[3];

float distances[3];
float old_distances[3];

bool heating_on[3] = { false, false, false };
long start_heating_time[3] = { 0, 0, 0 };
int heating_time = 1000; //heating time in ms

int activity_limits[3][3] = {{2, 5, 10}, {2, 5, 10}, {2, 5, 10}};
int temperature_limits[3][3] = {{65, 70, 80}, {65, 70, 80}, {65, 70, 80}};
int temperature_limit_index[3] = {-1, -1, -1};

int activity[3] = { 0, 0, 0 };
int last_interval_activity[3] = { 0, 0, 0};

int interval = 30000;

void setup() {
  pinMode(A_SSR_Pin, OUTPUT);
  pinMode(r_SSR_Pin, OUTPUT);
  pinMode(c_SSR_Pin, OUTPUT);

  set_heating_plate("off", 0);
  set_heating_plate("off", 1);
  set_heating_plate("off", 2);

  A_Temp_Sensor.begin();
  r_Temp_Sensor.begin();
  c_Temp_Sensor.begin();

  reset_activity_limits(0);
  reset_activity_limits(1);
  reset_activity_limits(2);

  Serial.begin(9600);
}

void set_heating_plate(char* state, int plate) {
  if (state == "on") {
    switch (plate) {
      case 0:
        digitalWrite(A_SSR_Pin, HIGH);
        start_heating_time[0] = millis();
        heating_on[0] = true;
        break;
      case 1:
        digitalWrite(r_SSR_Pin, HIGH);
        start_heating_time[1] = millis();
        heating_on[1] = true;
        break;
      case 2:
        digitalWrite(c_SSR_Pin, HIGH);
        start_heating_time[2] = millis();
        heating_on[2] = true;
        break;
    }
  } else {
    switch (plate) {
      case 0:
        digitalWrite(A_SSR_Pin, LOW);
        heating_on[0] = false;
        break;
      case 1:
        digitalWrite(r_SSR_Pin, LOW);
        heating_on[1] = false;
        break;
      case 2:
        digitalWrite(c_SSR_Pin, LOW);
        heating_on[2] = false;
        break;
    }
  }
}

void get_temperatures() {
  temps[0] = A_Temp_Sensor.readTemperature();
  temps[1] = r_Temp_Sensor.readTemperature();
  temps[2] = c_Temp_Sensor.readTemperature();
}

void save_old_temps() {
  if (temps[0] > 0) old_temps[0] = temps[0];
  if (temps[1] > 0) old_temps[1] = temps[1];
  if (temps[2] > 0) old_temps[2] = temps[2];
}

void get_distances() {
  distances[0] = distanceSensor1.measureDistanceCm();
  distances[1] = distanceSensor2.measureDistanceCm();
  distances[2] = distanceSensor3.measureDistanceCm();
}

long current_time;
long activity_clock = 0;
void save_old_activity() {
  current_time = millis();

  if (activity_clock + interval <= current_time) {
    last_interval_activity[0] = activity[0];
    last_interval_activity[1] = activity[1];
    last_interval_activity[2] = activity[2];
    activity[0] = 0;
    activity[1] = 0;
    activity[2] = 0;
    activity_clock = current_time;
  }
}

void count_activity() {
  if (old_distances[0] > distances[0] + 10) activity[0]++;
  if (old_distances[1] > distances[1] + 10) activity[1]++;
  if (old_distances[2] > distances[2] + 10) activity[2]++;
}

void save_old_distances() {
  old_distances[0] = distances[0];
  old_distances[1] = distances[1];
  old_distances[2] = distances[2];
}

void reset_activity_limits(int plate) {
  if (last_interval_activity[plate] < activity_limits[plate][0]) {
    activity_limits[plate][0] = 2;
    activity_limits[plate][1] = 5;
    activity_limits[plate][2] = 10;
  }
}

void set_activity_limits(int plate) {
  if (last_interval_activity[plate] > activity_limits[plate][2]) {
    activity_limits[plate][2] = last_interval_activity[plate];
    activity_limits[plate][1] = last_interval_activity[plate] - 5;
    activity_limits[plate][0] = last_interval_activity[plate] - 8;
  }
}

void set_temperature_limits(int plate) {
  if (last_interval_activity[plate] >= activity_limits[plate][2]) {
    temperature_limit_index[plate] = 2;
  }
  else if (last_interval_activity[plate] >= activity_limits[plate][1] && last_interval_activity[plate] < activity_limits[plate][2]) {
    temperature_limit_index[plate] = 1;
  }
  else if (last_interval_activity[plate] >= activity_limits[plate][0] && last_interval_activity[plate] < activity_limits[plate][1]) {
    temperature_limit_index[plate] = 0;
  }
  else {
    temperature_limit_index[plate] = -1;
  }
}

void log() {
  Serial.print("Time [s]:\t\t");
  Serial.print(millis() / 1000.0);
  Serial.print("\n\n");

  Serial.print("\t\t\tA\t\tr\t\tc\n\n");

  Serial.print("Distance [cm]:\t\t");
  Serial.print(distances[0]);
  Serial.print("\t\t");
  Serial.print(distances[1]);
  Serial.print("\t\t");
  Serial.print(distances[2]);
  Serial.print("\n");

  Serial.print("Temps [°C]:\t\t");
  Serial.print(temps[0]);
  Serial.print("\t\t");
  Serial.print(temps[1]);
  Serial.print("\t\t");
  Serial.print(temps[2]);
  Serial.print("\n");

  Serial.print("Activity:\t\t");
  Serial.print(activity[0]);
  Serial.print("\t\t");
  Serial.print(activity[1]);
  Serial.print("\t\t");
  Serial.print(activity[2]);
  Serial.print("\n");

  Serial.print("Last activity:\t\t");
  Serial.print(last_interval_activity[0]);
  Serial.print("\t\t");
  Serial.print(last_interval_activity[1]);
  Serial.print("\t\t");
  Serial.print(last_interval_activity[2]);
  Serial.print("\n");

  Serial.print("Activity limits:");
  Serial.print("\n\t\t\t");
  Serial.print(activity_limits[0][0]);
  Serial.print("\t\t");
  Serial.print(activity_limits[1][0]);
  Serial.print("\t\t");
  Serial.print(activity_limits[2][0]);
  Serial.print("\n\t\t\t");
  Serial.print(activity_limits[0][1]);
  Serial.print("\t\t");
  Serial.print(activity_limits[1][1]);
  Serial.print("\t\t");
  Serial.print(activity_limits[2][1]);
  Serial.print("\n\t\t\t");
  Serial.print(activity_limits[0][2]);
  Serial.print("\t\t");
  Serial.print(activity_limits[1][2]);
  Serial.print("\t\t");
  Serial.print(activity_limits[2][2]);
  Serial.print("\n");

  Serial.print("Temperature limits:");
  Serial.print("\n\t\t\t");
  Serial.print(temperature_limits[0][0]);
  Serial.print("\t\t");
  Serial.print(temperature_limits[1][0]);
  Serial.print("\t\t");
  Serial.print(temperature_limits[2][0]);
  Serial.print("\n\t\t\t");
  Serial.print(temperature_limits[0][1]);
  Serial.print("\t\t");
  Serial.print(temperature_limits[1][1]);
  Serial.print("\t\t");
  Serial.print(temperature_limits[2][1]);
  Serial.print("\n\t\t\t");
  Serial.print(temperature_limits[0][2]);
  Serial.print("\t\t");
  Serial.print(temperature_limits[1][2]);
  Serial.print("\t\t");
  Serial.print(temperature_limits[2][2]);
  Serial.print("\n\t\t\t");
  Serial.print(temperature_limit_index[0]);
  Serial.print("\t\t");
  Serial.print(temperature_limit_index[1]);
  Serial.print("\t\t");
  Serial.print(temperature_limit_index[2]);
  Serial.print("\n");
  
  Serial.print("\n\n");
}

void set_heating(int plate) {
  if (heating_on[plate]) {
    if (millis() - start_heating_time[plate] >= heating_time) {
      set_heating_plate("off", plate);
    }
  } else {    
    if (temperature_limit_index[plate] != -1) {
      if (temps[plate] < temperature_limits[plate][temperature_limit_index[plate]]) {
        set_heating_plate("on", plate);
      }
    }
  }
}

void loop() {
  get_distances();
  count_activity();
  
  get_temperatures();

  set_heating(0);
  set_heating(1);
  set_heating(2);
  
  set_activity_limits(0);
  set_activity_limits(1);
  set_activity_limits(2);

  reset_activity_limits(0);
  reset_activity_limits(1);
  reset_activity_limits(2);

  set_temperature_limits(0);
  set_temperature_limits(1);
  set_temperature_limits(2);
  
  save_old_activity();
  save_old_distances();
  save_old_temps();

  log();

  delay(125);
}
