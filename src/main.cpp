#include <Arduino.h>
#include <Thermistor.h>
#include <NTC_Thermistor.h>

#define MIN_RPM 15000
#define MAX_RPM 42000

#define START_TEMP 20
#define END_TEMP 45

#define UPDATE_INTERVAL 1000 // calc rpm every second

#define TEMP_SAMPLES  32
#define TEMP_INTERVAL 100

#define FAN_1_RPM_PIN 2
#define FAN_2_RPM_PIN 3

#define FAN_1_PWM_PIN 5
#define FAN_2_PWM_PIN 6

#define THERMISTOR_PIN         A0
#define REFERENCE_RESISTANCE   90000
#define NOMINAL_RESISTANCE     100000
#define NOMINAL_TEMPERATURE    25
#define B_VALUE                3950
#define ANALOG_RESOLUTION      1023

Thermistor* thermistor;

uint32_t fan_rpm[2] = {0, 0};
uint64_t fan_pulses[2] = {0, 0};
uint32_t target_rpm = MIN_RPM;

float temperature = 20.f;

uint64_t last_rpm_calc = 0;

void fan_1_pulse();
void fan_2_pulse();
void calc_rpm(uint64_t ms);
void update_temp();
void update_target_rpm();
void set_speed();

void setup() {
  Serial.begin(9600);

  pinMode(FAN_1_RPM_PIN, INPUT_PULLUP);
  pinMode(FAN_2_RPM_PIN, INPUT_PULLUP);

  pinMode(THERMISTOR_PIN, INPUT);

  thermistor = new NTC_Thermistor(
    THERMISTOR_PIN,
    REFERENCE_RESISTANCE,
    NOMINAL_RESISTANCE,
    NOMINAL_TEMPERATURE,
    B_VALUE,
    ANALOG_RESOLUTION
  );

  attachInterrupt(digitalPinToInterrupt(FAN_1_RPM_PIN), fan_1_pulse, FALLING);
  attachInterrupt(digitalPinToInterrupt(FAN_2_RPM_PIN), fan_2_pulse, FALLING);

  delay(1000);
}

void loop() {
  if (millis() - last_rpm_calc > UPDATE_INTERVAL) {
    last_rpm_calc = millis();
    calc_rpm(UPDATE_INTERVAL);
    update_temp();
    update_target_rpm();
    set_speed();

    Serial.print("\n-----------\n");
  }
}

void fan_1_pulse() {
  fan_pulses[0] ++;
}

void fan_2_pulse() {
  fan_pulses[1] ++;
}

void calc_rpm(uint64_t ms) {
  fan_rpm[0] = fan_pulses[0] * (60000 / ms);
  fan_rpm[1] = fan_pulses[1] * (60000 / ms);

  fan_pulses[0] = 0;
  fan_pulses[1] = 0;

  Serial.print("\nRPM 1: "); Serial.print(fan_rpm[0]); Serial.print(" RPM 2: "); Serial.print(fan_rpm[1]);
}

void update_temp() {
  float temp_sum = 0.f;
  Serial.print("\nSampling ");


  for (uint8_t i = 0; i < TEMP_SAMPLES; i++) {
    temp_sum += thermistor->readCelsius();
    delay(TEMP_INTERVAL);
    Serial.print(".");
  }

  temperature = temp_sum / (TEMP_SAMPLES * 1.f);

  Serial.print("\nTEMP: "); Serial.print(temperature);
}

void update_target_rpm() {
  if (temperature <= START_TEMP) target_rpm = MIN_RPM;
  else if (temperature >= END_TEMP) target_rpm = MAX_RPM;
  else target_rpm = map((long) (temperature * 100), START_TEMP * 100, END_TEMP * 100, MIN_RPM, MAX_RPM);

  Serial.print("\nTARGET_RPM: "); Serial.print(target_rpm);
}

void set_speed() {
  uint8_t speed = map(target_rpm, MIN_RPM, MAX_RPM, 160, 255);
  
  analogWrite(FAN_1_PWM_PIN, speed);
  analogWrite(FAN_2_PWM_PIN, speed);

  Serial.print("\nSPEED: "); Serial.print(speed);  
}