#include "PID_v1.h"

#define TMP_MID A5
const int SSR_TOP = 2;
const int SSR_BOT = 3;

// T in Degree Celsius, time in seconds
const float temp_preheat = 150.f;
const int duration_preheat = 60;

const float temp_soak = 200.f;
const int duration_soak = 60;

const float temp_reflow = 220.f;
const int duration_reflow = 10;

const int frame_size = 2000;
unsigned long frame_offset = 0;
unsigned long t_start = 0;
unsigned long t_relative = 0;
unsigned long duration;

// Sensor calibration
float u_mid = 0.f;
float k = 0.065001534f;
float d = 0.313680985f;

const int readtime = 2000;
unsigned int lastread = 0;

// PID parameters
const int pid_sample_time = 1000;
const double Kp = 10.0;
const double Ki = 0.1;
const double Kd = 10.0;
const unsigned int bias = 250;

double temperature, pid_output, target_temperature;
PID myPID(&temperature, &pid_output, &target_temperature, Kp, Ki, Kd, DIRECT);

enum states_t {
  state_idle,
  state_preheat,
  state_soak,
  state_reflow,
  state_cooling
} state;

double get_temp()
{
  u_mid = analogRead(TMP_MID) / 1024.f * 5000.f;
  return (u_mid * k + d);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  myPID.SetOutputLimits(0, frame_size);
  myPID.SetSampleTime(pid_sample_time);
  myPID.SetMode(AUTOMATIC); // Turn on PID control

  pinMode(SSR_TOP, OUTPUT);
  pinMode(SSR_BOT, OUTPUT);
  set_state_SSR(false);

  // Setup timers
  t_start = millis();
  frame_offset = millis();
  temperature = get_temp();

  if (isnan(temperature)) {
    Serial.println("Invalid reading, check thermocouple!");
  }
  else {
    Serial.print("Starting temperature: ");
    Serial.print(temperature);
    Serial.println(" °C");
  }

  state = state_preheat;
  target_temperature = temp_preheat;
}

void loop() {
  temperature = get_temp();
  // Debug pid_output
  if((millis() - lastread) > readtime){
        lastread = millis();
        Serial.print("Temperature: ");
        Serial.print(temperature);
        Serial.print("\t,Voltage: ");
        Serial.println(u_mid);
        Serial.print("pid_output: ");
        Serial.println(pid_output);
  }

  // How much time has passed? in ms
  duration = millis() - t_start;
  switch(state)
  {
    case state_idle:
    {
      // Idle, add hardware button to switch from idle to active etc.
    } break;
    case state_preheat:
    {
      static bool heating_done = false;
      if(temperature >= (temp_preheat - 3) && !heating_done)
      {
        // Heating done, hold on now
        t_relative = millis();
        heating_done = true;
        Serial.println("Preheat starting!");
      }
      if(heating_done)
      {
        if((millis() - t_relative) >= (duration_preheat * 1000))
        {
          state = state_soak;
          target_temperature = temp_soak;
          Serial.println("Preheat complete!");
        }
      }
    } break;
    case state_soak:
    {
      static bool heating_done = false;
      if(temperature >= (temp_soak - 3) && !heating_done)
      {
        // Heating done, hold on now
        t_relative = millis();
        heating_done = true;
        Serial.println("Soaking starting!");
      }
      if(heating_done)
      {
        if((millis() - t_relative) >= (duration_soak * 1000))
        {
          state = state_reflow;
          target_temperature = temp_soak;
          Serial.println("Soaking complete!");
        }
      }
    } break;
    case state_reflow:
    {
      static bool heating_done = false;
      if(temperature >= (temp_reflow - 3) && !heating_done)
      {
        // Heating done, hold on now
        t_relative = millis();
        heating_done = true;
        Serial.println("Reflow starting!");
      }
      if(heating_done)
      {
        if((millis() - t_relative) >= (duration_reflow * 1000))
        {
          state = state_cooling;
          target_temperature = 20; // 20 degree Celsius
          Serial.println("Reflow complete!");
          Serial.println("Cooling...");
        }
      }
    } break;
    case state_cooling:
    {
      static bool first_time = false;
      if(temperature <= 55 && !first_time)
      {
        Serial.println("Safe to touch!");
        first_time = true;
      }
    } break;
    default:
    {
    } break;
  }
  
  myPID.Compute();
  // Make sure frame_offset is not greater then frame_size
  long diff = millis() - frame_offset;
  if(diff >= frame_size)
  {
    frame_offset += frame_size;
    diff = millis() - frame_offset;
  }

  // PWM the SSR
  if(diff <= (pid_output + bias))
    set_state_SSR(true);
  else
    set_state_SSR(false);
}

void set_state_SSR(bool state) {
  if (state) {
    digitalWrite(SSR_TOP, HIGH);
    digitalWrite(SSR_BOT, HIGH);
  } else {
    digitalWrite(SSR_TOP, LOW);
    digitalWrite(SSR_BOT, LOW);
  }
}
