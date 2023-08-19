//* Librerías
#include <Arduino.h>
#include <Wire.h>

#include "U8g2lib.h"
#include "ADS1X15.h"


//* Enum para identificar los pines usados en placa
enum DemoPins : uint8_t {
  I2C_SDA   = 21,
  I2C_SCL   = 22,
  ANALOG_1  = 33,
  MOTOR_A_1A  = 18,
  MOTOR_A_1B  = 19,
  ADC_ALERT   = 25
};

//* Variable / potenciómetro leído
enum Variable : uint8_t {
  PROPORTIONAL,
  INTEGRAL,
  DERIVATIVE,
  SETPOINT,
  ROTOR,
  MAX
};

//* Creación de objetos
U8G2_SH1106_128X64_NONAME_F_HW_I2C display (U8G2_R0, U8X8_PIN_NONE);
ADS1115 adc(0x48);

//* Variables
struct AnalogInputs {
  uint16_t raw;
  uint16_t filtered;
  float scaled;
};

AnalogInputs analogs_inputs[MAX];
uint8_t ads_channel = 0;

volatile bool ready = false;

//* Variables del controlador
bool enable_controller = false;

float period = 0.1;

float max_kp = 10.0;
float max_ki = 10.0;
float max_kd = 10.0;

float max_out = 255.0;
float min_out = -255.0;

struct VariablesPID {
  float Kp;                   // Valor de sintonización proporcional
  float Ki;                   // Valor de sintonización integral
  float Kd;                   // Valor de sintonización derivativa
  float N_coef;               // Valor coeficiente filtro derivativo

  float setpoint;             // Valor del setpoint
  float error;                // Valor actual del error

  float proportional;         // Valor actual de la parte proporcional
  float integral;             // Valor actual de la parte integral
  float derivative;           // Valor actual de la parte derviativa
  float output;               // Valor actual de la salida  

  float last_error;           // Error anterior
  float last_integral;        // Valor integral anterior
  float last_derivative;      // Valor derivativo anterior
} pid;

//* Prototipos de funciones
void displayView();
void readAnalogsInputs();
void readConsole();
float PIDController(float in_value);
void IRAM_ATTR adcReady();

// PIN setup()
void setup() {
  // Inicializa puertos de comunicacion
  Wire.begin(I2C_SDA, I2C_SCL);
  Serial.begin(115200);

  // Inicia y limpia pantalla I2C
  display.begin();
  display.clear();

  // Inicia y configura el chip ADS1115
  adc.begin();
  adc.setGain(1);
  adc.setDataRate(4);
  adc.setComparatorThresholdHigh(0x8000);
  adc.setComparatorThresholdLow(0x0000);
  adc.setComparatorQueConvert(0);
  adc.setMode(1);

  // Establece y "apaga" pines de control de motor como salida
  pinMode((uint8_t) DemoPins::MOTOR_A_1A, OUTPUT);
  pinMode((uint8_t) DemoPins::MOTOR_A_1B, OUTPUT);

  digitalWrite((uint8_t) DemoPins::MOTOR_A_1A, LOW);
  digitalWrite((uint8_t) DemoPins::MOTOR_A_1B, LOW);

  // Establece el pin ADC alert como entrada, con pullup interno
  // y lo configura como interrupción.
  pinMode((uint8_t) DemoPins::ADC_ALERT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt((uint8_t) DemoPins::ADC_ALERT), adcReady, RISING);

  // Inicializa los valores del controlador PID en 0
  pid = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  // La constante N para el filtro derivativo se ignora por lo que se setea a 1.0
  pid.N_coef = 1.0;
}

// PIN loop()
void loop() {
  static uint32_t last_time = 0;
  // Lee datos del monitor serial
  readConsole();
  // Maneja la pantalla
  displayView();
  // Lee las entradas análogas
  readAnalogsInputs();

  // Lectura y actuación cada 100 ms
  if(millis() - last_time >= 100) {
    last_time = millis();
    // Como es una maqueta de ejemplo, si Ki = 0, los valores
    // de la integral se reinician.
    if(pid.Ki == 0) {
      pid.integral = 0;
      pid.last_integral = 0;
    }
    // Lee y escala la señal del potenciómetro conectado al rotor del motor
    analogs_inputs[ROTOR].raw = analogRead(ANALOG_1);
    analogs_inputs[ROTOR].scaled = analogs_inputs[ROTOR].raw * 180.0 / 3380.0;
    // Ingresa el valor leído al controlador PID
    float motor_speed = PIDController(analogs_inputs[ROTOR].scaled);
    // Dependiendo del signo de la velocidad del motor, 
    // los pintes de control del motor se apagan o no
    if((int32_t) motor_speed >= 0) {
      analogWrite(MOTOR_A_1A, 0); 
      analogWrite(MOTOR_A_1B, motor_speed);
    } else { // Si no, gira a la izquierda y hace positiva la velocidad
      motor_speed *= -1;
      analogWrite(MOTOR_A_1A, motor_speed);
      analogWrite(MOTOR_A_1B, 0);
    }
  }
}

// PIN  PIDController()
float PIDController(float in_value) {
  pid.error = pid.setpoint - in_value; // Calcula el error
  pid.proportional = pid.Kp * pid.error;      // Caclula la parte proporcional

  // Calcula la parte integral si no está saturada la salida.
  pid.integral = pid.Ki * 0.5 * period * (pid.error + pid.last_error) + pid.last_integral;
  // Anti Windup
  if(pid.integral > max_out) pid.integral = max_out;
  else if(pid.integral < min_out) pid.integral = min_out;

  // Calcula la parte derivativa del controlador
  pid.derivative =  pid.N_coef * pid.Kd * (pid.error - pid.last_error) 
                    + (1.0 - pid.N_coef * period) * pid.last_derivative;

  // Suma todas las partes para obtener la salida
  pid.output = pid.proportional + pid.integral + pid.derivative;

  if(pid.output > max_out) pid.output = max_out; 
  else if(pid.output < min_out) pid.output = min_out;
  
  Serial.printf("s: %.2f - p: %.2f - i: %.2f - d: %.2f - o: %.2f\n" , pid.setpoint
                                                                    , pid.proportional
                                                                    , pid.integral
                                                                    , pid.derivative
                                                                    , pid.output);

  pid.last_integral   = pid.integral;
  pid.last_derivative = pid.derivative;
  pid.last_error      = pid.error;

  if(pid.error == 0) pid.output = 0;

  return pid.output;
}

// PIN readConsole()
void readConsole() {
  if(Serial.available()) {
    char c = Serial.read();

    switch(c) {
      case 'p': max_kp = Serial.readStringUntil(',').toFloat(); break;
      case 'i': max_ki = Serial.readStringUntil(',').toFloat(); break;
      case 'd': max_kd = Serial.readStringUntil(',').toFloat(); break;
    }
  }
}

// PIN displayView()
void displayView() {
  static uint32_t last_time = 0;
  char text_buffer[20];

  if(millis() - last_time >= 100) {
    last_time = millis();
    display.setFont(u8g2_font_6x12_mf);
    display.enableUTF8Print();
    display.setFontMode(0);
    display.clearBuffer();
    sprintf(text_buffer, "P: %.1f", analogs_inputs[0].scaled);
    display.drawStr(0, 12, text_buffer);
    sprintf(text_buffer, "I: %.1f", analogs_inputs[1].scaled);
    display.drawStr(0, 24, text_buffer);
    sprintf(text_buffer, "D: %.1f", analogs_inputs[2].scaled);
    display.drawStr(0, 36, text_buffer);
    display.drawStr(0, 48, "Setpoint");
    sprintf(text_buffer, "%.0f°", analogs_inputs[3].scaled);
    uint8_t x = (display.getStrWidth("Setpoint") - display.getStrWidth(text_buffer)) / 2;
    display.drawUTF8(x, 60, text_buffer);

    display.drawStr(64, 12, "Position");
    display.setFont(u8g2_font_10x20_mf);
    sprintf(text_buffer, "%.0f°", analogs_inputs[4].scaled);
    display.drawUTF8(64, 30, text_buffer);

    display.sendBuffer();
  }
}

// PIN readAnalogsInputs()
void readAnalogsInputs() {
  static uint32_t last_time = 0;
  static bool firts = true;

  if(millis() - last_time >= 100) {
    if(firts) {
      ads_channel = 0;
      adc.requestADC(ads_channel);
      firts = false;
    }

    if(ready) {
      int16_t temp = adc.getValue();
      if(temp < 0) temp = 0;
      analogs_inputs[ads_channel].raw = temp;

      switch(ads_channel) {
        case PROPORTIONAL:
          analogs_inputs[PROPORTIONAL].scaled = temp * max_kp / 26363.0; // P
          break;
        case INTEGRAL:
          analogs_inputs[INTEGRAL].scaled = temp * max_ki / 26363.0;    // I
          break;
        case DERIVATIVE:
          analogs_inputs[DERIVATIVE].scaled = temp * max_kd / 26363.0;  // D
          break;
        case SETPOINT:
          analogs_inputs[SETPOINT].scaled = temp * 180.0 / 26363.0; // Setpoint
          break;
      }

      ads_channel++;

      if(ads_channel > 3) {
        last_time = millis();
        ads_channel = 0;
        pid.Kp        = analogs_inputs[PROPORTIONAL].scaled;
        pid.Ki        = analogs_inputs[INTEGRAL].scaled;
        pid.Kd        = analogs_inputs[DERIVATIVE].scaled;
        pid.setpoint  = analogs_inputs[SETPOINT].scaled;
      }

      adc.requestADC(ads_channel);
      ready = false;
    }
  }
}

// PIB adcReady()
void IRAM_ATTR adcReady() { ready = true; }