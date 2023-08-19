#include <Arduino.h>
#include <Wire.h>

#include "ESP32_Servo.h"
#include "VL53L0X.h"

// * Objetos
Servo servo;    // Objeto para manejo de servomotor
VL53L0X laser;  // Objeto para manejo de sensor láser

// * Pines
const uint8_t PIN_SDA = 21;
const uint8_t PIN_SCL = 22;
const uint8_t PIN_SERVO = 27;

// * Valores del servomotor 
const float MAX_OUT = 90;
const float MIN_OUT = -90;
static const uint8_t SERVO_INITIAL_ANGLE = 90;

// * Calibración de distancia medición
// (Calibración calculada por dos puntos medidos)
static const float M = 0.8772;
static const float B = 0;

// * Flags control
bool enable_controller = false;

// * Datalogger
bool enable_datalogger = false;
uint32_t counter = 0;
uint8_t angle = SERVO_INITIAL_ANGLE;

// * DataloggerOsc
bool enable_datalogger_osc = false;
uint32_t counter_osc = 0;
uint8_t angle_osc = SERVO_INITIAL_ANGLE;
uint8_t angle_osc_step = 0;
uint8_t factor_time = 0;

// * Variables para controlador PID
float period = 0.1;

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

uint32_t last_time = 0;

// * Prototipo de funciones
float PIDController(float in_value);
void readConsole();
void datalogger();
void dataloggerOsc();

// PIN setup()
void setup() {
  // Inicializa puertos seriales
  Serial.begin(115200);
  Wire.begin(PIN_SDA, PIN_SCL);

  // Inicializa valores PID en 0
  pid = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        , 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  pid.Kp = 0.84198;
  pid.Ki = 0.47872;
  pid.Kd = 0.33821;
  pid.N_coef = 7.9883;
  pid.setpoint = 150.0;

  // Inicia servomotor
  // Configura el pin del servo y los tiempos de trabajo para este
  servo.attach(PIN_SERVO, 1000, 2000);
  // Mueve el servo a la posición inicial
  servo.write(SERVO_INITIAL_ANGLE);

  // Inicia sensor de distancia
  laser.setTimeout(250);      // Setea timeout
  bool status = laser.init(); // Intenta iniciar el sensor

  if(status) { 
    // Configura parámetros del sensor
    laser.setSignalRateLimit(0.1);
    laser.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
    laser.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
    laser.setMeasurementTimingBudget(100e3);
  } else {
    // Si no inicia el sensor, bloquea el código
    while(1); 
  }
}

void loop() {
  // Lee los comandos escritos en consola
  readConsole();

  // Activa datalogger
  datalogger();
  
  // Activa datalogger 2
  dataloggerOsc();

  // Si el controlador está habilitado
  if(enable_controller) {
    // Lee el dato de distancia
    uint16_t distance = laser.readRangeSingleMillimeters();
    // Ajusta la distancia según la calibración
    float calibrated = ((float) (distance) * M) + B;
    // Ingresa el valor medido calibrado al controlador PID
    float servo_pos = PIDController(calibrated);
    // Agrega offset para valores aceptados por el actuador
    servo_pos += 90;
    servo.write(servo_pos);
  }
}

void readConsole() {

  if (Serial.available()) {
    char c = Serial.read();

    switch(c) {
      case 'c': //? habilita / deshabilita PID
      {
        enable_controller = !enable_controller;
        if(!enable_controller) servo.write(SERVO_INITIAL_ANGLE);
      } break;

      case 's': //? Setea setpoint 
      {
        pid.setpoint = Serial.readStringUntil(',').toFloat();
      } break;

      case 'P': //? Cambia la posición del servomotor
      {
        if(enable_controller) {
          break;
        }

        uint8_t position = Serial.readStringUntil(',').toInt();
        servo.write(position);
      } break;

      case 'm': //? Mide y muestra la distancia
      {
        uint16_t distance = laser.readRangeSingleMillimeters();
        Serial.printf("Distancia: %u\n", distance);
      } break;

      case 'p': //? Cambia el factor proporcional (Kp)
      {
        pid.Kp = Serial.readStringUntil(',').toFloat();
      } break;

      case 'i': //? Cambia el factor integral (Ki)
      {
        pid.Ki = Serial.readStringUntil(',').toFloat();
      } break;

      case 'd': //? Cambia el factor derivativo (Kd)
      {
        pid.Kd = Serial.readStringUntil(',').toFloat();
      } break;

      case 'n': //? Cambia el coeficiente del filtro
      {
        pid.N_coef = Serial.readStringUntil(',').toFloat();
      } break;

      case 'D': //? Habilita / deshabilita datalogger
      {
        enable_datalogger = !enable_datalogger;

        angle = SERVO_INITIAL_ANGLE;

        if(!enable_controller) {
          servo.write(angle);
          counter = 0;
        }
      } break;

      case '1': 
      {
        enable_datalogger_osc = !enable_datalogger_osc;

        angle_osc = SERVO_INITIAL_ANGLE;

        if(!enable_datalogger_osc) {
          servo.write(angle_osc);
          counter_osc = 0;
        }
      } break;

      case '2':
      {
        factor_time = Serial.readStringUntil(',').toInt();
      } break;

      case '3':
      {
        angle_osc_step = Serial.readStringUntil(',').toInt();
      } break;
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
  if(pid.integral > MAX_OUT) pid.integral = MAX_OUT;
  else if(pid.integral < MIN_OUT) pid.integral = MIN_OUT;

  // Calcula la parte derivativa del controlador
  pid.derivative =  pid.N_coef * pid.Kd * (pid.error - pid.last_error) 
                    + (1.0 - pid.N_coef * period) * pid.last_derivative;

  // Suma todas las partes para obtener la salida
  pid.output = pid.proportional + pid.integral + pid.derivative;

  if(pid.output > MAX_OUT) pid.output = MAX_OUT; 
  else if(pid.output < MIN_OUT) pid.output = MIN_OUT;
  
  Serial.printf("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\r" , pid.proportional
                                                  , pid.integral 
                                                  , pid.derivative
                                                  , in_value
                                                  , pid.output
                                                  , pid.error);

  pid.last_integral   = pid.integral;
  pid.last_derivative = pid.derivative;
  pid.last_error      = pid.error;

  if(pid.error == 0) pid.output = 0;

  return pid.output;
}

void datalogger() {
  if(enable_datalogger) {
    // Lee el dato de distancia
    uint16_t distance = laser.readRangeSingleMillimeters();
    // Ajusta la distancia según la calibración
    float calibrated = ((float) (distance) * M) + B;
    // Aumenta el contador
    counter++;

    // Cuando el contador llegue a 10, significa que habrá transcurrido
    // 1 segundo, por lo que cambiará la posición del servomotor para
    // mover la bola.
    if(counter == 5) {
      angle = SERVO_INITIAL_ANGLE + 20;
      servo.write(angle);
    }

    Serial.printf("%.1f,%u,%.3f\r", counter * 0.1, angle, calibrated);
  }
}

void dataloggerOsc() {
  if(enable_datalogger_osc) {
    // Lee el dato de distancia
    uint16_t distance = laser.readRangeSingleMillimeters();
    // Ajusta la distancia según la calibración
    float calibrated = ((float) (distance) * M) + B;
    // Aumenta el contador
    counter_osc++;

    // Cuando el contador llegue a 10, significa que habrá transcurrido
    // 1 segundo, por lo que cambiará la posición del servomotor para
    // mover la bola.
    if(counter_osc % factor_time == 0) {
      if(angle_osc >= SERVO_INITIAL_ANGLE) {
        angle_osc = SERVO_INITIAL_ANGLE - angle_osc_step;
        // Serial.printf("1: %u\n", angle_osc);
      } else {
        angle_osc = SERVO_INITIAL_ANGLE + angle_osc_step;
        // Serial.printf("2: %u\n", angle_osc);
      }

      servo.write(angle_osc);
    }

    Serial.printf("%.1f,%u,%.3f\r", counter_osc * 0.1, angle_osc, calibrated);
  }
}


/* ---------------------------------------------------------------------------------------------- */