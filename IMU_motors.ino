#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

// Variables globales
float gyroZOffset = 0.0; // Offset del giroscopio
float angleZ = 0.0;      // Ángulo calculado en el eje Z
unsigned long lastTime;  // Última marca de tiempo
unsigned long lastTime2 = 0;

bool init_1 = true;
// Definiciones para los motores
#define MOTORLEFT_PWM     25
#define MOTORLEFT_DIR_A   32    
#define MOTORLEFT_DIR_B   33

#define MOTORRIGH_PWM    26
#define MOTORRIGH_DIR_A  14      
#define MOTORRIGH_DIR_B  27

const int PWMFreq = 6000; /* 1 KHz */
const int PWMResolution = 8;
const int pwmChannelLeft  = 0;
const int pwmChannelRight = 1;

// Declaración de funciones
void setMotorLeft(int value);
void setMotorRigh(int value);
void motor(int left, int righ);
void freno(boolean left, boolean righ, int value);
void leerVelocidadYActualizarMotor();
void inicializarMotores();
void ControlPI(float current_angle);

// Calibrar el giroscopio
void calibrateGyro() {
  float total = 0.0;
  int numSamples = 100;

  Serial.println("Calibrando giroscopio...");
  for (int i = 0; i < numSamples; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    total += g.gyro.z * 57.2958; // Convertir rad/s a grados/s
    delay(10); // Esperar entre lecturas
  }
  gyroZOffset = total / numSamples; // Offset promedio
  Serial.println("Calibración completada.");
  Serial.print("Offset Z: ");
  Serial.println(gyroZOffset);
}

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // Esperar conexión serial (necesario en algunos dispositivos)

  if (!mpu.begin()) {
    Serial.println("Error al inicializar el MPU6050");
    while (1);
  }

  Serial.println("MPU6050 conectado exitosamente");

  // Calibrar el giroscopio
  calibrateGyro();

  // Inicializar la marca de tiempo
  lastTime = millis();

  // Inicializar motores
  inicializarMotores();
}

float Angle_Z() {
    // Leer datos del sensor
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Restar el offset calibrado del giroscopio
    float gyroZ = (g.gyro.z * 57.2958) - gyroZOffset; // °/s

    // Calcular el tiempo transcurrido
    unsigned long currentTime = millis();
    float dt = (currentTime - lastTime) / 1000.0; // Delta tiempo en segundos
    lastTime = currentTime;

    // Integrar la velocidad angular para obtener el ángulo
    angleZ += gyroZ * dt;

    // Normalizar valores
    if (angleZ > 360.0) angleZ -= 360.0;
    if (angleZ < 0.0) angleZ += 360.0; 


    return angleZ;
}


float setpoint = 0.0;  // Setpoint del ángulo (inicializado a 0)
bool controlActivo = false; // Control del ángulo desactivado al inicio
bool on_move = false;
float Kp = 6.5;  // Ganancia proporcional
float Ki = 0.001;
int pwm1,pwm2 = 0;
float integral = 0.0;


float current_angle_z = 0.0;

void leerVelocidadYActualizarMotor() {
    current_angle_z = Angle_Z();

    if (Serial.available() && !on_move) {
        String datos = Serial.readStringUntil('\n'); // Leer hasta salto de línea
        datos.trim(); // Eliminar espacios en blanco al inicio y final

        // Separar los valores por comas
        int comaIndex1 = datos.indexOf(',');
        int comaIndex2 = datos.lastIndexOf(',');
        if (comaIndex1 != -1 && comaIndex2 != -1) {
            String anguloStr = datos.substring(0, comaIndex1);
            String pwm1Str = datos.substring(comaIndex1 + 1, comaIndex2);
            String pwm2Str = datos.substring(comaIndex2 + 1);

            float anguloRecibido = anguloStr.toFloat();
            pwm1 = pwm1Str.toInt();
            pwm2 = pwm2Str.toInt();

            if (anguloRecibido != -1) {
                // Activar el control solo si se recibe un setpoint válido
                setpoint = anguloRecibido;
                controlActivo = true;
                on_move = true;
            }

            else if (anguloRecibido == -1 ){ controlActivo = false;}


        }
    }

            if (controlActivo && on_move) {
                // Si el control está activo, aplica el control proporcional
                ControlPI(current_angle_z);
            }
            else{
              motor(pwm1,pwm2);

            }
}


void ControlPI(float current_angle) {
    
    //float anguloActual = Angle_Z();

    // Calcular el error
    float error = setpoint - current_angle;
    unsigned long currentTime = millis();  // Tiempo actual
    float dt = (currentTime - lastTime2) / 1000.0; // Delta tiempo en segundos
    lastTime2 = currentTime;  // Actualizar lastTime para la siguiente iteración

    // Calcular la parte proporcional
    float proporcional = Kp * error;

    // Calcular la parte integral (acumulando el error multiplicado por el delta de tiempo)
    integral += error * dt;  // Multiplicar por el delta de tiempo para evitar acumulación excesiva
    float integralTerm = Ki * integral;

    // Sumar el término proporcional y el integral
    int pwm = proporcional + integralTerm;  
    // Limitar el valor de PWM


    pwm = constrain(pwm, -90, 90);

    // Control de los motores
    motor(-pwm, pwm);
    //Serial.print("Ang_z: ");
    //Serial.print(current_angle);
    //Serial.print("  Sp: ");
    //Serial.print(setpoint);
    //Serial.print("  Err: ");
    //Serial.print(error);
    //Serial.print("  Pwm ");
    //Serial.print(-pwm);
    //Serial.print(" , ");
    //Serial.println(pwm);

    if (abs(error) <= 6.0) {
        freno(true, true, 150); 
        delay(50);       
        motor(0, 0);     

        // Detener el control
        on_move = false;
        controlActivo = false;
        init_1 = false;
        //Serial.println("Setpoint alcanzado");
    }
}
void loop() {
    while (init_1){
      current_angle_z = Angle_Z();
      setpoint = 180;
      ControlPI(current_angle_z);
    }
    // Leer velocidades desde UART y actualizar motores
    leerVelocidadYActualizarMotor();

}







// Inicialización de motores
void inicializarMotores() {
  pinMode(MOTORRIGH_PWM   , OUTPUT);
  pinMode(MOTORRIGH_DIR_A , OUTPUT);
  pinMode(MOTORRIGH_DIR_B , OUTPUT);
  
  pinMode(MOTORLEFT_PWM   , OUTPUT);
  pinMode(MOTORLEFT_DIR_A , OUTPUT);
  pinMode(MOTORLEFT_DIR_B , OUTPUT);

  // Configuración PWM para los motores
  ledcSetup(pwmChannelLeft, PWMFreq, PWMResolution);
  ledcSetup(pwmChannelRight, PWMFreq, PWMResolution);  

  ledcAttachPin(MOTORRIGH_PWM , pwmChannelRight);
  ledcAttachPin(MOTORLEFT_PWM , pwmChannelLeft);

  motor(0, 0); // Apagar motores inicialmente
}

// Control individual de motores
void setMotorLeft(int value) {
  if (value >= 0) {
    digitalWrite(MOTORLEFT_DIR_A, LOW);
    digitalWrite(MOTORLEFT_DIR_B, HIGH);
  } else {
    digitalWrite(MOTORLEFT_DIR_A, HIGH);  
    digitalWrite(MOTORLEFT_DIR_B, LOW);
    value *= -1;
  }
  if (value > 10 && value <=255) {
  ledcWrite(pwmChannelLeft, abs(value+15));}
  if (value < -10) ledcWrite(pwmChannelLeft, abs(value-5));
  else ledcWrite(pwmChannelLeft, abs(value));
  
}

void setMotorRigh(int value) {
  if (value >= 0) {
    digitalWrite(MOTORRIGH_DIR_A, HIGH);
    digitalWrite(MOTORRIGH_DIR_B, LOW);
  } else {
    digitalWrite(MOTORRIGH_DIR_A, LOW);
    digitalWrite(MOTORRIGH_DIR_B, HIGH);
    value *= -1;
  }
  ledcWrite(pwmChannelRight, abs(value));
}

void motor(int left, int righ) {
  setMotorLeft(left);
  setMotorRigh(righ);
}

void freno(boolean left, boolean righ, int value) {
  if (left) {
    digitalWrite(MOTORLEFT_DIR_A, HIGH);
    digitalWrite(MOTORLEFT_DIR_B, HIGH);
    ledcWrite(pwmChannelLeft, abs(value));
  }

  if (righ) {
    digitalWrite(MOTORRIGH_DIR_A, HIGH);
    digitalWrite(MOTORRIGH_DIR_B, HIGH);
    ledcWrite(pwmChannelRight, abs(value));
  }
}
