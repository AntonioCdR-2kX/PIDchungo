#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"


/* SETTINGS */
#define ANGLE_FILTER_G     0.999 // gyro portion of complementary filter [0...1]
#define ANGLE_LIMIT        18    // stop program / start rise up sequence [deg]
#define RISEUP_END_ANGLE   5     // stop rise up sequence and start balancing [deg]
#define ANGLE_FIXRATE      1.0   // variate target angle [deg/s]
#define ANGLE_FIXRATE_2    0.1   // reduce continuous rotation
#define KP                 0.2   // PID proportional factor
#define KI                 0.5   // PID integral factor
#define KD                 0.005 // PID derivative factor
#define MOTOR_R            13    // motor resistance [Ohm]
#define MOTOR_Ke           0.26  // motor back EMF constant [Vs/rad]
#define SUPPLY_VOLTAGE     8.1   // battery box voltage [V]
#define WHEEL_AV_INTERVAL  100   // wheel angular velocity calculation interval [ms]
#define V_MAX              255   // 


/* VARIABLES */ 

// IMU
const int mpuAddress = 0x68;  // Puede ser 0x68 o 0x69
MPU6050 mpu(mpuAddress);

int ax, ay, az;
//int gx, gy, gz;

// motor
const int pinPWMA = 5;
const int pinAIN2 = 6;
const int pinAIN1 = 7;
const int pinMotorA[3] = { pinPWMA, pinAIN2, pinAIN1 };
const int pinSTBY = 8;

//enum moveDirection {
//   forward,
//   backward
//}
int speed = 0;      //velocidad de giro

// encoder
const int canalA = 5;
const int canalB = 6;

int objetivo;
volatile long contador = 0;


int startTime = 0;
int prevLoopTime = 0;
int prevTachoTime = 0;
int prevPrintTime = 0;


int loopCount = 0;
float gyroAngle = 0;
float measuredAngle = 0;
float prevTachoCount = 0;
float wheelAV = 0;
float targetAngle = 0;
float error = 0;
float prevError = 0;
float integral = 0;
float derivative = 0;
float PIDoutput = 0;
float motorCtrl = 0;


bool risingUp;

/* INICIALIZO */
void setup()
{
   // IMU
   Serial.begin(9600);
   Wire.begin();
   mpu.initialize();
   Serial.println(mpu.testConnection() ? F("IMU iniciado correctamente") : F("Error al iniciar IMU"));
   
   // motor
   pinMode(pinAIN2, OUTPUT);
   pinMode(pinAIN1, OUTPUT);
   pinMode(pinPWMA, OUTPUT);

   // encoder
   pinMode(canalA, INPUT_PULLUP);
   pinMode(canalB, INPUT_PULLUP);
   attachInterrupt(digitalPinToInterrupt(canalA), Lectura_encoderA, RISING);
   attachInterrupt(digitalPinToInterrupt(canalB), Lectura_encoderB, RISING);

   startTime = millis();
   prevLoopTime = millis();
   prevTachoTime = millis();
   prevPrintTime = millis();
}



/* BUCLE */
void loop() 
{
   int timeDelta = (millis() - prevLoopTime) / 1e9;  // [sec]
   prevLoopTime = millis();
   int secondsSinceStart = (millis() - startTime) / 1e9;


   // Leer las aceleraciones 
   mpu.getAcceleration(&ax, &ay, &az);
   //Calcular los angulos de inclinacion
   float accAngle = atan(ax / sqrt(pow(ay, 2) + pow(az, 2)))*(180.0 / 3.14);

   // Read gyroscope
   //mpu.readGyroscopeMaster(&gx, &gy, &gz);  // [deg/s]
   //float gyroAngleDelta = gz * timeDelta;
   //if isnan(gyroAngle) gyroAngle = accAngle;
   //gyroAngle += gyroAngleDelta;  // [deg]
   
   //calculate arm angle (complementary filter)
   //if isnan(measuredAngle) measuredAngle = accAngle;
   //measuredAngle = (ANGLE_FILTER_G * (measuredAngle + gyroAngleDelta) + (1-ANGLE_FILTER_G) * accAngle);  // [deg]
   
   // Cálculo sin filtro
   measuredAngle = accAngle;

   // Safety check
    if (abs(measuredAngle) >= ANGLE_LIMIT) 
    {
        if (secondsSinceStart < 0.01) {
            Serial.println("START RISE UP SEQUENCE");
            risingUp = true;
        }
        else if(!risingUp) {
            Serial.print("PROGRAM STOPPED, angle is too large: ");
            Serial.println(measuredAngle);
            fullStop();
        }        
    }

    
    // Calculate wheel angular velocity
    if (((millis() - prevTachoTime) / 1e6 )>= WHEEL_AV_INTERVAL)
    {
        float tachoTimeDelta = (millis() - prevTachoTime) / 1e9;  // [sec]
        prevTachoTime = millis();
        
        float pulses = contador - prevTachoCount;
        prevTachoCount = contador;
        float cycles = pulses / 360;   // 360 pulses per rotation
        wheelAV = cycles / tachoTimeDelta * 2 * 3.14;  // [rad/s]
    }
        

    // Rise up sequence, usamos el doble impulso meteoro inverso
    if (risingUp) 
    {
        if (secondsSinceStart < 1.0) {
            // speed up reaction wheel to full speed
            if (measuredAngle < 0) move(-1);
            else if (measuredAngle > 0) move(1);
        }
        else if (secondsSinceStart < 1.5){
            // change direction using full power
            if (measuredAngle > 0) move(V_MAX);
            else if (measuredAngle < 0){
                move(-V_MAX);
                // wait until close to top position, then start balancing
                if (abs(measuredAngle) < RISEUP_END_ANGLE) {
                    Serial.println("END RISE UP SEQUENCE");
                    risingUp = false;
                }   
            } 
        }
        else {
            Serial.println("RISE UP TIMEOUT");
        }     
    }
    else if (!risingUp) 
    {
        // variate target angle
        if (measuredAngle < targetAngle) targetAngle += ANGLE_FIXRATE * timeDelta;
        else targetAngle -= ANGLE_FIXRATE * timeDelta;
        
        // reduce continuous rotation
        targetAngle -= ANGLE_FIXRATE_2 * wheelAV * timeDelta;
        
        // PID controller
        error = targetAngle - measuredAngle;
        integral += error * timeDelta;
        derivative = (error - prevError) / timeDelta;
        prevError = error;
        PIDoutput = KP * error + KI * integral + KD * derivative;
        
        // compensate for motor back EMF voltage
        float current = -PIDoutput;
        float voltage = MOTOR_R * current + MOTOR_Ke * wheelAV;
        
        //convert voltage to pwm duty cycle
        motorCtrl = voltage / SUPPLY_VOLTAGE;
        
        // drive motor
        motorCtrl = min(max(motorCtrl, -1), 1);
        move(motorCtrl);
    }

    // debug print
    if (((millis() - prevPrintTime) / 1e9) >= 1.0)
    {
        float secondsSinceLastPrint = (millis() - prevPrintTime) / 1e9;
        prevPrintTime = millis();
        float loopInterval = secondsSinceLastPrint / loopCount * 1000;
        loopCount = 0;
        Serial.print("measuredAngle: ");
        Serial.print(measuredAngle);
        Serial.print(", motorCtrl: ");
        Serial.print(motorCtrl);
        Serial.print(", loopInterval: ");
        Serial.println(loopInterval);
    }

    loopCount += 1;
   
   // Mostrar resultados IMU
   Serial.print("Inclinacion en X: ");
   Serial.println(accAngle);

   // Mostrar resultado encoder
   Serial.print("Encoder: ");
   Serial.println(contador);
   
   
   
   delay(10);
}



/* FUNCIONES CONTADOR DEL ENCODER */
void Lectura_encoderA() {
     int b = digitalRead(CanalB);
     if (b > 0) {
         contador++;
     }
     else {
         contador--;
     }
}

void Lectura_encoderB() {
     int b = digitalRead(CanalA);
     if (b > 0) {
         contador++;
     }
     else {
         contador--;
     }
}



/* FUNCIONES CONTROL DEL MOTOR*/
void move(int per_speed) 
{
   // Speed puede tener valores positivos y negativos
   if (per_speed >= 0) moveMotorForward(pinMotorA, V_MAX * per_speed);
   else if (per_speed < 0) moveMotorBackward(pinMotorA, V_MAX * per_speed);
}

//void turn(int per_speed)
//{
//   // Speed puede tener valores positivos y negativos
//   if (per_speed >= 0) moveMotorForward(pinMotorA, V_MAX * per_speed);
//   else if (per_speed < 0) moveMotorBackward(pinMotorA, V_MAX * per_speed);
//}

void fullStop()
{
   disableMotors();
   stopMotor(pinMotorA);
}

//Funciones que controlan los motores
void moveMotorForward(const int pinMotor[3], int speed)
{
   digitalWrite(pinMotor[1], HIGH);
   digitalWrite(pinMotor[2], LOW);
   analogWrite(pinMotor[0], speed);
}

void moveMotorBackward(const int pinMotor[3], int speed)
{
   digitalWrite(pinMotor[1], LOW);
   digitalWrite(pinMotor[2], HIGH);
   analogWrite(pinMotor[0], speed);
}

void stopMotor(const int pinMotor[3])
{
   digitalWrite(pinMotor[1], LOW);
   digitalWrite(pinMotor[2], LOW);
   analogWrite(pinMotor[0], 0);
}

void enableMotors()
{
   digitalWrite(pinSTBY, HIGH);
}

void disableMotors()
{
   digitalWrite(pinSTBY, LOW);
}
