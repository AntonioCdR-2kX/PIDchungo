const int pinPWMA = 5;
const int pinAIN2 = 6;
const int pinAIN1 = 7;

const int pinSTBY = 8;

const int waitTime = 2000;   //espera entre fases
const int speed = 200;      //velocidad de giro

const int pinMotorA[3] = { pinPWMA, pinAIN2, pinAIN1 };

enum moveDirection {
   forward,
   backward
};

enum turnDirection {
   clockwise,
   counterClockwise
};

void setup()
{
   pinMode(pinAIN2, OUTPUT);
   pinMode(pinAIN1, OUTPUT);
   pinMode(pinPWMA, OUTPUT);
}

void loop()
{
   enableMotors();
   move(forward, 180);
   delay(waitTime);

   move(backward, 180);
   delay(waitTime);

   turn(clockwise, 180);
   delay(waitTime);

   turn(counterClockwise, 180);
   delay(waitTime);

   fullStop();
   delay(waitTime);
}

//Funciones que controlan el vehiculo
void move(int direction, int speed)
{
   if (direction == forward)
      moveMotorForward(pinMotorA, speed);
   else
      moveMotorBackward(pinMotorA, speed);
}

void turn(int direction, int speed)
{
   if (direction == forward)
      moveMotorForward(pinMotorA, speed);
   else
      moveMotorBackward(pinMotorA, speed);
}

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