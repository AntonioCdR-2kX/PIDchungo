
/*
    Motores con encoders
    Rojo: motor+
    Negro: motor-
    Blue: Vcc
    Green: GND
    Yellow: Flanco A (pin2)
    White: Flanco B (pin3)
 */

 //control de posicion
 //programa que por medio de interrupciones nos dice si estamos girando hacia adelante o hacia atras y mide los pulsos
 #define CanalA 2
 #define CanalB 3
 #define PWM 5
 #define IN2 6
 #define IN1 7
 
 // INICIALIZACION
 int objetivo;

 volatile long contador = 0;
 float pwr;
 int dir=0;


 void setup() {
     Serial.begin(9600);           // Inicializar puerto serie
     //Serial.println("Holiwi"); // Primer texto de control

     pinMode(CanalA, INPUT_PULLUP);
     pinMode(CanalB, INPUT_PULLUP);

     attachInterrupt(digitalPinToInterrupt(CanalA), Lectura_encoder, RISING);
 }


 // BUCLE
 void loop() { 

     //setMotor(dir, pwr, PWM, IN1, IN2);

     Serial.print(contador);
     Serial.println();

     delay(100);
 }


 void Lectura_encoder() {
     int b = digitalRead(CanalB);
     if (b > 0) {
         contador++;
     }
     else {
         contador--;
     }
 }

/*
 void setMotor(int dir, int pwmVal, int pwm, int in1, int in2) {   //dir de rot, velocidad de pwm, pin de pwm, in1 pin, in2 pin
     analogWrite(pwm, pwmVal); //set speed
     if (dir == 1) {
         digitalWrite(in1, HIGH);
         digitalWrite(in2, LOW);
     }
     else if (dir == -1) {
         digitalWrite(in1, LOW);
         digitalWrite(in2, HIGH);
     }
     else {
         digitalWrite(in1, LOW);
         digitalWrite(in2, LOW);
     }
 }
 */
 