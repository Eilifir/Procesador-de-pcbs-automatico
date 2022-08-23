//incluyo librerias y defino pines
#include <LiquidCrystal_I2C.h>      //liberia para LCD
#include <math.h>           //liberia para calculos
#include <PID_v1.h>           //libreria PID de br3ttb https://github.com/br3ttb/Arduino-PID-Library
#define SensorPIN  A0           //termistor
#define MOSFET_PIN 6          //pin de salida PWM
#define push 5              //boton del encoder
#define VELOCIDAD 2000          //velocidad motor paso a paso
//Variables para el programa
int calibrado = 0;             
int limpiado = 0;
int minuto;
int segundo;
int tiempoA;
int tiempoB;
int tiempoC;
int llamado = 0;
//motor paso a paso
int pasos;

//valores para poder utilizar el termistor
float Vcc = 5;              //voltaje
float Rc = 4920;            //resistencia en serie
//Valores para la ecuacion Steinhart–Hart
float A = 1.11492089e-3;
float B = 2.372075385e-4;
float C = 6.954079529e-8;
float K = 2.5;              //factor de disipacion en mW/C

//valores para usar el PID
const int PIN_OUTPUT = MOSFET_PIN;
double Kp=2, Ki=5, Kd=1;        // Constantes del controlador
double Input, Output, Setpoint;     // variables externas del controlador

PID pidController(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);


//variables para el encoder rotativo
int counter = 0;
int last_counter = 0;
bool clk_State;
bool Last_State; 
bool dt_State;  
int pushed = 0;
int tiempo = 0;

//definicion del lcd
LiquidCrystal_I2C lcd(0x20,16,2);

void setup()
{
  PCICR |= (1 << PCIE0);          //enable PCMSK0 scan                                                 
  PCMSK0 |= (1 << PCINT0);        //Set pin D8 trigger an interrupt on state change. 
  PCMSK0 |= (1 << PCINT1);        //Set pin D9 trigger an interrupt on state change.  
  DDRB &= B11111100;              //8, 9 as input for the encoder clock and data pins
  Last_State =   (PINB & B00000001);  //pin 8 state (clock pin)? 
  //defino pines
  pinMode(13, OUTPUT);          //motor
  pinMode(12, OUTPUT);          //motor
  pinMode(11, OUTPUT);          //motor
  pinMode(10, OUTPUT);          //motor
  pinMode(7, OUTPUT);           //motor
  pinMode(6, OUTPUT);           //cama caliente
  pinMode(4, OUTPUT);           //motor
  pinMode(3, OUTPUT);           //motor
  pinMode(A0, INPUT);           //termistor
  pinMode(A1, INPUT);           //fin de carrera
  pinMode(A2, INPUT);           //boton de clear
  pinMode(A3, INPUT);           //fin de carrera
  pinMode(A4, INPUT);           //lcd
  pinMode(A5, INPUT);           //lcd
  //defino parametros PID
  lcd.init();             //Arranca el LCD
  tiempoA = millis();
  tiempoB = 1000;
  Setpoint = 318.15;          //defino la temperatura deseada en kelvin
  pidController.SetMode(AUTOMATIC);     // encender el PID
  Serial.begin(9600);
}

void Limpieza(){
      //me fijo si se presiono el boton de limpiado
    int fin = 0;
    if(llamado = 1){
    do{
      digitalWrite(13, HIGH);
      delay(5000);
      digitalWrite(13,LOW);
      digitalWrite(12,HIGH);
      delay(5000); 
      digitalWrite(12,LOW);
    if(limpiado == 0){
      digitalWrite(11, HIGH);     //empiezo a rotar las cajas y espero antes de empezar a chequear el sensor
      delay(2000);
      while(A3 == 0){
      limpiado = 1;
      }                 //una vez detectado el segundo contenedor lo freno 
      digitalWrite(11,LOW);
    }else{
      digitalWrite(10, HIGH);     //empiezo a rotar las cajas y espero antes de empezar a chequear el sensor
      delay(2000);
      while(A3 == 0){
      limpiado = 0;
      }                 //una vez detectado el segundo contenedor lo freno 
      digitalWrite(10,LOW);
      fin = 1;
    }
    }while(fin == 0);
}
}
void Control(){
  if(llamado = 1){
  //calculo de temperatura
  float raw = analogRead(SensorPIN);        //leo el termistor
  float V =  raw / 1024 * Vcc;              //calculo el voltaje
  Serial.println(String(V) + "volts");      //lo pongo en el monitor serie
  float R = (Rc * V ) / (Vcc - V);          //calculo la resistencia
  Serial.println(String(R) + "ohms");       //lo pongo en el monitor serie
  //calculo para determinar la temperatura en base al termistor (ecuacion Steinhart–Hart. Esta separado en distintas lineas para asegurar que
  //no haya errores
  float logR  = log(R);
  float logR3 = logR * logR *logR;
  float logRB = logR * B;
  float logRC = logR3 * C;
  float logRABC = A + logRB + logRC;
  float R_th = 1/logRABC;
 
  float kelvin = R_th - V*V/(K * R)*1000;   //lo paso a kelvin
  float celsius = kelvin - 273.15;          //lo paso a celsius
  //hago control
  Input = analogRead(kelvin);               //le mando la temperatura actual
  pidController.Compute();                  //actualizar el PID
  analogWrite(PIN_OUTPUT, Output);          //pongo la salida con el pwm con el rango predefinido 0 a 255
  
  Serial.print("T = ");                     //pongo los valores en el monitor serie
  Serial.print(celsius);
  Serial.print("C\n");
  Serial.println(String(Output) + "pwm");
  delay(2500);
  }
}
}

void loop() 
{
//prendo el motor paso a paso y lo calibro
if(calibrado == 0){ //se calibraron los motores?
digitalWrite(3,LOW);            //apago el motor
delay(1000);
digitalWrite(3,HIGH);             //prendo el motor
digitalWrite(4,HIGH);             //le mando la direccion
delayMicroseconds(VELOCIDAD);
while(A1 == 0){               //sensor fin de carrera de los brazos
//cuenta los pasos hasta que se active el sensor
digitalWrite(7,HIGH);
delayMicroseconds(VELOCIDAD); 
digitalWrite(7,LOW);
pasos++;
}
//reseteo la posicion del motor
digitalWrite(4, LOW);             //invierto direccion del motor
for(int i = 0; i < pasos; i++){
  digitalWrite(7,HIGH);
  delayMicroseconds(VELOCIDAD);
  digitalWrite(7,LOW);
}
digitalWrite(3, LOW);
calibrado = 1;                //con esto no se repite la sequencia de calibrado
}

  //espero hasta que el usuario haga una interaccion con el encoder para mostrar la siguiente pantalla
  if((last_counter > counter) || (last_counter < counter)  || pushed){
    //utilizo el boton del encoder para confirmar el tiempo y le pido que seleccione el tiempo
    lcd.clear(); 
    lcd.setCursor (1, 0);
    lcd.print("Seleccione el");
    lcd.setCursor(3, 1);
    lcd.print("tiempo");
    do{
      if(A2 == HIGH){
        llamado = 1;
        Limpieza;
      }
      tiempo = counter;           //empiezo a definr el tiempo
      lcd.clear();
      lcd.setCursor(3, 0);
      lcd.print("Tiempo:");
      lcd.setCursor(2, 1);
      lcd.print(String(tiempo) + " Minutos");
      tiempoC = 1;
      //defino las posiciones del encoder
    last_counter = counter;
    if(counter > 60)
     {
      counter= 60;
    }
     if(counter < 0)
    {
    counter=0;
    }           //termino
    }while(pushed == 0);
    //depues de seleccionar el tiempo bajo los brazos roboticos
    digitalWrite(3,HIGH);
    digitalWrite(4,HIGH);
    for(int i = 0; i < pasos; i++){
     digitalWrite(7,HIGH);
     delayMicroseconds(VELOCIDAD);
     digitalWrite(7,LOW);
    }
    //hago el calculo de tiempo mientras que monitoreo la temperatura
    while(tiempo > 1 && tiempoC > 0){
      if (millis() - tiempoA > tiempoB)
      {
      tiempoA += tiempoB;
      minuto++;
      }else{
      llamado = 1;
      Control;
      }
      
      if(minuto == 60 ){
        tiempo--;
        lcd.clear();
        lcd.setCursor(3, 0);
        lcd.print("Tiempo:");
        lcd.setCursor(2, 1);
        lcd.print(String(tiempo) + " Minutos");
        minuto =0;
      }
      if(pushed){
        tiempoC = 0;
      }
    }
    //lo mismo pero en segundos
    tiempo = 60;
    while(tiempo > 0 && tiempoC > 0){
      delay(1000);
      if (millis() - tiempoA > tiempoB)
      {
      tiempoA += tiempoB;
      }else{
      Control;
      }
      tiempo--;
      lcd.clear();
      lcd.setCursor(3, 0);
      lcd.print("Tiempo:");
      lcd.setCursor(2, 1);
      lcd.print(String(tiempo) + " Segundos");
      segundo++;
      Control;
      segundo = 0;
      if(pushed){
        tiempoC = 0;
      }

     }
     }
digitalWrite(MOSFET_PIN, LOW); //apago el MOSFET para que no se queme el sistema
//devuelvo el motor a su posicion original
digitalWrite(4, LOW);
for(int i = 0; i < pasos; i++){
digitalWrite(7,HIGH);
digitalWrite(7,LOW);
delayMicroseconds(VELOCIDAD);
}
} 
//interrupicones
ISR(PCINT0_vect){
  
  clk_State =   (PINB & B00000001); //pin 8 state, clock pin? 
  dt_State  =   (PINB & B00000010); 
  if (clk_State != Last_State){   //sentido horario o antihorario??  
     // Si la data state es diferente al clock state, quiere decir que el encoder esta rotando en sentido horario
     if (dt_State != clk_State) { 
       counter ++;
     }
     else {
       counter --;
     } 
   } 
   Last_State = clk_State; // actualizo el estado de la data
  
