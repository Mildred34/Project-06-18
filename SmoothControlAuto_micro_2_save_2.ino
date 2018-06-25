#include <Servo.h> // library to control the linear actuator
#include "HX711-multi.h"

/* WIP
 *  Create a new timer
 *
 */
 
 // comment next line out when you don't need to debug
#define DEBUG 1

// Pins to the load cell amp
#define CLK A0      // clock pin to the load cell amp
#define DOUT1 A1    // data pin to the first lca
#define DOUT2 A2    // data pin to the second lca
#define DOUT3 A3    // data pin to the third lca

#define BOOT_MESSAGE "MIT_ML_SCALE V0.8"

#define TARE_TIMEOUT_SECONDS 4

byte DOUTS[3] = {DOUT1, DOUT2, DOUT3};

#define CHANNEL_COUNT sizeof(DOUTS)/sizeof(byte)

long int results[CHANNEL_COUNT];

HX711MULTI scales(CHANNEL_COUNT, DOUTS, CLK);
 
Servo myservo;  

int myservo_pin = 9; // pin that controls the servo

long myservo_movetime = 0; // next time in millis servo next moves

/*int myservo_gPos = 0; // target position to move towards
int myservo_cPos = 0; // current postion of servo*/

int mode_a;
int pos = 0;    // variable to store the servo position
int cPos[2];       // current and previous position
int gPos;       // goal position
float ang;
int tDelay = 20; // delay between moves, gives appearance of smooth motion

/*const byte interruptPin = 2; // Input pin for TX1
volatile int pwm_value = 0; // PWM_value sent by the TX1
volatile int prev_time = 0; // To count the Pulse width */

int bufferR[3]={0,0,0}; // buffer to store  last backward sensor data
int bufferA[3]={0,0,0}; // buffer to store last forward sensor data
int dF[2] = {0,0}; // buffer to store last differential values
int chgmt_signe = 0; // indique s'il y a eu un changement de signe (passage d'un capteur à un autre)

byte varCompteur = 0; // La variable compteur pour la fonction d'interruption

int mode; // mode = moving backward or forward

 unsigned long h; // pas de mesure


float derivee2 = 0;
float derivee = 0;

int i = 2; // buffer parameter

int compteur = 0; // To initialize the timer to determine if we push the stick during more than an amount of time
int compteur1 = 0;
int compteur2 = 0; // To initialize the timer to determine if we go back to the auto mode;

bool demarrage; // To know if we can tare the sensors, or launch the beginning of the loop, to avoid problem

unsigned long timer1 = 0;
unsigned long timer2 = 0;
unsigned long time3 = 0;
unsigned long timer11 = 0;
unsigned long timer21 = 0;
unsigned long timerh = 0;
unsigned long timer_mode = 0;
 unsigned long timer_signe = 0;
 unsigned long timer_M_A = 0;
 
void setup() {
  Serial.begin(9600);
  Serial.println(BOOT_MESSAGE);
  Serial.flush();

  
  
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object

  
  //gPos = 78; // Goal Pos
  
   gPos = 1330; // neutral pos in us
   
   for(int j=0;j<(sizeof(cPos)/sizeof(int));j++) cPos[j] = 1330;

  myservo.writeMicroseconds(gPos);
  
  //myservo.writeMicroseconds(gPos);
  
  delay(2000); // wait for the stick to get back to the neutral pos
  
  // cPos = myservo.read(); // read the servo position
  
  demarrage = 0;
  
  cli(); // Met le registre GIE à 0, disable any interruption
  
  /*pinMode(interruptPin, INPUT); // Interruption for PWM Reading
  attachInterrupt(digitalPinToInterrupt(interruptPin), rising, RISING);
  */
  bitClear (TCCR2A, WGM20); // WGM20 = 0 Mode compteur du timer
  bitClear (TCCR2A, WGM21); // WGM21 = 0 
  TCCR2B = 0b00000110; // Clock / 256 soit 16 micro-s et WGM22 = 0
  TIMSK2 = 0b00000000; // Interruption locale non-autorisée par TOIE2
  sei(); // Active l'interruption globale

  Serial.println("Remove the stick from the fork");
  Serial.println("Press t when it's ready");
  
  while(demarrage!=1){ // Attente pour tarer les capteurs
    
    if (Serial.available() > 0) { // wait for the serial port to be available
      demarrage = Demarrage();}
      delay(10);
  
  }
  
  Serial.print("Tare is getting ready");
  demarrage = 0;
   
  tare();
  
   Serial.println("Tare is ready ");
   Serial.println("Put back the stick");
   Serial.println("Press 't' when the stick is back");
   
   while(demarrage!=1)
   {if (Serial.available() > 0) {
    demarrage = Demarrage();}
    delay(10);
    }
    
    mode_a = 0; // initialisation sur le mode manuel
    
    TIMSK2 = 0b00000001; // Interruption locale autorisée par TOIE2
    
    delay(300);
    
    Serial.println("The motor will move now");
    timer_mode = millis();
    gPos = 1000;   
} 

void tare() {
  bool tareSuccessful = false;

  unsigned long tareStartTime = millis();
  while (!tareSuccessful && millis()<(tareStartTime+TARE_TIMEOUT_SECONDS*1000)) {
    tareSuccessful = scales.tare(20,10000);  //reject 'tare' if still ringing
  }
}

void sendRawData() {
  
  scales.read(results);
  
  for (int i=0; i<scales.get_count(); ++i) {
    Serial.print( results[i]);  
    Serial.print( (i!=scales.get_count()-1)?"\t":"\n");
  }  
  delay(10);
  
}


 
void loop() {
  
  if (millis()> timer_mode +100){
    
  mode_a = test_mode();
  timer_mode = millis();
  
  
  #ifdef DEBUG
  if (Serial.available() > 0){
  Serial.print(bufferA[2]);
  Serial.print('\t');
  Serial.print(bufferR[2]);
  Serial.print('\t');
  Serial.print(gPos);
  Serial.print('\t');
  Serial.println(cPos[1]);
  
  if(mode_a == 1)
  {
    Serial.println("Mode auto");
    }
  else{
     Serial.println("Mode manuel");
    }}
  #endif
  
  }
  
  if(bufferR[2]<20000 && bufferA[2]<20000){ // safety to avoid overload
    
  if(mode_a == 1)
  {
    mode_auto();
    
    }
  else{
    mode_manuel();
    }
  }
  else{
    while(1); // stop the program 
    Serial.println("Program is stopped");
    }
  
  
  // Serial.print(tDelay); Serial.println(" mSeconds");
  // if (Serial.available() > 0) { GetCommand(); }
} 

void moveServo() {
  
   #ifdef DEBUG
    if (Serial.available() > 0){
        Serial.println("ca marche avec serial");}
    #endif
    
    
  /*
   * Function which allow the linear actuator to move by step of 5 us ~ 1°
   * To add : prise en compte de la force de l'utilisateur, ajuster vitesse en conséquence... jouer sur le pas, si le vérin était plus rapide on pourrait également jouer sur le delay,...
   */

  
  if (cPos[1] < gPos) { // modifier hypothèses pour s'adapter au mode auto avec la TX1  gpos sera pas tjrs égale aux extréma
    cPos[0] = cPos[1];
    cPos[1] += 5; // modifier tdelay en conséquence
    cPos[1] = constrain(cPos[1],1000,2000);
    myservo.writeMicroseconds(cPos[1]);// 70 = neutral position  
  }
  
  if (cPos[1] > gPos){
    cPos[0] = cPos[1];
    cPos[1] -= 5; 
    cPos[1] = constrain(cPos[1],1000,2000);
    myservo.writeMicroseconds(cPos[1]);
  } 
  //if (cPos == gPos) // nothing
  myservo_movetime = millis() + tDelay;
  
}

void GetCommand() { 

  /*
   * Allow to increase or decrease the time delay between to PWM signal sent to the motor
   * 
   */

   
  if (Serial.available() > 0){
  
  char command = Serial.read();
  char mVal = command;
  
  
  if (mVal == 'x') {
    tDelay = tDelay + 10;
  } 
  
  else if(mVal == 'c'){
    tDelay = tDelay - 5;
    }
    
  else {
    tDelay = mVal;
  }
  
  Serial.print("Pauses changed to : "); Serial.print(tDelay); Serial.println(" mSeconds");
  Serial.print("command: "); Serial.println(command);
  Serial.flush();
  }
}

bool Demarrage(){

  // Function which is necessary to secure the start 
  
  char command = Serial.read();
  // Serial.println(command);
  int a;
  
  while (!Serial.available()){}
  
  if (command == 't') {
    // Serial.print(command);
    a = 1;
    return a;
  } 
  else{
    a = 0;
    return a;}
}

void Reading()
{

  /*if (millis()> timer_mode +100){
    
  mode_a = test_mode();
  timer_mode = millis();
  
  while(Serial.available()<1){}
  
  //Serial.print(bufferA[1]);
  //Serial.print('\t');
  //Serial.println(bufferR[1]);
  
  Serial.print(bufferA[2]);
  Serial.print('\t');
  Serial.print(bufferR[2]);
  Serial.print('\t');
  Serial.println(cPos[1]);
  
  if(mode_a == 1)
  {
    Serial.println("Mode auto");
    }
  else{
     Serial.println("Mode manuel");
    }
  }*/

  /* Function launched every 0.1 s
   *  Read the sensors and store data 
   *  
   */
  
  scales.read(results);
  
  for(int j=2;j>0;j--)
  {
    bufferR[j-1]= bufferR[j]; // met à jours les précédentes valeurs.
    bufferA[j-1]= bufferA[j];
    
    if (j<=1){
      dF[j-1] = dF[j];
    }
    
    }
    
  bufferR[2] = results[1]; // 1 = capteur arrière
  bufferA[2] = results[0]; // 0 = capteur avant
  dF[1] = bufferA[2] - bufferR[2]; // Difference of force
  
  
  }
  
  int test_mode(){ 
    
    /* Test if we have to pass into the manual or auto mode
     *  
     *  
     */
    
  
    if(mode_a == 1){ // si le mode auto est activé
      //Serial.println("Mode auto");
      if(chgmt_signe == 1){ // si un changement de signe a été détecté précédemment
        
        if(dF[1]*dF[0]<0){ // si un second changement de signe est détecté ne pas prendre en compte le précédent
          
          chgmt_signe = 0;
          
        }
        else{
          if(millis()-timer_signe > 1000){ // mode manuel activé
            // Serial.println("Mode manuel");
            chgmt_signe = 0;
            return 0; // forcer mode auto pour l'instant
            }
          
          }
        }
        else{
        if (dF[1]*dF[0]<0 &&  (abs(dF[1]) + abs(dF[0])) > 1000){
          chgmt_signe = 1;
          timer_signe = millis();  
        }
        }
      }
    else{
      // mode manuel est en cours
      int dist_neutre = cPos[1] - 1330; // on mesure la distance par rapport au neutre
      dist_neutre = abs(dist_neutre);
      //Serial.print("distance du neutre: ");
      //Serial.println(dist_neutre);

      
      if (dist_neutre < 120){ // si on est près du neutre et qu'il n'y a pas eu de déplacement brutal
          
        if(compteur2 == 0){ // on compte le temps qu'on reste au neutre
            compteur2 = 1;
            timer_M_A = millis();
        }
        
      if(compteur2 == 1){
        /*Serial.print("Attente avant mode auto");
        Serial.println(millis() - timer_M_A);*/
        if (millis() - timer_M_A > 5000){ // si ca fait plus de 500ms qu'on est resté au neutre, on repasse en mode auto
            compteur2 = 0;
          // Serial.println("Mode auto");
        return 0;
        }
      }
    
    }
    
    else{
      compteur2 = 0;
      
      }
    }
    }

  void mode_auto(){

    /*
     * Auto mode
     * For now it's just a sweep function
     */
    
    if (cPos[1] == gPos && gPos == 2000){
        gPos = 1000;
      }
    else if(cPos[1] == gPos && gPos == 1000) gPos = 2000;
    
    if (cPos[1] != gPos && millis() >= myservo_movetime) {
        moveServo(); // move servo after a certain amount of time ; amount of time proportional to the constraint read by the sensors
    }
    
      /*if(gPos == 1000 && cPos <= 1001){
        gPos = 2000;
        Serial.println("je suis la");
        }*/
    }

    void mode_manuel(){

      /*
       * Power steering mode
       * 
       * 
       */
      
     if (bufferR[2] > 2000 && bufferR[2] > bufferA[2]+500 && bufferR[2]-bufferR[1] < 5) // reculé
  { 
      if (compteur == 0) {timer1 = millis();
        compteur=1;
    }
    
    timer2 = millis()-timer1;
    timer21 = 0;
  
    
    if (timer2 > 500) {//Serial.println("Reculé dans la bonne boucle >65");
    
      gPos = 1000;
    
      while(bufferR[2] > 100){
        if (cPos != gPos && millis() >= myservo_movetime) {
          moveServo(); // move servo after a certain amount of time ; amount of time proportional to the constraint read by the sensors
    }
    // Serial.println("Mode reculer");
    }
    }
    
    
    
  }

  if (bufferR[2] < 400)
  {
    timer2 = 0;
    compteur = 0;
  }

  if (bufferA[2] > 2000 && bufferA[2] > bufferR[2]+500 && bufferA[2]-bufferA[1] < 5)
  { 
    if (compteur1 == 0) {timer1=millis();
      compteur1 = 1;
    } // initialize the timer1
    
    timer2 = 0; // put back to 0 timer2
    timer21 = millis()-timer1;
  
    
    if (timer21 > 500) {
      
      gPos = 2000;
    
      while(bufferA[2] > 100){
     // Serial.println("bloqué");
        if (cPos != gPos && millis() >= myservo_movetime) {
          moveServo(); // move servo after a certain amount of time ; amount of time proportional to the constraint read by the sensors
    }
    //Serial.println("Mode avancer");
    }
    }

    
    
  }

  if (bufferA[2] < 400)
  {
      timer21 = 0;
      compteur1 = 0;
  }   
      }
  
/*void rising() { // functions to call when there is an interruption
  if (mode_a == 1){
  attachInterrupt(0, falling, FALLING); // 0 = pin2
  prev_time = micros();
  }
}
 
void falling() {
  attachInterrupt(0, rising, RISING);
  pwm_value = micros()-prev_time;
  // Serial.println(pwm_value);
}*/

// Routine d'interruption
ISR(TIMER2_OVF_vect) {
  TCNT2 = 256 - 250; // 250 x 16 µS = 4 ms
  
  if (varCompteur++ > 25) { // 25 * 4 ms = 100 ms 
    varCompteur = 0;
    Reading();
  }
}


