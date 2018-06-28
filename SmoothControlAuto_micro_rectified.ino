#include <Servo.h> // library to control the linear actuator
#include "HX711-multi.h"

/* WIP
    Create a new timer

*/

// comment next line out when you don't need to debug
#define DEBUG 1

// Variables for the sensors and the amplifier module

// Pins to the load cell amp
#define CLK A0      // clock pin to the load cell amp
#define DOUT1 A1    // data pin to the first lca
#define DOUT2 A2    // data pin to the second lca
#define DOUT3 A3    // data pin to the third lca

#define BOOT_MESSAGE "2018_project_mower"

#define TARE_TIMEOUT_SECONDS 4

byte DOUTS[3] = {DOUT1, DOUT2, DOUT3};

#define CHANNEL_COUNT sizeof(DOUTS)/sizeof(byte)

long int results[CHANNEL_COUNT];

HX711MULTI scales(CHANNEL_COUNT, DOUTS, CLK);

// Variables where sensor data will be stored
int bufferR[3] = {0, 0, 0}; // buffer to store  last backward sensor data
int bufferL[3] = {0, 0, 0}; // buffer to store last forward sensor data (L for left)
int dF[2] = {0, 0}; // buffer to store last differential values

// Variable for the linear actuator
Servo myservo;
int myservo_pin = 9; // pin that controls the linear actuator
unsigned long myservo_movetime = 0; // next time in millis servo next moves
unsigned long tDelay = 20; // delay between moves, gives appearance of smooth motion


int mode_a; // mode_a = 1 --> autonomous mode ~~~ mode_a = 0 --> Power steering mode

// Position
int cPos[2];       // current and previous position
int gPos;       // goal position


// Variables to copy PWM_signal
/*const byte interruptPin = 2; // Input pin for TX1
  volatile int pwm_value = 0; // PWM_value sent by the TX1
  volatile int prev_time = 0; // To count the Pulse width */

// Variables to recognize if there is a changement of mode
int chgmt_signe = 0; // Boolean value to see if there is a sign changement on the variable Df

float derivee2 = 0; // first and second derivative will be stored here ( not used for now)
float derivee = 0;
byte Interrupt_counter = 0; // Counter variable for the interruption function

// Variable to measure how long last a function to check if it lasts for too long
unsigned long h; // pas de mesure

// Variables for the Power steering mode

int compteur = 0; // To initialize the timer to determine if we push the stick during more than an amount of time -->
int compteur1 = 0;
int compteur2 = 0; // To initialize the timer to determine if we go back to the auto mode;

bool demarrage; // To know if we can tare the sensors, or launch the beginning of the loop, to avoid problem

unsigned long Start_to_press_time = 0; // when the user will start to press the stick, the current time will be stored here
unsigned long timeR = 0; // duration 'til the user keep pressing of the stick (for the Right sensor)
unsigned long timeL = 0; // duration 'til the user keep pressing of the stick (for the Left sensor)
unsigned long timerh = 0;
unsigned long timer_mode = 0;
unsigned long timer_signe = 0;
unsigned long timer_neutre = 0;

void setup() {
  Serial.begin(9600);
  Serial.println(F(BOOT_MESSAGE));
   Serial.flush();



  myservo.attach(9);  // attaches the servo on pin 9 to the servo object


  //gPos = 78; // Goal Pos

  gPos = 1330; // neutral pos in us

  for (int j = 0; j < (sizeof(cPos) / sizeof(int)); j++) cPos[j] = 1330;

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

  Serial.println(F("Remove the stick from the fork"));
  Serial.println(F("Press t when it's ready"));

  while (demarrage != 1) { // Attente pour tarer les capteurs

    if (Serial.available() > 0) { // wait for the serial port to be available
      demarrage = Demarrage();
    }
    delay(10);

  }

  Serial.print(F("tare is getting ready"));
  demarrage = 0;

  tare();

  Serial.println(F("Tare is ready "));
  Serial.println(F("Put back the stick"));
  Serial.println(F("Press 't' when the stick is back"));

  while (demarrage != 1)
  { 
    if (Serial.available() > 0) {
      demarrage = Demarrage();
    }
    delay(10);
  }
  

  mode_a = 0; // Initialize in manual mode
 
  TIMSK2 = 0b00000001; // Interruption locale autorisée par TOIE2
  delay(300);

  Serial.println(F("The motor will move now"));
  timer_mode = millis();
  gPos = 1000;
  Serial.end();
}



void tare() {

  /*
     Set up the sensor; Just count the ticks, do an average of it, to estimate the zero and convert the data in gramms
     Read the .c for more information
  */

  bool tareSuccessful = false;

  unsigned long tareStartTime = millis();
  while (!tareSuccessful && millis() < (tareStartTime + TARE_TIMEOUT_SECONDS * 1000)) {
    tareSuccessful = scales.tare(20, 10000); //reject 'tare' if still ringing
  }
}



void sendRawData() {
  /*
     Read the data from all the sensors
  */
  scales.read(results);

  for (int i = 0; i < scales.get_count(); ++i) {
    Serial.print( results[i]);
    Serial.print( (i != scales.get_count() - 1) ? "\t" : "\n");
  }
  delay(10);

}

bool Demarrage() {

  /* Function which is necessary to secure the start
      Wait the user to write on the serial port to keep the programm going
  */
Serial.println("demarre");
  char command = Serial.read() ;

  while (!Serial.available()) {}

  if (command == 't') {
    // Serial.print(command);
    return 1;
  }
  else {
    return 0;
  }
}


void loop() {


  /*
     Print essential values for debug purposes
     Launch the test mode function every 0.1s --> To check if we switch of mode

  */
  if (millis() > timer_mode + 100) {
    mode_a = test_mode();
    timer_mode = millis();
    
#ifdef DEBUG

    if (Serial.available() > 0) {

      Serial.print(bufferL[2]);
      Serial.print('\t');
      Serial.print(bufferR[2]);
      Serial.print('\t');
      Serial.print(gPos);
      Serial.print('\t');
      Serial.println(cPos[1]);

      if (mode_a == 1)
      {
        Serial.println("Mode auto");
      }
      else {
        Serial.println("Mode manuel");
      }
    }

#endif

  }

  if (bufferR[2] < 20000 && bufferL[2] < 20000) { // safety to avoid overload

    if (mode_a == 1)
    {
      mode_auto();

    }
    else {
      mode_manuel();
    }
  }
  else {
    while (1); // stop the program
    Serial.println("Program is stopped");
  }


  // Serial.print(tDelay); Serial.println(" mSeconds");
  // if (Serial.available() > 0) { GetCommand(); }
}

void moveServo() {

#ifdef DEBUG
  /*if (Serial.available() > 0) {
    // Serial.println("");
  } */
#endif


  /*
     Function which allow the linear actuator to move by step of 5 us ~ 1°
     This function is called in the manual_mode function
  */


  if (cPos[1] < gPos) { // modifier hypothèses pour s'adapter au mode auto avec la TX1  gpos sera pas tjrs égale aux extréma
    cPos[0] = cPos[1];
    cPos[1] += 5; // If you modify this value, you have to change the delay between to PWM signal so that the linear actuator can reach the pos before a new signal sent
    cPos[1] = constrain(cPos[1], 1000, 2000);
    myservo.writeMicroseconds(cPos[1]);// 79 = neutral position in angle approx or 1330 in us
    myservo_movetime = millis() + tDelay;
    return ;
  }

  if (cPos[1] > gPos) {
    cPos[0] = cPos[1];
    cPos[1] -= 5;
    cPos[1] = constrain(cPos[1], 1000, 2000);
    myservo.writeMicroseconds(cPos[1]);
    myservo_movetime = millis() + tDelay;
    return ;
  }
  //if (cPos == gPos) // nothing



}

int test_mode() {

  /* Test if we have to pass into the manual or auto mode
  */


  if (mode_a == 1) { // If auto mode is activated ; the test changed according to the current mode

    if (chgmt_signe == 1) { // If a sign changement has been detected

      /*
         For instance the stick is going forward so the right sensor detect a high force value on its side ; we (i mean the user) push
         consequently the stick forward so that it pressed
         on the left sensor. This would make a sign change
      */

      if (dF[1]*dF[0] < 0) {

        /* If a second changement is detected afterwards, the first one is not taken into account
          This is to avoid unexpected behaviors, for instance if in the auto mode the direction of the linear actuator is changed, we don't want to proc the manual mode
        */

        chgmt_signe = 0;

      }
      else {
        if (millis() - timer_signe > 1000) {
          /* Manual mode activated*/
#ifdef DEBUG
          // Serial.println("Manual mode activated from auto_mode");
#endif

          chgmt_signe = 0; // put back the sign_chgmt indicator at 0
          return 0; // 0 mean manual ; 1 auto
        }

      }
    }
    else {
      /*
         If no sign change has been detected yet
      */

      if (dF[1]*dF[0] < 0 &&  (abs(dF[1]) + abs(dF[0])) > 1000) {

        /*
           First sign change detect + need to have a big difference between the previous difference value noted
           This value has to be reevalutated or i need to make a change like not between the current and the C-1 but more between the current and the C-2, difficult to make a 1kg difference
           in 0.1s
        */

        chgmt_signe = 1;
        timer_signe = millis();
      }
    }
  }
  else {
    /*
        The manual mode is currently active

    */

    int dist_neutre = cPos[1] - 1330;  // distance between the current position and the neutral position


    dist_neutre = abs(dist_neutre);

    //Serial.print("distance du neutre: ");
    //Serial.println(dist_neutre);


    if (dist_neutre < 120) { // If we are close enough to the neutral pos

      if (compteur2 == 0) { // we count the time we are close to this pos
        compteur2 = 1;
        timer_neutre = millis();
      }

      if (compteur2 == 1) {
        /*Serial.print("Time before auto mode:");
          Serial.println(millis() - timer_neutre);*/

        if (millis() - timer_neutre > 5000) { // si ca fait plus de 500ms qu'on est resté au neutre, on repasse en mode auto
          compteur2 = 0;
          // Serial.println("Mode auto activated");
          return 0;
        }
      }

    }

    else {
      compteur2 = 0;

    }
  }
}

void mode_auto() {

  /*
     Auto mode
     For now it's just a sweep function
  */

  if (cPos[1] == gPos && gPos == 2000) {
    gPos = 1000;
  }
  else if (cPos[1] == gPos && gPos == 1000) gPos = 2000;

  if (cPos[1] != gPos && millis() >= myservo_movetime) {
    moveServo(); // move servo after a certain amount of time ; amount of time proportional to the constraint read by the sensors
  }


}

void mode_manuel() {

  /*
     Power steering mode

  */
      

  if (bufferR[2] > 2000 && bufferR[2] > bufferL[2] + 500 && bufferR[2] - bufferR[1] < 5) // reculé
  {
    if (compteur == 0) {
      Start_to_press_time = millis();
      compteur = 1;
      compteur1 = 0;
      Serial.println("");
    }
    
    timeR = millis() - Start_to_press_time;
    timeL = 0;
    
    if (timeR > 500) {

      gPos = 1000;

      while (bufferR[2] > 100) {
        if (cPos != gPos && millis() >= myservo_movetime) {
          moveServo(); // move servo after a certain amount of time ; amount of time proportional to the constraint read by the sensors
        }
      }
    }



  }

  if (bufferR[2] < 400)
  {
    timeR = 0;
    compteur = 0;
  }
Serial.println("1");
  if (bufferL[2] > 2000 && bufferL[2] > bufferR[2] + 500 && bufferL[2] - bufferL[1] < 5)
  {
    if (compteur1 == 0) {
      Start_to_press_time = millis();
      compteur1 = 1;
      compteur = 0;
      
    } // initialize the timeR

    timeL = millis() - Start_to_press_time;
    timeR = 0;

    if (timeL > 500) {

      gPos = 2000;

      while (bufferL[2] > 100) { // may be remove the while and add an indicator
        //Serial.println("2");
        // Serial.println("bloqué");
        if (cPos != gPos && millis() >= myservo_movetime) {
          moveServo(); // move servo after a certain amount of time ; amount of time proportional to the constraint read by the sensors
          //Serial.println("3");
        }
        //Serial.println("Mode avancer");
      }
    }



  }

  if (bufferL[2] < 400)
  {
    timeL = 0;
    compteur1 = 0;
  }
  
}

// interrupt routine
ISR(TIMER2_OVF_vect) {
  TCNT2 = 256 - 250; // 250 x 16 µS = 4 ms

  if (Interrupt_counter++ > 25) { // 25 * 4 ms = 100 ms
    Interrupt_counter = 0;
    Reading();
  }
}

void Reading()
{

  /* Function launched every 0.1 s
      Read the sensors and store data

  */

  scales.read(results);

  for (int j = 2; j > 0; j--)
  {
    bufferR[j - 1] = bufferR[j]; // met à jours les précédentes valeurs.
    bufferL[j - 1] = bufferL[j];

    if (j <= 1) {
      dF[j - 1] = dF[j];
    }

  }

  bufferR[2] = results[1]; // 1 = capteur arrière
  bufferL[2] = results[0]; // 0 = capteur avant
  dF[1] = bufferL[2] - bufferR[2]; // Difference of force


}

void GetCommand() {

  /*
     Allow to increase or decrease the time delay between to PWM signal sent to the motor

  */


  if (Serial.available() > 0) {

    char command = Serial.read();
    char mVal = command;


    if (mVal == 'x') {
      tDelay = tDelay + 10;
    }

    else if (mVal == 'c') {
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
