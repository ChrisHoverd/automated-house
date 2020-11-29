

//initializing sensor, actuator and LED pins
#define buttonPin 2
#define buzzerPin 23
#define greenLED 50
#define redLED 51
#define motionSensor 3
#define flameSensor 18
#define valvePin 8

// including the LCD, keypad, and password libraries
#include <LiquidCrystal_I2C.h>
#include <Keypad.h>
#include <Password.h>

//create the password to be used on the keypad
Password password = Password("1379");

//set the number of rows and columns that are on the keypad
const byte rows = 4;
const byte cols = 3;

//setting up the keypad as it physcially appears
char keys[rows][cols] = {
  {'1', '2', '3'},
  {'4', '5', '6'},
  {'7', '8', '9'},
  {'*', '0', '#'}
};

//setup the row and column pinouts
byte rowPins[rows] = {26, 27, 28, 29};
byte colPins[cols] = {30, 31, 32};

//setup the keypad object
Keypad keypad = Keypad( makeKeymap(keys), rowPins, colPins, rows, cols);

// 0x27 is the address, 20 and 4 are the columns and rows respectively
LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27, 20, 4);

//variables used for timing in interrupts
unsigned long ms_from_strobe_start = 0;
unsigned long ms_previous_read_strobe = 0;
unsigned long redLED_interval = 250;
unsigned long ms_from_lcd_start = 0;
unsigned long ms_previous_read_lcd = 0;
unsigned long lcd_interval = 500;
unsigned long valve_interval = 5000;
unsigned long valve_ms_from_start = 0;
unsigned long valve_ms_previous_read = 0;

//variables used for the system to recognize each sensor's state. 0 being off. 1 being on.
int greenLED_state = 0;
int redLED_state = 0;
volatile int alarm_state = 0;
volatile int motion_sensor_1_state = 0;
volatile int flame_sensor_1_state = 0;
volatile int buzzer_1_state = 0;
int alarm_screen_state = 0;
int home_screen_state = 0;

//the column the cursor should start printing the password on the LCD
int password_curs = 10;

void setup() {
  lcd.init();  //initialize the lcd
  lcd.backlight();  //open the backlight 
  
  Serial.begin(9600);

  //set the pin modes for system sensors. INPUT or OUTPUT
  pinMode(buttonPin, INPUT);
  pinMode(buzzerPin, OUTPUT);
  pinMode(motionSensor, INPUT);
  pinMode(greenLED, OUTPUT);
  pinMode(redLED, OUTPUT);
  pinMode(valvePin, OUTPUT);

  //attach ISR's to the systems sensors
  attachInterrupt(digitalPinToInterrupt(buttonPin), buttonISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(motionSensor), motion_sensor_1_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(flameSensor), flame_sensor_1_ISR, RISING);

  //start with the green LED on
  digitalWrite(greenLED, HIGH);

  //tell the keypad to listen for events
  keypad.addEventListener(keypadEvent);

  //call the display home screen function
  displayHomeScreen();
}

void loop() {

  //the system begins in this state by default
  if(alarm_state == 0){ //no alarm state
    displayHomeScreen();
    greenLED_state = 1;
    redLED_state = 0;
    motion_sensor_1_state = 0;
    flame_sensor_1_state = 0;
    buzzer_1_state = 0;
    digitalWrite(greenLED, greenLED_state);   
    digitalWrite(redLED, redLED_state);       
    noTone(buzzerPin);
    digitalWrite(valvePin, LOW);
  }
  //an ISR from one of the sensors will put the system into the alarm state
  else{ //turns on buzzer alarm and red strobe LED
    displayAlarmScreen();   //displays the alarm state text onto the LCD
    buzzer_1_state = 1;
    tone(buzzerPin, 500);
    redLEDStrobe(); //strobes the red LED
  }
  keypad.getKey(); //passive function looking for a pressed key on the keypad...only useful when the alarm is on.
}

void redLEDStrobe() { //Flashes redLED
  digitalWrite(greenLED, LOW);
  ms_from_strobe_start = millis();  
  if (ms_from_strobe_start - ms_previous_read_strobe > redLED_interval) {  //this IF statement is used to create a strobe interval
    ms_previous_read_strobe = millis();
    if (redLED_state == 0) {
      redLED_state = 1;
    }
    else {
      redLED_state = 0;
    }
    digitalWrite(redLED, redLED_state);
  }
}

void buttonISR() { //PushButton ISR changes alarm state to On or Off
  if (alarm_state == 0){
    alarm_state = 1;
  }
  else {
    alarm_state = 0;
  }
}

void motion_sensor_1_ISR(){ //triggered on motion, raises alarm
  motion_sensor_1_state = 1;
  Serial.println("Motion Sensor Triggered");
  if (alarm_state == 0){
    alarm_state = 1;
  }
}

void flame_sensor_1_ISR() { //triggered on flame, raises alarm and dispenses water
      flame_sensor_1_state = 1;
      alarm_state = 1;   
      digitalWrite(valvePin, HIGH);  //opens valve
  }

void displayHomeScreen(){  //displays the home screen text "Hello, USer. System Active"

  if (home_screen_state == 0)
      {
      lcd.clear();
      lcd.setCursor ( 0, 0 );            
      lcd.print("Hello, User"); 
      lcd.setCursor ( 0, 1 );           
      lcd.print("System Active"); 
      home_screen_state = 1;
      alarm_screen_state = 0;
      }
    }

void displayAlarmScreen(){  //displays the alarm screen text "Password:    !!Alarm Triggered!!"
  if (alarm_screen_state == 0)
    {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Password: ");
    lcd.setCursor ( 0, 1 );            
    lcd.print("!!ALARM TRIGGERED!!"); 
    alarm_screen_state = 1;
    home_screen_state = 0;
    }
  }

void keypadEvent(KeypadEvent key){  //gets called when a keypad is pressed
  switch (keypad.getState()){
  case PRESSED: //when keypad is pressed, increment the cursor position on the LCD and continue to the next switch
  lcd.setCursor((password_curs++),0);
  switch (key){
    case '#':    //if the '#' symbol is pressed, check if the entered password is correct
    checkPassword();
    break;

    case '*':   //if the '*' symbole is pressed, reset the password attempt
    password.reset();
    break;

    default:    //if any other key is pressed, add the key to the password attempt and print a '*' in the current LCD cursor position
    password.append(key);
    lcd.print("*");
  }
  }
}

void checkPassword() { //function that is called to check if the entered password attempt is correct
  if (password.evaluate()){ //if password attempt is correct, turn off the alarm and reset the password attempt
    alarm_state = 0;
    password.reset();
  }
  else {
    password.reset(); //if password attempt is incorrect, reset the password attempt
  }
}
