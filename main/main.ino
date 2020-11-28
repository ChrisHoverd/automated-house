

#define buttonPin 2
#define buzzerPin 23
#define greenLED 50
#define redLED 51
#define motionSensor 3
#define flameSensor 18
#define valvePin 8
#include <LiquidCrystal_I2C.h>
#include <Keypad.h>
#include <Password.h>


Password password = Password("1379");
const byte rows = 4;
const byte cols = 3;

char keys[rows][cols] = {
  {'1', '2', '3'},
  {'4', '5', '6'},
  {'7', '8', '9'},
  {'*', '0', '#'}
};

byte rowPins[rows] = {26, 27, 28, 29};
byte colPins[cols] = {30, 31, 32};
Keypad keypad = Keypad( makeKeymap(keys), rowPins, colPins, rows, cols);

LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27, 20, 4);

unsigned long ms_from_strobe_start = 0;
unsigned long ms_previous_read_strobe = 0;
unsigned long redLED_interval = 250;
unsigned long ms_from_lcd_start = 0;
unsigned long ms_previous_read_lcd = 0;
unsigned long lcd_interval = 500;
unsigned long valve_interval = 5000;
unsigned long valve_ms_from_start = 0;
unsigned long valve_ms_previous_read = 0;


int greenLED_state = 0;
int redLED_state = 0;
volatile int alarm_state = 0;
volatile int motion_sensor_1_state = 0;
volatile int flame_sensor_1_state = 0;
volatile int buzzer_1_state = 0;
int alarm_screen_state = 0;
int home_screen_state = 0;

int password_curs = 10;

void setup() {
  lcd.init();  //initialize the lcd
  lcd.backlight();  //open the backlight 
  
  Serial.begin(9600);
  pinMode(buttonPin, INPUT);
  pinMode(buzzerPin, OUTPUT);
  pinMode(motionSensor, INPUT);
  pinMode(greenLED, OUTPUT);
  pinMode(redLED, OUTPUT);
  pinMode(valvePin, OUTPUT);
  
  attachInterrupt(digitalPinToInterrupt(buttonPin), buttonISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(motionSensor), motion_sensor_1_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(flameSensor), flame_sensor_1_ISR, RISING);

  digitalWrite(greenLED, HIGH);
  keypad.addEventListener(keypadEvent);
  displayHomeScreen();
}

void loop() {

  
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
  else{ //turns on buzzer alarm and red strobe LED
    displayAlarmScreen();
    buzzer_1_state = 1;
    tone(buzzerPin, 500);
    redLEDStrobe();
  }
  keypad.getKey();
}

void redLEDStrobe() { //Flashes redLED
  digitalWrite(greenLED, LOW);
  ms_from_strobe_start = millis();
  if (ms_from_strobe_start - ms_previous_read_strobe > redLED_interval) {
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

void flame_sensor_1_ISR() { //triggered on flame, raises alarm
  
  //valve_ms_from_start = millis();
   // if (valve_ms_from_start - valve_ms_previous_read > valve_interval){ 
      flame_sensor_1_state = 1;
      alarm_state = 1;   
      
      digitalWrite(valvePin, HIGH);
      //valve_ms_previous_read = millis();
    //}
  }

void displayHomeScreen(){
  //if (home_screen_state === 1) && (alarm_screen_state == 0){
  //ms_from_lcd_start = millis();
  //if (ms_from_lcd_start - ms_previous_read_lcd > lcd_interval) {
  // ms_previous_read_lcd = millis();
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

void displayAlarmScreen(){
  //  ms_from_lcd_start = millis();
  //  if (ms_from_lcd_start - ms_previous_read_lcd > lcd_interval) {
  //  ms_previous_read_lcd = millis();
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

void keypadEvent(KeypadEvent key){
  switch (keypad.getState()){
  case PRESSED:
  lcd.setCursor((password_curs++),0);
  switch (key){
    case '#': 
    checkPassword();
    break;

    case '*': 
    password.reset();
    break;

    default: 
    password.append(key);
    lcd.print("*");
  }
  }
}

void checkPassword() {
  if (password.evaluate()){
    alarm_state = 0;
    password.reset();
  }
  else {
    password.reset();
  }
}
