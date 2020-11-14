



#define buttonPin 2
#define buzzerPin 52
#define greenLED 50
#define redLED 51
#define motionSensor 3
#define flameSensor 18
#include <LiquidCrystal_I2C.h>
#include <Keypad.h>

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
int greenLED_state = 0;
int redLED_state = 0;

volatile int alarm_state = 0;
volatile int motion_sensor_1_state = 0;
volatile int flame_sensor_1_state =0;

void setup() {
  lcd.init();  //initialize the lcd
  lcd.backlight();  //open the backlight 
  
  Serial.begin(9600);
  pinMode(buttonPin, INPUT);
  pinMode(buzzerPin, OUTPUT);
  pinMode(motionSensor, INPUT);
  pinMode(greenLED, OUTPUT);
  pinMode(redLED, OUTPUT);
  
  attachInterrupt(digitalPinToInterrupt(buttonPin), buttonISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(motionSensor), motion_sensor_1_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(flameSensor), flame_sensor_1_ISR, RISING);

  digitalWrite(greenLED, HIGH);
}

void loop() {

  
  if(alarm_state == 0){ //no alarm state
    displayHomeScreen();
    greenLED_state = 1;
    redLED_state = 0;
    motion_sensor_1_state = 0;
    flame_sensor_1_state =0;
    digitalWrite(greenLED, greenLED_state);
    digitalWrite(redLED, redLED_state);
    noTone(buzzerPin);
  }
  else{ //turns on buzzer alarm and red strobe LED
    displayAlarmScreen();
    tone(buzzerPin, 500);
    redLEDStrobe();
  }

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
  flame_sensor_1_state = 1;
  Serial.println("Flame Sensor Triggered");
  if (alarm_state == 0){
    alarm_state = 1;
  }
}

void displayHomeScreen(){
  ms_from_lcd_start = millis();
  if (ms_from_lcd_start - ms_previous_read_lcd > lcd_interval) {
    lcd.clear();
    ms_previous_read_lcd = millis();
    lcd.setCursor ( 0, 0 );            
    lcd.print("Hello, Mr. Hoverd"); 
    lcd.setCursor ( 0, 1 );           
    lcd.print("System Active"); 
    }
  }
  

void displayAlarmScreen(){
  ms_from_lcd_start = millis();
  if (ms_from_lcd_start - ms_previous_read_lcd > lcd_interval) {
    ms_previous_read_lcd = millis();
  lcd.clear();
  lcd.setCursor ( 0, 1 );            
  lcd.print("!!ALARM TRIGGERED!!"); 
  }
}
