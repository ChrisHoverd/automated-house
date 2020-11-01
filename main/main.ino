
#define buttonPin 2
#define buzzerPin 52
#define greenLED 50
#define redLED 51
#define motionSensor 3
#define flameSensor 18
#define tempSensor A0

unsigned long ms_from_strobe_start = 0;
unsigned long ms_previous_read_strobe = 0;
unsigned long redLED_interval = 250;
int greenLED_state = 0;
int redLED_state = 0;

volatile int alarm_state = 0;
volatile int motion_sensor_1_state = 0;
volatile int flame_sensor_1_state =0;

int tempInput = 0
double temp = 0;

void setup() {

  Serial.begin(9600);
  pinMode(buttonPin, INPUT);
  pinMode(buzzerPin, OUTPUT);
  pinMode(motionSensor, INPUT);
  pinMode(tempSensor, INPUT);
  pinMode(greenLED, OUTPUT);
  pinMode(redLED, OUTPUT);
  
  attachInterrupt(digitalPinToInterrupt(buttonPin), buttonISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(motionSensor), motion_sensor_1_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(flameSensor), flame_sensor_1_ISR, RISING);

  digitalWrite(greenLED, HIGH);
}

void loop() {

  tempInput = analogRead(tempSensor);
  if(alarm_state == 0){ //turns off alarm
    greenLED_state = 1;
    redLED_state = 0;
    motion_sensor_1_state = 0;
    digitalWrite(greenLED, greenLED_state);
    digitalWrite(redLED, redLED_state);
    noTone(buzzerPin);
  }
  else{ //turns on buzzer alarm and red strobe LED
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

void flame_sensor_1_ISR() {
  flame_sensor_1_state = 1;
  Serial.println("Flame Sensor Triggered");
  if (alarm_state == 0){
    alarm_state = 1;
  }
}
