
#define buttonPin 2
#define buzzerPin 52
#define greenLED 50
#define redLED 51

unsigned long ms_from_start = 0;
unsigned long ms_previous_read_redLED = 0;
unsigned long redLED_interval = 250;
int greenLED_state = 0;
int redLED_state = 0;
volatile int alarm_state = 0;
void setup() {

  Serial.begin(9600);
  pinMode(buttonPin, INPUT);
  pinMode(buzzerPin, OUTPUT);

  pinMode(greenLED, OUTPUT);
  pinMode(redLED, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(buttonPin), buttonISR, FALLING);

  digitalWrite(greenLED, HIGH);
}

void loop() {

  if(alarm_state == 0){ //turns off alarm
    greenLED_state = 1;
    redLED_state = 0;
    digitalWrite(greenLED, greenLED_state);
    digitalWrite(redLED, redLED_state);
    noTone(buzzerPin);
  }
  else{ //turns on buzzer alarm and red strobe LED
    tone(buzzerPin, 500);
    redLEDAlert();
  }

}

void redLEDAlert() { //Flashes redLED
  digitalWrite(greenLED, LOW);
  ms_from_start = millis();
  if (ms_from_start - ms_previous_read_redLED > redLED_interval) {
    ms_previous_read_redLED = millis();
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
