
const int buttonPin = 52;
const int buzzerPin = 8;
const int ledPin;
double currentTime;
double lastTime;


void setup() {
  
  Serial.begin(9600);
  pinMode(buttonPin, INPUT);
  pinMode(buzzerPin, OUTPUT);
  
}

void loop() {
  checkButtonState();
  
  
}

void checkButtonState(){
  if(digitalRead(buttonPin) == LOW){
    Serial.println("Button Pushed");
    buzzerOn();
    //tone(buzzerPin, 500, 2000);
    }
  }
 

void buzzerOn(){
  tone(buzzerPin, 500, 2000);
  Serial.println("Buzzer has started");
}
