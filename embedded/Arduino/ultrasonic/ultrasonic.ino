//Ultrasound plane game
//by Yoruk for Instructables

//send commments to Yoruk16_72 AT yahoo DOT fr

//20 07 14 : main code

/*

 Wiring : 
 Vcc -> to Arduino 5v
 Trig -> Arduino pin 3
 Echo -> Arduino pin 2
 GND -> to Arduino GND 
 
 */


//settings

int echoPin= 2;
int triggerPin= 3;
unsigned long pulsetime = 0;
unsigned distance =0;
unsigned OldDistance =0;

void setup (){
  pinMode (echoPin, INPUT);
  pinMode (triggerPin, OUTPUT);
  Serial.begin(9600);  
}

void loop(){

  //compute the distance. Thanks for the ready-made code examples !!
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(100);
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(100);
  digitalWrite(triggerPin, LOW);
  pulsetime = pulseIn(echoPin, HIGH);
  distance = pulsetime / 58;
  delay(10);


  //sent the value only if it's a new value
  if (OldDistance != distance) {

    Serial.println(distance); 

    OldDistance = distance;
  }

  delay(50);  // wait 0.1 s between each measure
} 






