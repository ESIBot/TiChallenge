/*Está sin probar
* Falta adaptar para minimizar el consumo
*/

#include <Wire.h>




int redLed = 10;
int greenLed = 11;
int blueLed = 12;
int batPin = A0;

int led=0;
int vBat=0;



void setup()
{
  Wire.begin(2);                // join i2c bus with address #2
  Wire.onReceive(receiveEvent); // register event
  Wire.onRequest(requestEvent); // register event    
  
  pinMode(redLed, OUTPUT);  
  pinMode(greenLed, OUTPUT);  
  pinMode(blueLed, OUTPUT);  
}

void loop()
{
  vBat = analogRead(sensorPin);    
      
  switch(led){
    case 1:
      digitalWrite(redLed, HIGH);  
      digitalWrite(greenLed, LOW);  
      digitalWrite(blueLed, LOW);  
    break;
  
    case 2:
      digitalWrite(redLed, LOW);  
      digitalWrite(greenLed, HIGH);  
      digitalWrite(blueLed, LOW); 
    break;
  
    case 3:
      digitalWrite(redLed, LOW);  
      digitalWrite(greenLed, LOW);  
      digitalWrite(blueLed, HIGH); 
    break;
  
    default:      
      digitalWrite(redLed, LOW);  
      digitalWrite(greenLed, LOW);  
      digitalWrite(blueLed, LOW); 
  }
 
  delay(1000);
}


void receiveEvent(int howMany)
{
  while(Wire.available()) // loop through all but the last
  {
  led = Wire.read();
  }
}

void requestEvent()
{
  Wire.write(vBat); 
}
