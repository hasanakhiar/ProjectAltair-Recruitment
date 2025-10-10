const int lightThreshold = 78; //Using 25% as a Threshold 

void setup(){
	pinMode(A0, INPUT);
 	Serial.begin(9600); 
  	pinMode(13, OUTPUT);
  
}
void loop(){
  	int sensorValue = analogRead(A0);
  	Serial.println(sensorValue);
  
    if(sensorValue > lightThreshold){
		digitalWrite(13, HIGH);
    }
    else{
      digitalWrite(13, LOW);
    }
  	delay(1000);
}