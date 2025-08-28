int sensorPin = A2; //left: 963, right: 816

void setup() {

  Serial.begin(9600);

  pinMode(sensorPin, INPUT);

}

 

void loop() {

  int value = analogRead(sensorPin);

  Serial.println(value);

  delay(100);

}
