const int buzzerPin = 12;
const int led1 = 0;
const int led2 = 1;

void setup() {
  pinMode(buzzerPin, OUTPUT);
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
}

void loop() {
  
  for (int freq = 400; freq <= 1000; freq += 10) {
    tone(buzzerPin, freq);
    delay(2);
  }
  for (int freq = 1000; freq >= 400; freq -= 10) {
    tone(buzzerPin, freq);
    delay(2);
  }

  digitalWrite(led1, HIGH);
  digitalWrite(led2, LOW);
  delay(1000);

  digitalWrite(led1, LOW);
  digitalWrite(led2, HIGH);
  delay(1000);
}
