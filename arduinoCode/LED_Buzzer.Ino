const int buzzerPin = 12;
const int led1 = 0;
const int led2 = 1;

unsigned long previousMillis = 0;
int currentLED = 0;

void setup() {
  pinMode(buzzerPin, OUTPUT);
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
}

void loop() {
  for (int freq = 500; freq <= 1200; freq += 5) {
    tone(buzzerPin, freq);
    delay(2);
    updateLights();
  }

  for (int freq = 1200; freq >= 500; freq -= 5) {
    tone(buzzerPin, freq);
    delay(2);
    updateLights();
  }
}

void updateLights() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= 1000) {
    previousMillis = currentMillis;
    currentLED = !currentLED;

    if (currentLED == 0) {
      digitalWrite(led1, HIGH);
      digitalWrite(led2, LOW);
    } else {
      digitalWrite(led1, LOW);
      digitalWrite(led2, HIGH);
    }
  }
}
