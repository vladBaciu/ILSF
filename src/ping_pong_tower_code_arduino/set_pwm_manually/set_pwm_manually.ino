int ledPin = 9; // LED connected to digital pin 9

int val = 0; // variable to store the read value

char inputBuffer[4];
uint8_t number = 0;
void setup() {
   Serial.begin(9600);
   pinMode(ledPin, OUTPUT); // sets the pin as output
}

void loop()
{
  if (Serial.available() > 0) {

    Serial.readBytesUntil('\n', inputBuffer, 4);
    number = atoi(inputBuffer);
    Serial.print("PWM value: ");
    Serial.println(number);
    analogWrite(ledPin,number);
    number = 0;
    memset(inputBuffer,0x00,4);
  }
}
