#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

//--------Pin definitions for the TB6612FNG Motor Driver----
#define AIN1 4
#define BIN1 6
#define AIN2 3
#define BIN2 7
#define PWMA 9
#define PWMB 10
//------------------------------------------------------------

//--------Enter Line Details here---------
bool isBlackLine = 1;             //keep 1 in case of black line. In case of white line change this to 0
unsigned int lineThickness = 30;  //Enter line thickness in mm. Works best for thickness between 10 & 35
unsigned int numSensors = 7;      // Enter number of sensors as 7
//-----------------------------------------

int P, D, I, previousError, PIDvalue;
double error;
int lsp, rsp;
int lfSpeed = 100;
int currentSpeed = 20;
int sensorWeight[7] = { 4, 2, 1, 0, -1, -2, -4 };
int activeSensors;
float Kp = 0.15;
float Kd = 0.09;
float Ki = 0.001;

int onLine = 1;
int minValues[7], maxValues[7], threshold[7], sensorValue[7], sensorArray[7];

void setup() {
  sbi(ADCSRA, ADPS2);
  cbi(ADCSRA, ADPS1);
  cbi(ADCSRA, ADPS0);

  Serial.begin(9600);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(11, INPUT_PULLUP);
  pinMode(12, INPUT_PULLUP);
  pinMode(13, OUTPUT);

  pinMode(5, OUTPUT);
  digitalWrite(5, HIGH);
}

void loop() {
  while (digitalRead(11)) {}
  delay(1000);
  calibrate();
  while (digitalRead(12)) {}
  delay(1000);

  while (1) {
    readLine();
    if (currentSpeed < lfSpeed) currentSpeed++;
    if (onLine == 1) {
      linefollow();
      digitalWrite(13, HIGH);
    } else {
      digitalWrite(13, LOW);
      if (error > 0) {
        motor1run(-100);
        motor2run(lfSpeed);
      } else {
        motor1run(lfSpeed);
        motor2run(-100);
      }
    }
  }
}

void linefollow() {
  error = 0;
  activeSensors = 0;
  for (int i = 0; i < 7; i++) {
    error += sensorWeight[i] * sensorArray[i] * sensorValue[i];
    activeSensors += sensorArray[i];
  }
  error = error / activeSensors;

  P = error;
  I = I + error;
  D = error - previousError;

  PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
  previousError = error;

  lsp = currentSpeed - PIDvalue;
  rsp = currentSpeed + PIDvalue;

  lsp = constrain(lsp, 0, 255);
  rsp = constrain(rsp, 0, 255);
  
  motor1run(lsp);
  motor2run(rsp);
}

void calibrate() {
  for (int i = 0; i < 7; i++) {
    minValues[i] = analogRead(i);
    maxValues[i] = analogRead(i);
  }

  for (int i = 0; i < 18100; i++) {
    motor1run(50);
    motor2run(-50);

    for (int i = 0; i < 7; i++) {
      minValues[i] = min(minValues[i], analogRead(i));
      maxValues[i] = max(maxValues[i], analogRead(i));
    }
  }

  for (int i = 0; i < 7; i++) {
    threshold[i] = (minValues[i] + maxValues[i]) / 2;
    Serial.print(threshold[i]);
    Serial.print(" ");
  }
  Serial.println();

  motor1run(0);
  motor2run(0);
}

void readLine() {
  onLine = 0;
  for (int i = 0; i < 7; i++) {
    sensorValue[i] = map(analogRead(i), minValues[i], maxValues[i], isBlackLine ? 0 : 1000, isBlackLine ? 1000 : 0);
    sensorValue[i] = constrain(sensorValue[i], 0, 1000);
    sensorArray[i] = sensorValue[i] > 500;
    if (sensorArray[i]) onLine = 1;
  }
}

void motor1run(int motorSpeed) {
  motorSpeed = constrain(motorSpeed, -255, 255);
  digitalWrite(AIN1, motorSpeed > 0);
  digitalWrite(AIN2, motorSpeed < 0);
  analogWrite(PWMA, abs(motorSpeed));
}

void motor2run(int motorSpeed) {
  motorSpeed = constrain(motorSpeed, -255, 255);
  digitalWrite(BIN1, motorSpeed > 0);
  digitalWrite(BIN2, motorSpeed < 0);
  analogWrite(PWMB, abs(motorSpeed));
}
