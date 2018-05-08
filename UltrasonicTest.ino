
#define TRIG_PIN_1 A4
#define ECHO_PIN_1 A5
#define TRIG_PIN_2 11
#define ECHO_PIN_2 10

void setup() {

  pinMode(TRIG_PIN_1, OUTPUT);
  pinMode(ECHO_PIN_1, INPUT);
  pinMode(TRIG_PIN_2, OUTPUT);
  pinMode(ECHO_PIN_2, INPUT);

  Serial.begin(9600);

}


#define OBJECT_NONE 0
#define OBJECT_LEFT 1
#define OBJECT_RIGHT 2
#define OBJECT_CLOSE 3
#define OBJECT_THRESHOLD_CM 50
#define OBJECT_THRESHOLD_CM_CLOSE 15

int detectObject() {
  digitalWrite(TRIG_PIN_1, LOW);
  //digitalWrite(TRIG_PIN_2, LOW);
  delayMicroseconds(5);
  digitalWrite(TRIG_PIN_1, HIGH);
  //digitalWrite(TRIG_PIN_2, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN_1, LOW);
 // digitalWrite(TRIG_PIN_2, LOW);

  pinMode(ECHO_PIN_1, INPUT);
  double distLeft = pulseIn(ECHO_PIN_1, HIGH) / 58.2;   // left
//  enes.print(distLeft);


  digitalWrite(TRIG_PIN_2, LOW);
  delayMicroseconds(5);
  digitalWrite(TRIG_PIN_2, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN_2, LOW);
   
  pinMode(ECHO_PIN_2, INPUT);
  double distRight = pulseIn(ECHO_PIN_2, HIGH) / 58.2;
//  enes.print("  ");
 // enes.println(dist2);
//  enes.print("  ");

Serial.print("left: ");
Serial.println(distLeft);
Serial.print("right: ");
Serial.println(distRight);

  if (distLeft < OBJECT_THRESHOLD_CM_CLOSE || distRight < OBJECT_THRESHOLD_CM_CLOSE) {
    return OBJECT_CLOSE;
  }
  if (distLeft < OBJECT_THRESHOLD_CM || distRight < OBJECT_THRESHOLD_CM) {
    if (distLeft < distRight) {
      return OBJECT_LEFT;
    } else {
      return OBJECT_RIGHT;
    }
  } else {
    return OBJECT_NONE;
  }
}

void loop() {
  Serial.println(detectObject());
  delay(500);
}
