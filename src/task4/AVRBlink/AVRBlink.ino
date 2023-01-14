int R = 6;
int G = 8;
int B = 7;

void setup() {
  pinMode(R , OUTPUT);
  pinMode(G , OUTPUT);
  pinMode(B , OUTPUT);  
}

void loop() {
  digitalWrite(R , LOW);
  digitalWrite(G , HIGH);
  digitalWrite(B , HIGH);
  delay(1000);
  
  digitalWrite(R , HIGH);
  digitalWrite(G , LOW);
  digitalWrite(B , HIGH);
  delay(1000);

  digitalWrite(R , HIGH);
  digitalWrite(G , HIGH);
  digitalWrite(B , LOW);
  delay(1000);
}
