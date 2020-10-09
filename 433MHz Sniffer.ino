#define DATA 3

bool curVal = 0;
bool prev1Val = 0;
bool prev2Val = 0;
bool prev3Val = 0;
bool datum = 0;

void setup(){
  pinMode(DATA, INPUT);
  Serial.begin(250000);
}

void loop(){
  curVal = digitalRead(DATA);
  
  datum = prev2Val;
  
  if((prev1Val==0) and (prev2Val==1) and (prev3Val==0)){
    datum = 0;
  }
  
  Serial.println(datum);
  prev3Val = prev2Val;
  prev2Val = prev1Val;
  prev1Val = curVal;
}   
