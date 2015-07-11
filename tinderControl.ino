#include<Wire.h>

// Storage
int16_t Ac[] = {0,0,0},
        Gy[] = {0,0,0},
        Tmp = 0;

const int MPU[2] = {0x68,0x69};

float   GySNE[] = {0,0,0};

// History
// I only made one because by this time only one channel was
// necessary; feel free to double it up yo.
float   HistSNE[3][14] = {{0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.},
                          {0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.},
                          {0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.}};
int hist_i = 0;

int energy[] = {0,0,0};

float T[3][3] ={{ -0.5304,  -0.5673,   0.6299 },
               {  0.0590,   0.7165,   0.6950 },
               { -0.8457,   0.4058,  -0.3466 }};

void setup() {
  // Power setup
  pinMode(21, OUTPUT);
  digitalWrite(21, HIGH);
  pinMode(20, OUTPUT);
  digitalWrite(20, LOW);
  pinMode(17, OUTPUT);
  digitalWrite(17, HIGH);

  delay(500);

  // Serial.begin(9600);
  // while (!Serial) {
  //   // wait for serial port to connect
  // }
  Serial.println("Connected to TinderControl.");

  // Communicate with MPU 6050
  Wire.begin();
  initMPU6050(MPU[0]);
  Serial.println("Teensy connected to MPU6050.");  
}
void loop() {
  collect();

  if (energy[2] < 1000 && (energy[0] > 100 || energy[1] > 100)){
    if (energy[0] > 100 && energy[1] < 100) {
      tinder(false);
      while (energy[0] > 100 || energy[2] > 1000) {collect();delay(50);}
      clearHistory();
    }
    else if (energy[0] < 100 && energy[1] > 100) {
      tinder(true);
      while (energy[1] > 100 || energy[2] > 1000) {collect();delay(50);}
      clearHistory();
    }{}
  }
  delay(50);

}

void initMPU6050(int address) {
  Wire.beginTransmission(address);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  delay(10);
  Wire.beginTransmission(address);
  Wire.write(0x1B);
  Wire.write(B00001000);
  Wire.endTransmission(true);
}

void clearHistory() {
  for(int i=0; i<3; i++){
    for(int j=0; j<14; j++){
      HistSNE[i][j] = 0;
    }
  }
}

// int* apply3(T,v) {

// }

// void collect(float A[3][3]) {
void collect() {
  // Collect Data

  // Channel A
  Wire.beginTransmission(MPU[0]);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU[0],14,true);  // request a total of 14 registers
  Ac[0] =Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  Ac[1] =Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  Ac[2] =Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  Gy[0] =Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  Gy[1] =Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  Gy[2] =Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  for(int i=0; i<3; i++){
      Serial.print((int) Ac[i]);
      Serial.print(",");
  }
  for(int i=0; i<3; i++){
      Serial.print((int) Gy[i]);
      if(i!=2){Serial.print(",");}
      else{Serial.println("");}
  }

  GySNE[0] = - 0.0720*Gy[0] + 0.6697*Gy[1] - 0.7391*Gy[2];
  GySNE[1] = - 0.2095*Gy[0] - 0.7347*Gy[1] - 0.6453*Gy[2];
  GySNE[2] = - 0.9752*Gy[0] + 0.1084*Gy[1] + 0.1932*Gy[2];
  // GySNE[0] = A[0][0]*Gy[0] + A[0][1]*Gy[1] + A[0][2]*Gy[2];
  // GySNE[1] = A[1][0]*Gy[0] + A[1][1]*Gy[1] + A[1][2]*Gy[2];
  // GySNE[2] = A[2][0]*Gy[0] + A[2][1]*Gy[1] + A[2][2]*Gy[2];

  if (GySNE[0] == 0 || GySNE[1] == 0 || GySNE[2] == 0) {
    Serial.print("Restarting");
    initMPU6050(MPU[0]);
  }

  // Write to History as rolling buffer
  HistSNE[0][hist_i] = GySNE[0]/100;
  HistSNE[1][hist_i] = GySNE[1]/100;
  HistSNE[2][hist_i] = GySNE[2]/100;
  hist_i = (hist_i+1) % 14;

  // Calculate Energy of Window
  energy[0] = 0;
  for(int i=0; i<14; i++){ energy[0] += pow(HistSNE[0][i],2); }
  energy[1] = 0;
  for(int i=0; i<14; i++){ energy[1] += pow(HistSNE[1][i],2); }
  energy[2] = 0;
  for(int i=0; i<14; i++){ energy[2] += pow(HistSNE[2][i],2); }

  // Average energy per sample
  energy[0] = energy[0]/14;
  energy[1] = energy[1]/14;
  energy[2] = energy[2]/14;

  // print3(energy);
}

void print3(int* vals) {
  for(int i=0; i<3; i++){
      Serial.print((int) vals[i]);
      Serial.print(",");
  }
  Serial.println("");
}

void tinder(bool response) {
  for(int i=0; i<15; i++){
      Mouse.move(0,50);
      delay(10);
  }
  for(int i=0; i<15; i++){
      Mouse.move(-50,0);
      delay(10);
  }
  for(int i=0; i<4; i++){
      Mouse.move(50,0);
      delay(10);
  }
  // --
  if (response) {
    // Yes (Right)
    Mouse.move(50,0);
    delay(10);
  }
  else {
    // No (Left)
    Mouse.move(-30,0);
    delay(10);
  }
  // --
  for(int i=0; i<2; i++){
      Mouse.move(0,-50);
      delay(10);
  }
  // --
  Mouse.click();
  // --
  for(int i=0; i<3; i++){
      Mouse.move(0,60);
      delay(10);
  }
}