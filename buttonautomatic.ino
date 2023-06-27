#include <SoftwareSerial.h>
#include <LiquidCrystal.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
SoftwareSerial hc08(18, 19);  // TX: Pin 18 , TX: Pin 19

LiquidCrystal_I2C lcd(0x27, 16, 2); // Sesuaikan alamat I2C dengan alamat yang digunakan oleh LCD I2C Anda
//Address of the CMPS14 compass on i2c
#define _i2cAddress 0x60

#define CONTROL_Register 0

#define BEARING_Register 2
#define PITCH_Register 4
#define ROLL_Register 5
 
#define MAGNETX_Register 6
#define MAGNETY_Register 8
#define MAGNETZ_Register 10

#define ACCELEROX_Register 12
#define ACCELEROY_Register 14
#define ACCELEROZ_Register 16

#define GYROX_Register 18
#define GYROY_Register 20
#define GYROZ_Register 22

#define ONE_BYTE 1
#define TWO_BYTES 2
#define FOUR_BYTES 4
#define SIX_BYTES 6

#define choose_Satelite 50
#define Satelite1_Choosed 51
#define Satelite2_Choosed 52
#define Satelite3_Choosed 53
#define Satelite4_Choosed 54
#define SateliteHome_Choosed 55
#define SateliteWait_Choosed 56

//---------------------------------

byte _byteHigh;
byte _byteLow;


int bearing;
int nReceived;
signed char pitch;
signed char roll;

float magnetX = 0;
float magnetY = 0;
float magnetZ = 0;

float accelX = 0;
float accelY = 0;
float accelZ = 0;
// The acceleration along the X-axis, presented in mg
// See BNO080_Datasheet_v1.3 page 21
float accelScale = 9.80592991914f / 1000.f;  // 1 m/s^2

float gyroX = 0;
float gyroY = 0;
float gyroZ = 0;
// 16bit signed integer 32,768
// Max 2000 degrees per second - page 6
float gyroScale = 1.0f / 16.f;  // 1 Dps

int State_ChooseSatelite, State_FindPitch, State_FindRoll;
bool Status_FindPitch, Status_FindRoll, Status_PitchInPos, Status_RollInPos;
int SetPoint_Pitch, SetPoint_Roll, Calibrate_ValRoll, Calibrate_ValPitch;
bool Flag_Pitch, Flag_Roll;

//int Pitch_Satelite1, Pitch_Satelite2, Pitch_Satelite3, Roll_Satelite1, Roll_Satelite2, Roll_Satelite3;

int Pitch_Satelite1 = 40;
int Pitch_Satelite2 = 30;
int Pitch_Satelite3 = 35;
int Roll_Satelite1 = 40;
int Roll_Satelite2 = 45;
int Roll_Satelite3 = 35;

void setup() {
  Serial.begin(9600);
  hc08.begin(9600);

  // Mengatur HC-08 sebagai master
  hc08.write("AT+ROLE1\r\n");
  delay(500); // Tunggu respons dari HC-08

  // Initialize i2c
  Wire.begin();
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(11, OUTPUT);                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      
  pinMode(12, OUTPUT);
  pinMode(22, OUTPUT);
  pinMode(30, INPUT_PULLUP);
  pinMode(29, INPUT_PULLUP);
  pinMode(28, INPUT_PULLUP);
  pinMode(27, INPUT_PULLUP);
  pinMode(26, INPUT_PULLUP);
  pinMode(25, INPUT_PULLUP);
  pinMode(24, INPUT_PULLUP);
  pinMode(23, INPUT_PULLUP);

  State_ChooseSatelite = choose_Satelite;
  Calibrate_ValRoll = 7;
  State_FindPitch = 0;
  State_FindRoll = 0;

  lcd.begin(16, 2);  // Initialize the LCD
  lcd.backlight();  // Turn on the backlight
  lcd.setCursor(0, 0);  // Set the cursor to the first column, first row
  lcd.print("  Auto Pointing  ");  // Print the first line of text
  lcd.setCursor(0, 1);  // Set the cursor to the first column, second row
  lcd.print("  IoNLINK VSAT   ");  // Print the second line of text
}

void loop() {
  if (hc08.available()) {
    char data = hc08.read();
    Serial.print("Data yang diterima: ");  // Menampilkan data yang diterima dari HC-08 ke Serial Monitor
    Serial.println(data); // Menampilkan data di Serial Monitor
  }

  if (Serial.available()) {
    char data = Serial.read();
    hc08.write(data);  // Mengirim data dari Serial Monitor ke HC-08
  }

  // Read the Compass
  ReadCompass();

  // Read the Accelerator
  ReadAccelerator();

  // Read the Gyroscope
  ReadGyro();

  digitalWrite(4, 1);
  digitalWrite(5, 1);
  digitalWrite(8, 1);
  digitalWrite(11, 1);
  digitalWrite(12, 1);
  digitalWrite(22, 1);
  //analogWrite(2,0);
  //analogWrite(3,0);
  //analogWrite(6,0);
  //analogWrite(7,0);
  //
  //delay(2000);

  if (digitalRead(30) == LOW) {
    analogWrite(2, 0);
    analogWrite(3, 80);
  } else if (digitalRead(29) == LOW) {
    analogWrite(2, 80);
    analogWrite(3, 0);

  } else {
    if (State_ChooseSatelite == choose_Satelite) {
      analogWrite(2, 0);
      analogWrite(3, 0);
    }
    //analogWrite(2, 0);
    //analogWrite(3, 0);
  }

  if (digitalRead(28) == LOW) {
    analogWrite(6, 0);
    analogWrite(7, 40);
  } else if (digitalRead(27) == LOW) {
    analogWrite(6, 40);
    analogWrite(7, 0);
  } else {
    if (State_ChooseSatelite == choose_Satelite) {
      analogWrite(6, 0);
      analogWrite(7, 0);
    }
  }
  /*
  if (digitalRead(26) == LOW) {
    analogWrite(9, 0);
    analogWrite(10, 75);
  }
  else if (digitalRead(25) == LOW){
    analogWrite(9, 75);
    analogWrite(10, 0);
  }
  else {
    analogWrite(9, 0);
    analogWrite(10, 0);
  }
*/
  // Print data to Serial Monitor window
  
  Serial.print("$CMP,");
  Serial.print(bearing);
  Serial.print(",");
  Serial.print(pitch);
  Serial.print(",");
  Serial.print(roll);
  Serial.print(" (Sudut Azimuth");
  Serial.print(" 80,73)");
  Serial.print(" (Sudut Elevasi");
  Serial.print(" 45,64)");
  //Serial.print(" (Sudut Azimuth");
  //Serial.print(" 35,72)");
  //Serial.print(" (Sudut Elevasi");
  //Serial.print(" 79,48)");
  //Serial.print(" (Sudut Azimuth");
  //Serial.print(" 1,63)");
  //Serial.print(" (Sudut Elevasi");
  //Serial.print(" 81,43)");
  Serial.print(" degrees,");
  Serial.print(",Satelite ");
  Serial.print(State_ChooseSatelite);
  Serial.print(",State Pitch ");
  Serial.print(State_FindPitch);
  Serial.print(",State Roll ");
  Serial.print(State_FindRoll);
  Serial.print(",Home ");
  Serial.print(digitalRead(23));
  Serial.print(",Sat1 ");
  Serial.print(digitalRead(24));
  Serial.print(",Sat2 ");
  Serial.print(digitalRead(25));
  Serial.print(",Sat3 ");
  Serial.print(digitalRead(26));
  Serial.print(",Pitch In Pos ");
  Serial.print(Status_PitchInPos);
  Serial.print(",Roll In Pos ");
  Serial.print(Status_RollInPos);
  Serial.print(",SetPoint Pitch ");
  Serial.print(SetPoint_Pitch);
  Serial.print(",SetPoint Roll ");
  Serial.println(SetPoint_Roll);
  
  // =============================================== Case Moving ============================================================

  

  switch (State_ChooseSatelite) {

    case choose_Satelite:

      if (digitalRead(23) == LOW) {
        //if (Status_PitchInPos) {
          State_ChooseSatelite = SateliteHome_Choosed;
          //State_FindPitch = 1;
          //break;
        //}
      } else if (digitalRead(24) == LOW) {
        //if (Status_PitchInPos) {
          State_ChooseSatelite = Satelite1_Choosed;
          //State_FindPitch = 1;
          //break;
        //}
      } else if (digitalRead(25) == LOW) {
        //if (Status_PitchInPos) {
          State_ChooseSatelite = Satelite2_Choosed;
          //State_FindPitch = 1;
          //break;
        //}
      } else if (digitalRead(26) == LOW) {
        //if (Status_PitchInPos) {
          State_ChooseSatelite = Satelite3_Choosed;
          //State_FindPitch = 1;
          //break;
        //}
      } else {
        //break; // No Action
        ;
      }
      break;

    case Satelite1_Choosed:
      SetPoint_Pitch = Pitch_Satelite1;
      SetPoint_Roll = Calibrate_ValRoll + Roll_Satelite1;
      //if (Status_PitchInPos) {
        State_FindPitch = 1;
        State_ChooseSatelite = SateliteWait_Choosed;
        //break;
      //}
      break;
    case Satelite2_Choosed:
      SetPoint_Pitch = Pitch_Satelite2;
      SetPoint_Roll = Calibrate_ValRoll + Roll_Satelite2;
      //if (Status_PitchInPos) {
        State_FindPitch = 1;
        State_ChooseSatelite = SateliteWait_Choosed;
        //break;
      //}
      break;
    case Satelite3_Choosed:
      SetPoint_Pitch = Pitch_Satelite3;
      SetPoint_Roll = Calibrate_ValRoll + Roll_Satelite3;
      //if (Status_PitchInPos) {
        State_FindPitch = 1;
        State_ChooseSatelite = SateliteWait_Choosed;
        //break;
      //}
      break;
    case Satelite4_Choosed:
      break;
    case SateliteHome_Choosed:
      SetPoint_Pitch = 0;
      SetPoint_Roll = Calibrate_ValRoll + 0;
      //if (Status_PitchInPos) {
        State_FindPitch = 1;
        State_ChooseSatelite = SateliteWait_Choosed;
        //break;
      //}
      break;
    case SateliteWait_Choosed:
      break;
  }

  if (pitch >= SetPoint_Pitch - 5 && pitch <= SetPoint_Pitch + 5) {
    Status_PitchInPos = true;
  } else {
    Status_PitchInPos = false;
  }
  if (bearing >= SetPoint_Roll - 3 && bearing <= SetPoint_Roll + 3) {
    Status_RollInPos = true;
  } else {
    Status_RollInPos = false;
  }

  switch (State_FindPitch) {
    case 0:
      Flag_Pitch = 0;
      //if(Flag_Pitch > 0){
      break;
      //}
      //break; // No Action
    case 1:
      if (Status_PitchInPos == false) {
        if (pitch < SetPoint_Pitch) {
          analogWrite(2, 0);
          analogWrite(3, 50);
          //break;
        } else if (pitch > SetPoint_Pitch) {
          analogWrite(2, 50);
          analogWrite(3, 0);
          //break;
        }
      } else if (Status_PitchInPos) {
        analogWrite(2, 0);
        analogWrite(3, 0);
        State_FindPitch = 2;
        //break;
      }
      break;
    case 2:
      State_FindRoll = 1;
      State_FindPitch = 0;
      //break;
  }

  switch (State_FindRoll) {
    case 0:
      //break; // No Action
      Flag_Roll = 0;
      //if(Flag_Roll > 0){
      break;
      //}
    case 1:
      if (Status_RollInPos == false) {
        if (bearing < SetPoint_Roll) {
          analogWrite(7, 0);
          analogWrite(6, 25);
          //break;
        } else if (bearing > SetPoint_Roll) {
          analogWrite(7, 25);
          analogWrite(6, 0);
          //break;
        }
      } else if (Status_RollInPos) {
        analogWrite(6, 0);
        analogWrite(7, 0);
        State_FindRoll = 2;
        //break;
      }
      break;
    case 2:
      State_ChooseSatelite = choose_Satelite;
      State_FindRoll = 0;
      //break;
  }







  // ========================================================================================================================

  /*Serial.print("\t$ACC,");
  Serial.print(accelX, 4);
  Serial.print(",");
  Serial.print(accelY, 4);
  Serial.print(",");
  Serial.print(accelZ, 4);
  Serial.print(" m/s^2,");

  Serial.print("\t$GYR,");
  Serial.print(gyroX, 4);
  Serial.print(",");
  Serial.print(gyroY, 4);
  Serial.print(",");
  Serial.print(gyroZ, 4);
  Serial.println(" degrees/s");*/
}

int16_t getBearing() {
  // Begin communication with CMPS14
  Wire.beginTransmission(_i2cAddress);

  // Tell register you want some data
  Wire.write(BEARING_Register);

  // End the transmission
  int nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem
  if (nackCatcher != 0) { return 0; }

  // Request 2 bytes from CMPS14
  nReceived = Wire.requestFrom(_i2cAddress, TWO_BYTES);

  // Something has gone wrong
  if (nReceived != TWO_BYTES) return 0;

  // Read the values
  _byteHigh = Wire.read();
  _byteLow = Wire.read();

  // Calculate full bearing
  bearing = ((_byteHigh << 8) + _byteLow) / 10;

  return bearing;
}

byte getPitch() {
  // Begin communication with CMPS14
  Wire.beginTransmission(_i2cAddress);

  // Tell register you want some data
  Wire.write(PITCH_Register);

  // End the transmission
  int nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem
  if (nackCatcher != 0) { return 0; }

  // Request 1 byte from CMPS14
  nReceived = Wire.requestFrom(_i2cAddress, ONE_BYTE);

  // Something has gone wrong
  if (nReceived != ONE_BYTE) return 0;

  // Read the values
  pitch = Wire.read();

  return pitch;
}

byte getRoll() {
  // Begin communication with CMPS14
  Wire.beginTransmission(_i2cAddress);

  // Tell register you want some data
  Wire.write(ROLL_Register);

  // End the transmission
  int nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem
  if (nackCatcher != 0) { return 0; }

  // Request 1 byte from CMPS14
  nReceived = Wire.requestFrom(_i2cAddress, ONE_BYTE);

  // Something has gone wrong
  if (nReceived != ONE_BYTE) return 0;

  // Read the values
  roll = Wire.read();

  return roll;
}

int16_t getgyroX() {
  // Begin communication with CMPS14
  Wire.beginTransmission(_i2cAddress);

  // Tell register you want some data
  Wire.write(GYROX_Register);

  // End the transmission
  int nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem
  if (nackCatcher != 0) { return 0; }

  // Request 2 bytes from CMPS14
  nReceived = Wire.requestFrom(_i2cAddress, TWO_BYTES);

  // Something has gone wrong
  if (nReceived != TWO_BYTES) return 0;

  // Read the values
  _byteHigh = Wire.read();
  _byteLow = Wire.read();

  // Calculate GryoX
  return ((_byteHigh << 8) + _byteLow);
}

int16_t getgyroY() {
  // Begin communication with CMPS14
  Wire.beginTransmission(_i2cAddress);

  // Tell register you want some data
  Wire.write(GYROY_Register);

  // End the transmission
  int nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem
  if (nackCatcher != 0) { return 0; }

  // Request 2 bytes from CMPS14
  nReceived = Wire.requestFrom(_i2cAddress, TWO_BYTES);

  // Something has gone wrong
  if (nReceived != TWO_BYTES) return 0;

  // Read the values
  _byteHigh = Wire.read();
  _byteLow = Wire.read();

  // Calculate GryoY
  return ((_byteHigh << 8) + _byteLow);
}

int16_t getgyroZ() {
  // Begin communication with CMPS14
  Wire.beginTransmission(_i2cAddress);

  // Tell register you want some data
  Wire.write(GYROZ_Register);

  // End the transmission
  int nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem
  if (nackCatcher != 0) { return 0; }

  // Request 2 bytes from CMPS14
  nReceived = Wire.requestFrom(_i2cAddress, TWO_BYTES);

  // Something has gone wrong
  if (nReceived != TWO_BYTES) return 0;

  // Read the values
  _byteHigh = Wire.read();
  _byteLow = Wire.read();

  // Calculate GryoZ
  return ((_byteHigh << 8) + _byteLow);
}

int16_t getAcceleroX() {
  // Begin communication with CMPS14
  Wire.beginTransmission(_i2cAddress);

  // Tell register you want some data
  Wire.write(ACCELEROX_Register);

  // End the transmission
  int nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem
  if (nackCatcher != 0) { return 0; }

  // Request 2 bytes from CMPS14
  nReceived = Wire.requestFrom(_i2cAddress, TWO_BYTES);

  // Something has gone wrong
  if (nReceived != TWO_BYTES) return 0;

  // Read the values
  _byteHigh = Wire.read();
  _byteLow = Wire.read();

  // Calculate Accelerometer
  return (((int16_t)_byteHigh << 8) + (int16_t)_byteLow);
}

int16_t getAcceleroY() {
  // Begin communication with CMPS14
  Wire.beginTransmission(_i2cAddress);

  // Tell register you want some data
  Wire.write(ACCELEROY_Register);

  // End the transmission
  int nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem
  if (nackCatcher != 0) { return 0; }

  // Request 2 bytes from CMPS14
  nReceived = Wire.requestFrom(_i2cAddress, TWO_BYTES);

  // Something has gone wrong
  if (nReceived != TWO_BYTES) return 0;

  // Read the values
  _byteHigh = Wire.read();
  _byteLow = Wire.read();

  // Calculate Accelerometer
  return (((int16_t)_byteHigh << 8) + (int16_t)_byteLow);
}

int16_t getAcceleroZ() {
  // Begin communication with CMPS14
  Wire.beginTransmission(_i2cAddress);

  // Tell register you want some data
  Wire.write(ACCELEROZ_Register);

  // End the transmission
  int nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem
  if (nackCatcher != 0) { return 0; }

  // Request 2 bytes from CMPS14
  nReceived = Wire.requestFrom(_i2cAddress, TWO_BYTES);

  // Something has gone wrong
  if (nReceived != TWO_BYTES) return 0;

  // Read the values
  _byteHigh = Wire.read();
  _byteLow = Wire.read();

  // Calculate Accelerometer
  return (((int16_t)_byteHigh << 8) + (int16_t)_byteLow);
}

int16_t getMagnetX() {
  // Begin communication with CMPS14
  Wire.beginTransmission(_i2cAddress);

  // Tell register you want some data
  Wire.write(MAGNETX_Register);

  // End the transmission
  int nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem
  if (nackCatcher != 0) { return 0; }

  // Request 2 bytes from CMPS14
  nReceived = Wire.requestFrom(_i2cAddress, TWO_BYTES);

  // Something has gone wrong
  if (nReceived != TWO_BYTES) return 0;

  // Read the values
  _byteHigh = Wire.read();
  _byteLow = Wire.read();

  // Calculate value
  return (((int16_t)_byteHigh << 8) + (int16_t)_byteLow);
}

int16_t getMagnetY() {
  // Begin communication with CMPS14
  Wire.beginTransmission(_i2cAddress);

  // Tell register you want some data
  Wire.write(MAGNETY_Register);

  // End the transmission
  int nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem
  if (nackCatcher != 0) { return 0; }

  // Request 2 bytes from CMPS14
  nReceived = Wire.requestFrom(_i2cAddress, TWO_BYTES);

  // Something has gone wrong
  if (nReceived != TWO_BYTES) return 0;

  // Read the values
  _byteHigh = Wire.read();
  _byteLow = Wire.read();

  // Calculate value
  return (((int16_t)_byteHigh << 8) + (int16_t)_byteLow);
}

int16_t getMagnetZ() {
  // Begin communication with CMPS14
  Wire.beginTransmission(_i2cAddress);

  // Tell register you want some data
  Wire.write(MAGNETZ_Register);

  // End the transmission
  int nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem
  if (nackCatcher != 0) { return 0; }

  // Request 2 bytes from CMPS14
  nReceived = Wire.requestFrom(_i2cAddress, TWO_BYTES);

  // Something has gone wrong
  if (nReceived != TWO_BYTES) return 0;

  // Read the values
  _byteHigh = Wire.read();
  _byteLow = Wire.read();

  // Calculate value
  return (((int16_t)_byteHigh << 8) + (int16_t)_byteLow);
}

void ReadCompass() {
  // Begin communication with CMPS14
  Wire.beginTransmission(_i2cAddress);

  // Tell register you want some data
  Wire.write(BEARING_Register);

  // End the transmission
  int nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem
  if (nackCatcher != 0) {
    bearing = 0;
    pitch = 0;
    roll = 0;
    return;
  }

  // Request 4 bytes from CMPS14
  nReceived = Wire.requestFrom(_i2cAddress, FOUR_BYTES);

  // Something has gone wrong
  if (nReceived != FOUR_BYTES) {
    bearing = 0;
    pitch = 0;
    roll = 0;
    return;
  }

  // Read the values
  _byteHigh = Wire.read();
  _byteLow = Wire.read();
  bearing = ((_byteHigh << 8) + _byteLow) / 10;

  // Read the values
  pitch = Wire.read();

  // Read the values
  roll = Wire.read();
}


void ReadAccelerator() {
  // Begin communication with CMPS14
  Wire.beginTransmission(_i2cAddress);

  // Tell register you want some data
  Wire.write(ACCELEROX_Register);

  // End the transmission
  int nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem
  if (nackCatcher != 0) {
    accelX = 0;
    accelY = 0;
    accelZ = 0;
    return;
  }

  // Request 6 bytes from CMPS14
  nReceived = Wire.requestFrom(_i2cAddress, SIX_BYTES);

  // Something has gone wrong
  if (nReceived != SIX_BYTES) {
    accelX = 0;
    accelY = 0;
    accelZ = 0;
    return;
  }

  // Read the values
  _byteHigh = Wire.read();
  _byteLow = Wire.read();
  accelX = (((int16_t)_byteHigh << 8) + (int16_t)_byteLow) * accelScale;

  // Read the values
  _byteHigh = Wire.read();
  _byteLow = Wire.read();
  accelY = (((int16_t)_byteHigh << 8) + (int16_t)_byteLow) * accelScale;

  // Read the values
  _byteHigh = Wire.read();
  _byteLow = Wire.read();
  accelZ = (((int16_t)_byteHigh << 8) + (int16_t)_byteLow) * accelScale;
}

void ReadGyro() {
  // Begin communication with CMPS14
  Wire.beginTransmission(_i2cAddress);

  // Tell register you want some data
  Wire.write(GYROX_Register);

  // End the transmission
  int nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem
  if (nackCatcher != 0) {
    gyroX = 0;
    gyroY = 0;
    gyroZ = 0;
    return;
  }

  // Request 6 bytes from CMPS14
  nReceived = Wire.requestFrom(_i2cAddress, SIX_BYTES);

  // Timed out so return
  if (nReceived != SIX_BYTES) {
    accelX = 0;
    accelY = 0;
    accelZ = 0;
    return;
  }

  // Read the values
  _byteHigh = Wire.read();
  _byteLow = Wire.read();
  gyroX = (((int16_t)_byteHigh << 8) + (int16_t)_byteLow) * gyroScale;

  // Read the values
  _byteHigh = Wire.read();
  _byteLow = Wire.read();
  gyroY = (((int16_t)_byteHigh << 8) + (int16_t)_byteLow) * gyroScale;

  // Read the values
  _byteHigh = Wire.read();
  _byteLow = Wire.read();
  gyroZ = (((int16_t)_byteHigh << 8) + (int16_t)_byteLow) * gyroScale;
}


void changeAddress(byte i2cAddress, byte newi2cAddress) {
  // Reset the address on the i2c network
  // Ensure that you have only this module connected on the i2c network
  // The 7 bit i2c address must end with a 0. (even numbers please)
  // For example changeAddress(0x60, 0x64)

  // Address 0x60, 1 long flash, 0 short flashes
  // Address 0x62, 1 long flash, 1 short flashes
  // Address 0x64, 1 long flash, 2 short flashes
  // Address 0x66, 1 long flash, 3 short flashes
  // Address 0x68, 1 long flash, 4 short flashes
  // Address 0x6A, 1 long flash, 5 short flashes
  // Address 0x6C, 1 long flash, 6 short flashes
  // Address 0x6E, 1 long flash, 7 short flashes

  // Begin communication
  Wire.beginTransmission(i2cAddress);
  Wire.write(CONTROL_Register);
  Wire.write(byte(0xA0));

  // End the transmission
  int nackCatcher = Wire.endTransmission();

  //Wait 100ms
  delay(100);

  // Begin communication
  Wire.beginTransmission(i2cAddress);
  Wire.write(CONTROL_Register);
  Wire.write(byte(0xAA));

  // End the transmission
  nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem
  if (nackCatcher != 0) { return; }

  //Wait 100ms
  delay(100);

  // Begin communication
  Wire.beginTransmission(i2cAddress);
  Wire.write(CONTROL_Register);
  Wire.write(byte(0xA5));

  // End the transmission
  nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem
  if (nackCatcher != 0) { return; }

  //Wait 100ms
  delay(100);

  // Begin communication
  Wire.beginTransmission(i2cAddress);
  Wire.write(CONTROL_Register);
  Wire.write(newi2cAddress);

  // End the transmission
  nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem
  if (nackCatcher != 0) { return; }
}