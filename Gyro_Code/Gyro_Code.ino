#include <Wire.h>

//Set I2C address
const int MPU = 0x68;

//Set perameters for gyro           
float AccX, AccY, AccZ;                                                              //accelerometer values in all axis
float GyroX, GyroY, GyroZ;                                                           //Gyoroscope values in all axis
float accAngleX, accAngleY, accAngle, gyroAngleX, gyroAngleY, gyroAngleZ;            //Angle value in all axis
float roll, pitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;       //Accelerometer and Gyroscope error in all axis
float elapsedTime, currentTime, previousTime;
int d = 0;                                                            //Number of data sets


void setup() {

  Serial.begin(19200);            //19200baud
  Wire.begin();                   //Initialise comms
  Wire.beginTransmission(MPU);    //Begin I2C comms with MPU6050 at address 0x68
  Wire.write(0x6B);               //Talk to MPU power management register
  Wire.write(0x00);               //Reset PM register @0x6B
  Wire.endTransmission(true);     //stop comms @0x68

  //calculate_IMU_error();
  delay(20);
}


void loop() {
  //Reading accelerometer data
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);                //Write to accelerometer output register
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);  //Read 6 regusters, values for one axis are stored in 2 registers

  //Divide raw values by 16384 to get +-2g range, according to datasheet
  AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
  AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
  AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value

  //Calculate roll and pitch from acc data, with the complimentary filter data errors included
  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) + 1.95;
  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) + 4.74;

  //Reading gyroscope data
  previousTime = currentTime;       //Previous time stored beofre the actual time is read
  currentTime = millis();           //Current time reading in milliseconds
  elapsedTime = (currentTime - previousTime) / 1000;   //the elapsed time is calculated and divided by 1000, converting it into seconds

  Wire.beginTransmission(MPU);
  Wire.write(0x43);                 //write to gyroscope output data register address
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);   //read registers

  //Raw values are divided by 131.0 according to datasheet to produce range of 250deg/s
  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;

  //Output corrected
  GyroX = GyroX + 1.39;
  GyroY = GyroY + 0.23;
  GyroZ = GyroZ - 0.42;

  //Raw vlues converted to degrees
  gyroAngleX = gyroAngleX + GyroX * elapsedTime;
  gyroAngleY = gyroAngleY + GyroY * elapsedTime;
  yaw =  yaw + GyroZ * elapsedTime;

  //Complimentary filter
  roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
  pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;

  //Output data to serial monitor
  Serial.print(roll);
  Serial.print("/");
  Serial.print(pitch);
  Serial.print("/");
  Serial.println(yaw);

}

void calculate_IMU_error() {
  while (d < 250) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);               //Write to accelerometer output register
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true); //Read 6 regusters, values for one axis are stored in 2 registers

    //Divide raw values by 16384 to get +-2g range, according to datasheet
    AccX = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccY = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0 ;

    //Sum readings
    AccErrorX = AccErrorX + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
    AccErrorY = AccErrorY + ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));
    d++;
  }

  //Divide sum by 250 to get the error value
  AccErrorX = AccErrorX / 250;
  AccErrorY = AccErrorY / 250;
  d = 0;

  // Read gyro values 250 times
  while (d < 250) {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);               //Write to gyroscope output register
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    GyroX = Wire.read() << 8 | Wire.read();
    GyroY = Wire.read() << 8 | Wire.read();
    GyroZ = Wire.read() << 8 | Wire.read();

    //Sum all readings (for a 250deg/s range, raw value is divided by 131.0 according to the datasheet
    GyroErrorX = GyroErrorX + (GyroX / 131.0);
    GyroErrorY = GyroErrorY + (GyroY / 131.0);
    GyroErrorZ = GyroErrorZ + (GyroZ / 131.0);
    d++;
  }

  //Divide the sum by 250 to get the error value
  GyroErrorX = GyroErrorX / 250;
  GyroErrorY = GyroErrorY / 250;
  GyroErrorZ = GyroErrorZ / 250;

  // Print the error values on the Serial Monitor
  Serial.print("AccErrorX: ");
  Serial.println(AccErrorX);
  Serial.print("AccErrorY: ");
  Serial.println(AccErrorY);
  Serial.print("GyroErrorX: ");
  Serial.println(GyroErrorX);
  Serial.print("GyroErrorY: ");
  Serial.println(GyroErrorY);
  Serial.print("GyroErrorZ: ");
  Serial.println(GyroErrorZ);
}
