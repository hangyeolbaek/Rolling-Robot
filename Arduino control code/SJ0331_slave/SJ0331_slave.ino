// SLAVE BOARD #1 ~ #11
#include <Wire.h>
#include <Kalman.h> // Source: https://github.com/TKJElectronics/KalmanFilter
#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf
Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;
double roll_x;
char my_IMU[7], IMU_c[12][7];
String my_IMU_s;
String IMU_1_s, IMU_2_s , IMU_3_s , IMU_4_s, IMU_5_s, IMU_6_s , IMU_7_s , IMU_8_s , IMU_9_s , IMU_10_s , IMU_11_s , IMU_12_s;
float IMU_1, IMU_2 , IMU_3 , IMU_4, IMU_5 , IMU_6 , IMU_7 , IMU_8 , IMU_9 , IMU_10 , IMU_11 , IMU_12;
/* IMU Data */
double accX, accY, accZ, gyroX, gyroY, gyroZ, gyroXangle, gyroYangle, compAngleX, compAngleY, kalAngleX, kalAngleY;
int16_t tempRaw;
uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data
// TODO: Make calibration routine
int sensorValue0, sensorValue1, sensorValue2, sensorValue3;
float sensorValue0_1, sensorValue1_1, sensorValue2_1, sensorValue3_1;

void read_IMU();
//void print_data();
//void receive_n_send();
void send2master();
void wirePrint();
void receiveEvent(int howMany);
void setup() {
  memset(IMU_c, 0, sizeof(IMU_c)); // clear IMU_c array with 0
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  Serial.begin(115200);
  Wire.begin(11); //###################### CHANGE BOARD NUMBER 1 ~ 11
  Wire.onRequest(send2master);// M이 데이터 요청하면 뱉는거 S 2 M 할때
  Wire.onReceive(receiveEvent);// M이 데이터 보내는거 있으면 M 2 S  (7bytes)
#if ARDUINO >= 157
  Wire.setClock(400000UL); // Set I2C frequency to 400kHz
#else
  TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
#endif
}


void loop() {


}

void receiveEvent(int howMany) { //전송 데이터 읽기
  IMU_1_s = Wire.readStringUntil('!');//receive each IMU data by String
  IMU_2_s = Wire.readStringUntil('@');
  IMU_3_s = Wire.readStringUntil('#');
  IMU_4_s = Wire.readStringUntil('$');
  IMU_5_s = Wire.readStringUntil('%');
  IMU_6_s = Wire.readStringUntil('^');
  IMU_7_s = Wire.readStringUntil('&');
  IMU_8_s = Wire.readStringUntil('*');
  IMU_9_s = Wire.readStringUntil('(');
  IMU_10_s = Wire.readStringUntil(')');
  IMU_11_s = Wire.readStringUntil('_');
  IMU_12_s = Wire.readStringUntil('+');

  IMU_1_s.toCharArray(IMU_c[0], 7); // String 2 char array
  IMU_2_s.toCharArray(IMU_c[1], 7);
  IMU_3_s.toCharArray(IMU_c[2], 7);
  IMU_4_s.toCharArray(IMU_c[3], 7);
  IMU_5_s.toCharArray(IMU_c[4], 7);
  IMU_6_s.toCharArray(IMU_c[5], 7);
  IMU_7_s.toCharArray(IMU_c[6], 7);
  IMU_8_s.toCharArray(IMU_c[7], 7);
  IMU_9_s.toCharArray(IMU_c[8], 7);
  IMU_10_s.toCharArray(IMU_c[9], 7);
  IMU_11_s.toCharArray(IMU_c[10], 7);
  IMU_12_s.toCharArray(IMU_c[11], 7);

  IMU_1 = atof(IMU_c[0]);  // change data to float (prepare to use by value)
  IMU_2 = atof(IMU_c[1]);
  IMU_3 = atof(IMU_c[2]);
  IMU_4 = atof(IMU_c[3]);
  IMU_5 = atof(IMU_c[4]);
  IMU_6 = atof(IMU_c[5]);
  IMU_7 = atof(IMU_c[6]);
  IMU_8 = atof(IMU_c[7]);
  IMU_9 = atof(IMU_c[8]);
  IMU_10 = atof(IMU_c[9]);
  IMU_11 = atof(IMU_c[10]);
  IMU_12 = atof(IMU_c[11]);
}

void send2master() {
  read_IMU();
  wirePrint();
}
void wirePrint() {

  my_IMU_s = (String)roll_x;
  my_IMU_s.toCharArray(my_IMU, 7);

  for (int i = 0; i <= 6; i++) {
    Wire.print(my_IMU[i]);
  }
}
void read_IMU() {
  digitalWrite(7, HIGH);
  delay(1);
  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode
  while (i2cRead(0x75, i2cData, 1));

  while (i2cRead(0x3B, i2cData, 14));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
  tempRaw = (int16_t)((i2cData[6] << 8) | i2cData[7]);
  gyroX = (int16_t)((i2cData[8] << 8) | i2cData[9]);
  gyroY = (int16_t)((i2cData[10] << 8) | i2cData[11]);
  gyroZ = (int16_t)((i2cData[12] << 8) | i2cData[13]);;
  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();
  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
#endif
  double gyroXrate = gyroX / 131.0; // Convert to deg/s
#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif
  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  roll_x = (kalAngleX * -1) + 180;

  digitalWrite(7, LOW);
}
