#include "mbed.h"
#include "accelerometer.h"
#include "gyro.h"

AnalogOut Aout(PA_4);
InterruptIn btnRecord(BUTTON1);
EventQueue queue(32 * EVENTS_EVENT_SIZE);
Thread t;
Accelerometer acc;
Gyro gyro;
double Accel[3]={0};
double Gyro[3]={0};
double  accAngleX=0;
double  accAngleY=0;
double elapsedTime=0;
double roll_avg, pitch_avg, yaw_avg;
double roll_sum, pitch_sum, yaw_sum;
double roll[11], pitch[11], yaw[11];
double gyroAngleX=0;
double gyroAngleY=0;
int counter=0;
int idR[32] = {0};
int indexR = 0;

void record(void) {
roll_sum = 0;
pitch_sum = 0;
yaw_sum = 0;
for(int i = 1; i < 11; i++){
  yaw[0] = 0;
  acc.GetAcceleromterSensor(Accel);
  acc.GetAcceleromterCalibratedData(Accel);
  //printf("Calibrated ACC= %f, %f, %f\n", Accel[0], Accel[1], Accel[2]);
  // Calculating Roll and Pitch from the accelerometer data
  accAngleX = (atan(Accel[1] / sqrt(Accel[0]*Accel[1] + Accel[2]*Accel[2])) * 180 / SENSOR_PI_DOUBLE);
  accAngleY = (atan(-1 * Accel[1] / sqrt(Accel[1]*Accel[1] + Accel[2]*Accel[2])) * 180 / SENSOR_PI_DOUBLE);
  gyro.GetGyroSensor(Gyro);
  gyro.GetGyroCalibratedData(Gyro);
  //printf("Calibrated Gyro= %f, %f, %f\n", Gyro[0], Gyro[1], Gyro[2]);
  elapsedTime=0.1; //100ms by thread sleep time
  // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
  gyroAngleX = gyroAngleX + Gyro[0] * elapsedTime; // deg/s * s = deg
  gyroAngleY = gyroAngleY + Gyro[1] * elapsedTime;
  yaw[i] =  yaw[(i-1)] + Gyro[2] * elapsedTime;
  // Complementary filter - combine acceleromter and gyro angle values
  //roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
  //pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;
  //Use Acc data only
  roll[i] = accAngleX;
  pitch[i] = accAngleY;
  ThisThread::sleep_for(10ms);
}
  for(int i = 1; i < 11; i++){
      roll_sum += roll[i];
      pitch_sum += pitch[i];
      yaw_sum += yaw[i];
  }
  roll_avg = roll_sum/10;
  pitch_avg = pitch_sum/10;
  yaw_avg = yaw_sum/10;
  printf("%f/%f/%f\n", roll_avg, pitch_avg, yaw_avg);
  //ThisThread::sleep_for(10ms);

}

void startRecord(void) {
  //printf("---start---\n");
  idR[indexR++] = queue.call_every(100ms, record);
  indexR = indexR % 32;

}

void stopRecord(void) {
  //printf("---stop---\n");
  for (auto &i : idR)
    queue.cancel(i);
}

int main() {
  t.start(callback(&queue, &EventQueue::dispatch_forever));
  btnRecord.fall(queue.event(startRecord));
  btnRecord.rise(queue.event(stopRecord));
}