//连线方法
//MPU-UNO
//VCC-5V
//GND-GND
//SCL-A5
//SDA-A4
//ADO-GND
//未使用中断功能，即没有做 INT-digital pin 2 (interrupt pin 0) 这样的接线
 
//参考手册：MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2
 
 
#include <Wire.h>
#include <Arduino.h>
#include <SPI.h>
#include <U8x8lib.h>
#include "MPU6050.h"
//#include <BLEDevice.h>
//byte comdata;

//--------------------------------->> the OLED used输出口
U8X8_SSD1306_128X64_NONAME_SW_I2C u8x8(/* clock=*/ 15, /* data=*/ 4, /* reset=*/ 16);


//---------------------------------->> // 定义为全局变量，可直接在函数内部使用
long accelX, accelY, accelZ;     
float gForceX, gForceY, gForceZ;
MPU6050 accelgyro;
long gyroX, gyroY, gyroZ;
float rotX, rotY, rotZ;
int sum;
int count=0;//总的步数
char countStep[30];
char distance[30];
char tempStep[30];
double t1;
double t0;
double t;
int axoffs,ayoffs,azoffs;
int16_t ax, ay, az;
int16_t gx, gy, gz;
float distan=0.0;
float A_Total ,a_abs,a;
float threshold = 6;





void setup() {
  u8x8.begin();//打开oled显示
  Serial.begin(115200);
  Wire.begin();
  setupMPU();
  getoffs();
}
 
void loop() {
            recordAccelRegisters();
            recordGyroRegisters();
            jisuan();
//            Serial.println(count);
//            u8x8.setFont(u8x8_font_chroma48medium8_r);
//            sprintf(countStep, "%d",count); //产生数字
//            sprintf(distance, "%.2f",distan); //产生数字
//              u8x8.drawString(0, 0, "STEP:");
//              u8x8.drawString(5, 0, countStep);
//              u8x8.drawString(0, 1, "distan:");
//              u8x8.drawString(8, 1, distance);
//            delay(100);
           measure_temperature();

           //--------------->>新增加
//           while (Serial.available() > 0)  
//           {
//           comdata = Serial.read();
//           delay(2);
//           Serial.write(comdata);
//           }
}

//------------------------>>已经有了
void setupMPU(){
  // REGISTER 0x6B/REGISTER 107:Power Management 1
  Wire.beginTransmission(0b1101000); //This is the I2C address of the MPU (b1101000/b1101001 for AC0 low/high datasheet Sec. 9.2)
  Wire.write(0x6B); //Accessing the register 6B/107 - Power Management (Sec. 4.30) 
  Wire.write(0b00000000); //Setting SLEEP register to 0, using the internal 8 Mhz oscillator
  Wire.endTransmission();
 
  // REGISTER 0x1b/REGISTER 27:Gyroscope Configuration
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x1B); //Accessing the register 1B - Gyroscope Configuration (Sec. 4.4) 
  Wire.write(0x00000000); //Setting the gyro to full scale +/- 250deg./s (转化为rpm:250/360 * 60 = 41.67rpm) 最高可以转化为2000deg./s 
  Wire.endTransmission();
  
  // REGISTER 0x1C/REGISTER 28:ACCELEROMETER CONFIGURATION
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x1C); //Accessing the register 1C - Acccelerometer Configuration (Sec. 4.5) 
  Wire.write(0b00000000); //Setting the accel to +/- 2g（if choose +/- 16g，the value would be 0b00011000）
  Wire.endTransmission(); 
}


//--------------->>已经有了
void recordAccelRegisters() {
  // REGISTER 0x3B~0x40/REGISTER 59~64
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x3B); //Starting register for Accel Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); //Request Accel Registers (3B - 40)
 
  // 使用了左移<<和位运算|。Wire.read()一次读取1bytes，并在下一次调用时自动读取下一个地址的数据
  while(Wire.available() < 6);  // Waiting for all the 6 bytes data to be sent from the slave machine （必须等待所有数据存储到缓冲区后才能读取） 
  accelX = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX （自动存储为定义的long型值）
  accelY = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
  accelZ = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ
  zhi();
}
 
void recordGyroRegisters() {
  // REGISTER 0x43~0x48/REGISTER 67~72
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x43); //Starting register for Gyro Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); //Request Gyro Registers (43 ~ 48)
  while(Wire.available() < 6);
  gyroX = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX
  gyroY = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
  gyroZ = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ
  processGyroData();
}
 
void processGyroData() {
  rotX = gyroX / 131.0;
  rotY = gyroY / 131.0; 
  rotZ = gyroZ / 131.0;
}

//------------------------->>计算加速度
void jisuan()//计算
{
      sum = (short)sqrt(gForceX * gForceX + gForceY * gForceY + gForceZ * gForceZ);//计算三维空间的加速度总和
      Serial.print("sum");
      Serial.println(sum);
      if(sum>8)
      {
      count++;
      sum=0;
      distan+=0.4524;
      };
      A_Total = (short)sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ);//计算三维空间的加速度总和
      a = (float)A_Total / 16384 * 10;
      a_abs = (float)sqrt(a*a);   
//      Serial.print("a_abs");
//      Serial.println(a_abs);
//      Serial.print("distan");
//      Serial.println(distan);  

//      Serial.println(count);
      u8x8.setFont(u8x8_font_chroma48medium8_r);
      sprintf(countStep, "%d",count); //产生数字
      sprintf(distance, "%f",distan); //产生数字
      u8x8.drawString(0, 0, "STEP:");
      u8x8.drawString(5, 0, countStep);
      u8x8.drawString(0, 1, "distan:");
      u8x8.drawString(8, 1, distance);
      delay(1000);
}
//---------------------------取值，
void zhi()
{
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    t0=t1;
    t1=micros();
    t=(t1-t0)/1000000;
    ax=ax+axoffs;
    ay=ay+ayoffs;
    az=az+azoffs;
    accelX=ax;
    accelY=ay;
    accelZ=az;
//    Serial.print(" accelX:");
//    Serial.print(accelX);
//    Serial.print(" accelY:");
//    Serial.print(accelY);
//    Serial.print(" accelZ:");
//    Serial.println(accelZ);
    gForceX=(accelX/16384)*9.80;
    gForceY=(accelY/16384)*9.80;
    gForceZ=(accelZ/16384)*9.80;
  
  }
void getoffs()//
{
      int16_t ax, ay, az;
      int16_t gx, gy, gz;
      long int axsum=0;
      long int aysum=0;
      long int azsum=0;
      int i;
          for(i=1;i<=2000;i++)
          {
              accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
              axsum=ax+axsum;
              aysum=ay+aysum;
              azsum=az+azsum;
              //azsum=az+azsum-16384;
          }
      axoffs=-axsum/2000;
      ayoffs=-aysum/2000;
      azoffs=-azsum/2000;
}


 


//--------------------->>测量温度
double measure_temperature(){
      //--------------->>温度初始化
      int va;
      double temp;
      double true_temp;
      float vat;
      double av= -1.064200E-09;
      double b= -5.759725E-06;
      double c= -1.789883E-01;
      double d=  2.048570E+02;
      double one_temp,two_temp;
      va = analogRead(0);  //读取模拟量
      va -=2900;
      vat=va*(3.3/4095)*1000;//3.3v为参考电压，4095为12位分辨率，*1000单位转换为mv
      temp=av*(vat)*(vat)*(vat)+b*(vat)*(vat)+c*(vat)+d;
      true_temp=temp;
     // true_temp=sqrt(true_temp*true_temp);
      true_temp=abs(true_temp);
//      one_temp=av*(val)*(val)*(val)+b*(val)*(val)+c*(val)+d;
//      delayMicroseconds(1200);
//      two_temp=av*(val)*(val)*(val)+b*(val)*(val)+c*(val)+d;

//       if(true_temp<24.0){
//        true_temp+=16.0;
//       }
//       float t_temp;
//       switch(true_temp){
//       case:true_temp<24  t_temp=24.253;
//       break;
//       case:true_temp>37  t_temp=36.945;
//        
//       }
       //在oled屏上显示温度
       sprintf(tempStep, "%.2f",true_temp); //产生数字
       u8x8.drawString(0, 3, "temp:");
       u8x8.drawString(5, 3, tempStep);

       Serial.print(" va:");
       Serial.println(va);
       Serial.print(" true_temp:");
       Serial.println(true_temp);
}
