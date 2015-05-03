//Project Name : Dolphi System with Proto-Saber Evolution
//Start from 2015/04 by Frogofmagic.

//========================================================================================================//
#include <SoftwareSerial.h>
#include "mpu.h"
#include "I2Cdev.h"
//========================================================================================================//
#define SYSTEM_DEBUGMODE    1
//========================================================================================================//

int iMpuUpdateStatus;
int iSystem_STATUS = 0;
unsigned long time_last;



SoftwareSerial mySerial(10, 9); // RX, TX

void setup() {
    Fastwire::setup(800,0);                //Initial I2C
    Serial.begin(115200);                  //Initial UART to PC Baud Rate 115200
    mySerial.begin(115200);                //Initial UART to BLE Baud Rate 115200
    iMpuUpdateStatus = mympu_open(200);    //Initial mpu and read status.
    
    if(SYSTEM_DEBUGMODE){                  //Show mpu initial result.
        Serial.print("MPU init: "); 
        Serial.println(iMpuUpdateStatus);
    }	
}

/*
unsigned int c = 0; //cumulative number of successful MPU/DMP reads
unsigned int np = 0; //cumulative number of MPU/DMP reads that brought no packet back
unsigned int err_c = 0; //cumulative number of MPU/DMP reads that brought corrupted packet
unsigned int err_o = 0; //cumulative number of MPU/DMP reads that had overflow bit set
*/
int iReadfromBLE[4];

void loop() {
    
    iMpuUpdateStatus = mympu_update();                //run once need 9~11ms.
    if (mySerial.available()>=4){                     //check data from BLE
      
        for(int i=0; i<4; i++)                        //clear array first.
            iReadfromBLE[i] = 0; 
         
        for(int i=0; i<4; i++)                        //Read data from BLE
            iReadfromBLE[i] = mySerial.read(); 
            
        if(iReadfromBLE[0] == 0x41)    
            if(iReadfromBLE[1] == 0x54)
                if(iReadfromBLE[2] == 0x2B)
                    if(iReadfromBLE[3] == 0x01){
                        iMpuUpdateStatus = mympu_update();            //Read MPU data.
                        
                        for(int i=0; i<3; i++)                        //send ypr data
                            mySerial.write(mympu.ypr[i]); 
                        for(int i=0; i<3; i++)                        //send gyr data
                            mySerial.write(mympu.gyro[i]); 
                        
                        if(SYSTEM_DEBUGMODE){                         //Show mpu initial result.      
                            Serial.print(" Y: "); Serial.print(mympu.ypr[0]);
                            Serial.print(" P: "); Serial.print(mympu.ypr[1]);
                            Serial.print(" R: "); Serial.print(mympu.ypr[2]);
                            Serial.print("\tgy: "); Serial.print(mympu.gyro[0]);
                            Serial.print(" gp: "); Serial.print(mympu.gyro[1]);
                            Serial.print(" gr: "); Serial.println(mympu.gyro[2]);    
                        }     
                    }  

            
    }  
    /*
    time_now = millis();
    Serial.print("Time_after update: ");
    Serial.println(time_now); 
    */
    /*   
    //SHOW DATA        
    Serial.print(np); Serial.print("  "); Serial.print(err_c); Serial.print(" "); Serial.print(err_o);
    Serial.print(" Y: "); Serial.print(mympu.ypr[0]);
    Serial.print(" P: "); Serial.print(mympu.ypr[1]);
    Serial.print(" R: "); Serial.print(mympu.ypr[2]);
    Serial.print("\tgy: "); Serial.print(mympu.gyro[0]);
    Serial.print(" gp: "); Serial.print(mympu.gyro[1]);
    Serial.print(" gr: "); Serial.println(mympu.gyro[2]);
    */


}


