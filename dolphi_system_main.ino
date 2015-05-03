#include <MsTimer2.h>

//Project Name : Dolphi System with Proto-Saber Evolution
//Start from 2015/04 by Frogofmagic.
//Pin define : 
//    D9  = UART-TX connect to BLE-RX
//    D10 = UART-RX connect to BLE-TX
//========================================================================================================//
#include <SoftwareSerial.h>
#include "mpu.h"
#include "I2Cdev.h"
//========================================================================================================//
#define SYSTEM_DEBUGMODE    1
#define pinDebug            2
//========================================================================================================//

int iMpuUpdateStatus;
int iSystem_STATUS = 0;
int iPackagecounter = 0;
unsigned long time_last;



SoftwareSerial mySerial(10, 9); // RX, TX

void setup() {
    Fastwire::setup(800,0);                  //Initial I2C
    
    Serial.begin(115200);                    //Initial UART to PC Baud Rate 115200
    mySerial.begin(115200);                  //Initial UART to BLE Baud Rate 115200
    
    iMpuUpdateStatus = mympu_open(200);      //Initial mpu and read status.

    if(SYSTEM_DEBUGMODE){                    //Show mpu initial result.
        pinMode(pinDebug, OUTPUT);           //Set D2(pinDebug) = output
        digitalWrite(pinDebug, LOW);         //Set D2  = Low
        Serial.print("MPU init: "); 
        Serial.println(iMpuUpdateStatus);
    }	
    
    MsTimer2::set(10, Timer2_INTERRUPT);     //10ms period
    MsTimer2::start();
}

/*
unsigned int c = 0; //cumulative number of successful MPU/DMP reads
unsigned int np = 0; //cumulative number of MPU/DMP reads that brought no packet back
unsigned int err_c = 0; //cumulative number of MPU/DMP reads that brought corrupted packet
unsigned int err_o = 0; //cumulative number of MPU/DMP reads that had overflow bit set
*/
int iReadfromBLE[4];

void loop() {
    /*
    if (mySerial.available()>=4){                     //check data from BLE
        digitalWrite(pinDebug, HIGH);       //Set D2  = HIGH
        
        for(int i=0; i<4; i++)                        //clear array first.
            iReadfromBLE[i] = 0; 
         
        for(int i=0; i<4; i++)                        //Read data from BLE
            iReadfromBLE[i] = mySerial.read(); 
        
        //=================0x01 check======================//
        if(iReadfromBLE[0] == 0x41)                   //Check CMD = "AT+"(0x41542B) 0x01
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
        //=================0x01 END======================//
       digitalWrite(pinDebug, LOW);        //Set D2  = Low  
    }  // END of check data from BLE
    */
    
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


void Timer2_INTERRUPT(){                //Read MPU Data every interrupt
    iPackagecounter++;
    iMpuUpdateStatus = mympu_update();  //run once need about 4ms. 
    
    mySerial.write(iPackagecounter);    //sned package number first.
    
    for(int i=0; i<3; i++)              //send ypr data to BLE
        mySerial.write(mympu.ypr[i]); 
    for(int i=0; i<3; i++)              //send gyr data
        mySerial.write(mympu.gyro[i]); 
    
    if(SYSTEM_DEBUGMODE){               //Show mpu data 
        Serial.print(" Y: "); Serial.print(mympu.ypr[0]);
        Serial.print(" P: "); Serial.print(mympu.ypr[1]);
        Serial.print(" R: "); Serial.print(mympu.ypr[2]);
        Serial.print("\tgy: "); Serial.print(mympu.gyro[0]);
        Serial.print(" gp: "); Serial.print(mympu.gyro[1]);
        Serial.print(" gr: "); Serial.println(mympu.gyro[2]);    
    }     
    
}
