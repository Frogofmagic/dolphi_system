
#include "mpu.h"
#include "I2Cdev.h"

int ret;
unsigned long time_last;
void setup() {
    Fastwire::setup(800,0);
    Serial.begin(38400);
    ret = mympu_open(200);
    Serial.print("MPU init: "); 
    Serial.println(ret);
    time_last = millis();	
}

unsigned int c = 0; //cumulative number of successful MPU/DMP reads
unsigned int np = 0; //cumulative number of MPU/DMP reads that brought no packet back
unsigned int err_c = 0; //cumulative number of MPU/DMP reads that brought corrupted packet
unsigned int err_o = 0; //cumulative number of MPU/DMP reads that had overflow bit set

void loop() {
    unsigned long time_now;
    time_now = millis();
    Serial.print("Time_now: ");
    Serial.println(time_now); 
    
    ret = mympu_update();  //run once need 9~11ms.
    
    time_now = millis();
    Serial.print("Time_after update: ");
    Serial.println(time_now); 

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


