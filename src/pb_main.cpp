#include <Arduino.h>
/*  
   Created by:    Wim Boone
   Project:       Drone flight computer
   Date Started:  24/08/2019
   Does:          -Monitoring battery status and current flow
                  -Managing temperature?
                  -Calculate estimated flight time and send data over I2C bus to FCU
 */
#ifdef TARGET_ATTINY84
#include <Wire.h>

#define A_battery  A7       //current sensor battery pack
#define A_m1 8              //current sensor motor 1
#define A_m2 A5             //current sensor motor 2
#define NTC A1              //temperature sensor for battery
#define MSFT_gate A0        //mosfet gate for switching 12v
#define vcc_pack A3         //voltage in pack
#define vcc_cell A2         //voltage in cell 

/*variables for current sensors*/
const float factor = 0.02;  //20mV/A is the factor
float voltage_a;            //calculated voltage from sensor
float battery_current;
int sensitivity = 66;       //66mV/A is the factor
int offsetVoltage = 2500;
double voltage_b = 0;       //calculated voltage from sensor
double m1_current = 0;      //calculated current from sensor
double voltage_c = 0;
double m2_current = 0;

int V_ntc;                  
float R1 = 10000;           //resistance of NTC
float logR2, R2, T, Tc;
float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;

float voltage_pack = 0;
int voltage_cell = 0;

void setup() {
  //Wire.begin();

}

void loop() {
  voltage_a =   (5.0 / 1023.0)* analogRead(A_battery);          //read the voltage from ACS758 current sensor
  voltage_a =  voltage_a - (5.0 * 0.5) + 0.007 ;                //0.007 is a value to make voltage zero when there is no current
  battery_current = voltage_a / factor;
                                                      
  voltage_b = (5000 / 1024.0) * analogRead(A_m1);               //calculate the voltage from ACS712 motor 1 
  m1_current = ((voltage_b - offsetVoltage) / sensitivity);

  voltage_c = (5000 / 1024.0) * analogRead(A_m1);               //calculate the voltage from ACS712 motor 2
  m2_current = ((voltage_c - offsetVoltage) / sensitivity);

  V_ntc = analogRead(NTC);
  R2 = R1 * (1023.0 / (float)V_ntc - 1.0);
  logR2 = log(R2);
  T = (1.0 / (c1 + c2*logR2 + c3*logR2*logR2*logR2));
  Tc = T - 273.15;

  voltage_cell = analogRead(vcc_cell);
  voltage_pack = analogRead(vcc_cell) * 9;                      //multiply reading by 9 because of voltage divider

  
}


#endif