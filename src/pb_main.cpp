#include <Arduino.h>
/*  
   Created by:    Wim Boone
   Project:       Drone flight computer
   Date Started:  24/08/2019
   Does:          -Monitoring battery status and current flow
                  -Calculate estimated flight time and send data over I2C bus to FCU
 */
#ifdef TARGET_ATTINY84
#include <Wire.h>

#define PWB_ADDR_IIC 1 // Powerboard address

#define A_TOT A7    //current sensor battery pack
#define A_M1 8      //current sensor motor 1
#define A_M2 A5     //current sensor motor 2
#define NTC A1      //temperature sensor for battery
#define FET_GATE A0 //mosfet gate for switching 12v
#define V_TOT A3    //voltage in pack
#define V_CELL A2   //voltage in cell

// float pointers
volatile byte *battery_amps_fptr;
volatile byte *motor1_amps_fptr;
volatile byte *motor2_amps_fptr;
volatile byte *battery_voltage_fptr;
volatile byte *cell_voltage_fptr;
volatile byte *battery_temp_fptr;

/* variables for current sensors */
const int FACTOR_ACS758 = 10; //10mV/A
const int FACTOR_ACS712 = 66; //66mV/A

float battery_amps;
float motor1_amps = 0; //calculated current from sensor
float motor2_amps = 0;

float R1 = 10000; //resistance of NTC
float logR2, R2, T, battery_temp;
float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;

float battery_voltage = 0;
float cell_voltage = 0;

void requestEvent();

void setup()
{
  Wire.begin(PWB_ADDR_IIC);
  Wire.setClock()
  Wire.onRequest(requestEvent); // register event
}

void loop()
{
  float voltage_a = (5000 / 1023) * analogRead(A_TOT); //read the voltage from ACS758 current sensor
  battery_amps = (voltage_a * 0.5 + 7) / FACTOR_ACS758; //7 is offset value

  float voltage_b = (5000 / 1023) * analogRead(A_M1); //calculate the voltage from ACS712 motor 1
  motor1_amps = (voltage_b * 0.5) / FACTOR_ACS712;

  float voltage_c = (5000 / 1023) * analogRead(A_M2); //calculate the voltage from ACS712 motor 2
  motor2_amps = (voltage_c * 0.5) / FACTOR_ACS712;

  int V_ntc = analogRead(NTC);
  R2 = R1 * (1023.0 / (float)V_ntc - 1.0);
  logR2 = log(R2);
  T = (1.0 / (c1 + c2 * logR2 + c3 * logR2 * logR2 * logR2));
  battery_temp = T - 273.15;

  cell_voltage = analogRead(V_CELL);
  battery_voltage = analogRead(V_TOT) * 9; //multiply reading by 9 because of voltage divider
}

void requestEvent()
{
  byte *Data;
  battery_amps_fptr = (byte *)&battery_amps;
  motor1_amps_fptr = (byte *)&motor1_amps;
  motor2_amps_fptr = (byte *)&motor2_amps;
  battery_voltage_fptr = (byte *)&battery_voltage;
  cell_voltage_fptr = (byte *)&cell_voltage;
  battery_temp_fptr = (byte *)&battery_temp;

  Data[0] = battery_amps_fptr[0];
  Data[1] = battery_amps_fptr[1];
  Data[2] = battery_amps_fptr[2];
  Data[3] = battery_amps_fptr[3];

  Data[4] = motor1_amps_fptr[0];
  Data[5] = motor1_amps_fptr[1];
  Data[6] = motor1_amps_fptr[2];
  Data[7] = motor1_amps_fptr[3];

  Data[8] = motor2_amps_fptr[0];
  Data[9] = motor2_amps_fptr[1];
  Data[10] = motor2_amps_fptr[2];
  Data[11] = motor2_amps_fptr[3];

  Data[12] = battery_voltage_fptr[0];
  Data[13] = battery_voltage_fptr[1];
  Data[14] = battery_voltage_fptr[2];
  Data[15] = battery_voltage_fptr[3];

  Data[16] = battery_temp_fptr[0];
  Data[17] = battery_temp_fptr[1];
  Data[18] = battery_temp_fptr[2];
  Data[19] = battery_temp_fptr[3];

  Data[20] = battery_temp_fptr[0];
  Data[21] = battery_temp_fptr[1];
  Data[22] = battery_temp_fptr[2];
  Data[23] = battery_temp_fptr[3];

  Wire.write(Data, 21);
}
#endif