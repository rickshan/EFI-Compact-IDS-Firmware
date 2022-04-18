/*
 * Compact IDS Control Firmware
 * Rev 1.0
 * Guerin Rowland 9-9-2019
 * Rev 2.0
 * Guerin Rowland 10-1-2019
 * Added support for all I/O on prototype PCB
 * Rev 3.0
 * Guerin Rowland 3-24-2020
 * Added comments eveywhere
 * Cleaned up formatting
 * Removed unused fragments
 * Changed from 1 global firmware version to one for each system because
 * each system needs specific tunings for each head
 * Guerin Rowland 10/23/2020
 * Added median filtering and smoothing for circulation pump control to reject 
 * feed tank level reading glitches
 * Guerin Rowland 2/26/2021
 * Remove unused fragments
 * Add support for new devices on Josh Bailey redesigned system
 * Guerin Rowland 7/7/2021
 * Add bulk level check before running suupply pump 
 * Add shutoff when trap fills
 * Add foam sensor support
 * Retune pressure PIDs
 * Guerin Rowland 1/31/2022
 * Circulation Pump wont run until return is at 150mL full
 * Air pumps will run at a gentle "venting" vacuum until both reservoirs are at 150mL
 */

/************************Included Libraries*****************/
#include "Arduino.h"                    //Includes core arduino functions such as pinMode, Serial.print, etc.
#include "PID_v2.h"                     //PID Library for pump control, modified to use 12 bit 4095 count adcs
#include "HX711.h"                      //Load cell library for a chip that is similar enough it works for our ADS chips
#include "SPI.h"                        //SPI library for communicating with pressure sensors and RTD chips
#include "Adafruit_MAX31865.h"          //Library for RTD amplifier chips
#include "Adafruit_MAX31855.h"          //Library for thermocouple amplifier chips
#include "EEPROM.h"                     //EEPROM library so we can store variables in memory when the board loses power
#include "MCP4728.h"                    //DAC library for one of our DAC chips
#include "Adafruit_MCP4725.h"           //DAC library for our other DAC chip
#include "Wire.h"                       //I2C Library
#include "RunningMedian.h"              //what do you think ;)
#include "Adafruit_ADS1015.h"
/************************Pin Definitions**************************/
#define CLK 45                          //Clock pin for inlet load cell
#define CLK2 43                          //Clock pin for outlet load cell
#define CLK3 36      
#define CLK4 40      
#define DOUT1 44                         //Data out pin for inlet load cell
#define DOUT2 42                         //Data out pin for outlet load cell
#define DOUT3 37      
#define DOUT4 41       
#define degas 7                          //Pin to run degas pump
#define degasTransducer 19 
#define coreThermocouple 31              //SPI chipselect pin for heater element thermocouple reading
#define chillerThermocouple 32           //SPI chipselect pin for heater element thermocouple reading
#define spareThermocouple1 33            //SPI chipselect pin for heater element thermocouple reading
#define spareThermocouple2 34            //SPI chipselect pin for heater element thermocouple reading
#define supply 6                         //Supply pump pin
#define inRTD 47                         //SPI chipselect for inlet RTD
#define outRTD 46                        //SPI chipselect for outlet RTD
#define RREF 400.0                       //Reference resistor value for RTD amplifier
#define RNOMINAL 100.0                   //Nominal RTD value (Using PT100 RTDs)
#define OUTPUT_MIN 1638.4                // 1638 counts (10% of 2^14 counts or 0x0666), this is for pressure sensor reading
#define OUTPUT_MAX 14745.6               // 14745 counts (90% of 2^14 counts or 0x3999), this is for presure sensor reading
#define PRESSURE_MIN -29.5223            // min is 0 for sensors that give absolute values, negative for differential sensors
#define PRESSURE_MAX 29.5223             // 29.5223inhg we want results in inhg
#define inletFoam A10
#define outletFoam A9
#define degasFoam A8
#define inletLock 48
#define outletLock 49

/************************PID Parameters****************************/
int Kp=180, Ki=50, Kd=0;                //CIRCULATION PARAMETERS
int cPKp=60, cPKi=20, cPKd=0;           //Conservative inlet pressure
int cVKp=60, cVKi=20, cVKd=0;           //Conservative outlet pressure
int TKp=800, TKi=100, TKd=300;          //TEMPERATURE PARAMETERS
/***********************Variable Definitions********************/
float calibration_factor = 2161;        //calibration factor for load cells. This number gives an accurate reading
int WindowSize = 5000;                 //This is a time constraint for the heater relay output PID function
int FILLSize = 4000;                   //This is a time constraint for the heater relay output PID function
int i = 0, c = 0, r = 1, j = 0;        //These letters are counters and for defining system modes
unsigned long windowStartTime;         //These are for the heating PID Relay output algorithm
double pressureSetpoint, PVOutput, vacuumSetpoint, targetMl, tempSetpoint, inletTemp, outletTemp, maxOutput;
double headTemp, inletT, outletT, VOutput, returnMl, gap, vgap, Output, feedMass, returnMass, POutput, TOutput, VPOutput, returnSetpoint, FILLOutput;
int bulkMass, trapMass;
char incomingByte, T, P, V, E;           //These variables help sort incoming data
double filterVal, filterVal2;            // This determines smoothness; 0.0001 is max and 1 is off (no smoothing)
double smoothedCIRC=0;                   // This holds the last loop value just use a unique variable for every different sensor that needs smoothing
double smoothMass, smoothIP, smoothOP;   // This holds the last loop value just use a unique variable for every different sensor that needs smoothing
double feedMl, lastFeedMl;
double feedMedian, meniscus, bulkMl, trapMl, degasPressure, CHOutput;
int circRate, supplyPump, degasSetpoint, IF, OF, DF, BL, TF;
int coreTempLimit, SerialNumber;
int chillTempLimit, trapMedian;
int feedOffset, returnOffset, bulkOffset, trapOffset;
double inletPressure, outletPressure;
//int boardTemp;
int z = 0;
int inFirst;
double coreTemp, chillTemp, boardTemp;
double underfillSetpoint;
double pressureHolder;
double vacuumHolder;
/************Chip and PID initlializations**********************/
MCP4728 dac;
Adafruit_MAX31855 heaterCore(coreThermocouple);
Adafruit_MAX31855 chillerCore(chillerThermocouple);
Adafruit_MAX31855 thermSpare1(spareThermocouple1);
Adafruit_MAX31855 thermSpare2(spareThermocouple2);
Adafruit_MAX31865 inletRTD = Adafruit_MAX31865(inRTD);
Adafruit_MAX31865 outletRTD = Adafruit_MAX31865(outRTD);
PID Circulation(&feedMedian, &Output, &targetMl, Kp, Ki, Kd, DIRECT);
PID Temperature(&headTemp, &TOutput, &tempSetpoint, TKp, TKi, TKd, DIRECT);
//PID Chiller(&headTemp, &CHOutput, &tempSetpoint, TKp, TKi, TKd, REVERSE);
PID Pressure(&smoothIP, &POutput, &pressureSetpoint, cPKp, cPKi, cPKd, DIRECT);
PID PVacuum(&smoothIP, &PVOutput, &pressureSetpoint, cPKp, cPKi, cPKd, REVERSE);
PID VPressure(&smoothOP, &VPOutput, &vacuumSetpoint, cVKp, cVKi, cVKd, DIRECT);
PID Vacuum(&smoothOP, &VOutput, &vacuumSetpoint, cVKp, cVKi, cVKd, REVERSE);
HX711 feedTank;
HX711 returnTank;
HX711 trapTank;
HX711 bulkTank;
Adafruit_MCP4725 CIRCULATE;
Adafruit_ADS1015 PressureADC;
RunningMedian samples = RunningMedian(15);
RunningMedian trapp = RunningMedian(5);
int8_t psetpoint, vsetpoint;
/******************************************Setup Loop****************************************************/
void setup() 
{                                  //This loop runs once at startup to get everything ready
  EEPROM.update(20, 0);
  SPI.begin();                                  //Begin SPI communication
  Wire.setClock(400000);                        //set clock frequency for I2C comm
  dac.attatch(Wire, 13);                        //second argument is Arduino pin connected to LDAC(not), we are controlling LDAC manually so just entered pin we are not using
  PressureADC.begin();
  CIRCULATE.begin(0x61);                        //begins CIRCULATE device at 0x61 I2C address (this is our circulation pump DAC)
  tempSetpoint = (int8_t)EEPROM.read(0);        //read memory location for temperature setpoint
  EEPROM.get(1, pressureSetpoint);              //read memory location for pressure setpoint
  pressureHolder = pressureSetpoint;
  EEPROM.get(10, vacuumSetpoint);               //read memory location for vacuum setpoint
  vacuumHolder = vacuumSetpoint;
  EEPROM.get(5, degasSetpoint);                 //read memory location for vacuum setpoint
  targetMl = 180;                               //Sets target reservoir fill level to 180mL
  maxOutput = 4000;                             //We are trying to limit how fast the circulation pump runs in waterbased systems to reduce foaming and cavitation
  dac.readRegisters();                          //Used to read current settings from MCP4728
  dac.selectVref(MCP4728::VREF::VDD, MCP4728::VREF::VDD, MCP4728::VREF::VDD, MCP4728::VREF::VDD);                                   //setup voltage ref for each DAC channel
  dac.selectGain(MCP4728::GAIN::X1, MCP4728::GAIN::X1, MCP4728::GAIN::X1, MCP4728::GAIN::X1);                                       //set gain on output
  windowStartTime = millis();                   //initialize windowStartTime for heater output to current value of Millis
  feedTank.begin(DOUT1, CLK);                   //Begin SPI communication for feed side load cell
  returnTank.begin(DOUT2, CLK2);                //Begin SPI communication return side load cell
  trapTank.begin(DOUT4, CLK4);
  bulkTank.begin(DOUT3, CLK3);
  inletRTD.begin(MAX31865_4WIRE);               //Initialize feed side RTD, we are using a 3-wire
  outletRTD.begin(MAX31865_4WIRE);              //Initialize return side RTD, we are using a 3-wire
  feedTank.set_scale(calibration_factor);       //initialize load cell at calibration factor
  returnTank.set_scale(calibration_factor);     //initialize load cell at calibration factor
  bulkTank.set_scale(calibration_factor);       //initialize load cell at calibration factor
  trapTank.set_scale(calibration_factor);       //initialize load cell at calibration factor
  filterVal = .001;                             // Smoothing function sensitivity. 1 is off, 0.0001 is maximum
  filterVal2 = 0.1;
  smoothedCIRC = 10;                            // Initialize this value to avoid spikes in Arduino plotter on startup
 // filterValp = 0.95;                          // Smoothing function sensitivity. 1 is off, 0.0001 is maximum
  smoothMass = 180;                             // Initialize this value to avoid spikes in Arduino plotter on startup
  Serial.begin(9600);                           //Open serial port at 9600 Baud
  lastFeedMl = 100;
  coreTempLimit = 70;
  chillTempLimit = 60;
  underfillSetpoint = -20;
  while (!Serial) delay(1);
  //EEPROM.update(8, 9);     //leave commented, this is used upon the first setup of each board to burn serial number
 
 /*int feedOffset = 1050;
 int returnOffset = 1030;
 int trapOffset = 1597;
 int bulkOffset = 905;
 */
/****************PID Initializations*************/
  Circulation.SetMode(AUTOMATIC);
  Circulation.SetTunings(Kp, Ki, Kd);
  Temperature.SetMode(AUTOMATIC);
  Temperature.SetTunings(TKp, TKi, TKd);
 // Chiller.SetMode(AUTOMATIC);
  //Chiller.SetTunings(TKp, TKi, TKd);
  Pressure.SetMode(AUTOMATIC);
  Vacuum.SetMode(AUTOMATIC);
  PVacuum.SetMode(AUTOMATIC);
  VPressure.SetMode(AUTOMATIC);
  Pressure.SetTunings(cPKp, cPKi, cPKd);
  PVacuum.SetTunings(cPKp, cPKi, cPKd);
  Vacuum.SetTunings(cVKp, cVKi, cVKd);
  VPressure.SetTunings(cVKp, cVKi, cVKd);

/**************Pin Mode Definitions**************/
  pinMode(supply, OUTPUT);
  digitalWrite(supply, LOW);
  pinMode(degas, OUTPUT);
  pinMode(degasTransducer, OUTPUT);
  digitalWrite(degasTransducer, HIGH);
  pinMode(inletFoam, INPUT);
  pinMode(outletFoam, INPUT);
  pinMode(degasFoam, INPUT);
  pinMode(inletLock, OUTPUT);
  pinMode(outletLock, OUTPUT);
  delay(500);
  DDRJ = B01111000;
  SerialNumber = (EEPROM.read(8));
  r = 0;
}

/***********************************Main Code Body********************************************************/

void loop() {
/*************************Read Sensors and Serial Port*********************************/
  degasPressure = ((float)readSensor(degasTransducer) - OUTPUT_MIN) * (PRESSURE_MAX - PRESSURE_MIN) / (OUTPUT_MAX - OUTPUT_MIN) + PRESSURE_MIN;     //Read outlet pressure sensor
  inletPressure = (0.2329*(PressureADC.readADC_SingleEnded(0))-231.82); 
  outletPressure = (0.2329*(PressureADC.readADC_SingleEnded(1))-231.82);  
  smoothIP = smooth(inletPressure, filterVal2, smoothIP);
  smoothOP = smooth(outletPressure, filterVal2, smoothOP);

    switch(z)       //breaks up more time consuming processes (temperatrue readings with rtd mainly) to speed up code
    {
      case 0:
        {
          inletT = inletRTD.readRTD();                          //Read 16 bit integer from RTD
          inletTemp = inletRTD.temperature(RNOMINAL, RREF);     //Take RTD count reading and make it a Celcius reading
          z++;
          break;
        }
        case 1:
        {
          outletT = outletRTD.readRTD();                        //Read 16 bit integer from RTD
          outletTemp = outletRTD.temperature(RNOMINAL, RREF);   //Take RTD count reading and make it a Celcius reading
          headTemp = ((inletTemp + outletTemp)/2);              //This assumes we lose an even amount of heat between the temp reading points, the printhead is halfway between.
          z++;
          break;
        }
         case 2:
        {
          coreTemp = (heaterCore.readCelsius());                //Read heater core thermocouple and apply calibration
          z++;
          break;
        }
          case 3:
        {
          chillTemp = (chillerCore.readCelsius());              //Read heater core thermocouple and apply calibration
          z = 0;
          break;
        }
    }

  feedMass = feedTank.get_units();                      //Read feed side load cell
  returnMass = returnTank.get_units();                  //Read return side load cell
  bulkMass = bulkTank.get_units();                      //Read bulk load cell
  trapMass = trapTank.get_units();                      //Read trap load cell
//  IF = (!digitalRead(inletFoam));                     //inlet foam fault
 // OF = (!digitalRead(outletFoam));                    //outlet foam fault
//  DF = (!digitalRead(degasFoam));                     //degas foam fault
  readSerial();                                         //Read serial port and set modes and setpoints
  
/**************************Do Conversion Math******************************************/
  feedMl = ((feedMass - 550)/1.0);                     //Account for the weight of lids, tubes and fittings in the load cell reading, convert gram to mL
  samples.add(feedMl);
  trapp.add(trapMl);
  feedMedian = samples.getMedian();
  trapMedian = trapp.getMedian();
  smoothMass = smooth(feedMl, filterVal, smoothMass);
  returnMl = ((returnMass - 630)/1.0);                 //Account for the weight of lids, tubes and fittings in the load cell reading, convert gram to mL
  trapMl = ((trapMass - 750)/1.0);
  bulkMl = ((bulkMass - 480)/1.0);
  meniscus = inletPressure - outletPressure;
  circRate = (Output/4095)*100;
/**************************Write Outputs to Pins and Serial Port***********************/  
 if((feedMl > 120))
 { 
  pressureSetpoint = pressureHolder;              //read memory location for pressure setpoint
 }
 
 if((returnMl > 120))
 { 
  vacuumSetpoint = vacuumHolder;              //read memory location for pressure setpoint
 }
 
 if((feedMl <= 120))
 { 
  pressureSetpoint = underfillSetpoint;              //read memory location for pressure setpoint
 }
 
 if((returnMl <= 120))
 { 
  vacuumSetpoint = underfillSetpoint;              //read memory location for pressure setpoint
 }
  
  if(r > 0)
  {
  Pressure.Compute();                                   //Runs PID math for the pressure pump on the feed side
  dac.analogWrite(MCP4728::DAC_CH::A, POutput);         //Writes output of PID to DAC
  Vacuum.Compute();                                     //Runs PID math for the vacuum pump on the feed side
  dac.analogWrite(MCP4728::DAC_CH::C, VOutput);         //Writes output of PID to DAC
  PVacuum.Compute();                                    //Runs PID math for the vacuum pump on the return side
  dac.analogWrite(MCP4728::DAC_CH::D, PVOutput);        //Writes output of PID to DAC
  VPressure.Compute();                                  //Runs PID math for the pressure pump on the return side
  dac.analogWrite(MCP4728::DAC_CH::B, VPOutput);        //Writes output of PID to DAC
   }
   
   else
   {
    dac.analogWrite(MCP4728::DAC_CH::A, 0);               //Writes output of PID to DAC
    dac.analogWrite(MCP4728::DAC_CH::C, 0);               //Writes output of PID to DAC
    dac.analogWrite(MCP4728::DAC_CH::D, 0);               //Writes output of PID to DAC
    dac.analogWrite(MCP4728::DAC_CH::B, 0);               //Writes output of PID to DAC
   }
 
 
   
  Temperature.Compute();                                //Runs PID math for temperature
 // Chiller.Compute();
  Circulate();                                          //Call function to run circulation pump
  Heat();                                               //Call function to run heater
  Supply();                                             //Call function to run supply pump
  Degas();
  Trap();
  lastFeedMl = feedMl;
  Faults();
  Lock();
 
}


/*************************************functions******************************************************/

/********************Presure Sensor Read Function***********************************************/

int16_t readSensor (uint8_t selectPin) {

  SPI.beginTransaction(SPISettings(800000, MSBFIRST, SPI_MODE1)); // Set to 700kHz, MSB and MODE1
  digitalWrite(selectPin, LOW);         //pull Chipselect Pin to Low

  int inByte_1 = SPI.transfer(0x00);    // Read first Byte of Pressure
  int inByte_2 = SPI.transfer(0x00);    // Read second Byte of Pressure

  digitalWrite(selectPin, HIGH);        //pull Chipselect Pin to High
  SPI.endTransaction();                 //end SPI Transaction
 
  int16_t pressure_dig = inByte_1 << 8 | inByte_2;
  return pressure_dig; //return digital Pressure Value
}
/************************Read Serial Port Function****************/
int16_t readSerial(){
  if (Serial.available() > 0) {
    int inFirst = Serial.read();      //Reads in first character and decides what to do
    switch (inFirst){
     
  case 't':                           //When a "t" comes in, set temperature and place value in eeprom
    if (Serial.available() > 0){
    tempSetpoint = Serial.parseInt(); //Parse input into a multiple digit setpoint
    EEPROM.update(0,tempSetpoint);}   //EEPROM.update instead of EEPROM.write only writes to eeprom if the value is different than whats stored extending eeprom life
    break;

  case 'p':                           //When a "p" comes in, set pressure and place value in eeprom
    if (Serial.available() > 0){
    pressureSetpoint = Serial.parseInt();
    pressureHolder = pressureSetpoint;
    EEPROM.put(1, pressureSetpoint);}     //EEPROM location for pressure is 1
    break;

  case 'v':                           //When a "v" comes in, set pressure and place value in eeprom
    if (Serial.available() > 0){
    vacuumSetpoint = Serial.parseInt();
    vacuumHolder = vacuumSetpoint;
    EEPROM.put(10,vacuumSetpoint);
    }
    break;

  case 'g':                           //When a "g" comes in, set degas setpoint and place value in eeprom
    if (Serial.available() > 0){
    degasSetpoint = Serial.parseInt();
    EEPROM.update(5, degasSetpoint);}
    break;

  case 'f':                           //When a "f" comes in, set counters for flowrate function
    i = 1;
    break;

  case 'c':                           //When a "c" comes in, set counters to run flush function
    c = 1;
    r = 1;
    break;

  case 'd':                           //When a "d" comes in, set counters to stop flush function
    c = 0;
    break;

  case 'i':                           //When a "i" comes in, set counters for idle mode
    r = 0;
    break;

  case 'r':                           //When a "r" comes in, set counters for run mode
    r = 1;
    break;

  case 'a':
    writeSerial(); 
    break;

  default:                            //if something besides the above defined letters comes in, do nothing
    break;
    }
 }
}

/********************Write to Serial Port Function********/
int16_t writeSerial()
{
  Serial.print(inletPressure);     //Print inlet pressure to serial port
  Serial.print(" , "); 
  Serial.print(meniscus);          //Print meniscus pressure to serial port
  Serial.print(" , ");    
  Serial.print(outletPressure);    //Print outlet pressure to serial port
  Serial.print(" , ");  
  Serial.print(degasPressure);     //Print degas pressure to serial port
  Serial.print(" , ");  
  Serial.print(inletTemp);         //Print intlet temperature to serial port
  Serial.print(" , ");
  Serial.print(headTemp);          //Print computed head temperature to serial port
  Serial.print(" , ");
  Serial.print(outletTemp);        //Print outlet temperature to serial port
  Serial.print(" , ");
  Serial.print(coreTemp);          //Print heater core temperature to serial port
  Serial.print(" , ");
  Serial.print(chillTemp);         //Print chiller core temperature to serial port
  Serial.print(" , ");
  Serial.print(feedMl);            //Print feed tank mL to serial port
  Serial.print(" , ");
  Serial.print(returnMl);          //Print return tank mL to serial port
  Serial.print(" , ");
  Serial.print(trapMl);            //Print overflow trap tank mL to serial port
  Serial.print(" , ");
  Serial.print(bulkMl);            //Print bulk tank mL to serial port
  Serial.print(" , ");
  Serial.print(circRate);          //Print % circulation pump output vctrl to serial port
  Serial.print(" , ");
  Serial.print(supplyPump);        //Print supply pump on/off to serial port
  Serial.print(" , ");
  Serial.print(pressureSetpoint);  //Print pressure setpoint to serial port
  Serial.print(" , ");
  Serial.print(vacuumSetpoint);    //Print vacuum setpoint to serial port
  Serial.print(" , ");
  Serial.print(tempSetpoint);      //Print temperature setpoint to serial port
  Serial.print(" , ");
  Serial.print(degasSetpoint);     //Print degassing setpoint to serial port
  Serial.print(" , ");
  Serial.print(IF);                // inlet foam fault
  Serial.print(" ");
  Serial.print(OF);                // outlet foam fault
  Serial.print(" ");
  Serial.print(DF);                // degas foam fault
  Serial.print(" ");
  Serial.print(BL);                // bulk low fault
  Serial.print(" ");
  Serial.print(TF);                // trap fluid detected fault
  Serial.print(" ");
  Serial.print(r);
  Serial.print(" ");
  Serial.print("0");
  Serial.print(" ");
  Serial.print("0");
  Serial.print(" ");
  Serial.print("0");
  Serial.print(" ");
  Serial.print("0");
  Serial.print(" ");
  Serial.print("0");
  Serial.print(" ");
  Serial.print("0");
  Serial.print(" ");
  Serial.print("0");
  Serial.print(" ");
  Serial.print("0");
  Serial.print(" ");
  Serial.print("0");
  Serial.print(" ");
  Serial.print("0");
  Serial.println(SerialNumber);
  }

  /**************************faults************************/
int16_t Faults()
{
  if(bulkMl < 100 )
    {
      BL = 1;
    }
    else
    {
      BL = 0;
    }
 }

 /*********************************Locking******************/
int16_t Lock()
{
  if(r < 1)
  {
    digitalWrite(inletLock, LOW);
    digitalWrite(outletLock, LOW);
  }
  else
  {
    digitalWrite(inletLock, HIGH);
    digitalWrite(outletLock, HIGH);
  }
}
/**************************HEATING*****************************************/
  int16_t Heat()
  {                                                     //Google millis function for help
        
   if (millis() - windowStartTime > WindowSize)         //If current milliseconds-window start time is bigger than set windowsize
  { //time to shift the Relay Window
    windowStartTime += WindowSize;                      //Windowstarttime = windowstarttime + windowsize
  }

  //checks for moving fluid, full reservoirs, and sets heater core temp limit
 if ((TOutput > millis() - windowStartTime) && (coreTemp < coreTempLimit) && (feedMl > (targetMl - 20)) && (returnMl > (targetMl - 20)) && (Output > 500)&& (headTemp > 0)) 
  {
     PORTJ = PORTJ|B01000000;
  }

   else 
  {
     PORTJ = PORTJ&= ~B01000000;
  }

  // if ((CHOutput > millis() - windowStartTime) && (chillTemp < chillTempLimit) && (feedMl > (targetMl - 20)) && (returnMl > (targetMl - 20)) && (Output > 500)&& (headTemp > 0)) 
          if ( (chillTemp < chillTempLimit) && (feedMl > (targetMl - 10)) && (returnMl > (targetMl - 10)) && (Output > 500)&& (headTemp > 0) && (tempSetpoint <= 23))
  {
     PORTJ = PORTJ|B00010000;
  }

  else 
  {
     PORTJ = PORTJ&= ~B00010000;
  }
  }

/***************************CIRCULATION***********************************************/
int16_t Circulate()
{
    
if(r > 0)                                 //If we are in run mode
{
if ((i > 0) || (c > 0))                   //If Flowrate or Flush function is called
{
    Output = 4095;                        //Run pump maxed out
    CIRCULATE.setVoltage(Output, false);
}

if ((i < 1) && (c < 1) && (returnMl >= (targetMl - 25)))                   //If we are in run but flush or flowrate is not called
{
    Circulation.Compute();                //Computer normal circulation rate
   
    //Output = map(Output, 0,4095, 0, maxOutput);   //Map output from 0-4095 to 0-max output. This is also to try to reduce foaming in WB ink
    smoothedCIRC = smooth(Output, filterVal, smoothedCIRC);     //smoothing function
    CIRCULATE.setVoltage(smoothedCIRC, false);
}
}
else                                            //turns pump off in idle mode
{
    Output = 0;
    CIRCULATE.setVoltage(Output, false);
}
  
}

/********************************SUPPLY PUMP*****************************/
int16_t Supply()
{       //Checks for a full system and if we are in run mode or flush mode
   if((((returnMl + feedMl) < (targetMl + targetMl)) || (c > 0)) && (r > 0))
 {
 j++;         //j counter makes sure system is empty for 20 cycles through code before running supply pump. This keeps any glitches or people bumping the system from filling it.
 if((j > 20) && (bulkMl > 200))
 {
    analogWrite(supply, 150);         //If conditions are met, fill system
    supplyPump = 1;
 }
  else
  {
   digitalWrite(supply, LOW);          //If conditions are not met, do not fill system
   supplyPump = 0;
  }
 }
 
 
 else                                 //Default to not filling system and turn off pump in idle mode
 {
  digitalWrite(supply, LOW);
   supplyPump = 0;
 }


 if (feedMl >= 200)                   //Resets counter when feed tank gets to 200mL full for flowrate functionality
 {
  i = 0;
 }
  if (returnMl >= 180)                //if return reservoir is full, reset j counter so supply pump doesent try to fill anything
 {
  j = 0;
 }
}

/***************Degassing***********************/
int16_t Degas()
{
  if((degasPressure > degasSetpoint) && (r > 0))
  {
    digitalWrite(degas, HIGH);
  }
  else
  {
    digitalWrite(degas, LOW);
  }
}
/**************************SMOOTHING***************************/

double smooth(double Input, double filterVal, double smoothedVal)
{
  smoothedVal = (Input * (1 - filterVal)) + (smoothedVal * filterVal);      // Smoothing function
  return (double)smoothedVal;
}


/*****************************TRAP******************************/
int16_t Trap()
{
  if(trapMedian >= 100)
  {
    r = 0;
    TF = 1;
  }
  
  else
  {
    TF = 0;
  }
}
