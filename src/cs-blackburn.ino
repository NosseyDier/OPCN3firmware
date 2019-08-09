// This #include statement was automatically added by the Particle IDE.
#include <opcn3.h>
#include "Pca9554.h"
SYSTEM_MODE(MANUAL);

#include <string>

#define GPIO_A                 0x20   //GPIO Extender A (7 bit address)
#define GPIO_B                 0x21   //GPIO Extender B

OPCN3 alpha(D5);
HistogramData hist;
ConfigVars vars;

void setup(){

 Serial.begin();
    Serial.println("here");

  Wire.begin(); 
  Pca9554.begin(GPIO_A);
  Pca9554.begin(GPIO_B);

  Pca9554.pinMode(GPIO_A,0,INPUT);    //Stat2
  Pca9554.pinMode(GPIO_A,1,INPUT);    //Stat1
  Pca9554.pinMode(GPIO_A, 5, OUTPUT); //EN_PW_OPC
  Pca9554.pinMode(GPIO_A, 6, OUTPUT); //PMS_SET
  Pca9554.pinMode(GPIO_A, 7, OUTPUT); //PMS_RST
  Pca9554.pinMode(GPIO_B, 6, OUTPUT); //EN_5V
  Pca9554.pinMode(GPIO_B, 7, OUTPUT); //SEL_SERV
  pinMode(A0,OUTPUT);                 //ONOFF_GPS
  pinMode(C4,OUTPUT);                 //EN_GPS
  pinMode(B0,INPUT);                  //SYSTEM-ON_GPS

  Pca9554.digitalWrite(GPIO_B, 6, LOW); //EN5V
  Pca9554.digitalWrite(GPIO_A, 5, HIGH);//EN_PW_OPC
  Pca9554.digitalWrite(GPIO_A, 6, HIGH);//SET PMS_SET
  Pca9554.digitalWrite(GPIO_A, 7, HIGH);//SET PMS_RST
  //delay(1000);

    delay(1000);
    alpha.begin(D5);
    delay(1000);
    alpha.on();


   
    Serial.println("Testing OPC-N3 v" + String(alpha.firm_ver.major) + "." + String(alpha.firm_ver.minor));
    // Read and print the configuration variables
    vars = alpha.read_configuration_variables();
    Serial.println("\nConfiguration Variables");
    Serial.print("\tGSC:\t"); Serial.println(vars.gsc);
    Serial.print("\tSFR:\t"); Serial.println(vars.sfr);
    Serial.print("\tLaser DAC:\t"); Serial.println(vars.laser_dac);
    Serial.print("\tFan DAC:\t"); Serial.println(vars.fan_dac);
    Serial.print("\tToF-SFR:\t"); Serial.println(vars.tof_sfr);
   
    delay(1000);
}

void loop(){

    delay(5000);

    hist = alpha.read_histogram();

    // Print out the histogram data
    Serial.print("\nInformation string:\t"); Serial.println(alpha.read_information_string());

    Serial.print("\nFirmware version major:\t"); Serial.println(alpha.read_firmware_version().major);
    Serial.print("\nFirmware version minor:\t"); Serial.println(alpha.read_firmware_version().minor);

    Serial.print("\nSampling Period:\t"); Serial.println(hist.period);
    Serial.print("PM1: "); Serial.println(hist.pm1);
    //Particle.publish("PM1: ", String::format("%.2f", hist.pm1), PUBLIC);
    Serial.print("PM2.5: "); Serial.println(hist.pm25);
    //Particle.publish("PM2.5: ", String::format("%.2f", hist.pm25), PUBLIC);
    Serial.print("PM10: "); Serial.println(hist.pm10);
    //Particle.publish("PM10: ", String::format("%.2f", hist.pm10), PUBLIC);
}