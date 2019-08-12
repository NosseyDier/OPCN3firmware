/*
    opcn3.h - Library for operating the Alphasense OPC-N3 Particle counter.
    Created by Reid A. Yesson August 2019.
    Released with an MIT license.
*/

#ifndef Opcn3_h
#define Opcn3_h

// Includes
#include <application.h>

struct Status {
    int fanON;
    int laserON;
    int fanDAC;
    int laserDAC;
};

struct Firmware {
    int major;
    int minor;
};

struct HistogramData {
    double bin0;
    double bin1;
    double bin2;
    double bin3;
    double bin4;
    double bin5;
    double bin6;
    double bin7;
    double bin8;
    double bin9;
    double bin10;
    double bin11;
    double bin12;
    double bin13;
    double bin14;
    double bin15;
    double bin16;
    double bin17;
    double bin18;
    double bin19;
    double bin20;
    double bin21;
    double bin22;
    double bin23;

    // Mass Time-of-Flight
    float bin1MToF;
    float bin3MToF;
    float bin5MToF;
    float bin7MToF;

    // Sample Flow Rate
    float sfr;

    // The temperature
    //unsigned long temp_pressure;
    unsigned int temp;

    // Humidity
    unsigned int humidity;

    // Sampling Period
    float period;

    // Checksum
    unsigned int checksum;

    float pm1;
    float pm25;
    float pm10;
};

struct PMData {
  float pm1;
  float pm25;
  float pm10;
};

struct ConfigVars {
    // Bin Boundaries
    int bb0;
    int bb1;
    int bb2;
    int bb3;
    int bb4;
    int bb5;
    int bb6;
    int bb7;
    int bb8;
    int bb9;
    int bb10;
    int bb11;
    int bb12;
    int bb13;
    int bb14;
    int bb15;
    int bb16;
    int bb17;
    int bb18;
    int bb19;
    int bb20;
    int bb21;
    int bb22;
    int bb23;
    int bb24;

    int bbd0;
    int bbd1;
    int bbd2;
    int bbd3;
    int bbd4;
    int bbd5;
    int bbd6;
    int bbd7;
    int bbd8;
    int bbd9;
    int bbd10;
    int bbd11;
    int bbd12;
    int bbd13;
    int bbd14;
    int bbd15;
    int bbd16;
    int bbd17;
    int bbd18;
    int bbd19;
    int bbd20;
    int bbd21;
    int bbd22;
    int bbd23;
    int bbd24;

    int bw0;
    int bw1;
    int bw2;
    int bw3;
    int bw4;
    int bw5;
    int bw6;
    int bw7;
    int bw8;
    int bw9;
    int bw10;
    int bw11;
    int bw12;
    int bw13;
    int bw14;
    int bw15;
    int bw16;
    int bw17;
    int bw18;
    int bw19;
    int bw20;
    int bw21;
    int bw22;
    int bw23;

    unsigned int max_tof;
    unsigned int AMSamplingInterval;
    unsigned int AMIntervalCount;
    unsigned int AMFanOnIdle;
    unsigned int AMLaserOnIdle;
    unsigned int AMMaxDataArraysInFile;
    unsigned int AMOnlySavePMData;

    // Time of Flight to Sample Flow Rate Conversion Factor
    unsigned int tof_sfr;
};

class OPCN2
{
private:
    // attributes
    uint8_t _CS;
    int _fv;

    // methods
    uint16_t _16bit_int(byte MSB, byte LSB);
    bool _compare_arrays(byte array1[], byte array2[], int length);
    float _calculate_float(byte val0, byte val1, byte val2, byte val3);
    uint32_t _32bit_int(byte val0, byte val1, byte val2, byte val3);

public:
    OPCN2(uint8_t chip_select);

    // attributes
    Firmware firm_ver;

    // methods
    void begin(uint8_t chip_select);
    void set_firmware_version();
    bool ping();
    bool on();
    bool off();
    bool write_config_variables(byte values[]);
    bool write_config_variables2(byte values[]);
    bool write_serial_number_string(byte values[]);
    bool save_config_variables();
    bool enter_bootloader();
    bool set_fan_power(uint8_t value);
    bool set_laser_power(uint8_t value);
    bool toggle_fan(bool state);
    bool toggle_laser(bool state);

    String read_information_string();
    String read_serial_number();
    Firmware read_firmware_version();
    Status read_status();
    ConfigVars read_configuration_variables();
    PMData read_pm_data();
    HistogramData read_histogram(bool convert_to_conc = true);
};

#endif
