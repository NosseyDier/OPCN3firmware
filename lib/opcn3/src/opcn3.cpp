#include <opcn3.h>

OPCN3::OPCN3(uint8_t chip_select)
{

}

void OPCN3::begin(uint8_t chip_select)
{
    // Initiate an instance of the OPCN2 class
  // Ex. OPCN2 alpha(chip_select = A2);
  _CS = chip_select;

  // Set up SPI1
  SPI1.begin(_CS);
  SPI1.setBitOrder(MSBFIRST);
  SPI1.setDataMode(SPI_MODE1);
  SPI1.setClockSpeed(500000);

  // Set the firmware version
  _fv = this->read_information_string().replace(".", "").trim().substring(24, 26).toInt();

  if (_fv < 18) {
    firm_ver.major = _fv;
    firm_ver.minor = 0;
  }
  else {
    firm_ver.major = 99;    // only temporarily...
    Firmware tmp = this->read_firmware_version();

    firm_ver.major = tmp.major;
    firm_ver.minor = tmp.minor;
  }

}

void OPCN3::set_firmware_version( void )
{
  // Set the firmware version in case it previously failed
  // Set the firmware version
  _fv = this->read_information_string().replace(".", "").trim().substring(24, 26).toInt();

  if (_fv < 18) {
    this->firm_ver.major = _fv;
    this->firm_ver.minor = 0;
  } else {
    Firmware tmp = this->read_firmware_version();

    this->firm_ver.major = tmp.major;
    this->firm_ver.minor = tmp.minor;
  }
}

uint16_t OPCN3::_16bit_int(byte LSB, byte MSB)
{
  // Combine two bytes into a 16-bit unsigned int
  return ((MSB << 8) | LSB);
}

bool OPCN3::_compare_arrays(byte array1[], byte array2[], int length)
{
  // Compare two arrays and return a boolean
  bool result = true;

  for (int i = 0; i < length; i++){
    if (array1[i] != array2[i]){
      result = false;
    }
  }

  return result;
}

float OPCN3::_calculate_float(byte val0, byte val1, byte val2, byte val3)
{
  // Return an IEEE754 float from an array of 4 bytes
  union u_tag {
    byte b[4];
    float val;
  } u;

  u.b[0] = val0;
  u.b[1] = val1;
  u.b[2] = val2;
  u.b[3] = val3;

  return u.val;
}

uint32_t OPCN3::_32bit_int(byte val0, byte val1, byte val2, byte val3)
{
  // Return a 32-bit unsigned int from 4 bytes
  return ((val3 << 24) | (val2 << 16) | (val1 << 8) | val0);
}

bool OPCN3::ping()
{
  // Isse the check status command
  // ex.
  // $ alpha.ping();
  // $ true
  byte resp[1];
  byte expected[] = {0xF3};

  digitalWrite(this->_CS, LOW);       // pull the pin low
  resp[0] = SPI1.transfer(0xCF);       // issue the command byte
  digitalWrite(this->_CS, HIGH);      // pull the pin high

  return this->_compare_arrays(resp, expected, 1);
}

bool OPCN3::on()
{
  // Turn ON the OPC and return a boolean
  // Ex.
  // $ alpha.on()
  // $ true
  byte vals[2];
  byte expected[] = {0xF3, 0x03};

  digitalWrite(this->_CS, LOW);
  vals[0] = SPI1.transfer(0x03);
  digitalWrite(this->_CS, HIGH);

  delayMicroseconds(10000);

  digitalWrite(this->_CS, LOW);
  vals[1] = SPI1.transfer(0x00);
  digitalWrite(this->_CS, HIGH);

  return this->_compare_arrays(vals, expected, 2);
}

bool OPCN3::off()
{
  // Turn OFF the OPC and return a boolean
  // Ex.
  // $ alpha.off()
  // $ true
  byte vals[2];
  byte expected[] = {0xF3, 0x03};

  digitalWrite(this->_CS, LOW);
  vals[0] = SPI1.transfer(0x03);
  digitalWrite(this->_CS, HIGH);

  delayMicroseconds(10000);

  digitalWrite(this->_CS, LOW);
  vals[1] = SPI1.transfer(0x01);
  digitalWrite(this->_CS, HIGH);

  return this->_compare_arrays(vals, expected, 2);
}

String OPCN3::read_information_string()
{
  // Read the information string and return a string
  // Ex.
  // $ alpha.read_information_string()
  String result = "";
  String tmp;
  byte vals[61];

  digitalWrite(this->_CS, LOW);
  SPI1.transfer(0x3F);
  digitalWrite(this->_CS, HIGH);

  delayMicroseconds(3000);

  // Iterate to read the entire string
  digitalWrite(this->_CS, LOW);
  for (int i = 0; i < 60; i++){
    vals[i] = SPI1.transfer(0x00);
    result += String((char)vals[i]);
    delayMicroseconds(4);
  }

  digitalWrite(this->_CS, HIGH);

  return result;
}

struct Status OPCN3::read_status()
{
  // Read key status variables from the OPC-N2
  // Only available with Alphasense OPC-N2 > firmware v18
  // Ex.
  // $ alpha.read_status()
  Status data;
  byte vals[4];

  // Read the status
  digitalWrite(this->_CS, LOW);
  SPI1.transfer(0x13);
  digitalWrite(this->_CS, HIGH);

  delayMicroseconds(10000);

  // Send the read command and build the array of data
  digitalWrite(this->_CS, LOW);
  for (int i = 0; i < 4; i++){
    vals[i] = SPI1.transfer(0x13);
    delayMicroseconds(4);
  }

  digitalWrite(this->_CS, HIGH);

  // Calculate the values!
  data.fanON    = (unsigned int)vals[0];
  data.laserON  = (unsigned int)vals[1];
  data.fanDAC   = (unsigned int)vals[2];
  data.laserDAC = (unsigned int)vals[3];

  return data;
}

struct Firmware OPCN3::read_firmware_version()
{
  // Only available with Alphasense OPC-N2 > firmware v18
  // Ex.
  // $ alpha.read_firmware_version()
  Firmware res;

  // Read the Firmware version
  digitalWrite(this->_CS, LOW);
  SPI1.transfer(0x12);
  digitalWrite(this->_CS, HIGH);

  delayMicroseconds(10000);

  digitalWrite(this->_CS, LOW);
  res.major = (unsigned int)SPI1.transfer(0x00);
  delayMicroseconds(4);
  res.minor = (unsigned int)SPI1.transfer(0x00);
  digitalWrite(this->_CS, HIGH);

  return res;
}

bool OPCN3::write_config_variables(byte values[])
{
  // Write the configuration [NOT IMPLEMENTED]
  return true;
}

bool OPCN3::write_serial_number_string(byte values[])
{
  // NOT IMPLEMENTED
  // Only available with Alphasense OPC-N2 > firmware v18
  return true;
}

bool OPCN3::save_config_variables()
{
  // Save the configuration variables
  // Ex.
  // $ alpha.save_config_variables()
  byte resp[6];
  byte commands[] = {0x43, 0x3F, 0x3C, 0x3F, 0x3C, 0x43};
  byte expected[] = {0xF3, 0x43, 0x3f, 0x3c, 0x3f, 0x3c};

  digitalWrite(this->_CS, LOW);
  resp[0] = SPI1.transfer(commands[0]);
  digitalWrite(this->_CS, HIGH);

  delayMicroseconds(10000);

  digitalWrite(this->_CS, LOW);
  for (int i = 1; i < (int)sizeof(commands); i++){
    resp[i] = SPI1.transfer(commands[i]);
    delayMicroseconds(4);
  }

  digitalWrite(this->_CS, HIGH);      // Pull the pin high

  return this->_compare_arrays(resp, expected, 6);
}

bool OPCN3::enter_bootloader()
{
  // Enter into bootloader mode
  byte resp[1];
  byte expected[] = {0xF3};

  digitalWrite(this->_CS, LOW);
  resp[0] = SPI1.transfer(0x41);
  digitalWrite(this->_CS, HIGH);

  return this->_compare_arrays(resp, expected, 1);
}

bool OPCN3::set_fan_power(uint8_t value)
{
  // Set the fan power
  // Value must be between 0-255
  // Ex.
  // $ alpha.set_fan_power(255);
  byte resp[3];
  byte expected[] = {0xF3, 0x42, 0x00};

  digitalWrite(this->_CS, LOW);
  resp[0] = SPI1.transfer(0x42);
  digitalWrite(this->_CS, HIGH);

  delayMicroseconds(10000);

  digitalWrite(this->_CS, LOW);
  resp[1] = SPI1.transfer(0x00);

  delayMicroseconds(4);

  resp[2] = SPI1.transfer(value);
  digitalWrite(this->_CS, HIGH);

  return this->_compare_arrays(resp, expected, 3);
}

bool OPCN3::set_laser_power(uint8_t value)
{
  // Set the laser power
  // Value must be between 0-255
  // Ex.
  // $ alpha.set_laser_power(255);
  byte resp[3];
  byte expected[] = {0xF3, 0x42, 0x01};

  digitalWrite(this->_CS, LOW);
  resp[0] = SPI1.transfer(0x42);
  digitalWrite(this->_CS, HIGH);

  delayMicroseconds(10000);

  digitalWrite(this->_CS, LOW);
  resp[1] = SPI1.transfer(0x01);

  delayMicroseconds(4);

  resp[2] = SPI1.transfer(value);
  digitalWrite(this->_CS, HIGH);

  return this->_compare_arrays(resp, expected, 3);
}

bool OPCN3::toggle_fan(bool state)
{
  // Toggle the state of the fan
  // Ex.
  // $ alpha.toggle_fan(true); // turns fan on
  byte resp[2];
  byte expected[] = {0xF3, 0x03};

  digitalWrite(this->_CS, LOW);
  resp[0] = SPI1.transfer(0x03);
  digitalWrite(this->_CS, HIGH);

  delayMicroseconds(10000);

  // turn either on or off
  digitalWrite(this->_CS, LOW);
  if (state == true){
    resp[1] = SPI1.transfer(0x04);
  }
  else {
    resp[1] = SPI1.transfer(0x05);
  }

  digitalWrite(this->_CS, HIGH);

  return this->_compare_arrays(resp, expected, 2);
}

bool OPCN3::toggle_laser(bool state)
{
  // Toggle the state of the laser
  // Ex.
  // $ alpha.toggle_laser(true);
  byte resp[2];
  byte expected[] = {0xF3, 0x03};

  digitalWrite(this->_CS, LOW);
  resp[0] = SPI1.transfer(0x03);
  digitalWrite(this->_CS, HIGH);

  delayMicroseconds(10000);

  digitalWrite(this->_CS, LOW);
  if (state == true){
    resp[1] = SPI1.transfer(0x02);
  }
  else {
    resp[1] = SPI1.transfer(0x03);
  }

  digitalWrite(this->_CS, HIGH);

  return this->_compare_arrays(resp, expected, 2);
}


//Checked
struct ConfigVars OPCN3::read_configuration_variables()
{
  // Read the configuration variables and return the structure
  // Ex.
  // $ alpha.read_configuration_variables();
  ConfigVars results;       // empty structure for the data
  int configVarsReturns = 167;
  byte vals[configVarsReturns + 1];

  if (this->firm_ver.major < 18) {
    results.AMSamplingInterval    = -999;
    results.AMIntervalCount       = -999;
    results.AMFanOnIdle           = -999;
    results.AMLaserOnIdle         = -999;
    results.AMMaxDataArraysInFile = -999;
    results.AMOnlySavePMData      = -999;
  }

  // Read the config variables
  digitalWrite(this->_CS, LOW);
  SPI1.transfer(0x3c);
  digitalWrite(this->_CS, HIGH);

  delayMicroseconds(10000);

  digitalWrite(this->_CS, LOW);
  for (int i = 0; i < configVarsReturns + 1; i++){
    vals[i] = SPI1.transfer(0x00);
    delayMicroseconds(4);
  }

  digitalWrite(this->_CS, HIGH);

  // Fill in the results
  results.bb0 = this->_16bit_int(vals[0], vals[1]);
  results.bb1 = this->_16bit_int(vals[2], vals[3]);
  results.bb2 = this->_16bit_int(vals[4], vals[5]);
  results.bb3 = this->_16bit_int(vals[6], vals[7]);
  results.bb4 = this->_16bit_int(vals[8], vals[9]);
  results.bb5 = this->_16bit_int(vals[10], vals[11]);
  results.bb6 = this->_16bit_int(vals[12], vals[13]);
  results.bb7 = this->_16bit_int(vals[14], vals[15]);
  results.bb8 = this->_16bit_int(vals[16], vals[17]);
  results.bb9 = this->_16bit_int(vals[18], vals[19]);
  results.bb10 = this->_16bit_int(vals[20], vals[21]);
  results.bb11 = this->_16bit_int(vals[22], vals[23]);
  results.bb12 = this->_16bit_int(vals[24], vals[25]);
  results.bb13 = this->_16bit_int(vals[26], vals[27]);
  results.bb14 = this->_16bit_int(vals[28], vals[29]);
  results.bb15 = this->_16bit_int(vals[30], vals[31]);
  results.bb16 = this->_16bit_int(vals[32], vals[33]);
  results.bb17 = this->_16bit_int(vals[34], vals[35]);
  results.bb18 = this->_16bit_int(vals[36], vals[37]);
  results.bb19 = this->_16bit_int(vals[38], vals[39]);
  results.bb20 = this->_16bit_int(vals[40], vals[41]);
  results.bb21 = this->_16bit_int(vals[42], vals[43]);
  results.bb22 = this->_16bit_int(vals[44], vals[45]);
  results.bb23 = this->_16bit_int(vals[46], vals[47]);
  results.bb24 = this->_16bit_int(vals[48], vals[49]);

  results.bbd0 = this->_16bit_int(vals[50], vals[51]);
  results.bbd1 = this->_16bit_int(vals[52], vals[53]);
  results.bbd2 = this->_16bit_int(vals[54], vals[55]);
  results.bbd3 = this->_16bit_int(vals[56], vals[57]);
  results.bbd4 = this->_16bit_int(vals[58], vals[59]);
  results.bbd5 = this->_16bit_int(vals[60], vals[61]);
  results.bbd6 = this->_16bit_int(vals[62], vals[63]);
  results.bbd7 = this->_16bit_int(vals[64], vals[65]);
  results.bbd8 = this->_16bit_int(vals[66], vals[67]);
  results.bbd9 = this->_16bit_int(vals[68], vals[69]);
  results.bbd10 = this->_16bit_int(vals[70], vals[71]);
  results.bbd11 = this->_16bit_int(vals[72], vals[73]);
  results.bbd12 = this->_16bit_int(vals[74], vals[75]);
  results.bbd13 = this->_16bit_int(vals[76], vals[77]);
  results.bbd14 = this->_16bit_int(vals[78], vals[79]);
  results.bbd15 = this->_16bit_int(vals[80], vals[81]);
  results.bbd16 = this->_16bit_int(vals[82], vals[83]);
  results.bbd17 = this->_16bit_int(vals[84], vals[85]);
  results.bbd18 = this->_16bit_int(vals[86], vals[87]);
  results.bbd19 = this->_16bit_int(vals[88], vals[89]);
  results.bbd20 = this->_16bit_int(vals[90], vals[91]);
  results.bbd21 = this->_16bit_int(vals[92], vals[93]);
  results.bbd22 = this->_16bit_int(vals[94], vals[95]);
  results.bbd23 = this->_16bit_int(vals[96], vals[97]);
  results.bbd24 = this->_16bit_int(vals[98], vals[99]);

  results.bw0 = this->_16bit_int(vals[100], vals[101]);
  results.bw1 = this->_16bit_int(vals[102], vals[103]);
  results.bw2 = this->_16bit_int(vals[104], vals[105]);
  results.bw3 = this->_16bit_int(vals[106], vals[107]);
  results.bw4 = this->_16bit_int(vals[108], vals[109]);
  results.bw5 = this->_16bit_int(vals[110], vals[111]);
  results.bw6 = this->_16bit_int(vals[112], vals[113]);
  results.bw7 = this->_16bit_int(vals[114], vals[115]);
  results.bw8 = this->_16bit_int(vals[116], vals[117]);
  results.bw9 = this->_16bit_int(vals[118], vals[119]);
  results.bw10 = this->_16bit_int(vals[120], vals[121]);
  results.bw11 = this->_16bit_int(vals[122], vals[123]);
  results.bw12 = this->_16bit_int(vals[124], vals[125]);
  results.bw13 = this->_16bit_int(vals[126], vals[127]);
  results.bw14 = this->_16bit_int(vals[128], vals[129]);
  results.bw15 = this->_16bit_int(vals[130], vals[131]);
  results.bw16 = this->_16bit_int(vals[132], vals[133]);
  results.bw17 = this->_16bit_int(vals[134], vals[135]);
  results.bw18 = this->_16bit_int(vals[136], vals[137]);
  results.bw19 = this->_16bit_int(vals[138], vals[139]);
  results.bw20 = this->_16bit_int(vals[140], vals[141]);
  results.bw21 = this->_16bit_int(vals[142], vals[143]);
  results.bw22 = this->_16bit_int(vals[144], vals[145]);
  results.bw23 = this->_16bit_int(vals[146], vals[147]);
  
  results.max_tof               = this->_16bit_int(vals[154], vals[155]);
  results.AMSamplingInterval    = this->_16bit_int(vals[156], vals[157]);
  results.AMIntervalCount       = this->_16bit_int(vals[158], vals[159]);
  results.AMMaxDataArraysInFile = this->_16bit_int(vals[160], vals[161]);
  results.AMOnlySavePMData      = (unsigned int)vals[162];
  results.AMFanOnIdle           = (unsigned int)vals[163];
  results.AMLaserOnIdle         = (unsigned int)vals[164];

  // Time-of-Flight to Sample Flow Rate ratio
  results.tof_sfr               = (unsigned int)vals[165];
  return results;
}

// Checked
String OPCN3::read_serial_number()
{
  // Read the serial number of the OPC
  // Only available on OPCN2's with firmware >v18
  // Ex.
  // $ alpha.read_serial_number();
  String result = "";
  byte vals[59];

  if (this->firm_ver.major < 18) {
    result = "";
  }
  else {
    digitalWrite(this->_CS, LOW);       // Pull the CS low
    SPI1.transfer(0x10);                 // Send the start command
    digitalWrite(this->_CS, HIGH);       // Pull the CS High

    delayMicroseconds(3000);

    // Iterate to read the entire string
    digitalWrite(this->_CS, LOW);
    // OPCN2 - 61
    for (int i = 0; i < 60; i++){
        vals[i] = SPI1.transfer(0x00);
        result += String((char)vals[i]);
        delayMicroseconds(4);
    }

    digitalWrite(this->_CS, HIGH);
  }

  result = result.trim();

  return result;
}

// Checked
struct PMData OPCN3::read_pm_data()
{
  // Read the PM Data and reset the histogram, return the struct
  // Only available on OPCN2's with firmware >v18
  // Ex.
  // $ alpha.read_pm_data();
  PMData data;
  byte vals[12];

  if (this->firm_ver.major < 18) {
      data.pm1  = -999;
      data.pm25 = -999;
      data.pm10 = -999;
  }
  else {
      // Read the data and clear the local memory
      digitalWrite(this->_CS, LOW);       // Pull the CS Low
      SPI1.transfer(0x32);                 // Transfer the command byte
      digitalWrite(this->_CS, HIGH);

      delayMicroseconds(12000);           // Delay for 12 ms

      // Send commands and build array of data
      digitalWrite(this->_CS, LOW);

      for (int i = 0; i < 12; i++){
          vals[i] = SPI1.transfer(0x00);
          delayMicroseconds(4);
      }

      digitalWrite(this->_CS, HIGH);      // Pull the CS High

      data.pm1  = this->_calculate_float(vals[0], vals[1], vals[2], vals[3]);
      data.pm25 = this->_calculate_float(vals[4], vals[5], vals[6], vals[7]);
      data.pm10 = this->_calculate_float(vals[8], vals[9], vals[10], vals[11]);
  }

  return data;
}

// Checked
struct HistogramData OPCN3::read_histogram(bool convert_to_conc)
{
  // Read the Histogram Data and reset the histogram, return the struct
  // convert_to_conc can be set to true if you would like the result
  // returned as concentrations (rather than raw counts) with units of
  // particles per cubic centimeter [#/cc]
  // Ex.
  // $ alpha.read_histogram(true)
  HistogramData data;
  int histogramReturns = 85;
  byte vals[histogramReturns + 1];
  //byte vals[62];

  // Read the data and clear the local memory
  digitalWrite(this->_CS, LOW);       // Pull the CS Low
  SPI1.transfer(0x30);                 // Transfer the command byte
  digitalWrite(this->_CS, HIGH);

  delayMicroseconds(12000);           // Delay for 12 ms

  // Send commands and build array of data
  digitalWrite(this->_CS, LOW);

  for (int i = 0; i < histogramReturns + 1; i++){
      vals[i] = SPI1.transfer(0x00);
      delayMicroseconds(4);
  }

  digitalWrite(this->_CS, HIGH);      // Pull the CS High

  // If convert_to_conc = True, convert from raw data to concentration
  double conv;

  if ( convert_to_conc != true ) {
      conv = 1.0;
  }
  else {
      conv = data.sfr * data.period;
  }

  // Calculate all of the values!
  data.bin0 = (double)this->_16bit_int(vals[0], vals[1]) / conv;
  data.bin1   = (double)this->_16bit_int(vals[2], vals[3]) / conv;
  data.bin2   = (double)this->_16bit_int(vals[4], vals[5]) / conv;
  data.bin3   = (double)this->_16bit_int(vals[6], vals[7]) / conv;
  data.bin4   = (double)this->_16bit_int(vals[8], vals[9]) / conv;
  data.bin5   = (double)this->_16bit_int(vals[10], vals[11]) / conv;
  data.bin6   = (double)this->_16bit_int(vals[12], vals[13]) / conv;
  data.bin7   = (double)this->_16bit_int(vals[14], vals[15]) / conv;
  data.bin8   = (double)this->_16bit_int(vals[16], vals[17]) / conv;
  data.bin9   = (double)this->_16bit_int(vals[18], vals[19]) / conv;
  data.bin10  = (double)this->_16bit_int(vals[20], vals[21]) / conv;
  data.bin11  = (double)this->_16bit_int(vals[22], vals[23]) / conv;
  data.bin12  = (double)this->_16bit_int(vals[24], vals[25]) / conv;
  data.bin13  = (double)this->_16bit_int(vals[26], vals[27]) / conv;
  data.bin14  = (double)this->_16bit_int(vals[28], vals[29]) / conv;
  data.bin15  = (double)this->_16bit_int(vals[30], vals[31]) / conv;
  data.bin16  = (double)this->_16bit_int(vals[32], vals[33]) / conv;
  data.bin17  = (double)this->_16bit_int(vals[34], vals[35]) / conv;
  data.bin18  = (double)this->_16bit_int(vals[36], vals[37]) / conv;
  data.bin19  = (double)this->_16bit_int(vals[38], vals[39]) / conv;
  data.bin20  = (double)this->_16bit_int(vals[40], vals[41]) / conv;
  data.bin21  = (double)this->_16bit_int(vals[42], vals[43]) / conv;
  data.bin22  = (double)this->_16bit_int(vals[44], vals[45]) / conv;
  data.bin23  = (double)this->_16bit_int(vals[46], vals[47]) / conv;

  data.bin1MToF = int(vals[48]) / 3.0;
  data.bin3MToF = int(vals[49]) / 3.0;
  data.bin5MToF = int(vals[50]) / 3.0;
  data.bin7MToF = int(vals[51]) / 3.0;

  //data.period = this->_calculate_float(vals[44], vals[45], vals[46], vals[47]);
  data.period = this->_16bit_int(vals[52], vals[53]);

  //data.sfr    = this->_calculate_float(vals[36], vals[37], vals[38], vals[39]);
  data.sfr = this->_16bit_int(vals[54], vals[55]);
  // This holds either temperature or pressure
  // If temp, this is temp in C x 10
  // If pressure, this is pressure in Pa
  //data.temp= this->_32bit_int(vals[40], vals[41], vals[42], vals[43]);
  data.temp = this->_16bit_int(vals[56], vals[57]);
  // Relative humidity
  data.humidity = this->_16bit_int(vals[58], vals[59]);
  
  data.pm1 = this->_calculate_float(vals[60], vals[61], vals[62], vals[63]);
  data.pm25 = this->_calculate_float(vals[64], vals[65], vals[66], vals[67]);
  data.pm10 = this->_calculate_float(vals[68], vals[69], vals[70], vals[71]);

  data.checksum = this->_16bit_int(vals[84], vals[85]);

  return data;
}
