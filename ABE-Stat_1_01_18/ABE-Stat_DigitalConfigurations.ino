/*
 * Creator: Daniel M. Jenkins
 * Date: February 14, 2017
 * Description: Basic digital communication and configuration functions; including basic functions to 
 * set analog switches to configure operation of analog circuits...
 * 
 * Partitioned into separate libraries, including for higher level Analog Circuit reading / writing, and 
 * for configuring and reading network analyzer results (7/30/2017)
 */
 
/* 
 *  basic functions for reading / writing 1 byte of data from / to AD5933
 *  Uses single byte read/writes because even for the 2 or 3 byte control registers is almost as or is more efficient
 *  than using the block write / read instructions + address pointer instructions for AD5933
 */
byte i2c_Read(int i2cAddr, int regAddr)
{
  byte value;
  Wire.beginTransmission(i2cAddr);  // address of device to read data from
  Wire.write(regAddr); // Address of register to be read
  Wire.endTransmission(); 
  Wire.requestFrom(i2cAddr, 1);  // request 1 byte of data starting at regAddr
  while (Wire.available()) value = Wire.read();  // read and return the data returned by device
  return value;
}
/*
 * To write block of data to different registers on AD5933 requires separate block write and address pointer instructions/ codes for AD5933
 * So to write a block of 3 bytes using this approach requires transmission of 8 bytes + Start , Read/Write, Acknowledge, and stop bits
 * compared to transmission of 9 bytes and flow control bits for writing 3 bytes with individual register write instructions
 * For less than 3 blocks it's actually faster/more efficient to just write bytes using individual register write instructions
 * So just leave it simple here and have single byte write instructions
 */
void i2c_Write(int i2cAddr, int regAddr, byte reg_value)
{
  Wire.beginTransmission(i2cAddr);
  Wire.write(regAddr);
  Wire.write(reg_value); // write number of bytes from array, starting at address of array
  Wire.endTransmission();
}

/*
 * Separate instruction for Writing to ADG715 (device only has one data register, which does not need to be designated/addressed to be written to;
 * similarly, cannot write block of more than 1 byte at a time...
 */
 void i2c_WriteADG715(byte switchStates)
 {
    Wire.beginTransmission(ADG715OctalSwitch);  // note on this device don't need to designate register address to write to...
    Wire.write(switchStates);
    Wire.endTransmission();
 }

/*
 * Function to write individual bytes from a double data type into EEPROM
 */
 void EEPROM_Double_Write(int address, double number) {
    doubleValue.floatingValue = number;
    for (int a = 0; a < sizeof(double); a++) EEPROM.write(address + a, doubleValue.doubleBytes[a]);
 }

/*
 * Function to read back individual bytes from EEPROM and recompose double data type
 */
 double EEPROM_Double_Read(int address) {
    for (int a = 0; a < sizeof(double); a++) doubleValue.doubleBytes[a] = EEPROM.read(address + a);
    return (doubleValue.floatingValue);
 }

/*
 * Function to write a 16 bit integer to EEPROM
 */
 void EEPROM_Int_Write(int address, unsigned int value) {
    byte val = (value & 0xff00) >> 8;
    EEPROM.write(address, val);
    val = value & 0x00ff;
    EEPROM.write(address + 1, val);
 }

 /*
 * Function to write a 16 bit integer to EEPROM // only used for measuring 100% battery charge- full charge cant be negative number, so make unsigned
 * to accommodate sum of more individual observations (form of averaging / rejecting random noise)
 */
 unsigned int EEPROM_Int_Read(int address) {
    unsigned int value = (EEPROM.read(address) << 8) | EEPROM.read(address + 1);
    return value;
 }

 void resetElectrodeConfig() {      
      switch (electrodeConfig) {
          case TWO_ELECTRODE_CONFIG:
            twoElectrodeConfig();
            Serial.print("mABE in two electrode config\t");
            break;
          case THREE_ELECTRODE_CONFIG:
            threeElectrodeConfig();
            Serial.print("mABE in three electrode config\t");
            break;
          case OPEN_CIRCUIT_CONFIG:
            openCircuitConfig();
            Serial.print("mABE in open circuit config\t");
            break;
          default:
            openCircuitConfig();
            Serial.print("mABE defaulted to open circuit config\t");
            break;
      }
  }

/*
 * bring O11 low to remove battery power from device (note that USB power always powers device when connected)
 */
 void devicePower(boolean affirmative) {
    if (affirmative) ShiftRegisterValue |= 0x0800; // set bit O11 in shift register as high to hold power on...
    else ShiftRegisterValue &= 0xf7ff;  // set bit O11 in shift register as low to power down...
    sh_reg(ShiftRegisterValue);
 }

/*
 * Read the AD5933 Status register (just to test i2c communication with device
 */
 byte AD5933StatusUpdate() {
    return i2c_Read(AD5933NetworkAnalyzer, AD5933_Status);
 }

/*
 * Determine if Bluetooth is connected...
 */
 boolean BT_Connected() {
    pinMode(0, INPUT);
    if (((ShiftRegisterValue & 0x000c) >> 2) != 0x0000) { // make sure BT_Connected signal is connected to GPIO00 through switch- if not connect it...
      ShiftRegisterValue &= 0xfff3; // mask off bits 2 & 3 from shift register value
      sh_reg(ShiftRegisterValue);
    }
    return digitalRead(0); // now return the inverse of the state read on GPIO00 (data ready on logic low) 
 }

/* 
  Connect !CS of referenced device to !CS of ESP8266 (to enable SPI communication)
  On this design there are only two "slave" SPI devices (ADS1220 ADC or AD5061 DAC)
*/
void SPI_Chip_Select(int device) {
    if (device > 1) device = 1;  // only have 2 devices- 0 (ADS1220) or 1 (AD5061) connected to SPI...
    if (device < 0) device = 0;
    int address = (device & 0x0001);  // add address, and make sure that !enable bit (O01) for switch is off (otherwise no device will connect)
    ShiftRegisterValue &= 0xfffc;  // mask off outputs O00 - O01 on shift register value
    ShiftRegisterValue |= address;  // and write the new values into them...
    sh_reg (ShiftRegisterValue);
}

/*
 * set !enable high on !CS select switch (for using CS pin / GPIO15  signal for SDA and i2c communication, instead of for SPI)
 */
 void Release_Chip_Select() {
  ShiftRegisterValue |= 0x0002; // force O01 high, leave all others unchanged
  sh_reg(ShiftRegisterValue);
 }

/*
 * set !enable high on switch routing BT_Connected and !DRDY signals to GPIO00 (Generally the latter signals will be high, so will prevent MCU going into programming mode on reset)
 */
 void Release_GPIO00() {
   ShiftRegisterValue |= 0x0008; // force O03 high- switch to GPIO00 disabled, leave all others unchanged
   sh_reg(ShiftRegisterValue);
 }

/*
 * Connect working electrode through Trans-Impedance Amplifier / LMP7721 network to ADS1220 Analog to Digital Converter
 * sends switch signal of "0" to both ADG779 (Shift register outputs O04 and O05)
 */
 void TIA_LMP7721() {
    ShiftRegisterValue &= 0xffcf; // force O04 and O05 low (connect working electrode to LMP7721 TIA network)
    sh_reg(ShiftRegisterValue);
 }

/*
 * Connect working electrode through Trans-Impedance Amplifier on AD5933 network analyzer
 * sends switch signal of "1" to both ADG779 (Shift register outputs O04 and O05)
 */
 void TIA_AD5933() {
    ShiftRegisterValue |= 0x0030; // force O04 and O05 high (connect working electrode to AD5933 TIA network)
    sh_reg(ShiftRegisterValue);
 }

/*
 * Set the gain / feedback resistance of Trans-impedance network (through switch ADG704)
 * Sets address bits A0 and A1 on ADG704 (shift register O08 and O09)
 * argument of 0 (b00) connects 1k resistor, 1 (b01) connects 10k, 2 (b10) connects 100k, 3 (b11) connects 1M
 * algorithm automatically turns enable input into ADG704 on (shift register O10)
 */
 void setTIAGain(byte gain_code) {
    if (gain_code <= 0) gain_code = 0x00;
    else if (gain_code >= 3) gain_code = 0x03;  // apply limits to range of gain_code, and swap 0x00 and 0x03
              // after swap, b00 => b03(1kOhm); b01 => b01(10k); b10 => b10(1M); b11 => b00(100M) (bit swapping is vestigial from early prototypes of NC-1194_Potentiostat
              // which in which resistor layout was out of order by magnitude
    unsigned int shifty = gain_code << 8; // move code to corresponding shift register outputs for A0 and A1 (O08 and O09)
    shifty |= 0x0400; // also turn on bit O10 for shift register (enable pin for ADG704)
    ShiftRegisterValue &= 0xf8ff; // mask off bits O08, O09, and O10 
    ShiftRegisterValue |= shifty; // and fill in with gain code and enable bit for ADG704
    sh_reg(ShiftRegisterValue);
    TIAGainCode = gain_code;
    switch (gain_code) {
       case 0x00:
          TIAGain = Gain0;
          break;
       case 0x01:
          TIAGain = Gain1;
          break;
       case 0x02:
          TIAGain = Gain2;
          break;
       case 0x03:
          TIAGain = Gain3;
          break;
       default:
          break;
    }
 }

/*
 * Turn on signal for red light of 3-color LED (also need to invoke illuminateLED() to conduct through PMOS to supply)
 */
 void redLED(boolean red) {
    if (red) ShiftRegisterValue |= 0x0040; // force O06 high (drive red LED NMOS)
    else ShiftRegisterValue &= 0xffbf; // force O06 low (don't drive red LED NMOS)
    sh_reg(ShiftRegisterValue);
 }

/*
 * illuminate tri-color LED (green if circuit powered, blue when connected to bluetooth, unless redLED selected above);
 * logic low- PMOS switch to supply tri-color LED...
 */
 void illuminateLED() {
    ShiftRegisterValue &= 0xff7f; // force O07 low (PMOS channel closed- conduct Vdd to LEDs)
    sh_reg(ShiftRegisterValue);
 }

/*
 * remove power to tri-color LED (green if circuit powered, blue when connected to bluetooth, unless redLED selected above);
 * logic low- PMOS switch to supply tri-color LED...
 */
 void noLED() {
    ShiftRegisterValue |= 0x0080; // force O08 high (PMOS channel open / remove Vdd from LEDs)
    sh_reg(ShiftRegisterValue);
 }

/*
 * Connect (sum) DAC(AD5061) / LPF output to Potentiostat network (in revised design this is done through one of the ADG715 switches set though i2c)
 */
 void DAC_AD5061_Connect(boolean decider) {
   if (decider) ADG715SwitchStates |= 0x80;  // set most significant bit (Switch 07), leave all others untouched
   else ADG715SwitchStates &= 0x7f;  // force most significant (Switch 07) low, leave all others unchanged
   i2c_WriteADG715(ADG715SwitchStates);
 }

/*
 * Connect (sum) AD5933 Network analyzer output to potentiostat network
 */
 void AD5933_Connect(boolean decider) {
   if (decider) ADG715SwitchStates |= 0x40;  // set second most significant bit (Switch 06), leave all others untouched
   else ADG715SwitchStates &= 0xbf;  // force second most significant bit (Switch 06) low, leave all others untouched
   i2c_WriteADG715(ADG715SwitchStates);
 }

/*
 * Conduct through PMOS to power AD5933 and it's external clock (if true), otherwise close channel / remove power
 */
 void AD5933_PowerOn(boolean powerStatus) {
    if (powerStatus) ShiftRegisterValue &= 0xefff;  // set bit 12 to 0 (pull inversion layer in PMOS to power network analyzer DVDD)
    else ShiftRegisterValue |= 0x1000;  // set bit 12 to 1 (cut off inversion- no power to network)
    sh_reg(ShiftRegisterValue);
 }

 /*
  * use control register of AD5933 to put device into power down mode (better
  * alternative than removing power from supply pin, as this slows down i2c data 
  * transfer, resulting in missed commands / failure to update ADG715 switch states
  * for electrode configuration and excitation signal composition
  */
  void AD5933_PowerDown() {
      i2c_Write(AD5933NetworkAnalyzer, AD5933_Control, 0xa0);
  }

 /*
 * Conduct through PMOS to power external 250kHz clock for AD5933
 */
 void MCLK_Enable(boolean powerStatus) {
    if (powerStatus) ShiftRegisterValue &= 0xdfff;  // set bit 13 to 0 (pull inversion layer in PMOS to power clock)
    else ShiftRegisterValue |= 0x2000;  // set bit 12 to 1 (cut off inversion- no power to network)
    sh_reg(ShiftRegisterValue);
 }

 /*
  * Connect potentiostat network in 3-electrode configuration 
  * working to TIA (S0 = 1)
  * feedback from reference (S3 = 1)
  * isolated counter and reference (S4 = 0)
  * amplifier output to counter (S5 =1)
  */
  void threeElectrodeConfig() {
      ADG715SwitchStates |= 0x29; // force S5, S3, S0 high, don't change others
      ADG715SwitchStates &= 0xeb; // force S4 & S2 low- isolate reference and counter electrodes
      i2c_WriteADG715(ADG715SwitchStates);
  }

 /*
  * Connect potentiostat network in 2-electrode configuration 
  * working to TIA (S0 = 1)
  * feedback from reference (S3 = 1)
  * isolated counter and reference (S4 = 1)
  * amplifier output to counter (S5 =1)
  * 
  * (non-working electrode can be connected to counter or reference terminal)
  */
  void twoElectrodeConfig() {
      ADG715SwitchStates |= 0x39; // force S5, S4, S3, S0 high, don't change others
      ADG715SwitchStates &= 0xfb; // force S2 low (no direct voltage follower / ref bypass on biasing amplifier); others unchanged

      //Test below use voltage follower configuration on first amplifier (S2), connect ref and counter electrodes to output (S4 & S5)
      // and open the feedback from reference amplifier to biasing amplifier (S3). Working electrode still needs to connect (S0)

      /*ADG715SwitchStates |= 0x35; // force S0, S2, S4 & S5 high
      ADG715SwitchStates &= 0xf7; // force S3 low*/
      
      i2c_WriteADG715(ADG715SwitchStates);
  }

  /*
  * Configure potentiostat to measure open circuit potential
  * working to TIA (S0 = 1)
  * no feedback from reference (just use voltage follower to measure reference potential; S3 = 0)
  * disconnect counter and reference (S4 = 0)
  * disconnect counter electrode from amplifier (S5 = 0)
  * 
  * circuit measures open circuit potential at "working" contact relative to "reference" contact
  */
  void openCircuitConfig() {
    ADG715SwitchStates |= 0x05; // make sure S0  & S2 high (connect working electrode to virtual ground / feedback / ADC; bypass biasing amplifier to stabilize output), don't change others
    ADG715SwitchStates &= 0xc7;  // make sure S5, S4, S3 low (auxiliary connections and reference to voltage sources); all others don't change (S0  & S2 already high, S1 doesn't matter)
    i2c_WriteADG715(ADG715SwitchStates);
  }

/*
 * Read !DRDY pin from ADS1220 to determine if data conversion is completed (returns true for completed conversion; false if not)
 */
boolean ADS1220DataReady() {
  pinMode(0, INPUT);
  if (((ShiftRegisterValue & 0x000c) >> 2) != 0x0001) { // make sure !DRDY is connected to GPIO00 through switch- if not connect it...
    ShiftRegisterValue &= 0xfff3; // mask off bits 2 & 3 from shift register value
    ShiftRegisterValue |= 0x0004; // force bit 2 high (3 is already low)- connects !DRDY to GPIO00
    sh_reg(ShiftRegisterValue);
  }
  return !digitalRead(0); // now return the inverse of the state read on GPIO00 (data ready on logic low) 
}

/*
 * wait for ADS1220DataReady (polls !DRDY pin to determine end of conversion, but if not converted within 51 ms 
 * (greater than largest possible conversion time) will exit
 * use this function so application doesn't get hung up if !DRDY pin becomes inoperable or disconnected...
 */
void waitADS1220DataReady() {
    long startT = millis();
    boolean timeOut = false;
    while (!ADS1220DataReady() && !timeOut) {
        if ((millis() - startT) > 51) timeOut = true; // make timeout > 50 ms; if only wait 50 ms may read value before conversion complete for normal low data rate...
    }
    Release_GPIO00(); // already received !DRDY signal or timed out- disconnect signals from GPIO00 so high signal does not prevent ESP8266 from going into reprogram mode on reset
}

/*
  write 24 bits to the shift register chips 74HC595;
  Careful with data types! if statements appear to only operate on least significant 16 bits (?)
*/
void sh_reg(unsigned int val) {
  unsigned int chosenbit;
  
  digitalWrite(CS_RCK, LOW);  // prime register clock (active on positive edge)
  digitalWrite(CLOCK, LOW);  // prime the data/ serial clock (active on positive edge)
  for (int i = 0; i < 16; i++) {
    chosenbit = (val >> i);  // mask for the current bit in shift register
    //dummy = val & mask;

    if (chosenbit & 0x0001) digitalWrite(SER, HIGH);

    else digitalWrite(SER, LOW);  // write appropriate value to input

    digitalWrite(CLOCK, HIGH);
    digitalWrite(CLOCK, LOW);  // and clock it into the register
  }
  
  digitalWrite(CS_RCK, HIGH);
  digitalWrite(CS_RCK, LOW);  // clock the register into the output
  ShiftRegisterValue = val;
}
