/*
 * Creator: Daniel M. Jenkins
 * Date: July 30, 2017
 * Description: Basic calibration routines for ABE-Stat (i.e. calibrate DAC, transimpedance gains, 
 * system phase and admittance / impedance coefficients...
 * 
 * Partitioned into separate libraries, including for higher level Analog Circuit reading / writing, and 
 * for configuring and reading network analyzer results (7/30/2017)
 */
 
/*
 * After startup, call into Calibration function to manage calibrations with 'c'
 */
 void Calibrate() {

    delay(1500); // hard code delay just so that information doesn't get garbled with incoming data...
    Serial.print("mCalibration Data:");
    Serial.print(vslope, 4);
    Serial.print(',');
    Serial.print(voffset, 4);
    Serial.print(',');
    Serial.print(vlowlimit, 3);
    Serial.print(',');
    Serial.print(vhighlimit, 3);
    Serial.print(',');
    Serial.print(Gain0, 1);
    Serial.print(',');
    Serial.print(Gain1, 1);
    Serial.print(',');
    Serial.print(Gain2, 1);
    Serial.print(',');
    Serial.print(Gain3, 1);
    Serial.print(',');
    Serial.print(AD5933_GperVoltOut,1);
    Serial.print(',');
    Serial.print(fCLK_Factor, 4);
    Serial.print(',');
    Serial.print(AD5933_BiasV);
    Serial.print(',');
    for (int AD5933Settings = 0; AD5933Settings < 48; AD5933Settings++) {
      Serial.print(phaseSlope[AD5933Settings], 6);
      Serial.print(',');
      Serial.print(phaseOffset[AD5933Settings], 1);
      Serial.print(',');
      Serial.print(ZSlope[AD5933Settings], 6);
      Serial.print(',');
      Serial.print(ZOffset[AD5933Settings], 1);
      Serial.print(',');
    }
    Serial.print('\t');

    AD5933_PowerOn(true); // turn power on to AD5933 so we can change electrode configuration to 2 electrode
    twoElectrodeConfig();
    AD5933_PowerOn(false);  // then turn it back off to start...

    completedCalibrationsCode = 0x0000;
    calibratedAD5933biasAndGperVolt = false;
    boolean ex = false;
    while (!ex) {
      while (!Serial.available());
      command = Serial.read();
      switch(command) {
        case 'b':
          calibrateBattery();
          break;
        case 'd':
          vslope = Serial.parseFloat();
          voffset = Serial.parseFloat();
          EEPROM_Double_Write(DAC_SLOPE_EEPROM_ADDRESS, vslope);
          EEPROM_Double_Write(DAC_OFFSET_EEPROM_ADDRESS, voffset);
          EEPROM.commit();
          break;
        case 'e':
          writePhaseCoefficients();
          break;
        case 'g':
          calibrateGain();
          break;
        case 'u': // instruction calibrate ratio of actual AD5933 fCLK to nominal / specification value
          calibratefCLK();
          break;
        case 'v':// Android says set voltage...
          calibrateDAC();
          break;
        case 'x':// exit calibration routine
          ex = true;
          break;
        case 'y':
          writeAdmittanceCoefficients();
          break;
        case 'z':
          calibrateZ();
          break;
        default:
          break;
      }
    }
 }

 /*
  * read the returning coefficients of system phase calibration and write into EEPROM
  */
  void writePhaseCoefficients() {
          int settingsIndex = Serial.parseInt();
          phaseSlope[settingsIndex] = Serial.parseFloat();
          phaseOffset[settingsIndex] = Serial.parseFloat();
          int EEPROMOffsetAddress = (4 * EEPROMBlockSize) * settingsIndex;  // offset from given base address for given index
          EEPROM_Double_Write(PHASE_SLOPE_EEPROM_ADDRESS + EEPROMOffsetAddress, phaseSlope[settingsIndex]);
          EEPROM_Double_Write(PHASE_OFFSET_EEPROM_ADDRESS + EEPROMOffsetAddress, phaseOffset[settingsIndex]);
          EEPROM.commit();
  }

  /*
  * read the returning coefficients of admittance* load resistance calibration and write into EEPROM
  */
  void writeAdmittanceCoefficients() {
          int settingsIndex = Serial.parseInt();
          ZSlope[settingsIndex] = Serial.parseFloat();
          ZOffset[settingsIndex] = Serial.parseFloat();
          int EEPROMOffsetAddress = (4 * EEPROMBlockSize) * settingsIndex;  // offset from given base address for given index
          EEPROM_Double_Write(Z_SLOPE_EEPROM_ADDRESS + EEPROMOffsetAddress, ZSlope[settingsIndex]);
          EEPROM_Double_Write(Z_OFFSET_EEPROM_ADDRESS + EEPROMOffsetAddress, ZOffset[settingsIndex]);
          EEPROM.commit();
  }

 /*
  * Calibrate the full charge value of battery (assumes that battery is fully charged)
  */
 void calibrateBattery() {
          unsigned int fullCharge = 0;
          for (int refV = 0; refV < 64; refV++) {
            fullCharge += analogRead(0);
          }
          Serial.print("mFull battery reading ");
          Serial.print(fullCharge);
          Serial.print('\t');
          EEPROM_Int_Write(V_BATT_FULL_EEPROM_ADDRESS, fullCharge);
          EEPROM.commit();
 }

 /*
  * Calibration routine for observed AD5933 clock frequency vs. nominal value: system first starts up digital synthesizer (DDS) at predefined value near 2kHz,
  * then user enters observed frequency and sends information back
  */
 void calibratefCLK() {
          MCLKext = false;  // make sure we use the internal AD5933 clock that we're calibrating!
          double nominalFrequency = fCLK / (8.0 * 1024.0);  // this value should be 2047.85 Hz for fCLK nominal value of 16.776 MHz
          byte lowControlNybble = startAD5933Synthesizer(nominalFrequency, 0x03); // start AD5933 DDS at nominal frequency and highest amplitude so user can easily see DDS signal!
          while (!Serial.available());  // wait for user to send back observed frequency...
          if (Serial.read() == 'w') { // code with returning observed frequency should be "w"
              double observedFrequency = Serial.parseFloat();
              fCLK_Factor = observedFrequency / nominalFrequency;
              EEPROM_Double_Write(FCLK_CORRECTION_EEPROM_ADDRESS, fCLK_Factor);
              EEPROM.commit();
          }
          byte AD5933_PowerDown = (0xa0 | lowControlNybble);  // control register 1 value to put AD5933 in "power-down" mode
          i2c_Write(AD5933NetworkAnalyzer, AD5933_Control, AD5933_PowerDown);   // write power down byte to control register...
          AD5933_PowerOn(false);  // turn power off
 }

/*
 * Set desired nominal voltage, and report this with actual voltage so Android can determine calibration equation
 * Then find the actual voltage for lowest and highest voltage sets...
 */
 void calibrateDAC() {
      AD5933_PowerOn(false);  // remove power from AD5933 for calibrating DAC
      DAC_AD5061_Connect(true); // connect AD5061 Digital to Analog Converter from network
      AD5933_Connect(false);  // AND connect network analyzer input (here we are testing the superposition of the two signal sources)
      setTIAGain(0x01);  // make sure selected feedback resistor connected for TIA (otherwise non-ideal op-amp performance can bias the "reference"
      TIA_LMP7721();  // connect to TIA amplifier LMP7721 to ensure virtual ground reference...
      twoElectrodeConfig(); // to allow calibration with single resistor between Working and reference contacts
      double nominalVoltage = -1.50;
      while (nominalVoltage <= 1.51) {
        setAndReportVoltage(nominalVoltage);
        delay(40);
        nominalVoltage += 0.05;
      }
      DAC_AD5061_SetVoltage(-1.65);
      delay(10);  // let transients settle...
      vlowlimit = -ADS1220_Diff_Voltage(0x04, 0x00) / 1000000.0;
      DAC_AD5061_SetVoltage(1.65);
      delay(10);  // let transients settle...
      vhighlimit = -ADS1220_Diff_Voltage(0x04, 0x00) / 1000000.0;
      EEPROM_Double_Write(DAC_LOW_LIMIT_EEPROM_ADDRESS, vlowlimit);
      EEPROM_Double_Write(DAC_HIGH_LIMIT_EEPROM_ADDRESS, vhighlimit);
      EEPROM.commit();
      Serial.print('l');  // flag that voltage range limits coming...
      Serial.print(vlowlimit, 4);
      Serial.print('\t');
      Serial.print(vhighlimit, 4);
      Serial.print('\t');
      Serial.print('c');  // flag that the linear calibration range has been completed.
 }

/*
 * Set voltage to desired level, then report back observed voltage
 */
void setAndReportVoltage(double volts) {
      DAC_AD5061_SetVoltage(volts);
      delay(10);  // let transients settle...
      double actualVoltage = -ADS1220_Diff_Voltage(0x04, 0x00) / 1000000.0;
      Serial.print('v');
      Serial.print(volts, 4);
      Serial.print('\t');
      Serial.print(actualVoltage, 4);
      Serial.print('\t');
 }

/*
 * Calibrate system phase (folded into calibrateZ function as of ABE-Stat1_0_06
 */
/*void calibratePhase() {
    //DAC_AD5061_Connect(false); // remove DAC_AD5061 signal...
    DAC_AD5061_Connect(true); // connect DAC from AD5061 to network...  // calibration appears to be different with and without AD5061, so go ahead and connect,
          // then for calibrations set bias to 0- so make all measurements connected to this network (and apply "0" bias / output voltage)
    DAC_AD5061_SetCalibratedVoltage(0.0); // here we'll apply unbiased voltage
    AD5933_Connect(true);  // AND connect network analyzer voltage source to network
    TIA_AD5933();  // connect network back to TIA on network analyzer AD5933
    twoElectrodeConfig(); // to allow calibration with single resistor between Working and reference contacts
    long frequent = 6000; // start at frequency of 6000 (at lower frequencies, finite samples at given clock rate do not cover enough cycles for accurate FFT
    autoRangeTestImpedance(frequent); // use autoranging function at first to find settings to get reliable admittance values at frequency
    while (frequent < 100500) {
       TestImpedance(frequent, excitationCode, PGAx5Setting);
       Serial.print('f');
       Serial.print(frequent);
       Serial.print('\t');
       Serial.print(rawPhase(), 2); // send the raw phase value (during calibration, with resistor, raw phase is the system phase...
       Serial.print('\t');
       frequent += 1000;  // increment frequency and take / report new calibration measurement
    }
    Serial.print('c');  // flag that the linear calibration range has been completed (Android can now Execute_LSR() with compiled data)
}*/

/*
 * Calibrate impedances
 * Note that for calibration using settings with internal AD5933 clock, calibrations use "nominal" frequencies assuming that fCLK is highly accurate;
 * Starting version 1.01.13, frequency values reported back in EIS function are corrected for deviation from fCLK in datasheet...
 */
void calibrateZ() {
    double networkR = cellResistance(); // As of ABE-Stat1_0_06, evaluate the load resistance for AD5933 impedance calibrations- don't just read Android input from user Serial.parseFloat();
    AD5933_PowerOn(true); // now turn on the network analyzer- this is what we're actually calibrating!
    delay(100);  // small delay to let AD5933 power up and get itself sorted out...
    Serial.print('r');
    Serial.print(networkR, 1);
    Serial.print('\t'); // reports the load resistance (impedance)
    boolean completedAvailableSettings = false;
    double admitCoefficient = 1.0;
    DAC_AD5061_Connect(true); // connect DAC from AD5061 to network...  // calibration appears to be different with and without AD5061, so go ahead and connect,
          // then for calibrations set bias to 0- so make all measurements connected to this network (and apply "0" bias / output voltage)
    biasVoltage = 0.0;
    //DAC_AD5061_SetCalibratedVoltage(biasVoltage); // here we'll apply unbiased voltage
    AD5933_Connect(true);  // AND connect network analyzer voltage source to network
    TIA_AD5933();  // connect network back to TIA on network analyzer AD5933
    electrodeConfig = TWO_ELECTRODE_CONFIG;
    resetElectrodeConfig();
    PGAx5Setting = true;
    TIAGainCode = 0x02;
    setTIAGain(TIAGainCode);
    excitationCode = 0x03;
    while (!completedAvailableSettings) {
        int settingIndex = excitationCode + (TIAGainCode * 4);
        if (PGAx5Setting) settingIndex += 12;
        double Voutexpected = (TIAGain / networkR) * excitationVoltage(); // estimate expected TIA output voltage based on settings and load impedance
        if (PGAx5Setting) Voutexpected *= 5.0;
        MCLKext = false;  // make first test measurement with internal clock / higher frequency
        /*if ((Voutexpected > 0.2) && (Voutexpected < 1.0)) {  // we haven't calibrated the admittance coeffient corresponding to 1V, or bias, and conditions appear to be good for it...
            AD5933_biasMeasure(20000);
            //DAC_AD5061_SetCalibratedVoltage(AD5933_BiasV); // here we'll apply unbiased voltage
            DAC_AD5061_SetCalibratedVoltage(0.0);
            if (testImpedance(20000, excitationCode)) { // test impedance at given conditions- at 20kHz (make sure we're well above bottom of range so we get good number of cycles sampled.
                  AD5933_GperVoltOut = rawAdmittance() / Voutexpected;
            }
            else AD5933_GperVoltOut = 10000;
            Serial.print("h");
            Serial.print(AD5933_GperVoltOut, 1);
            Serial.print("\t");
            Serial.print(AD5933_BiasV, 4);
            Serial.print("\t");
            if (!calibratedAD5933biasAndGperVolt) {
                  EEPROM_Double_Write(AD5933_G_PER_V_EEPROM_ADDRESS, AD5933_GperVoltOut);
                  EEPROM_Double_Write(AD5933_BIAS_V_EEPROM_ADDRESS, AD5933_BiasV);
                  EEPROM.commit();
                  calibratedAD5933biasAndGperVolt = true; // we've calibrated values, so don't overwrite again this invocation of calibration app...
                  }
        }*/
        long bitTest = (0x0001 << settingIndex);
        boolean conditionTested = (completedCalibrationsCode & bitTest) >> settingIndex; // determine whether this condition has been calibrated yet...
        if (!conditionTested && (Voutexpected > 0.09) && (Voutexpected < 1.01)) {  // if we haven't calibrated current settings, and predicted output looks like a suitable range calibrate this setting!
            MCLKext = false;  // for higher frequencies use the internal AD5933 clock
            double freeIncrement = fCLK / (16.0 * 1024.0);//fCLK / (16.0 * 1024.0);  // start internal frequency calibration at 4kHz (not 1kHz- admittance at first data point very small
            double frequent = 2 * freeIncrement; // start at frequency of 15000 (at lower frequencies, finite samples at given clock rate do not cover enough cycles for accurate FFT

            //AD5933_biasMeasure(frequent);
            //DAC_AD5061_SetCalibratedVoltage(0.0);
            DAC_AD5061_SetCalibratedVoltage(-AD5933_BiasV);
            if (testImpedance(frequent, excitationCode)) { // test impedance at given conditions- at 20kHz (make sure we're well above bottom of range so we get good number of cycles sampled.
                  AD5933_GperVoltOut = rawAdmittance() / Voutexpected;
            }
            Serial.print("h");
            Serial.print(AD5933_GperVoltOut, 1);
            Serial.print("\t");
            Serial.print(AD5933_BiasV, 4);
            Serial.print("\t");
            //DAC_AD5061_SetCalibratedVoltage(0.0);
            /*for (int junky = 0; junky < 6; junky++) {
                if (TestImpedance(frequent, excitationCode, PGAx5Setting)) {
                    admitCoefficient = networkR * rawAdmittance();
                    Serial.print('z');
                    Serial.print(AD5933settingIndex);
                    Serial.print('\t');
                    Serial.print(frequent);
                    Serial.print('\t');
                    Serial.print(admitCoefficient, 2);
                    Serial.print('\t');
                    Serial.print(rawPhase(), 2);
                    Serial.print('\t');
                }
            }*/
            while (frequent < 100500) {
              if (testImpedance(frequent, excitationCode)) {
                  admitCoefficient = networkR * rawAdmittance();
                  Serial.print('z');
                  Serial.print(AD5933settingIndex());
                  Serial.print('\t');
                  Serial.print(frequent);
                  Serial.print('\t');
                  Serial.print(admitCoefficient, 2);
                  Serial.print('\t');
                  Serial.print(rawPhase(), 2);
                  Serial.print('\t');
                  frequent += freeIncrement;  // increment frequency and take / report new calibration measurement
                  delay(1); // don't let MCU timeout...
              } // if not valid result, make network analyzer repeat measurement at same frequency (after cycling power in TestImpedance())
            }
            Serial.print('c');  // flag that the linear calibration range has been completed (Android can now Execute_LSR() with compiled data)
            // completed calibration of performance at given settings over frequencies with internal AD5933 clock

            AD5933_PowerOn(false);  // turn off power to let device rest between calibration settings
            delay(200);  // wait a little to make sure calibration data come back, and to prevent MCU from timing out...
            while (Serial.available()) {
              char commandant = Serial.read();
              if (commandant == 'y') writeAdmittanceCoefficients();
              else if (commandant == 'e') writePhaseCoefficients();
            }

            MCLKext = true;  // for lower frequencies use the external 250kHz clock for AD5933
            AD5933_PowerOn(true);  // restore power to analyze calibration settings with external clock...
            delay(100);  // turn chip back on, and let it settle a little bit...
            settingIndex += 24; // new range of setting indeces for calibrations with external clock (otherwise index order for AD5933 settings the same)
            freeIncrement = fMCLK / (16.0 * 1024.0); // for external MCLK frequency increment, to cover integer number of cycles with 1024 ADC samples at fclk / 16
            frequent = freeIncrement * 4.0;
            double fmax = 2000.0;//(fMCLK / 32.0) - 1.0;  // ~7.8kHz for 250kHz clock  // just leave it at 4kHz since we wont use internal clock above this...
            /*for (int junky = 0; junky < 6; junky++) {
                if (TestImpedance(frequent, excitationCode, PGAx5Setting)) {
                    admitCoefficient = networkR * rawAdmittance();
                    Serial.print('z');
                    Serial.print(AD5933settingIndex);
                    Serial.print('\t');
                    Serial.print(frequent);
                    Serial.print('\t');
                    Serial.print(admitCoefficient, 2);
                    Serial.print('\t');
                    Serial.print(rawPhase(), 2);
                    Serial.print('\t');
                }
            }*/
            //AD5933_biasMeasure(frequent);
            //DAC_AD5061_SetCalibratedVoltage(0.0);
            DAC_AD5061_SetCalibratedVoltage(-AD5933_BiasV);
            if (testImpedance(frequent, excitationCode)) { // test impedance at given conditions- at 20kHz (make sure we're well above bottom of range so we get good number of cycles sampled.
                  AD5933_GperVoltOut = rawAdmittance() / Voutexpected;
            }
            Serial.print("h");
            Serial.print(AD5933_GperVoltOut, 1);
            Serial.print("\t");
            Serial.print(AD5933_BiasV, 4);
            Serial.print("\t");
            //DAC_AD5061_SetCalibratedVoltage(AD5933_BiasV); // here we'll apply unbiased voltage
            DAC_AD5061_SetCalibratedVoltage(0.0);
            
            while (frequent < (fmax)) {
              /*Serial.print("mEvaluating ");
              Serial.print(frequent);
              Serial.print("Hz\t");*/
              if (testImpedance(frequent, excitationCode)) {
                    admitCoefficient = networkR * rawAdmittance();
                    Serial.print('z');
                    Serial.print(AD5933settingIndex());
                    Serial.print('\t');
                    Serial.print(frequent);
                    Serial.print('\t');
                    Serial.print(admitCoefficient, 2);
                    Serial.print('\t');
                    Serial.print(rawPhase(), 2);
                    Serial.print('\t');
                    frequent += freeIncrement * 16.0;
                    /*if (frequent < 1200.0) frequent += freeIncrement;  // increment frequency and take / report new calibration measurement
                    else frequent += (freeIncrement * 8.0); // increase frequency */
                    delay(1); // don't let MCU timeout...
              }// if result returned doesn't make sense, frequency is not incremented and same frequency value is requested again after cycling power on AD5933...
            }
            Serial.print('c');  // flag that the linear calibration range has been completed (Android can now Execute_LSR() with compiled data)

            AD5933_PowerOn(false);  // turn off power to let device rest between calibration settings
            delay(200);  // wait a little to make sure calibration data come back, and to prevent MCU from timing out...
            while (Serial.available()) {
              char commandant = Serial.read();
              if (commandant == 'y') writeAdmittanceCoefficients();
              else if (commandant == 'e') writePhaseCoefficients();
            }
            
            completedCalibrationsCode = (completedCalibrationsCode | bitTest);  // add current conditions to set of that calibration is complete
            Serial.print('i');  // now send Android the latest completed settings code to display (so user knows if more conditions need to be calibrated, with different networkR
            Serial.print(completedCalibrationsCode, BIN);  // send as binary number to facilitate identification of settings remaining to calibrate...
            Serial.print('\t');
        } // now we've either completed calibration at given setting, or determined that it has already been completed or is expected to result in sub-optimal results with given networkR

        if (excitationCode > 0x00) {  // so update settings and run through cycle again...
            excitationCode--;
        }
        else if (PGAx5Setting) {
            PGAx5Setting = false;
            excitationCode = 0x03;  // start back at excitation code = 0x03 at new PGA setting
        }
        else if (TIAGainCode > 0x00) {
            TIAGainCode--;
            setTIAGain(TIAGainCode);
            PGAx5Setting = true;
            excitationCode = 0x03;
        }
        else {
            completedAvailableSettings = true; // if excitation, gain, and PGA settings are *all* at lowest values at completion of calibration, we've completed every possible combination...
            Serial.print("y");  // flag for Android to know that we've calibrated as many settings as give good results with given networkR (so swap out resistor and start calibration again)
        }
    }
}

/*
 * Calibrate TIA gain
 */
void calibrateGain() {
    AD5933_PowerOn(false);  // remove power from AD5933- just a noise source for measuring feedback resistors / gains...
    int index = Serial.parseInt();
    if (index < 0) index = 0;
    else if (index > 0x03) index = 0x03;
    double networkR = Serial.parseFloat();
    DAC_AD5061_Connect(true); // connect AD5061 Digital to Analog Converter from network
    AD5933_Connect(false);  // AND connect network analyzer input (here we are testing the superposition of the two signal sources)
    setTIAGain(index);  // make sure selected feedback resistor connected for TIA (otherwise non-ideal op-amp performance can bias the "reference"\
    TIA_LMP7721();  // connect to TIA amplifier LMP7721 to ensure virtual ground reference...
    twoElectrodeConfig(); // to allow calibration with single resistor between Working and reference contacts
    double vamp = 2.1;
    double vsource = 1.5;
    double vworking = 0.0;
    while (vamp > (vhighlimit - 0.02)) {  // set threshold somewhat low- is it not trigering near the voltage reference value 2.048 (?)
      DAC_AD5061_SetVoltage(vsource);
      delay(10); // let transient settle
      vworking = -ADS1220_Diff_Voltage(0x04, 0x00) / 1000000.0;
      vamp = ADS1220_Diff_Voltage(0x02, 0x00) / 1000000.0;
      vsource /= 2.0; // if offscale divide source voltage by 2- keep repeating until get onto scale for valid measurement
    }
    double gain = networkR * vamp / vworking;
    switch (index) {
      case 0x00:
        Gain0 = gain;
        EEPROM_Double_Write(GAIN_0_EEPROM_ADDRESS, gain);
        break;
      case 0x01:
        Gain1 = gain;
        EEPROM_Double_Write(GAIN_1_EEPROM_ADDRESS, gain);
        break;
      case 0x02:
        Gain2 = gain;
        EEPROM_Double_Write(GAIN_2_EEPROM_ADDRESS, gain);
        break;
      case 0x03:
        Gain3 = gain;
        EEPROM_Double_Write(GAIN_3_EEPROM_ADDRESS, gain);
        break;
      default:
        break;
    }
    EEPROM.commit();
    Serial.print('g');
    Serial.print(index);
    Serial.print('\t');
    Serial.print(gain, 1);
    Serial.print('\t');

    if (!calibratedAD5933biasAndGperVolt && (TIAGainCode == 0x02)) { // if we haven't calibrated the digitized admittance response of AD5933 this iteration of power cycle, let's do it now...
        double networkR = cellResistance(); // As of ABE-Stat1_0_06, evaluate the load resistance for AD5933 impedance calibrations- don't just read Android input from user Serial.parseFloat();
        Serial.print('r');
        Serial.print(networkR, 1);
        Serial.print('\t'); // reports the load resistance (impedance)
        DAC_AD5061_SetCalibratedVoltage(0.0); // apply zero bias then configure the rest of network...
        PGAx5Setting = true;
        AD5933_PowerOn(true); // now turn on the network analyzer- this is what we're actually calibrating!

        //AD5933_biasMeasure(20000);
        DAC_AD5061_SetCalibratedVoltage(-AD5933_BiasV);
        double Voutexpected = (TIAGain / networkR) * excitationVoltage(); // estimate expected TIA output voltage based on settings and load impedance
        if (PGAx5Setting) Voutexpected *= 5.0;
        
        MCLKext = false;  // make first test measurement with internal clock / higher frequency

        while (Voutexpected > 1.01) { // start at most sensitive settings- adjust down until we expect output signal to fall within range of analog rails
            if (PGAx5Setting) PGAx5Setting = false;
            else if (excitationCode > 0x00) {
                excitationCode--;
            }
            Voutexpected = (TIAGain / networkR) * excitationVoltage();
            if (PGAx5Setting) Voutexpected *= 5.0;
        }
        DAC_AD5061_Connect(true); // connect DAC from AD5061 to network...  // calibration appears to be different with and without AD5061, so go ahead and connect,
        AD5933_Connect(true);  // AND connect network analyzer voltage source to network
        TIA_AD5933();  // connect network back to TIA on network analyzer AD5933
        electrodeConfig = TWO_ELECTRODE_CONFIG;
        resetElectrodeConfig();
        delay(1000);
        testImpedance(20000.0, excitationCode);  // test impedance at given conditions- at 20kHz (make sure we're well above bottom of range so we get good number of cycles sampled.
        AD5933_GperVoltOut = rawAdmittance() / Voutexpected;
        Serial.print("h");
        Serial.print(AD5933_GperVoltOut, 1);
        Serial.print("\t");
        Serial.print(AD5933_BiasV, 4);
        Serial.print("\t");
        EEPROM_Double_Write(AD5933_G_PER_V_EEPROM_ADDRESS, AD5933_GperVoltOut);
        EEPROM_Double_Write(AD5933_BIAS_V_EEPROM_ADDRESS, AD5933_BiasV);
        EEPROM.commit();
        calibratedAD5933biasAndGperVolt = true; // we've calibrated values, so don't overwrite again this invocation of calibration app...
        AD5933_PowerOn(false);  // remove power from AD5933- just a noise source for measuring feedback resistors / gains...
        AD5933_Connect(false);  // AND connect network analyzer input (here we are testing the superposition of the two signal sources)
        MCLKext = false;  // also make sure external clock of AD5933 is disabled
        MCLK_Enable(false);  // leave off external clock on AD5933 for this measurement (make measurement with internal clock)
        TIA_LMP7721();  // connect to TIA amplifier LMP7721 to ensure virtual ground reference...
    }
}

