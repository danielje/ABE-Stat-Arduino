/*
 * Creator: Daniel M. Jenkins
 * Date: February 14, 2017
 * Description: Basic calls to AD5933 network analyzer
 * 
 * partitioned into separate library for ABE-Stat (7/30/2017) to facilitate function management
 */

 /*
  * Function to compare magnitudes of two double values and determine if they are within a designated level of tolerance of each other...
  */
  boolean Ais_sameAs_B(double A, double B, double precision) { // precision number between 0 and 1
        double avgA_B = abs((A + B) / 2.0); // estimate magnitude of average of numbers
        double diffA_B = abs(A - B);
        if (diffA_B < (precision * avgA_B)) return true;
        else return false;        
  }

 /* 
  *  Function to compare if A is equal or greater than B (within a margin of error / precision- i.e. if A < B but within precision returns true
  */
  boolean Ais_greaterThan_B(double A, double B, double precision) {
        double margin = precision * (A + B) / 2.0;
        if (A > (B - margin)) return true;
        else return false;
  }

 /*
  * invoke formula to identify correct calibration index for current AD5933 Settings
  */
  int AD5933settingIndex() {
      int myIndex = excitationCode + (TIAGainCode * 4);
      if (PGAx5Setting) myIndex += 12;
      if (MCLKext) myIndex += 24;
      return myIndex;
  }

 /*
  * restore hardware settings from a given settings index value
  */
  void setAD5933configuration(int setterVal) {
      if (setterVal > 23) { // settings above 23 mean we are using external clock (slower frequencies)
            MCLKext = true;
            setterVal -= 24;  // now decrement by 24 since other settings are modulo 24...
      }
      else MCLKext = false;
      if (setterVal > 11) { // modulo 24 settings above 11 mean x5 programmable gain amplifier is on...
            PGAx5Setting = true;
            setterVal -= 12;
      }
      else PGAx5Setting = false;
      excitationCode = setterVal % 4;
      setTIAGain((setterVal - excitationCode) / 4);
      //TIAGainCode = (setterVal - excitationCode) / 4;
  }

/*
 * Function to estimate the AD5933 TIA reference voltage relative to board analog reference at the current setting (this is necessary to apply the correct bias voltage)
 */
  void AD5933_biasMeasure(double f) {
        int rememberExcitationCode = excitationCode;  // save excitation Code (because startAD5933Synthesizer is going to overwrite the value)
        TIA_AD5933();
        AD5933_Connect(false);  // disconnect the excitation signal of AD5933 (trying to isolate any other source of bias on network)
        DAC_AD5061_Connect(false);
        openCircuitConfig();  // put in open circuit configuration to test the bias voltage (if resistor connects working to reference, the ref voltage will be working voltage bias
        byte dummyStart = startAD5933Synthesizer(f, 0x00); // start signal on AD5933 just to get AD5933 TIA connected, so we can measure it's reference potential
              // from AD5933 relative to circuit board analog reference; AD5933 internal reference and board reference are both supposed to be Vcc / 2, but not actually physically connected
        delay(1000);  // let transient occur after removing Vout signal of AD5933 following impedance measurement
        AD5933_BiasV = (ADS1220_Diff_Voltage(0x04, 0x00) / 1000000);// - 2 * voffset / vslope; // when working and reference electrodes connected in open circuit config (and working connected to AD5933)
        resetElectrodeConfig(); // put electrodes back where you found them!
        AD5933_Connect(true);
        DAC_AD5061_Connect(true);
        excitationCode = rememberExcitationCode;  // put excitationCode value back!
        delay(50);
  }

/*
 * Measure cell capacitance (value returned is in pF) (in version 1.0.07 included check to make sure does not result in divide by 0
 */
double cellCapacitance() {
      AD5933_PowerOn(true); // have to turn on AD5933 to use it!
      DAC_AD5061_Connect(true); // connect DAC from AD5061 to network...  // calibration appears to be different with and without AD5061, so go ahead and connect,
          // then for calibrations set bias to 0- so make all measurements connected to this network (and apply "0" bias / output voltage)
      biasVoltage = 0.0;
      //DAC_AD5061_SetCalibratedVoltage(biasVoltage - (AD5933_BiasV)); // here we'll apply unbiased voltage
      DAC_AD5061_SetCalibratedVoltage(biasVoltage);
      AD5933_Connect(true); // connect cell input to network analyzer
      TIA_AD5933(); // connect cell output to network analyzer...
      setTIAGain(0x02); // use 2nd most sensitive gain- 1000000 ohm resistor results in poor amplifier performance on network analyzer
      excitationCode = 0x03;  // start with greatest excitation amplitude
      boolean q = false;
      double f = 100000.0;
      double capacitance = 0.0;
      MCLKext = false;
      while (!q) {  // keep analyzing until we get a good measurement- start at most sensitive settings i.e. highest frequency, largest gain and excitation voltage
          while (!testImpedance(f, excitationCode));  // first try to evaluate impedance at 100kHz, highest sensitivity settings, but without PGA which can distort signal...
          capacitance = (1000000000000 / (2 * PI * f * impedanceMagnitude()));
          double graw = rawAdmittance();
          if ((graw < (1.01 * AD5933_GperVoltOut)) && (graw > (0.05 * AD5933_GperVoltOut))) {
              q = true; // looks like we got a good value- stop analysis here...
          }
          else {
              Serial.print("j");  // instruction to Android not to timeout, or request new analysis (we're iteratively trying to complete an analysis)
              if (graw > (1.01 * AD5933_GperVoltOut)) {
                  if (TIAGainCode > 0x00) {
                      TIAGainCode--;
                      setTIAGain(TIAGainCode);
                  }
                  else if (f > 10001.0) f -= 10000.0;
                  else if (excitationCode > 0x00) {
                      excitationCode--;
                  }
                  else q = true; // flag end of routine if we're at the least sensitive setting and still appear to be saturating...
              }
              else { // only other condition in this loop is that we have a small value of TIA current (can be symptom of really high saturation if signal stays saturated at one rail
                  f = 10000.0;
                  excitationCode = 0x00;
                  setTIAGain(0x00);
                  while (!q) {  // if value is really small at most sensitive settings, go to least sensitive settings and work our way up to make sure we're not supersaturating TIA
                      Serial.print("j");  // instruction to Android not to timeout, or request new analysis (we're iteratively trying to complete an analysis)
                      if (excitationCode < 0x03) excitationCode++;
                      else if (f < 90001.0) f += 10000.0;
                      else if (TIAGainCode < 0x02) {
                          TIAGainCode++;
                          setTIAGain(TIAGainCode);
                      }
                      else q = true;  // if we get back up to the most sensitive settings, we know we've tried all conceivable settings to get valid measurement...
                      while (!testImpedance(f, excitationCode));  // first try to evaluate impedance at 100kHz, highest sensitivity settings, but without PGA which can distort signal...
                      capacitance = (1000000000000 / (2 * PI * f * impedanceMagnitude()));
                      double graw = rawAdmittance();
                      if (graw > (0.05 * AD5933_GperVoltOut)) q = true;
                  }
              }
          }
      }
      capacitance = (1000000000000 / (2 * PI * f * impedanceMagnitude()));
      return capacitance;
}

/* 
 *  Estimate the raw phase from realZ and imaginaryZ values; 
 *  The vector magnitudes are actually proportional to admittance, but the phase corresponds to the impedance phase (phase relationship of voltage to current)
 *  in version 1.0.07 modified to make sure real number (phasor 90 or -90 degree) returned if real component is zero- prevent divide by zero / nan result
 */
double rawPhase() {
    double phi = 0.0;
    double phasor = 0.0;
    if (realZ == 0) {
      if (imaginaryZ > 0) phasor = 90.0;
      else if (imaginaryZ < 0) phasor = -90.0;
    }
    else {
      double ratio = imaginaryZ * 1.0 / (realZ * 1.0);
      phi = 180.0 * atan(ratio) / PI;
      if (realZ > 0) phasor = phi;  // in 1st and 4th quadrants arctan gives direct phase angle
      else if (imaginaryZ > 0) phasor = 180.0 + phi;  // in second quadrant, arctan returns negative of angle, with magnitude equivalent to deviation from 180 deg (so just add the negative number)
      else phasor = -180.0 + phi; // in third quadrant, arctan returns positive angle; so just add the angle to -180...
    }
    return phasor;
}

/*
 * return the impedance phase from realZ and imaginaryZ values, corrected for calibrated system phase
 */
 double correctPhase() {
    int setting = AD5933settingIndex();
    /*double sysfase = (phaseOffset[AD5933settingIndex] + phaseSlope[AD5933settingIndex] * frequency);
    Serial.print("mZslope ");
    Serial.print(phaseSlope[AD5933settingIndex], 4);
    Serial.print(" ZOffset ");
    Serial.print(phaseOffset[AD5933settingIndex]);
    Serial.print(" systemPhase ");
    Serial.print(sysfase);
    Serial.print(" settingIndex ");
    Serial.print(AD5933settingIndex);
    Serial.print("\t");*/
    double frase = rawPhase() - (phaseOffset[setting] + phaseSlope[setting] * frequency);
    while (frase > 180.0) frase -= 360.0;
    while (frase < -180.0) frase += 360.0;  // add or subtract 360 degrees until result falls in range -180 to +180...
    return frase;
    
 }

/*
 * Return the raw admittance value
 */
 double rawAdmittance() {
    double realzsq = 1.0 * pow(realZ, 2);
    double imagzsq = 1.0 * pow(imaginaryZ, 2);
    double Gval = sqrt(realzsq + imagzsq);
    /*Serial.print("mRaw G value ");
    Serial.print(Gval, 2);
    Serial.print("\t");*/
    return Gval;
 }

/*
 * Return estimated impedance magnitude (corrected based on TIA gain setting, PGAx5, and excitation...
 * 
 * impedance is inversely proportional to rawAdmittance value, which itself is directly proportional to 
 * excitation voltage, feedback / TIAGainSetting, and PGAx5 setting (so these latter must be factored into the actual impedance estimate
 */
 double impedanceMagnitude() {
    int setting = AD5933settingIndex();
    /*double Zfcoefficient = (ZSlope[AD5933settingIndex] * frequency * 1.0) + ZOffset[AD5933settingIndex];
    Serial.print("mZslope ");
    Serial.print(ZSlope[AD5933settingIndex]);
    Serial.print(" ZOffset ");
    Serial.print(ZOffset[AD5933settingIndex]);
    Serial.print(" Zcoeff ");
    Serial.print(Zfcoefficient);
    Serial.print(" settingIndex ");
    Serial.print(AD5933settingIndex);
    Serial.print("\t");*/
    double Impedance = 1e9; // if raw admittance is zero, just use this value for impedance magnitude (pretty much upper limit or beyond range on device...
    double rawG = rawAdmittance();
    if (rawG != 0.0) Impedance = (((ZSlope[setting] * frequency * 1.0) + ZOffset[setting]) / (rawG * 1.0));
    /*Serial.print("mZabs ");
    Serial.print(Impedance, 0);
    Serial.print(" exc_code ");
    Serial.print(excitationCode);
    Serial.print(" TIAGain ");
    Serial.print(TIAGain, 0);
    if (PGAx5Setting) Serial.print("PGAx5");
    Serial.print("\t");*/
    return Impedance;
 }

 /*
 * returns peak excitation voltage at current excitation code setting
 */
 double excitationVoltage() {
      double Vexc = 0.1;
      switch (excitationCode) {
            case 0x00:
              Vexc  = 0.01;
              break;
            case 0x01:
              Vexc  = 0.02;
              break;
            case 0x02:
              Vexc  = 0.05;
              break;
            case 0x03:
              Vexc  = 0.1;
              break;
            default:
              break;
        }
       return Vexc;
 }

/*
 * 
 */
 double AD5933_RefTemperature() {
    AD5933_PowerOn(true);
    i2c_Write(AD5933NetworkAnalyzer, AD5933_Control, 0x90); // this part of register has bits we set to ask to start temperature measurement...
    //i2c_Write(AD5933NetworkAnalyzer, AD5933_Control + 1, 0x00); //this is the rest of bits for control, but most are reserved / 0 so just leave them. Only exceptions are reset bit (D4) 
              // and clock source which we generally want to leave as 0 (no reset; internal clock source)
    delayMicroseconds(810); // from documentation conversion requires 800 microseconds..., other option is to poll status register d0 but this is actually probably faster...
    int TData = (i2c_Read(AD5933NetworkAnalyzer, AD5933_TemperatureData) << 8) | i2c_Read(AD5933NetworkAnalyzer, AD5933_TemperatureData + 1);  // just read 16 bits of 4 byte data block returned into local variable- only reading two bytes
    if ((TData & 0x2000) != 0) { // if 14 bit twos complement conversion indicates negative number- sign extend to ...
        TData = 0xE000 | ((TData) & 0x1FFF);
    }
    double deviceTemp = TData * 0.03125;
    return deviceTemp;
 }
 
/*
 * Crude function to go through simple frequency "sweep" cycle of only a single frequency increment
 * Note for programming frequency registers, internal oscillator is 16.776 MHz
 * Start Frequency code = (4 * desired frequency / (clock freq)) * 2^27 
 * (same formula for frequency interval code)
 * Using an the internal oscillator as clock, code for start frequency of 30kHz is 960070  (0x0EA646)
 * frequency increment code for 10 Hz is 320 (0x000140)
 * latest version with freq and excitation arguments; excitation is code for 
 * 
 * returns boolean indicating if valid result occurs (i.e. if repeated measurements are identical, something is wrong so power is cycled, user can ask for new measurement
 */
 boolean testImpedance(double freq, byte excitation) {
      byte lowControlNybble = startAD5933Synthesizer(freq, excitation);
      delay(500); // let transients die out from reconnecting synthesizer signal to output pin from ground, now that we've started the synthesizer...
      AD5933_Connect(true); // now DC bias of synthesizer signal should have settled to near analog reference..
      delay(500); // let more time pass to allow equilibrium behavior to be established with the new AC component of signal...
      double rz = 0;
      double iz = 0;
      int sampleNumber = 3;
      for (int a = 0; a < sampleNumber; a++) {
            readAD5933Data(lowControlNybble);
            repeatedAD5933Measurement = true; // here we'll simply make 5 readings to see how they change over time
            rz += realZ;
            iz += imaginaryZ;
      }
      realZ = rz / sampleNumber;
      imaginaryZ = iz / sampleNumber;
      return true;
 }

 /*
 * Finds optimal settings for measuring impedance of an unknown network at a given frequency and excitation voltage, by evaluating all available settings
 * and returning the one that results in combination of lowest value of impedance AND highest digitized admittance value (this should only occur in the highest sensitivity
 * settings that return the largest admittance value, without saturating which would result in a quantifiable drop in estimated impedance... Just to be certain
 * method also checks to make sure we're not near the admittanceMargin...
 * 
 * Returns the settings index for optimal measurement...
 */
 void optimalImpedanceSetting(double freak, byte excited, boolean returnSetting) {
      int currentSetting = AD5933settingIndex();  // record current setting index so we can put it back later!
      frequency = freak;  // need this value to estimate impedance magnitudes from calibration data!
      //Serial.print("mOptimizing AD5933 settings!\t");
      double lowCredibleZ;  // local storage variable to save lowest predicted value of Z (within margin of precision) at setting that also has the highest digitized admittance value
      double highG; // local storage variable to save highest observed value of digitized admittance corresponding with lowest predicted value of Z (within margin of precision)
      
      setTIAGain(0x00); // start at lowest sensitivity...
      PGAx5Setting = false; // and lowest PGA gain...
      
      byte loControlNybble = startAD5933Synthesizer(freak, excited);  // get the signal started!

      delay(500); // at start of signal AD5933 Vout goes from Gnd to defined DC offset; let AD5933 offset settle with AC coupling before trying to take measurement

      readAD5933Data(loControlNybble); // issue command to start impedance measurement with given settings, and read data registers when measurement complete...
      repeatedAD5933Measurement = true;  // set this flag so readAD5933 function knows next time to request repeat measurement at same frequency...
      lowCredibleZ = impedanceMagnitude();  // lowest credible value of impedance
      highG = rawAdmittance();  // first readings are automatically the "optimal" of the set that we've seen so far...

      double gExpectedRatio = ((highG * lastKnownZ) / (excitationVoltage() * AD5933_GperVoltOut * TIAGain));
      if (PGAx5Setting) gExpectedRatio /= 5.0;
      
      int optimalSetting = AD5933settingIndex(); // stores the setting index for the optimal setting...
      /*Serial.print("mSetting index ");
      Serial.print(AD5933settingIndex());
      Serial.print(" impedance ");
      Serial.print(lowCredibleZ);
      Serial.print(" gRatio ");
      Serial.print(gExpectedRatio, 2);
      Serial.print(" phase ");
      Serial.print(correctPhase());
      Serial.print('\t');*/
      boolean eoOptimization = false;
      //while ((!PGAx5Setting) && (TIAGainCode >= 0x02)) {
      while (!eoOptimization) {
            if (TIAGainCode < 0x02) {
                  TIAGainCode++;
                  setTIAGain(TIAGainCode);  // if not at most sensitive TIA gain, increment gain...
            }
            else if (!PGAx5Setting) { // if we haven't already been evaluating with x5 programmable gain amplifier on, turn it on now (and start TIA gain from least sensitive settings again)
                //Serial.print("mGot to PGAx5!\t");
                PGAx5Setting = true;
                setTIAGain(0x00);
                loControlNybble &= 0x0e;  // mask off the last bit of control nybble (turns on PGAx5 setting)
            }
            /*Serial.print("moperating setting ");
            Serial.print(AD5933settingIndex());
            Serial.print('\t');*/
            delay(1);
            readAD5933Data(loControlNybble); // issue command to start impedance measurement with given settings, and read data registers when measurement complete...
            double g = rawAdmittance();
            double gRatio = ((g * lastKnownZ) / (excitationVoltage() * AD5933_GperVoltOut * TIAGain));
            if (PGAx5Setting) gRatio /= 5.0;
            double z = impedanceMagnitude();
            /*Serial.print("mSetting index ");
            Serial.print(AD5933settingIndex());
            Serial.print(" impedance value ");
            Serial.print(z);
            Serial.print(" gRatio ");
            Serial.print(gRatio, 2);
            Serial.print(" phase ");
            Serial.print(correctPhase());
            Serial.print('\t');*/
            //if (Ais_greaterThan_B(g, highG, 0.01) && Ais_greaterThan_B(lowZ, z, 0.1) && (g < admittanceMargin)) {
            //if (Ais_greaterThan_B(g, highG, 0.01) && Ais_greaterThan_B(lowZ, z, 0.1) && (g < admittanceMargin)) {  // remove constraints on Z, because at low sensitivity settings accuracy of Z estimate is poor...
            //if (gRatio > (0.99 * gExpectedRatio)) {  // should be a reliable way to find optimal setting- at low sensitivities, noise will increase digitized admittance
            /*
             * compare results to see which setting gives most plausible result; first set of criteria is to use newest setting if digitized admittance from 
             * previous setting reflects a TIA voltage less than 20 mV, and ratio of new admittance / expected new admittance is lower than similar ratio from old setting
             * (this suggests that the old low sensitivity setting is significantly biased due to noise); the other criteria is if digitized admittance is larger on new setting
             * but estimated impedance from the new setting is still lower (this pushes us to the highest sensitivity setting where we don't have evidence of saturation
             * based on our limited set of observations...
             */
            boolean swapSetting = false;
            if (highG < 150.0) {
                  //Serial.print("mOld setting not sensitive!\t");
                  if (gRatio < gExpectedRatio) {
                      //Serial.print("mNew setting results closer to expected!\t");
                      swapSetting = true;
                  }
            }
            else if (Ais_sameAs_B(gRatio, gExpectedRatio, 0.05)) {
                  //Serial.print("msettings approximately consistent!\t");
                  if (g > highG) {
                      //Serial.print("mNew setting is more sensitive- use it!\t");
                      swapSetting = true;
                  }
            }
            //if (((highG < (AD5933_GperVoltOut / 10.0)) && (gRatio < gExpectedRatio)) || (Ais_greaterThan_B(g, highG, 0.01) && Ais_greaterThan_B(lowCredibleZ, z, 0.1) && (g < admittanceMargin))) {
            if (swapSetting) {
                  optimalSetting = AD5933settingIndex();
                  lowCredibleZ = z;
                  gExpectedRatio = gRatio;
                  highG = g;
                  //Serial.print("mNew optimum setting!\t");
            }
            if (PGAx5Setting && (TIAGainCode >= 0x02)) eoOptimization = true;
      }
      /*Serial.print("mOptimal setting index ");
      Serial.print(optimalSetting);
      Serial.print(" impedance value ");
      Serial.print(lowCredibleZ);
      Serial.print(" digitized admittance ");
      Serial.print(highG);
      Serial.print('\t');*/
      //MCLK_Enable(false); // make sure to turn off external clock at end of measurement...
      if (returnSetting) setAD5933configuration(optimalSetting);
      else setAD5933configuration(currentSetting);
 }

 /* 
 *  Function reads the real and imaginary data registers of AD5933 (need to pass the low nybble of control (LCN) register so excitation and PGA gain are set correctly)...
 *  
 */
 void readAD5933Data(byte LCN) {
      byte regValue = (0x20 | LCN);
      if (repeatedAD5933Measurement) regValue = (0x40 | LCN); // compose command to repeat measurement if we've already made a measurement...
      i2c_Write(AD5933NetworkAnalyzer, AD5933_Control, regValue); // now issue frequency sweep command (now device will start measuring impedance after equilibration cycles)
      
      while((i2c_Read(AD5933NetworkAnalyzer, AD5933_Status) & 0x02) == 0x00)
        delay(1); // wait for status register to confirm that valid conversion/ impedance data measured; don't let MCU timeout...
       
      realZ = (i2c_Read(AD5933NetworkAnalyzer, AD5933_RealData) << 8) | i2c_Read(AD5933NetworkAnalyzer, AD5933_RealData + 1); // first read initial data to get baseline
      imaginaryZ = (i2c_Read(AD5933NetworkAnalyzer, AD5933_ImaginaryData) << 8) | i2c_Read(AD5933NetworkAnalyzer, AD5933_ImaginaryData + 1);
 }

 /*
 * Function issues all commands to AD5933 to start synthesizer signal with the given settings; returns the "lowControlNybble" written to the AD5933 control register
 * that sets the excitation voltage amplitude and enable bit (high nybble of register composes command to start synthesizer, intiate measurement, repeat measurement, etc)
 */
 byte startAD5933Synthesizer(double freaky, byte exciter) {
      TIA_AD5933();  // set TIA to return to AD5933 network analyzer...
      //DAC_AD5061_Connect(true); // connect DAC from AD5061 to network (all AD5933 calibrations are conducted with this switch connected/ closed, so connect it even if 0 bias)
      
      //AD5933_Connect(true); // for arbitrary bias need to connect voltage sources from DAC (AD5061) and network analyzer (AD5933)
      //resetElectrodeConfig();
      //DAC_AD5061_SetCalibratedVoltage(biasVoltage - (AD5933_BiasV));

      frequency = freaky; // make sure global variable knows the actual frequency so that impedance calibrations can be applied...
      
      //Serial.print("mStarting Synthesizer!\t");
      AD5933_PowerOn(true);
      
      double extClockMinF = (fMCLK / (16.0 * 1024.0));  // minimum frequency increment with external clock to sample integer number of cycles
      double intClockMinF = (fCLK / (16.0 * 1024.0)); // minimum frequency increment with internal clock to sample integer number of cycles
      
      if (freaky < extClockMinF) freaky = extClockMinF; // minimum signal frequency is ~15 Hz (assuming we are using external clock)
      else if (freaky > 100000.0) freaky = 100000.0;  // maximum signal frequency is 100 kHz (limited by AD5933 Low Pass Filtering of synthesizer signal)
      
      if (freaky > ((fMCLK / 32.0) + 0.9)) MCLKext = false; // don't use external clock if requested frequency exceeds value that can be programmed with external clock
      else if (freaky < (intClockMinF - 1.0)) MCLKext = true; // for frequencies below where we can't sample integer number of cycles with internal clock, use external clock...
      
      if (exciter > 0x03) exciter = 0x03;
      else if (exciter < 0x00) exciter = 0x00;  // set boundaries on excitation code, from datasheet: b00 is 2.0Vp-p, b01 is 200mV, b10 is 400mV, b11 is 1V
      excitationCode = exciter;  // record argument sent to this function in external variable (before it's transformed to the excitation map of AD5933 control register)
      exciter = (exciter + 0x01) & 0x03;// after adding one and "AND"ing off all but bits 0 & 1, map of input argument to actual datasheet excitation codes: b00 => b01(200mV);
                                //b01 => b10(400mV); b10 => b11(1.0V); b11 => b00 (2.0V)
                                
      long fAD5933CLK = fCLK; // by default use frequency of internal clock
      // check the desired setting for external clock, and write appropriate setting to AD5933 control register
      if (MCLKext) {
          fAD5933CLK = fMCLK; // only overwrite default clock frequency if using external clock...
      }
      
      long DDSphaseAccumulatorFactor = pow(2, 29);  // # of increments of clock frequency for setting synthesizer frequency ((2^27) * fclk divisor to Phase Accumulator (4))
      
      byte lowCtrlNybble = (exciter << 1); // left shift excitation code into corresponding bits in low nybble of control register, append 1 to LSbit (PGA gain = 1)
      if (!PGAx5Setting) lowCtrlNybble |= 0x01;  // set lowest bit on if PGA gain is only 1 (not 5)
      
      long freq_code = (long) (DDSphaseAccumulatorFactor * (freaky / fAD5933CLK));  // using system clock 16.776 MHz and formula freq_code = 4 * f * 2^27/ fclock
      byte regContent = (byte) ((freq_code & 0x00ff0000) >> 16);
      i2c_Write(AD5933NetworkAnalyzer, AD5933_StartFrequency, regContent);
      regContent = (byte) ((freq_code & 0x0000ff00) >> 8);
      i2c_Write(AD5933NetworkAnalyzer, AD5933_StartFrequency + 1, regContent);
      regContent = (byte) (freq_code & 0x000000ff);
      i2c_Write(AD5933NetworkAnalyzer, AD5933_StartFrequency + 2, regContent);  // start frequency is now programmed...
      
      /*i2c_Write(AD5933NetworkAnalyzer, AD5933_FrequencyIncr, 0x00);
      i2c_Write(AD5933NetworkAnalyzer, AD5933_FrequencyIncr + 1, 0x01);
      i2c_Write(AD5933NetworkAnalyzer, AD5933_FrequencyIncr + 2, 0x40); // frequency increment is now 10 Hz (if set to internal clock), even though we're not going to use it...
      */
      
      i2c_Write(AD5933NetworkAnalyzer, AD5933_IncrNumber, 0x00);
      i2c_Write(AD5933NetworkAnalyzer, AD5933_IncrNumber + 1, 0x01);  // just program a single frequency interval / measurement
      
      i2c_Write(AD5933NetworkAnalyzer, AD5933_SettleCycleNumber, 0x00);
      if (freaky < 100) i2c_Write(AD5933NetworkAnalyzer, AD5933_SettleCycleNumber + 1, 0x10);  //allow 16 full cycles of new frequency to be applied for low frequencies
      else if (freaky < 1000) i2c_Write(AD5933NetworkAnalyzer, AD5933_SettleCycleNumber + 1, 0x40); // 64 full cycles equilibrarion for freq < 1000 Hz
      else i2c_Write(AD5933NetworkAnalyzer, AD5933_SettleCycleNumber + 1, 0x80); // 128 full cycles equilibration for freq > 1000 Hz

      AD5933_Connect(false);  // just make absolute certain synthesizer is disconnected from network before major transient occurs from resetting in next lines of code...
      delay(1);
      //openCircuitConfig();
      regContent = (0xb0 | lowCtrlNybble);
      i2c_Write(AD5933NetworkAnalyzer, AD5933_Control, regContent);   // put device on standby (PLEASE DISCONNECT FROM NETWORK FIRST AS SYNTHESIZER WILL BE GROUNDED UNTIL FREQUENCY SWEEP STARTS!)...
      //resetElectrodeConfig();// also put network in open circuit config while resetting digital synthesizer- voltage reset with capacitive input coupling still results in big transient into network!


      MCLK_Enable(MCLKext); // enable the external clock if it is to be used...
      if (MCLKext) {  // once all other settings are finished, set clock source before starting sweep...
          i2c_Write(AD5933NetworkAnalyzer, AD5933_Control + 1, 0x08); // no reset, use external clock 250kHz
      }
      else i2c_Write(AD5933NetworkAnalyzer, AD5933_Control + 1, 0x00);
      
      regContent = (0x10 | lowCtrlNybble);
      i2c_Write(AD5933NetworkAnalyzer, AD5933_Control, regContent);   // issue "initialize" command (start Vout from digital synthesizer)!

      repeatedAD5933Measurement = false;
      return lowCtrlNybble; // make sure we send this back to the calling function so they can reassemble commands to AD5933 with same excitation and PGA settings...
 }
