
#if defined(QUADP) || defined(QUADX) || defined(Y4)
  #define NUMBER_MOTOR 4
#endif

uint8_t PWM_PIN[8] = {MOTOR_ORDER};

//TODO:
//volatile uint8_t atomicServo[4] = {125,125,125,125};

void writeMotors() { // [1000;2000] => [125;250]
    for(uint8_t i=0;i<min(NUMBER_MOTOR,4);i++)
      analogWrite(PWM_PIN[i], motor[i]>>3);
}

void writeAllMotors(int16_t mc) {   // Sends commands to all motors
  for (uint8_t i =0;i<NUMBER_MOTOR;i++)
    motor[i]=mc;
  writeMotors();
}

#if defined(LOG_VALUES) || (POWERMETER == 1)
void logMotorsPower() {
  uint32_t amp;
  /* true cubic function; when divided by vbat_max=126 (12.6V) for 3 cell battery this gives maximum value of ~ 1000 */
  const uint32_t amperes[64] =   {0,4,13,31,60,104,165,246,350,481,640,831,1056,1319,1622,1969,2361,2803,3297,3845,4451,5118,5848,6645,
	                             7510,8448,9461,10551,11723,12978,14319,15750,17273,18892,20608,22425,24346,26374,28512,30762,33127,35611,
	                             38215,40944,43799,46785,49903,53156,56548,60081,63759,67583,71558,75685,79968,84410,89013,93781,98716,103821,
	                             109099,114553,120186,126000 };
  if (vbat) { // by all means - must avoid division by zero 
    for (uint8_t i =0;i<NUMBER_MOTOR;i++) {
      amp = amperes[(motor[i] - 1000)>>4] / vbat; // range mapped from [1000:2000] => [0:1000]; then break that up into 64 ranges; lookup amp
      #ifdef LOG_VALUES
         pMeter[i]+= amp; // sum up over time the mapped ESC input 
      #endif
      #if (POWERMETER == 1)
         pMeter[PMOTOR_SUM]+= amp; // total sum over all motors
      #endif
    }
  }
}
#endif

void initOutput() {
  for(uint8_t i=0;i<NUMBER_MOTOR;i++)
    pinMode(PWM_PIN[i],OUTPUT);
  writeAllMotors(1000);
  delay(300);
    #if defined(A0_A1_PIN_HEX)
      pinMode(5,INPUT);pinMode(6,INPUT);     // we reactivate the INPUT affectation for these two PINs
      pinMode(A0,OUTPUT);pinMode(A1,OUTPUT);
    #endif
}

void mixTable() {
  int16_t maxMotor;
  uint8_t i,axis;
  static uint8_t camCycle = 0;
  static uint8_t camState = 0;
  static uint32_t camTime = 0;
  
  #define PIDMIX(X,Y,Z) rcCommand[THROTTLE] + axisPID[ROLL]*X + axisPID[PITCH]*Y + YAW_DIRECTION * axisPID[YAW]*Z

  #if NUMBER_MOTOR > 3
    //prevent "yaw jump" during yaw correction
    axisPID[YAW] = constrain(axisPID[YAW],-100-abs(rcCommand[YAW]),+100+abs(rcCommand[YAW]));
  #endif
  #ifdef QUADX
    motor[0] = PIDMIX(-1,+1,-1); //REAR_R
    motor[1] = PIDMIX(-1,-1,+1); //FRONT_R
    motor[2] = PIDMIX(+1,+1,+1); //REAR_L
    motor[3] = PIDMIX(+1,-1,-1); //FRONT_L
  #endif

  maxMotor=motor[0];
  for(i=1;i< NUMBER_MOTOR;i++)
    if (motor[i]>maxMotor) maxMotor=motor[i];
  for (i = 0; i < NUMBER_MOTOR; i++) {
    if (maxMotor > MAXTHROTTLE) // this is a way to still have good gyro corrections if at least one motor reaches its max.
      motor[i] -= maxMotor - MAXTHROTTLE;
    motor[i] = constrain(motor[i], MINTHROTTLE, MAXTHROTTLE);    
    if ((rcData[THROTTLE]) < MINCHECK)
      #ifndef MOTOR_STOP
        motor[i] = MINTHROTTLE;
      #else
        motor[i] = MINCOMMAND;
      #endif
    if (armed == 0)
      motor[i] = MINCOMMAND;
  }

  #if defined(LOG_VALUES) || (POWERMETER == 1)
    logMotorsPower();
  #endif
}

