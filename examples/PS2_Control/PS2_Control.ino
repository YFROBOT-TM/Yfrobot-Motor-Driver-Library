/*
  PS2 control - PS2手柄控制4轮麦克纳姆轮小车

  YFROBOT-ZL
  yfrobot@qq.com
  WWW.YFROBOT.COM
  08/20/2020
*/

//#define DEBUGESERIAL

#include <PS2X_lib.h>  //for v1.8
#include <PinChangeInterrupt.h>
#include <MotorDriver_PCA9685.h>

/******************************************************************
   set pins connected to PS2 controller:
     - 1e column: original
     - 2e colmun: Stef?
   replace pin numbers by the ones you use
 ******************************************************************/
#define PS2_DAT        10  //
#define PS2_CMD        11  //
#define PS2_CS         12  //
#define PS2_CLK        14  //

/******************************************************************
   select modes of PS2 controller:
     - pressures = analog reading of push-butttons
     - rumble    = motor rumbling
   uncomment 1 of the lines for each mode selection
 ******************************************************************/
//#define pressures   true
#define pressures   false
#define rumble      true
//#define rumble      false

#define TRIGGER RISING // RISING FALLING
// choose a valid PinChangeInterrupt pin of your Arduino board
#define M1PHASEAPIN 4
#define M1PHASEBPIN 2
#define M2PHASEAPIN 5
#define M2PHASEBPIN 3
#define M3PHASEAPIN 7
#define M3PHASEBPIN 6
#define M4PHASEAPIN 9
#define M4PHASEBPIN 8

PS2X ps2x; // create PS2 Controller Class
MotorDriver_PCA9685 motorDirver = MotorDriver_PCA9685();// called this way, it uses the default address 0x40

//right now, the library does NOT support hot pluggable controllers, meaning
//you must always either restart your Arduino after you connect the controller,
//or call config_gamepad(pins) again after connecting the controller.

int error = 0;
byte type = 0;
byte vibrate = 0;

#define MAXSPEED 4096
#define MINSPEED 2000

int maxSpeed = 4000;            // 最大速度限制
#define SPEEDMOTOR 1500;
int MotorSpeedM1 = SPEEDMOTOR; // range(-4096 ~ 4096)
int MotorSpeedM2 = SPEEDMOTOR; // range(-4096 ~ 4096)
int MotorSpeedM3 = SPEEDMOTOR; // range(-4096 ~ 4096)
int MotorSpeedM4 = SPEEDMOTOR; // range(-4096 ~ 4096)

volatile long M1PulseValue = 0;
volatile long M2PulseValue = 0;
volatile long M3PulseValue = 0;
volatile long M4PulseValue = 0;
unsigned long previousTime = 0;
int intervalTime = 200;             // 采样时间
int changeWidth = 3;

unsigned long encoderFlag = 0;
int moveDir = 0;
bool testFlag = false;

int speedm_x = 0;
int speedm_y = 0;

// Reset func
void (* resetFunc) (void) = 0;

void setup() {
  Wire.begin();  // join the TWI as the master
  Serial.begin(115200);
  delay(500);  //added delay to give wireless ps2 module some time to startup, before configuring it
  //setup pins and settings: GamePad(clock, command, attention, data, Pressures?, Rumble?) check for error
  error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_CS, PS2_DAT, pressures, rumble);
  if (error == 0) {
    Serial.println("Found Controller, configured successful.");
    Serial.print("pressures = ");
    if (pressures)
      Serial.println("true ");
    else
      Serial.println("false");
    Serial.print("rumble = ");
    if (rumble)
      Serial.println("true");
    else
      Serial.println("false");
  } else if (error == 1)
    Serial.println("No controller found, check wiring, see readme.txt to enable debug. visit www.billporter.info for troubleshooting tips");
  else if (error == 2)
    Serial.println("Controller found but not accepting commands. see readme.txt to enable debug. Visit www.billporter.info for troubleshooting tips");
  else if (error == 3)
    Serial.println("Controller refusing to enter Pressures mode, may not support it. ");

  type = ps2x.readType();
  switch (type) {
    case 0:
      Serial.println("Unknown Controller type found ");
      break;
    case 1:
      Serial.println("DualShock Controller found ");
      break;
    case 2:
      Serial.println("GuitarHero Controller found ");
      break;
    case 3:
      Serial.println("Wireless Sony DualShock Controller found ");
      break;
  }
  motorDirver.begin();
  motorDirver.setMotorDirReverse(0, 1, 0, 1); // 设置电机方向, 0-默认,1-反向.

  // set pins to input with a pullup
  pinMode(M1PHASEAPIN, INPUT_PULLUP);
  pinMode(M2PHASEAPIN, INPUT_PULLUP);
  pinMode(M3PHASEAPIN, INPUT_PULLUP);
  pinMode(M4PHASEAPIN, INPUT_PULLUP);

  previousTime = millis();
}

void loop() {
  /* You must Read Gamepad to get new values and set vibration values
     ps2x.read_gamepad(small motor on/off, larger motor strenght from 0-255)
     if you don't enable the rumble, use ps2x.read_gamepad(); with no values
     You should call this at least once a second
  */
  if (error == 1) { //skip loop if no controller found
    Serial.println("ERROR!!");
    delay(1);
    resetFunc();
  }

  ps2x.read_gamepad(false, vibrate); //read controller and set large motor to spin at 'vibrate' speed

  if (ps2x.Button(PSB_START))        //will be TRUE as long as button is pressed
    Serial.println("Start is being held");
  if (ps2x.Button(PSB_SELECT))
    Serial.println("Select is being held");

  // 按下 上下左右 按钮
  if (ps2x.ButtonPressed(PSB_PAD_UP)) {     //will be TRUE as long as button is pressed
    encoderFlag++;
    moveDir = 0;
  }
  if (ps2x.ButtonPressed(PSB_PAD_RIGHT)) {
    encoderFlag++;
    moveDir = 1;
  }
  if (ps2x.ButtonPressed(PSB_PAD_LEFT)) {
    encoderFlag++;
    moveDir = 2;
  }
  if (ps2x.ButtonPressed(PSB_PAD_DOWN)) {
    encoderFlag++;
    moveDir = 3;
  }
  // 左旋转
  if (ps2x.ButtonPressed(PSB_SQUARE)) {            //will be TRUE if button was JUST pressed
    //    Serial.println("Square just released");
    encoderFlag++;
    moveDir = 4;
  }
  // 右旋转
  if (ps2x.ButtonPressed(PSB_CIRCLE)) {         //will be TRUE if button was JUST pressed
    //    Serial.println("Circle just pressed");
    encoderFlag++;
    moveDir = 5;
  }

  // 释放 上下左右 按钮
  if (ps2x.ButtonReleased(PSB_PAD_UP) || ps2x.ButtonReleased(PSB_PAD_RIGHT) ||
      ps2x.ButtonReleased(PSB_PAD_LEFT) || ps2x.ButtonReleased(PSB_PAD_DOWN) ||
      ps2x.ButtonReleased(PSB_SQUARE) || ps2x.ButtonReleased(PSB_CIRCLE)) {   //will be TRUE if button was JUST released
    //    motorDirver.stopMotor(MAll);
    motorDirver.setMotor(0, 0, 0, 0);
    encoderFlag = 0;
    closeInterrupt(); //关闭引脚中断
    //    Serial.println("free stop!");
  }

  if (encoderFlag == 1) {
    startInterrupt(); //开启引脚中断
    previousTime = millis(); //更新时间
    //    Serial.println("interrupt");
    encoderFlag++;
  } else if (encoderFlag > 1) {
    //    Serial.println(previousTime);
    pulseCount();
    if (moveDir == 0)
      motorDirver.setMotor(MotorSpeedM1, MotorSpeedM2, MotorSpeedM3, MotorSpeedM4);
    else if (moveDir == 1)
      motorDirver.setMotor(MotorSpeedM1, -MotorSpeedM2, -MotorSpeedM3, MotorSpeedM4);
    else if (moveDir == 2)
      motorDirver.setMotor(-MotorSpeedM1, MotorSpeedM2, MotorSpeedM3, -MotorSpeedM4);
    else if (moveDir == 3)
      motorDirver.setMotor(-MotorSpeedM1, -MotorSpeedM2, -MotorSpeedM3, -MotorSpeedM4);
    else if (moveDir == 4)
      motorDirver.setMotor(-MotorSpeedM1, MotorSpeedM2, -MotorSpeedM3, MotorSpeedM4);
    else if (moveDir == 5)
      motorDirver.setMotor(MotorSpeedM1, -MotorSpeedM2, MotorSpeedM3, -MotorSpeedM4);
  }

  if (ps2x.NewButtonState()) {        //will be TRUE if any button changes state (on to off, or off to on)
    if (ps2x.Button(PSB_L2))
      Serial.println("L2 pressed");
  }
  // 最大速度限制
  if (ps2x.Button(PSB_L3)) {
    maxSpeed += 50;
    maxSpeed = maxSpeed > MAXSPEED ? MAXSPEED : maxSpeed ;
    Serial.print("maxSpeed:");
    Serial.println(maxSpeed);
  }
  if (ps2x.Button(PSB_R3)) {
    maxSpeed -= 50;
    maxSpeed = maxSpeed < MINSPEED ? MINSPEED : maxSpeed ;
    Serial.print("maxSpeed:");
    Serial.println(maxSpeed);
  }
  // 恢复初始速度 1500
  if (ps2x.ButtonPressed(PSB_R2)) {
    // reset speed;
    MotorSpeedM1 = SPEEDMOTOR; // range(-4096 ~ 4096)
    MotorSpeedM2 = SPEEDMOTOR; // range(-4096 ~ 4096)
    MotorSpeedM3 = SPEEDMOTOR; // range(-4096 ~ 4096)
    MotorSpeedM4 = SPEEDMOTOR; // range(-4096 ~ 4096)
    Serial.print("Reset speed  ");
    Serial.print(MotorSpeedM1);
    Serial.println();
  }

  // 增加速度 +50
  if (ps2x.Button(PSB_TRIANGLE)) {             //will be TRUE if button was JUST pressed
    int sp = 50;
    MotorSpeedM1 += sp; // range(-4096 ~ 4096)
    MotorSpeedM2 += sp; // range(-4096 ~ 4096)
    MotorSpeedM3 += sp; // range(-4096 ~ 4096)
    MotorSpeedM4 += sp; // range(-4096 ~ 4096)
    MotorSpeedM1 = MotorSpeedM1 > maxSpeed ? maxSpeed : MotorSpeedM1;
    MotorSpeedM2 = MotorSpeedM2 > maxSpeed ? maxSpeed : MotorSpeedM2;
    MotorSpeedM3 = MotorSpeedM3 > maxSpeed ? maxSpeed : MotorSpeedM3;
    MotorSpeedM4 = MotorSpeedM4 > maxSpeed ? maxSpeed : MotorSpeedM4;
    vibrate = 100;
    Serial.print("+  ");
    Serial.print(MotorSpeedM1);
    Serial.println();
  }
  // 减小速度 -50
  if (ps2x.Button(PSB_CROSS)) {             //will be TRUE if button was JUST pressed
    int sp = 50;
    MotorSpeedM1 -= sp; // range(-4096 ~ 4096)
    MotorSpeedM2 -= sp; // range(-4096 ~ 4096)
    MotorSpeedM3 -= sp; // range(-4096 ~ 4096)
    MotorSpeedM4 -= sp; // range(-4096 ~ 4096)
    MotorSpeedM1 = MotorSpeedM1 < 0 ?  0: MotorSpeedM1;
    MotorSpeedM2 = MotorSpeedM2 < 0 ?  0: MotorSpeedM2;
    MotorSpeedM3 = MotorSpeedM3 < 0 ?  0: MotorSpeedM3;
    MotorSpeedM4 = MotorSpeedM4 < 0 ?  0: MotorSpeedM4;
    vibrate = 100;
    Serial.print("-  ");
    Serial.print(MotorSpeedM1);
    Serial.println();
  }
  if (ps2x.ButtonReleased(PSB_TRIANGLE) || ps2x.ButtonReleased(PSB_CROSS)) {
    vibrate = 0;
    //    Serial.println("vibrate = 0");
  }


  // 左摇杆控制 需按下L1或者R1
  if (ps2x.Button(PSB_L1) || ps2x.Button(PSB_R1)) { //drive motor if either is TRUE
    if (ps2x.Analog(PSS_RX) == 128) {
      if (ps2x.Analog(PSS_LY) < 127 && ps2x.Analog(PSS_LY) >= 0) {
        speedm_y = map(ps2x.Analog(PSS_LY), 127, 0, 0, maxSpeed);
      } else if (ps2x.Analog(PSS_LY) > 127 && ps2x.Analog(PSS_LY) <= 255) {
        speedm_y = map(ps2x.Analog(PSS_LY), 255, 127, -maxSpeed, 0);
      } else
        speedm_y = 0;

      if (ps2x.Analog(PSS_LX) < 128 && ps2x.Analog(PSS_LX) >= 0) {
        speedm_x = map(ps2x.Analog(PSS_LX), 0, 128, -maxSpeed, 0);
      } else if (ps2x.Analog(PSS_LX) > 128 && ps2x.Analog(PSS_LX) <= 255) {
        speedm_x = map(ps2x.Analog(PSS_LX), 128, 255, 0, maxSpeed);
      } else
        speedm_x = 0;

      //#ifdef DEBUGESERIAL
      Serial.print("y:");
      Serial.print(speedm_y);
      Serial.print(",x:");
      Serial.print(speedm_x);
      Serial.print(",M1:");
      Serial.print(speedm_y + speedm_x);
      Serial.print(",M2:");
      Serial.print(speedm_y - speedm_x);
      Serial.print(",M3:");
      Serial.print(speedm_y - speedm_x);
      Serial.print(",M4:");
      Serial.print(speedm_y + speedm_x);
      Serial.println();
      //#endif
      moveMotor(speedm_y + speedm_x, speedm_y - speedm_x, speedm_y - speedm_x, speedm_y + speedm_x); // 电机M1,M2,M3,M4 运动
    } else {
      int speedm_w;
      if (ps2x.Analog(PSS_RX) < 128 && ps2x.Analog(PSS_RX) >= 0) {
        speedm_w = map(ps2x.Analog(PSS_RX), 0, 128, -maxSpeed, 0);
      } else if (ps2x.Analog(PSS_RX) > 128 && ps2x.Analog(PSS_RX) <= 255) {
        speedm_w = map(ps2x.Analog(PSS_RX), 128, 255, 0, maxSpeed);
      }
      //#ifdef DEBUGESERIAL
      Serial.print("M1:");
      Serial.print(speedm_w);
      Serial.print(",M2:");
      Serial.print( - speedm_w);
      Serial.print(",M3:");
      Serial.print(speedm_w);
      Serial.print(",M4:");
      Serial.print( - speedm_w);
      Serial.println();
      //#endif
      moveMotor(speedm_w, - speedm_w, speedm_w, - speedm_w); // 电机M1,M2,M3,M4 运动
    }
  }

  if (ps2x.ButtonReleased(PSB_L1) || ps2x.ButtonReleased(PSB_R1)) {   //will be TRUE if button was JUST released
    motorDirver.stopMotor(MAll);
    //    Serial.println("stop");
  }

  delay(10);
}

void moveMotor(int16_t speed1, int16_t speed2, int16_t speed3, int16_t speed4) {
  if (speed1 > 0)
    speed1 = min(speed1, maxSpeed);
  else
    speed1 = max(speed1, -maxSpeed);
  if (speed2 > 0)
    speed2 = min(speed2, maxSpeed);
  else
    speed2 = max(speed2, -maxSpeed);
  if (speed3 > 0)
    speed3 = min(speed3, maxSpeed);
  else
    speed3 = max(speed3, -maxSpeed);
  if (speed4 > 0)
    speed4 = min(speed4, maxSpeed);
  else
    speed4 = max(speed4, -maxSpeed);
  motorDirver.setMotor(speed1, speed2, speed3, speed4);
#ifdef DEBUGESERIAL
  Serial.print("M1speed:");
  Serial.print(speed1);
  Serial.print(",M2speed:");
  Serial.print(speed2);
  Serial.print(",M3speed:");
  Serial.print(speed3);
  Serial.print(",M4speed:");
  Serial.print(speed4);
  Serial.println();
#endif
}

/*
   中断脉冲计数 调速
*/
void pulseCount() {
  if (millis() - previousTime >= intervalTime) {
    closeInterrupt();
    //#ifdef DEBUGESERIAL
    // print values
    Serial.print(millis());
    Serial.print(" - M1:");
    Serial.print(M1PulseValue);
    Serial.print(" M2:");
    Serial.print(M2PulseValue);
    Serial.print(" M3:");
    Serial.print(M3PulseValue);
    Serial.print(" M4:");
    Serial.print(M4PulseValue);
    //#endif
    //    long pulseValue = (M1PulseValue + M2PulseValue + M3PulseValue + M4PulseValue) / 4;
    //    long pulseValue = 100;
    //    Serial.print(" pVal:");
    //    Serial.print(pulseValue);
    if (M2PulseValue > 0)
      MotorSpeedM2 += (abs(M1PulseValue) - M2PulseValue) * changeWidth;
    else
      MotorSpeedM2 += (abs(M1PulseValue) + M2PulseValue) * changeWidth;
    if (M3PulseValue > 0)
      MotorSpeedM3 += (abs(M1PulseValue) - M3PulseValue) * changeWidth;
    else
      MotorSpeedM3 += (abs(M1PulseValue) + M3PulseValue) * changeWidth;
    if (M4PulseValue > 0)
      MotorSpeedM4 += (abs(M1PulseValue) - M4PulseValue) * changeWidth;
    else
      MotorSpeedM4 += (abs(M1PulseValue) + M4PulseValue) * changeWidth;
#ifdef DEBUGESERIAL
    Serial.print("    M1:");
    Serial.print(MotorSpeedM1);
    Serial.print(" M2:");
    Serial.print(MotorSpeedM2);
    Serial.print(" M3:");
    Serial.print(MotorSpeedM3);
    Serial.print(" M4:");
    Serial.print(MotorSpeedM4);
#endif
    Serial.println();
    startInterrupt();

    M1PulseValue = 0;
    M2PulseValue = 0;
    M3PulseValue = 0;
    M4PulseValue = 0;
    previousTime = millis();
  }
}

void startInterrupt() {
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(M1PHASEAPIN), m1_phase_a, TRIGGER);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(M2PHASEAPIN), m2_phase_a, TRIGGER);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(M3PHASEAPIN), m3_phase_a, TRIGGER);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(M4PHASEAPIN), m4_phase_a, TRIGGER);
}

void closeInterrupt() {
  detachPinChangeInterrupt(digitalPinToPinChangeInterrupt(M1PHASEAPIN));
  detachPinChangeInterrupt(digitalPinToPinChangeInterrupt(M2PHASEAPIN));
  detachPinChangeInterrupt(digitalPinToPinChangeInterrupt(M3PHASEAPIN));
  detachPinChangeInterrupt(digitalPinToPinChangeInterrupt(M4PHASEAPIN));
}

void m1_phase_a(void) {
  bool nowM1PhaseB = digitalRead(M1PHASEBPIN);
  if (!nowM1PhaseB)
    M1PulseValue--;
  else
    M1PulseValue++;
}
void m2_phase_a(void) {
  bool nowM2PhaseB = digitalRead(M2PHASEBPIN);
  if (nowM2PhaseB)
    M2PulseValue--;
  else
    M2PulseValue++;
}
void m3_phase_a(void) {
  bool nowM3PhaseB = digitalRead(M3PHASEBPIN);
  if (!nowM3PhaseB)
    M3PulseValue--;
  else
    M3PulseValue++;
}
void m4_phase_a(void) {
  bool nowM4PhaseB = digitalRead(M4PHASEBPIN);
  if (nowM4PhaseB)
    M4PulseValue--;
  else
    M4PulseValue++;
}
