/* Program name:
      EBB.mcpds
   Author:
      Alessandro Dago & Maksymilian Kowalczuk
   Description:
      This is a main program for Electronic Bias Braking system,
      developed for FSAE racing competition and a racing car project,
      in Dynamis PRC Team at Politecnico di Milano, Milano, 2016.
   Car:
      DP8 - Dynamis PRC
   Configuration:
      Microcontroller: dsPIC4011
      Oscillator:      XT 10Mhz, PLL 8x, Tcy = 20Mhz
      PWM frequency:   20 kHz
      SW:              microC PRO for dsPIC v.6.2.0 
   CAN input:
      brk_P_front - 50Hz
      brk_P_rear - 50Hz
      ebbtarget - 10Hz (1 byte) spec: 0 to 8 = EBB values from -4 to 4, 15 = zero calilbration
                                       50 = set zero, 99 to 101 manual calibration
      Battery_Voltage - 25Hz
   CAN output:
      ebbvalue - 10Hz spec: 100 = calibration, from 0 to 8 position from -4 to 4, 122 = ebb error, 10 = rotateing, 124 = low battery
      motor_sense - 10Hz
      motor_state - 10Hz (1 byte) spec: 0 = ready, 1 = busy, 2 = broken, 10 = manual calibration ack
      */

//Digital i/o pins
sbit REVERSE at LATE2_bit;
sbit FORWARD at LATE1_bit;
sbit ENABLE1 at LATE3_bit;
sbit ENABLE2 at LATE4_bit;
sbit LED_B at LATD1_bit;
sbit LED_G at LATD3_bit;
sbit BUZZER at LATD2_bit;

//Global constants
#define SATURATION 1800
#define MIN_PW 840                //900 before
#define DELTA 110                  //120 before
#define MAX_Sense 920             //mA   //not more than 900mA
#define Sense_Conversion 1.5625   //mA
#define MAX_Pressure 5000         //
#define V_MIN 680 // 12v
#define LOOP_T 6
#define OSC_FREQ_MHZ 80
#define ONE_TURN 1787.5 //ebb2016
#define TOL 20
#define CENTRAL 32768
#define RESET 0
#define CALIBRATION 100
#define CALIBRATION_OFF 0
#define READY 0
#define BUSY 1
#define ERROR 122
#define LOW_BATTERY 124
#define ROTATEING 10
#define CENTRAL_CALIBRATION 15
#define OFF 0
#define ON 1

      
//Libraries
#include "CAN/macro.c"
#include "CAN/d_can.h"
#include "TIMER/dsPIC.c"



//Global variables declaration
int cnt, ii, RefPosition, MotorPosition, Difference, yPID, proportional, feedback, start, sound, spec, RIO_on_flag;
float Kp, Ki, integral;
unsigned long MemAddrP, MemAddrR, MemAddrC, MemAddrM;
unsigned char SPI_send, SPI_fault, buffer;
char string[20];
unsigned int old_ebbvalue=0, old_motor_sense=0, old_motor_state=0, vbatt = 0, ebbvalue = 0, motor_state = 0, motor_sense = 0, ebbtarget, brk_P_front = 0, brk_P_rear = 0;



//Functions declaration
void initVariables(void);
void initPorts(void);
void turnMotorOff(void);
void turnMotorOn(int spec);
void motorControl(void);
void zeroCalibration(void);
void Counter(void);
void CAN(void);
void setZero(void);
void demoPos(void);



onTimer1Interrupt {         // 5ms ISR timer 1
Counter();
clearTimer1();
}

onTimer2Interrupt {         // 100ms ISR timer 2
CAN();
clearTimer2();
}

onTimer4Interrupt {         // 200us ISR timer 4
if(sound == 1){
     BUZZER = !BUZZER;
     }
clearTimer4();
}


//Interrupt pin INT0 ISR - Low side comparator (H-bridge)
/*void LSCMP_ISR() iv IVT_ADDR_INT0INTERRUPT ics ICS_AUTO {
     IFS0bits.INT0IF = 0;                       // Clear INT0 flag
}   */

void main() {
initVariables();
initPorts();
sound = ON;
delay_ms(800);
sound = OFF;

while(1){
     if(ebbtarget >= 0 && ebbtarget <= 8){
          //demoPos();                                     //demo positioning function
          RefPosition = ebbtarget - 4;
          EEPROM_Write(MemAddrR, RefPosition);
          while(WR_bit);
          MotorPosition = POSCNT - CENTRAL;
          EEPROM_Write(MemAddrM, MotorPosition);
          while(WR_bit);
          if((MotorPosition <= RefPosition*ONE_TURN - TOL) || (MotorPosition >= RefPosition*ONE_TURN + TOL)){
               motorControl();
               }
          }
     else if(ebbtarget == CENTRAL_CALIBRATION){
          if(RIO_on_flag == ON){
               zeroCalibration();
               }
          else{
               ebbvalue = ERROR;
               }
          }
     else if(ebbtarget == 50){
          setZero();
          }
     else if(ebbtarget == 100){
          LED_B = ON;
          ebbvalue = CALIBRATION;
          motor_state = 10;
          while(ebbtarget > 98 && ebbtarget < 102){
               if(ebbtarget == 99){
                    ENABLE1 = 1;
                    FORWARD = 0;
                    REVERSE = 1;
                    PDC1 = 1000;
                    ebbvalue = ROTATEING;
                    }
               if(ebbtarget == 101){
                    ENABLE1 = 1;
                    FORWARD = 1;
                    REVERSE = 0;
                    PDC1 = 1000;
                    ebbvalue = ROTATEING;
                    }
               if(ebbtarget == 100){
                    turnMotorOff();
                    ebbvalue = CALIBRATION;
                    }
               motor_sense = getAnalogValue()*Sense_Conversion;
               if(motor_sense >= 2*MAX_Sense){
                    ebbvalue = ERROR;
                    break;
                    }
               }
          turnMotorOff();
          motor_state = READY;
          setZero();
          delay_ms(100);
          LED_B = OFF;
          }
     else ebbvalue = RefPosition + 4;
     }
}


void initVariables(){
feedback = 0, cnt = 0, sound = OFF;
yPID = 0, integral = 0, proportional = 0, Kp = 2, Ki = 7;
ii = 0;
RIO_on_flag = OFF;
motor_state = READY;
motor_sense = 0;
buffer = 0;        //???
SPI_send = 129;                   //Retain Faults, 4A current limit, 2X slew time
MemAddrR = 0x7FFDA0; //Memory address for Reference Position
MemAddrP = 0x7FFDB0; //Memory address for Position
MemAddrC = 0x7FFDC0; //Memory address for Calibration status
MemAddrM = 0x7FFDD0; //Memory address for Motor Position
}

void initPorts(){
ADPCFG = 0b1111111111111110;                    //analog input on AN0 (Current Sense)
TRISDbits.TRISD1 = 0;                           //green led;
TRISDbits.TRISD3 = 0;                           //blue led;
TRISDbits.TRISD2 = 0;                           //buzzer;
TRISEbits.TRISE0 = 0;                           //PWM output
TRISEbits.TRISE1 = 0;                           //Forward output
TRISEbits.TRISE2 = 0;                           //Reverse output
TRISEbits.TRISE3 = 0;                           //Enable1 output
TRISEbits.TRISE4 = 0;                           //Enable2 output
TRISBbits.TRISB2 = 0;                           //Chip Select SPI interface output (low = true)
TRISBbits.TRISB0 = 1;                           //set ADC pin as input (Current sense: Vcsns = Iout x 3.1)
TRISEbits.TRISE8 = 1;                           //LSCMP (Low Side Comparator) input (high = true)

//SPI1_Init_Advanced(_SPI_MASTER,_SPI_8_BIT,_SPI_PRESCALE_SEC_1,_SPI_PRESCALE_PRI_16,_SPI_SS_DISABLE,_SPI_DATA_SAMPLE_MIDDLE,_SPI_CLK_IDLE_LOW,_SPI_IDLE_2_ACTIVE);
//SPI1_Init();                                    //Initialize SPI1 interface with default settings
//SPI1_Write(SPI_send);
//LATBbits.LATB2 = 0;                          //Ther's only one SPI device, ever in communication

// Quadrature Encoder
QEICON = 0b0000010100000010;                 //Set Quadrature Encoder
POSCNT = EEPROM_Read(MemAddrM) + CENTRAL;    //Position Counter starter value
//POSCNT = CENTRAL;

//Can init
CAN_Init();
Can_resetWritePacket();
Can_addIntToWritePacket(RefPosition);                   //ebbvalue
Can_addIntToWritePacket(motor_sense);                   //motor_sense
Can_addIntToWritePacket(motor_state);                   //motor_state
Can_write(EBB_ID);

delay_ms(20);

RefPosition = EEPROM_READ(MemAddrR);

//Timer init
setTimer(TIMER1_DEVICE,0.005);
setTimer(TIMER2_DEVICE,0.1);
setTimer(TIMER4_DEVICE,0.0002);

// Analog Init on pin AN0
setupAnalogSampling();
setAnalogPin(AN0);
turnOnAnalogModule();

//PWM Init
PWMCON1 = 0b0000000100000001;                //independent mode, only PWM1L enabled
PTPER = 999;                                 //PWM frequency = 20 kHz
PDC1 = 0;                                    //initial 0% of duty cycle - motor is off;        MAX_Value = 2000;
PTMR = 0;                                    //to clear the PWM time base
PTCON = 0b1000000000000000;                  //prescaler 1:1, postscaler 1:1, free running mode, PWM on
ENABLE2 = 1;                                 //Never sleep mode on H-bridge
FORWARD = 0;
REVERSE = 0;

BUZZER = 0;
LED_B = 0;
LED_G = 0;
motor_sense = getAnalogValue()*Sense_Conversion;
//UART1_Init(9600);
}

void Counter(){
     feedback = feedback + 1;
     if(feedback > 20){
          feedback = 0;
          }
}

void CAN(){
     //if(ebbvalue != old_ebbvalue || motor_sense != old_motor_sense || motor_state != old_motor_state){
          Can_resetWritePacket();
          Can_addIntToWritePacket(ebbvalue);
          Can_addIntToWritePacket(motor_sense);
          Can_addIntToWritePacket(motor_state);
          Can_write(EBB_ID);
          //}
     old_ebbvalue = ebbvalue;
     old_motor_state = motor_state;
     old_motor_sense = motor_sense;
     //inttostr(brk_P_front, string);
     //inttostr(vbatt, string);
     //UART1_Write_Text(string);
}

void setZero(){
     EEPROM_WRITE(MemAddrC, CALIBRATION);
     while(WR_bit);
     POSCNT = CENTRAL;                  //Set zero
     MotorPosition = 0;
     EEPROM_WRITE(MemAddrM, MotorPosition);
     while(WR_bit);
     RefPosition = 0;
     EEPROM_WRITE(MemAddrR, RefPosition);
     while(WR_bit);
     ebbvalue = 4;
     EEPROM_WRITE(MemAddrC, CALIBRATION_OFF);
     while(WR_bit);
     sound = 1;
     delay_ms(600);
     sound = 0;
}

void demoPos(){
     if(cnt == 0)    ebbtarget = 5;      //
     if(cnt == 1)    ebbtarget = 3;      //
     if(cnt == 2)    ebbtarget = 6;      //
     if(cnt == 3)    ebbtarget = 2;      //
     if(cnt == 4)    ebbtarget = 7;      //
     if(cnt == 5)    ebbtarget = 1;      //
     if(cnt == 6)    ebbtarget = 8;      //
     if(cnt == 7)    ebbtarget = 0;      //
     if(cnt == 8)    cnt = 0;            //
     cnt++;
     delay_ms(1000);
}

void turnMotorOff(){
     ENABLE1 = 0;
     PDC1 = 0;
     motor_sense = getAnalogValue()*Sense_Conversion;
}

void turnMotorOn(int spec){
     if(yPID >= SATURATION){                          //anti wind-up
          yPID = SATURATION;
          }
     if(yPID <= -SATURATION){
          yPID = -SATURATION;
          }
     if(yPID >= 0){
          REVERSE = 0;
          FORWARD = 1;         //Forward
          }
     else if(yPID < 0){
          FORWARD = 0;
          REVERSE = 1;          //Reverse
          yPID = -yPID;
          }
     if(yPID < MIN_PW){
          yPID = MIN_PW;
          }
     motor_sense = getAnalogValue()*Sense_Conversion;
     if(start > 0) start = start + 1;
     if(start >= 3) start = 0;
     if(start == 0 && motor_sense > MAX_Sense){
          turnMotorOff();
          ebbvalue = ERROR;
          while(TIMER2_OCCURRED != FALSE);
          delay_ms(1600);
          start = 1;
          }
     else{
          PDC1 = yPID;
          ENABLE1 = 1;
          ebbvalue = spec;
          }
}

void motorControl(){
     LED_G = ON;
     motor_state = BUSY;
     integral = 0;
     start = 1;
     MotorPosition = POSCNT - CENTRAL;                           //encoder Position Counter register (value from 0 to 65536);
     while(((MotorPosition <= RefPosition*ONE_TURN - TOL) || (MotorPosition >= RefPosition*ONE_TURN + TOL) || yPID > MIN_PW) && vbatt > V_MIN){
          if(feedback > LOOP_T){                         //30ms
               while(brk_P_front >= MAX_Pressure || brk_P_rear >= MAX_Pressure){
                    turnMotorOff();
                    ebbvalue = ERROR;
                    }
               MotorPosition = POSCNT - CENTRAL;
               EEPROM_Write(MemAddrM, MotorPosition);
               while(WR_bit);
               if(ebbtarget >= 0 && ebbtarget <= 8){
                    RefPosition = ebbtarget - 4;
                    EEPROM_Write(MemAddrR, RefPosition);
                    while(WR_bit);
                    }
               Difference = RefPosition*ONE_TURN - MotorPosition;
               proportional = Difference * Kp;
               if((proportional < (MIN_PW+DELTA) && proportional >= 0) || (proportional > -(MIN_PW+DELTA) && proportional <= 0)) integral = integral + Difference / Ki;
               else integral = 0;
               yPID = proportional + integral;
               turnMotorOn(ROTATEING);
               feedback = RESET;
               }
          }
     turnMotorOff();
     MotorPosition = POSCNT - CENTRAL;
     EEPROM_Write(MemAddrM, MotorPosition);
     while(WR_bit);
     if(vbatt <= V_MIN) ebbvalue = LOW_BATTERY;
     else ebbvalue = RefPosition + 4;
     delay_ms(100);
     motor_state = READY;
     LED_G = OFF;
}

void zeroCalibration(void){
     LED_B = ON;
     motor_state = BUSY;
     brk_P_front = 100;                       //read oil pressure on can_bus
     brk_P_rear = 100;
     integral = 0;
     start = 1;
     ebbvalue = CALIBRATION;
     MotorPosition = POSCNT - CENTRAL;
     EEPROM_Write(MemAddrM, MotorPosition);
     while(WR_bit);
     EEPROM_Write(MemAddrC, CALIBRATION);
     while(WR_bit);
     while(((brk_P_front > brk_P_rear + TOL || brk_P_front < brk_P_rear - TOL) || yPID > MIN_PW) && vbatt > V_MIN){
          if(feedback > LOOP_T){                    //30ms
               while(brk_P_front >= MAX_Pressure || brk_P_rear >= MAX_Pressure){
                    turnMotorOff();
                    ebbvalue = ERROR;
                    }
               MotorPosition = POSCNT - CENTRAL;
               EEPROM_Write(MemAddrM, MotorPosition);
               while(WR_bit);
               Difference = brk_P_front - brk_P_rear;
               proportional = Difference * Kp;
               if((proportional < (MIN_PW+DELTA) && proportional > 0) || (proportional > -(MIN_PW+DELTA) && proportional < 0)) integral = integral + Difference / Ki;
               else integral = 0;
               yPID = proportional + integral;
               turnMotorOn(CALIBRATION);
               brk_P_front = brk_P_front - yPID/100;            //demo
               brk_P_rear = brk_P_rear + yPID/100;              //demo
               feedback = RESET;
               }
          }
     turnMotorOff();
     setZero();
     if(vbatt <= V_MIN) ebbvalue = LOW_BATTERY;
     motor_state = READY;
     LED_B = OFF;
}


onCanInterrupt{
     unsigned long int CAN_id;
     char CAN_datain[8];
     unsigned int CAN_dataLen, CAN_flags;
     Can_read(&CAN_id, CAN_datain, &CAN_dataLen, &CAN_flags);
     Can_clearInterrupt();
     switch(CAN_id){
          case EFI_OIL_BATT_ID: vbatt = (unsigned int) ((CAN_datain[6] << 8) | (CAN_datain[7] & 0xFF));
                                break;
          case SW_EBB_ID:       ebbtarget = CAN_datain[0];
                                break;
          case SW_RIO_GEAR_BRK_STEER_ID:
                                         brk_P_front = (unsigned int) ((CAN_datain[0] << 8) | (CAN_datain[1] & 0xFF));
                                         brk_P_rear = (unsigned int) ((CAN_datain[2] << 8) | (CAN_datain[3] & 0xFF));
                                         /*brk_P_front = CAN_datain[1];
                                         brk_P_rear = CAN_datain[2];
                                         brk_P_front = (brk_P_front & 0xFF)*100 + (brk_P_front >> 8);
                                         brk_P_rear = (brk_P_rear & 0xFF)*100 + (brk_P_rear >> 8);*/
                                         RIO_on_flag = ON;
                                         break;
     }
}