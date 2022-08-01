/*  01 08 2022 
PENDULUM - by Francesco Ganassin
    a pendulum with these elements
    Leonardo Micro Pro
    MPU6050 gyro
    MOZZI byte
    Li-Po battery
    
    based on Tim Barrass 2012, CC by-nc-sa codes
    and other non blocking codes for MPU6050
*/

#include <MozziGuts.h>
#include <Oscil.h>
#include <tables/sin512_int8.h>
#include <mozzi_midi.h>
#include <twi_nonblock.h>

#define CONTROL_RATE 32
float gain;

static volatile byte acc_status = 0;
#define ACC_IDLE 0
#define ACC_READING 1
#define ACC_WRITING 2

int accbytedata[14];

#define MPU6050_ADDR                  0x68
#define MPU6050_SMPLRT_DIV_REGISTER   0x19
#define MPU6050_CONFIG_REGISTER       0x1a
#define MPU6050_GYRO_CONFIG_REGISTER  0x1b
#define MPU6050_PWR_MGMT_1_REGISTER   0x6b
#define MPU6050_ACCEL_OUT_REGISTER    0x3b

Oscil<SIN512_NUM_CELLS, AUDIO_RATE> aSin1(SIN512_DATA);
Oscil<SIN512_NUM_CELLS, AUDIO_RATE> aSin2(SIN512_DATA);
Oscil<SIN512_NUM_CELLS, AUDIO_RATE> aSin3(SIN512_DATA);
Oscil<SIN512_NUM_CELLS, AUDIO_RATE> aSin4(SIN512_DATA);

void setup_accelero(){
  initialize_twi_nonblock();

  acc_writeTo(MPU6050_SMPLRT_DIV_REGISTER, 0x00);
  acc_writeTo(MPU6050_CONFIG_REGISTER, 0x00);
  acc_writeTo(MPU6050_GYRO_CONFIG_REGISTER, 0x00);
  acc_writeTo(MPU6050_PWR_MGMT_1_REGISTER, 0x01);
  
  acc_status = ACC_IDLE;
}

void initiate_read_accelero(){
  txAddress = MPU6050_ADDR;
  txBufferIndex = 0;
  txBufferLength = 0;

  txBuffer[txBufferIndex] = MPU6050_ACCEL_OUT_REGISTER; 
  ++txBufferIndex;
  txBufferLength = txBufferIndex;

  twi_initiateWriteTo(txAddress, txBuffer, txBufferLength);
  acc_status = ACC_WRITING;
}

void initiate_request_accelero(){
  txBufferIndex = 0;
  txBufferLength = 0;

  byte read = twi_initiateReadFrom(MPU6050_ADDR, 14);
  acc_status = ACC_READING;
}

void finalise_request_accelero() {
  byte read = twi_readMasterBuffer( rxBuffer, 14 ); 
  rxBufferIndex = 0;
  rxBufferLength = read;

  byte i = 0;
  while( rxBufferLength - rxBufferIndex > 0) { 
    accbytedata[i] = rxBuffer[rxBufferIndex];
    ++rxBufferIndex;
    i++;
  }

  acc_status = ACC_IDLE;
}

void acc_writeTo(byte address, byte val) {
  twowire_beginTransmission(MPU6050_ADDR);
  twowire_send( address );
  twowire_send( val );
  twowire_endTransmission();
}

void setup(){
  aSin1.setFreq(4435);
  aSin2.setFreq(2794);
  aSin3.setFreq(1865);
  aSin4.setFreq(2349);
 

  startMozzi(CONTROL_RATE);
  
  Serial.begin(115200); 
  setup_accelero();
}

int gyrox;
int gyroy;
int gyroz;

unsigned long ms = millis();
unsigned long readTime = ms;

void loop(){
  audioHook();
}


void updateControl(){
  ms = millis();
  if (ms > readTime) {
    readTime += 50;
 
    switch( acc_status ){
    case ACC_IDLE:
     gyrox = (accbytedata[8] << 8 | accbytedata[9]) >> 7;
      gyroy = (accbytedata[10] << 8 | accbytedata[11]) >> 7;
      gyroz = (accbytedata[12] << 8 | accbytedata[13]) >> 7;
      Serial.print("\tgX ");Serial.print(gyrox);
      Serial.print("\tgY ");Serial.print(gyroy);
      Serial.print("\tgZ ");Serial.print(gyroz);
      Serial.println();
      initiate_read_accelero();
  
        break;
    case ACC_WRITING:
      if ( TWI_MTX != twi_state ){
        initiate_request_accelero();
      }
      break;
    case ACC_READING:
      if ( TWI_MRX != twi_state ){
        finalise_request_accelero();
      }
      break;
    }
  }
}

AudioOutput_t updateAudio(){
  long asig = (long)
    aSin1.next() * (gyroz) +
    aSin2.next() * (gyroy) +
    aSin3.next() * (gyrox) +
    aSin4.next() * (gyroz) ;
  
  return MonoOutput::fromAlmostNBit(18, asig);
}
