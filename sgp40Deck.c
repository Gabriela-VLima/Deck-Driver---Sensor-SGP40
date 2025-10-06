/*
Deck Driver para o Sensor SGP40
Sensor para medição de compostos orgânicos voláteis (VOC) para qualidade do ar interno.
Esse deck driver foi implementado na aeronave Crazyflie 2.1

*/
#define DEBUG_MODULE "SGP40Deck"

// All includes
#include "debug.h"
#include "i2cdev.h"
#include "deck.h"
#include "log.h"
#include "FreeRTOS.h"
#include "task.h"
#include "system.h"
#include "param.h"
#include "pm.h"

// Addresses, constants for SGP40
#define SGP40I2CAddr 0x59 
#define SGP40_TASK_STACKSIZE    (2*configMINIMAL_STACK_SIZE)
#define SGP40_TASK_PRI 3
#define SGP40_TASK_NAME "SGP40"

#define SGP40_address 0x59 

#define SGP40_measure_RAW {0x26, 0x0F} 
#define sgp40_execute_self_test {0x28, 0x0E} 
#define sgp40_turn_heater_off {0x36, 0x15} 
#define SGP40_SERIAL_ID {0x36, 0x82} 
#define SGP40_Soft_Reset {0x00, 0x06}

// Tabela do CheckSum 
  const uint8_t CRC8LookupTable[16][16] = {
      {0x00, 0x31, 0x62, 0x53, 0xC4, 0xF5, 0xA6, 0x97, 0xB9, 0x88, 0xDB, 0xEA, 0x7D, 0x4C, 0x1F, 0x2E},
      {0x43, 0x72, 0x21, 0x10, 0x87, 0xB6, 0xE5, 0xD4, 0xFA, 0xCB, 0x98, 0xA9, 0x3E, 0x0F, 0x5C, 0x6D},
      {0x86, 0xB7, 0xE4, 0xD5, 0x42, 0x73, 0x20, 0x11, 0x3F, 0x0E, 0x5D, 0x6C, 0xFB, 0xCA, 0x99, 0xA8},
      {0xC5, 0xF4, 0xA7, 0x96, 0x01, 0x30, 0x63, 0x52, 0x7C, 0x4D, 0x1E, 0x2F, 0xB8, 0x89, 0xDA, 0xEB},
      {0x3D, 0x0C, 0x5F, 0x6E, 0xF9, 0xC8, 0x9B, 0xAA, 0x84, 0xB5, 0xE6, 0xD7, 0x40, 0x71, 0x22, 0x13},
      {0x7E, 0x4F, 0x1C, 0x2D, 0xBA, 0x8B, 0xD8, 0xE9, 0xC7, 0xF6, 0xA5, 0x94, 0x03, 0x32, 0x61, 0x50},
      {0xBB, 0x8A, 0xD9, 0xE8, 0x7F, 0x4E, 0x1D, 0x2C, 0x02, 0x33, 0x60, 0x51, 0xC6, 0xF7, 0xA4, 0x95},
      {0xF8, 0xC9, 0x9A, 0xAB, 0x3C, 0x0D, 0x5E, 0x6F, 0x41, 0x70, 0x23, 0x12, 0x85, 0xB4, 0xE7, 0xD6},
      {0x7A, 0x4B, 0x18, 0x29, 0xBE, 0x8F, 0xDC, 0xED, 0xC3, 0xF2, 0xA1, 0x90, 0x07, 0x36, 0x65, 0x54},
      {0x39, 0x08, 0x5B, 0x6A, 0xFD, 0xCC, 0x9F, 0xAE, 0x80, 0xB1, 0xE2, 0xD3, 0x44, 0x75, 0x26, 0x17},
      {0xFC, 0xCD, 0x9E, 0xAF, 0x38, 0x09, 0x5A, 0x6B, 0x45, 0x74, 0x27, 0x16, 0x81, 0xB0, 0xE3, 0xD2},
      {0xBF, 0x8E, 0xDD, 0xEC, 0x7B, 0x4A, 0x19, 0x28, 0x06, 0x37, 0x64, 0x55, 0xC2, 0xF3, 0xA0, 0x91},
      {0x47, 0x76, 0x25, 0x14, 0x83, 0xB2, 0xE1, 0xD0, 0xFE, 0xCF, 0x9C, 0xAD, 0x3A, 0x0B, 0x58, 0x69},
      {0x04, 0x35, 0x66, 0x57, 0xC0, 0xF1, 0xA2, 0x93, 0xBD, 0x8C, 0xDF, 0xEE, 0x79, 0x48, 0x1B, 0x2A},
      {0xC1, 0xF0, 0xA3, 0x92, 0x05, 0x34, 0x67, 0x56, 0x78, 0x49, 0x1A, 0x2B, 0xBC, 0x8D, 0xDE, 0xEF},
      {0x82, 0xB3, 0xE0, 0xD1, 0x46, 0x77, 0x24, 0x15, 0x3B, 0x0A, 0x59, 0x68, 0xFF, 0xCE, 0x9D, 0xAC}};


// Misc global variables
static bool isInit;
float intensity1;
float intensity2;
uint16_t dado2_e1;
uint16_t dado2_e2;
uint16_t dado2_e3;
uint16_t dado2_e4;
uint16_t dado2_e5;
uint16_t dado2_e6;
uint16_t dado2_e7;


uint8_t SGP40_LOOKUP_TABLE(uint16_t data){
  uint8_t CRC_ = 0xFF;                          //inital value
  CRC_ ^= (uint8_t)(data >> 8);                 //start with MSB
  CRC_ = CRC8LookupTable[CRC_ >> 4][CRC_ & 0xF]; //look up table [MSnibble][LSnibble]
  CRC_ ^= (uint8_t)data;                        //use LSB
  CRC_ = CRC8LookupTable[CRC_ >> 4][CRC_ & 0xF]; //look up table [MSnibble][LSnibble]
  return CRC_;
}

uint8_t CRC8(uint16_t data){
  uint8_t crc_ = 0xFF; //Init with 0xFF

  crc_ ^= (data >> 8); // XOR-in the first input byte

  for (uint8_t i = 0; i < 8; i++)
  {
    if ((crc_ & 0x80) != 0)
      crc_ = (uint8_t)((crc_ << 1) ^ 0x31);
    else
      crc_ <<= 1;
  }
  crc_ ^= (uint8_t)data; // XOR-in the last input byte

  for (uint8_t i = 0; i < 8; i++)
  {
    if ((crc_ & 0x80) != 0)
      crc_ = (uint8_t)((crc_ << 1) ^ 0x31);
    else
      crc_ <<= 1;
  }

  return crc_; //No output reflection
}

void appSGP40(void* arg)
{

  TickType_t xLastWakeTime_1;
  TickType_t xLastWakeTime_2;
  TickType_t xLastWakeTime_3;

  uint8_t measure_raw[] = {0x26, 0x0F};
  
  IAQBaselineTVOC_1 = IAQBaselineTVOC_0 >> 8;
  IAQBaselineTVOC_2 = IAQBaselineTVOC_0 & 0x00FF;
  IAQBaselineCO2_1 = IAQBaselineCO2_0 >> 8;
  IAQBaselineCO2_2 = IAQBaselineCO2_0 & 0x00FF;

  uint8_t IAQ_baseline[] = {IAQBaselineTVOC_1, IAQBaselineTVOC_2, IAQBaselineCO2_1, IAQBaselineCO2_2};    

  uint8_t check_baseline_1 = CRC8(IAQ_baseline[0] << 8 | IAQ_baseline[1]);
  uint8_t check_baseline_2 = CRC8(IAQ_baseline[2] << 8 | IAQ_baseline[3]);
  
  uint8_t intensitydata[] = {0, 0, 0, 0, 0, 0};
  uint8_t intensitydata_1[] = {0, 0, 0, 0, 0, 0};
  uint8_t intensitydata_a[] = {0, 0, 0, 0, 0, 0};
  uint8_t intensitydata_b[] = {0, 0, 0, 0, 0, 0};

  uint16_t dadoa1;
  uint16_t dadoa2;
  uint16_t dadob1;
  uint16_t dadob2;
  uint16_t dado1;
  uint16_t dado2;
  uint16_t dado3;
  uint16_t dado4;
  uint8_t check_a1;
  uint8_t check_a2;
  uint8_t check_b1;
  uint8_t check_b2;
  uint8_t check_1;
  uint8_t check_2;
  uint8_t check_3;
  uint8_t check_4;
  uint16_t iaq_baseline;

  xLastWakeTime_1 = xTaskGetTickCount();
 
   if (i2cdevWriteReg8(I2C1_DEV, SGP40_address, I2CDEV_NO_MEM_ADDR, 2, IAQ_init)){
      //DEBUG_PRINT("IAQ Init OK \n");
    }
    else{
      //DEBUG_PRINT("IAQ Init Fail \n");
    } 
    vTaskDelay(M2T(10));


  while (1) {
    // Leitura TVOC e CO2eq
    //Escrita do Comando
    xLastWakeTime_2 = xTaskGetTickCount();  
       
    //Leitura
    if(i2cdevReadReg8(I2C1_DEV, SGP40_address, I2CDEV_NO_MEM_ADDR, (uint8_t)6, intensitydata)){
      int a = sizeof(intensitydata) / sizeof(intensitydata[0]);
      //DEBUG_PRINT("Size of intensitydata: %d\n", a);
      if (a != 6){
        //DEBUG_PRINT("TIMEOUT \n");
      }
      else{
        dado1 = intensitydata[0] << 8 | intensitydata[1];
        check_1 = intensitydata[2];
        dado2 = intensitydata[3] << 8 | intensitydata[4];
        check_2 = intensitydata[5];
        dado2_e1 = dado1;
        dado2_e2 = dado2;        
      }
    }
    else{
      //DEBUG_PRINT("Reading Fail \n");
      //intensity1 = intensitydata[1];
    }

    // Leitura Raw
    uint8_t d_2[] = {0x20, 0x50};    
    if (i2cdevWriteReg8(I2C1_DEV, SGP40_address, I2CDEV_NO_MEM_ADDR, 2, measure_raw)){
      //DEBUG_PRINT("Writing_1 OK \n");
    }
    else{
      //DEBUG_PRINT("Writing 1 Fail \n");
    }

    vTaskDelay(M2T(25));
    
    //Leitura
    if(i2cdevReadReg8(I2C1_DEV, SGP40_address, I2CDEV_NO_MEM_ADDR, (uint8_t)6, intensitydata_1)){
      int a = sizeof(intensitydata_1) / sizeof(intensitydata_1[0]);
      //DEBUG_PRINT("Size of intensitydata: %d\n", a);
      if (a != 6){
        //DEBUG_PRINT("TIMEOUT \n");
      }
      else{
        dado3 = intensitydata_1[0] << 8 | intensitydata_1[1];
        check_3 = intensitydata_1[2];
        dado4 = intensitydata_1[3] << 8 | intensitydata_1[4];
        check_4 = intensitydata_1[5];
        dado2_e3 = dado3;
        dado2_e4 = dado4;        
      }
    }
    else{
      //DEBUG_PRINT("Reading Fail \n");

    }
        //DEBUG_PRINT("CO2eq: * %lu\n", dado1);
        //DEBUG_PRINT("TVOC: * %lu\n", dado2);
        //DEBUG_PRINT("Raw H2: * %lu\n", dado3);
        //DEBUG_PRINT("Raw Ethanol: * %lu\n", dado4);
        //DEBUG_PRINT("Tempo: * %d\n", T2M(&xLastWakeTime_2));
    
  }
}
void appMain(void* arg)
{
  xTaskCreate(appSGP40, SGP40_TASK_NAME, SGP40_TASK_STACKSIZE, NULL, SGP40_TASK_PRI, NULL);
}

static void sgpInit()
{
  if (isInit)
    return;

  //DEBUG_PRINT("Initializing SGP...\n");
  i2cdevInit(I2C1_DEV);
  isInit = true;
  //DEBUG_PRINT("SGP initialization complete!\n");
  
  uint8_t d_1[] = {0x20, 0x03};
    if (i2cdevWriteReg8(I2C1_DEV, (uint8_t)0x59, I2CDEV_NO_MEM_ADDR, 2, d_1)){
      //DEBUG_PRINT("Writing Air Quality OK\n");
    }
    else{
      //DEBUG_PRINT("Writing Air Quality FAIL \n");
    }    
    vTaskDelay(M2T(10));

}

// Deck driver test function
static bool sgpTest()
{
  return true;
}

static const DeckDriver sgp40Driver = {
  .name = "sgp40",
  .init = sgpInit,
  .test = sgpTest,
};


PARAM_GROUP_START(SGP40Baseline)
PARAM_ADD_CORE(PARAM_UINT16, TVOC, &IAQBaselineTVOC)
PARAM_ADD_CORE(PARAM_UINT16, CO2, &IAQBaselineCO2)
PARAM_GROUP_STOP(SGP40Baseline)

DECK_DRIVER(sgp40Driver);
LOG_GROUP_START(SGP40)
LOG_ADD(LOG_UINT16, co2eq, &dado2_e1)
LOG_ADD(LOG_UINT16, tvoc, &dado2_e2)
LOG_ADD(LOG_UINT16, rawh2, &dado2_e3)
LOG_ADD(LOG_UINT16, raweth, &dado2_e4)
LOG_ADD(LOG_UINT16, co2eqbl, &dado2_e5)
LOG_ADD(LOG_UINT16, tvocbl, &dado2_e6)
LOG_ADD(LOG_UINT16, tvocibl, &dado2_e7)
LOG_GROUP_STOP(SGP40)

