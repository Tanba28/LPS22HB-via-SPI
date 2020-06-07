/* ========================================
 *
 * Copyright (c) 2020 Takumi Niwa
 * This software is released under the MIT license.
 *
 * ========================================
*/

#include "LPS22HB.h"

static void SpiReadWrite(const uint8_t reg, uint8_t *data, const uint8_t size, const SPI_DIRECTION direction);
static uint8_t SpiMask(const uint8_t reg, const SPI_DIRECTION direction);

static void InterruptConfigApply(LPS22HB_CONFIG_t config);
static void CtrlReg1Apply(LPS22HB_CONFIG_t config);
static void CtrlReg2Apply(LPS22HB_CONFIG_t config);
static void CtrlReg3Apply(LPS22HB_CONFIG_t config);
static void FifoCtrlApply(LPS22HB_CONFIG_t config);

void LPS22HBStart(){
    SPIM_LPS_Start();
}

LPS22HB_CONFIG_t LPS22HBInitializeConfig(){
    LPS22HB_INTERRUPT_CFG_BYTE_t interrupt_cfg = {
    LPS22HB_CONFIG_PHE,
    LPS22HB_CONFIG_PLE,
    LPS22HB_CONFIG_LIR,
    LPS22HB_CONFIG_DIFF_EN,
    LPS22HB_CONFIG_RESET_AZ,
    LPS22HB_CONFIG_AUTOZERO,
    LPS22HB_CONFIG_RESET_ARP,
    LPS22HB_CONFIG_AUTORIFP
    };

    LPS22HB_CTRL_REG1_BYTE_t ctrl_reg1 = {
    LPS22HB_CONFIG_SIM,
    LPS22HB_CONFIG_BDU,
    LPS22HB_CONFIG_LPFP_CFG,
    LPS22HB_CONFIG_ODR,
    0
    };

    LPS22HB_CTRL_REG2_BYTE_t ctrl_reg2 = {
    LPS22HB_CONFIG_ONE_SHOT,
    0,
    LPS22HB_CONFIG_SWRESET,
    LPS22HB_CONFIG_I2CDIS,
    LPS22HB_CONFIG_IF_ADC_INC,
    LPS22HB_CONFIG_STOP_ON_FTH,
    LPS22HB_CONFIG_FIFO_EN,
    LPS22HB_CONFIG_BOOT
    };

    LPS22HB_CTRL_REG3_BYTE_t ctrl_reg3 = {
    LPS22HB_CONFIG_INT_S,
    LPS22HB_CONFIG_DRDY,
    LPS22HB_CONFIG_D_OVR,
    LPS22HB_CONFIG_F_FTH,
    LPS22HB_CONFIG_F_FSS5,
    LPS22HB_CONFIG_PP_OD,
    LPS22HB_CONFIG_INT_H_L,
    };

    LPS22HB_FIFO_CTRL_BYTE_t fifo_ctrl = {
    LPS22HB_CONFIG_WTM,
    LPS22HB_CONFIG_FIFO_MODE
    };
    
    LPS22HB_CONFIG_t config = {
        interrupt_cfg,
        ctrl_reg1,
        ctrl_reg2,
        ctrl_reg3,
        fifo_ctrl
    };
    
    LPS22HBSoftwareReset();
    
    InterruptConfigApply(config);
    CtrlReg1Apply(config);
    CtrlReg2Apply(config);
    CtrlReg3Apply(config);
    FifoCtrlApply(config);
    
    return config;
}

LPS22HB_BIT_t LPS22HBWhoAmI(){
    uint8_t cmd;
       
    SpiReadWrite(LPS22HB_WHO_AM_I,&cmd,1,READ);

    if(cmd == I_AM_LPS22HB){
        return LPS22HB_TRUE;
    }
    return LPS22HB_FALSE;
}

LPS22HB_BIT_t LPS22HBSoftwareReset(){
    LPS22HB_CONFIG_t config;
    config.ctrl_reg2.swreset = 1;
    
    CtrlReg2Apply(config);
    
    return LPS22HB_TRUE;    
}
LPS22HB_BIT_t LPS22HBSetODR(LPS22HB_ODR_t odr,LPS22HB_CONFIG_t *config){
    config->ctrl_reg1.odr = odr;
    
    CtrlReg1Apply(*config);
    
    if(config->ctrl_reg1.odr == odr){
        return LPS22HB_TRUE;
    }
    return LPS22HB_FALSE;
}

LPS22HB_BIT_t LPS22HBSetDRDY(LPS22HB_BIT_t drdy,LPS22HB_CONFIG_t *config){    
    config->ctrl_reg3.drdy = drdy;
    
    CtrlReg3Apply(*config);
    
    if(config->ctrl_reg3.drdy == drdy){
        return LPS22HB_TRUE;
    }
    return LPS22HB_FALSE;
}

LPS22HB_BIT_t LPS22HBSetLPFP(LPS22HB_LPFP_CFG_t lpfp_cfg,LPS22HB_CONFIG_t *config);

LPS22HB_DATA_CONTEINER_t LPS22HBUpdateData(){
    uint8_t cmd[5];
    
    LPS22HB_DATA_CONTEINER_t lps22hb_data;
    
    SpiReadWrite(LPS22HB_PRESS_OUT_XL,cmd,3,READ);
    
    lps22hb_data.press_row.press_out_XL = cmd[0];
    lps22hb_data.press_row.press_out_L = cmd[1];
    lps22hb_data.press_row.press_out_H = cmd[2];
    
    SpiReadWrite(LPS22HB_TEMP_OUT_L,cmd,2,READ);
    lps22hb_data.temp_row.temp_out_L = cmd[0];
    lps22hb_data.temp_row.temp_out_H = cmd[1];
    
    
    return lps22hb_data;
}

uint32_t LPS22HBGetPressRow(LPS22HB_DATA_CONTEINER_t lps22hb_data){
    return lps22hb_data.press_row.press;
}
uint16_t LPS22HBGetTempRow(LPS22HB_DATA_CONTEINER_t lps22hb_data){
    return lps22hb_data.temp_row.temp;
}
float LPS22HBGetPress(LPS22HB_DATA_CONTEINER_t lps22hb_data){
    return lps22hb_data.press_row.press/4096.0;
}
float LPS22HBGetTemp(LPS22HB_DATA_CONTEINER_t lps22hb_data){
    return lps22hb_data.temp_row.temp/100.0;
}

static void SpiReadWrite(const uint8_t reg, uint8_t *data, const uint8_t size, const SPI_DIRECTION direction){
    uint8_t address;
    uint8_t state;
    
    address = SpiMask(reg,direction);
    SPIM_LPS_ClearFIFO();
    
    state = CyEnterCriticalSection();
    SPIM_LPS_WriteTxData(address);
    SPIM_LPS_PutArray(data,size);
    
    while(!(SPIM_LPS_ReadTxStatus() & SPIM_LPS_STS_SPI_DONE));
    
    SPIM_LPS_ReadRxData();
    
    for(uint8_t i = 0;i < size; i++){
        data[i] = SPIM_LPS_ReadRxData();
        
    }
    
    CyExitCriticalSection(state);
}

static uint8_t SpiMask(const uint8_t reg, const SPI_DIRECTION direction){
    if(direction == READ){
        return reg | 0x80;
    }
    return reg;
}

static void InterruptConfigApply(LPS22HB_CONFIG_t config){   
    uint8_t *ptr = (uint8_t*)&config.interrupt_cfg;
    
    SpiReadWrite(LPS22HB_INTERRUPT_CFG,ptr,1,WRITE);
    
    SpiReadWrite(LPS22HB_INTERRUPT_CFG,ptr,1,READ);
}
static void CtrlReg1Apply(LPS22HB_CONFIG_t config){
    uint8_t *ptr = (uint8_t*)&config.ctrl_reg1;
    
    SpiReadWrite(LPS22HB_CTRL_REG1,ptr,1,WRITE);
    
    SpiReadWrite(LPS22HB_CTRL_REG1,ptr,1,READ);
}
static void CtrlReg2Apply(LPS22HB_CONFIG_t config){
    uint8_t *ptr = (uint8_t*)&config.ctrl_reg2;
    
    SpiReadWrite(LPS22HB_CTRL_REG2,ptr,1,WRITE);
    
    SpiReadWrite(LPS22HB_CTRL_REG2,ptr,1,READ);
}
static void CtrlReg3Apply(LPS22HB_CONFIG_t config){
    uint8_t *ptr = (uint8_t*)&config.ctrl_reg3;
    
    SpiReadWrite(LPS22HB_CTRL_REG3,ptr,1,WRITE);
    
    SpiReadWrite(LPS22HB_CTRL_REG3,ptr,1,READ);
}
static void FifoCtrlApply(LPS22HB_CONFIG_t config){
    uint8_t *ptr = (uint8_t*)&config.fifo_ctrl;
    
    SpiReadWrite(LPS22HB_FIFO_CTRL,ptr,1,WRITE);
    
    SpiReadWrite(LPS22HB_FIFO_CTRL,ptr,1,READ);    
}
