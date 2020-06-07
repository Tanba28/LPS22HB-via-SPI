/* ========================================
 *
 * Copyright (c) 2020 Takumi Niwa
 * This software is released under the MIT license.
 *
 * ========================================
*/

#ifndef LPS22HB_H
#define LPS22HB_H


#include "project.h"
/* Initial LPS22HB Config */
/* Interrupt Config */
#define LPS22HB_CONFIG_AUTORIFP    0 /* default:0 1にした場合、最初に測定した気圧を閾値ベース割り込みの基準値REF_Pに使用する。最初の測定後bitは0になる */
#define LPS22HB_CONFIG_RESET_ARP   0 /* default:0 1にした場合、AUTORIFPモードを終了し、REF_Pをリセットして通常モードに戻る　*/
#define LPS22HB_CONFIG_AUTOZERO    0 /* default:0 1にした場合、最初に測定した気圧を閾値ベース割り込みの基準値REF_Pに使用する。更に、出力PRESS_OUTは測定値からREF_Pを引いたものになる。最初の測定後bitは0になる */
#define LPS22HB_CONFIG_RESET_AZ    0 /* default:0 1にした場合、AUTOZEROモードを終了し、REF_Pをリセットして通常モードに戻る　*/
#define LPS22HB_CONFIG_DIFF_EN     0 /* default:0 1にした場合、閾値ベース割り込みを有効にする。機能の使用にはTHS_Pレジスタに閾値を格納する必要がある */
#define LPS22HB_CONFIG_LIR         0 /* default:0 1にした場合、割り込み信号をラッチする */
#define LPS22HB_CONFIG_PLE         0 /* default:0 1にした場合、REF_Pと比較して測定値が下回ったときに割り込みを発生させる */
#define LPS22HB_CONFIG_PHE         0 /* default:0 1にした場合、REF_Pと比較して測定値が上回ったときに割り込みを発生させる */

/* CTRL REG1 Config */
#define LPS22HB_CONFIG_SIM         0 /* default:0 SPIに関する設定。0:4線SPI, 1:3線SPI */
#define LPS22HB_CONFIG_BDU         0 /* default:0 1にした場合、気圧3バイトをすべて読みださないと次のデータが更新されなくなる */
#define LPS22HB_CONFIG_LPFP_CFG    LPS22HB_LPFP_DISABLE /* default:0x00 LPFに関する設定。列挙型参照 */
#define LPS22HB_CONFIG_ODR         LPS22HB_ODR_75Hz /* defaul:0x00 データレートに関する設定。列挙型参照 */

/* CTRL REG2 Config */
#define LPS22HB_CONFIG_ONE_SHOT    0 /* default:0 1回だけ計測を行う。計測後bitは0になる */
#define LPS22HB_CONFIG_SWRESET     0 /* default:0 1にした場合、ソフトウェアリセットをかける。リセット後bitは0になる */
#define LPS22HB_CONFIG_I2CDIS      0 /* default:0 I2Cの有効化に関する設定。1にした場合I2Cが無効になる */
#define LPS22HB_CONFIG_IF_ADC_INC  1 /* default:1 連続でレジスタを読んだときにアドレスのインクリメントを有効にするかどうか。0の場合インクリメントが無効になる */
#define LPS22HB_CONFIG_STOP_ON_FTH 0 /* default:0 FIFOのウォーターマークを有効にするかどうか */
#define LPS22HB_CONFIG_FIFO_EN     1 /* default:0 FIFOを有効にするかどうか */
#define LPS22HB_CONFIG_BOOT        0 /* default:0 1にした場合、内部メモリをリブートする */

/* CTRL REG3 Config */
#define LPS22HB_CONFIG_INT_S       0 /* default:0x00 データレディ割り込みの条件をどうするか。列挙型参照 */
#define LPS22HB_CONFIG_DRDY        0 /* default:0 データレディ割り込みを有効にするかどうか */
#define LPS22HB_CONFIG_D_OVR       0 /* default:0 FIFOがオーバーランしたときに割り込みを発生させるかどうか */
#define LPS22HB_CONFIG_F_FTH       0 /* default:0 FIFOがウォーターマークに達したときに割り込みを発生させるかどうか */
#define LPS22HB_CONFIG_F_FSS5      1 /* default:0 FIFOがFULLになったときに割り込みを発生させるかどうか */
#define LPS22HB_CONFIG_PP_OD       0 /* default:0 割り込みピンの動作設定。0のときプッシュプル。1のときオープンドレイン */
#define LPS22HB_CONFIG_INT_H_L     0 /* default:0 割り込みのアクティブハイ/アクティブロー設定。0のときアクティブハイ。1のときアクティブロー */

/* FIFO CTRL Config */
#define LPS22HB_CONFIG_WTM         0 /*default:0x00 ウォーターマークの閾値 */
#define LPS22HB_CONFIG_FIFO_MODE   LPS22HB_FIFOMODE_FIFO /*default:0x00 FIFOに関する設定。列挙型参照 */

/* LPS22HB Register Mapping */
#define LPS22HB_INTERRUPT_CFG   0x0B /* R/W Interrupt Register */
#define LPS22HB_THS_P_L         0x0C /* R/W Pressure Threshold Register */
#define LPS22HB_THS_P_H         0x0D
#define LPS22HB_WHO_AM_I        0x0F /* R   WHO AM I */
#define LPS22HB_CTRL_REG1       0x10 /* R/W Control Register */
#define LPS22HB_CTRL_REG2       0x11
#define LPS22HB_CTRL_REG3       0x12
#define LPS22HB_FIFO_CTRL       0x14 /* R/W FIFO Configuration Register */
#define LPS22HB_REF_P_XL        0x15 /* R/W Reference Pressure Register */
#define LPS22HB_REF_P_L         0x16
#define LPS22HB_REF_P_H         0x17
#define LPS22HB_RPDS_L          0x18 /* R/W Pressure Offset Register */
#define LPS22HB_RPDS_H          0x19
#define LPS22HB_RSD_CONF        0x1A /* R/W Resolution Register */
#define LPS22HB_INT_SOURCE      0x25 /* R   Interrupt Register */
#define LPS22HB_FIFO_STATUS     0x26 /* R   FIFO Status Register */
#define LPS22HB_STATUS          0x27 /* R   Status Register */
#define LPS22HB_PRESS_OUT_XL    0x28 /* R   Pressure Output Register */
#define LPS22HB_PRESS_OUT_L     0x29
#define LPS22HB_PRESS_OUT_H     0x2A
#define LPS22HB_TEMP_OUT_L      0x2B /* R   Temperature Output Register */
#define LPS22HB_TEMP_OUT_H      0x2C
#define LPS22HB_LPFP_RES        0x33 /* R   Filter Reset Register */

#define I_AM_LPS22HB 0xB1
    
typedef enum {
    LPS22HB_FALSE = 0x00,
    LPS22HB_TRUE  = 0x01,
} LPS22HB_BIT_t;

typedef enum {
    LPS22HB_PWR_DOWN = 0x00,
    LPS22HB_ODR_1Hz  = 0x01,
    LPS22HB_ODR_10Hz = 0x02,
    LPS22HB_ODR_25Hz = 0x03,
    LPS22HB_ODR_50Hz = 0x04,
    LPS22HB_ODR_75Hz = 0x05
} LPS22HB_ODR_t;

typedef enum {
    LPS22HB_LPFP_DISABLE      = 0x00,
    LPS22HB_LPFP_BANDWIDTH_9  = 0x02,
    LPS22HB_LPFP_BANDWIDTH_20 = 0x03
} LPS22HB_LPFP_CFG_t;

typedef enum {
    LPS22HB_FIFOMODE_BYPASS           = 0x00,
    LPS22HB_FIFOMODE_FIFO             = 0x01,
    LPS22HB_FIFOMODE_STREAM           = 0x02,
    LPS22HB_FIFOMODE_STREAM_TO_FIFO   = 0x03,
    LPS22HB_FIFOMODE_BYPASS_TO_STREAM = 0x04,
    LPS22HB_FIFOMODE_DYNAMIC_STREAM   = 0x06,
    LPS22HB_FIFOMODE_BYPASS_TO_FIFO   = 0x07
} LPS22HB_FIFO_MODE_t;
    
typedef enum {
    LPS22HB_INTSOURCE_DATASIGNAL  = 0x00,
    LPS22HB_INTSOURCE_PRESS_HI    = 0x01,
    LPS22HB_INTSOURCE_PRESS_LOW   = 0x02,
    LPS22HB_INTSOURCE_PRESS_HILOW = 0x03
} LPS22HB_INT_SOURCE_CFG_t;

typedef uint8_t LPS22HB_WTM_t;
typedef uint8_t LPS22HB_FSS_t;
typedef uint8_t LPS22HB_DATA_t;

typedef struct {
    LPS22HB_BIT_t phe       : 1;
    LPS22HB_BIT_t ple       : 1;
    LPS22HB_BIT_t lir       : 1;
    LPS22HB_BIT_t diff_en   : 1;
    LPS22HB_BIT_t reset_az  : 1;
    LPS22HB_BIT_t autozero  : 1;
    LPS22HB_BIT_t reset_arp : 1;
    LPS22HB_BIT_t autorifp  : 1;
} LPS22HB_INTERRUPT_CFG_BYTE_t;

typedef struct {
    LPS22HB_BIT_t sim           : 1;
    LPS22HB_BIT_t bdu           : 1;
    LPS22HB_LPFP_CFG_t lpfg_cfg : 2;
    LPS22HB_ODR_t odr           : 3;
    LPS22HB_BIT_t zero          : 1;
} LPS22HB_CTRL_REG1_BYTE_t;

typedef struct {
    LPS22HB_BIT_t one_shot    : 1;
    LPS22HB_BIT_t zero        : 1;
    LPS22HB_BIT_t swreset     : 1;
    LPS22HB_BIT_t i2c_dis     : 1;
    LPS22HB_BIT_t if_adc_inc  : 1;
    LPS22HB_BIT_t stop_on_fth : 1;
    LPS22HB_BIT_t fifo_en     : 1;
    LPS22HB_BIT_t boot        : 1;
} LPS22HB_CTRL_REG2_BYTE_t;

typedef struct {
    LPS22HB_INT_SOURCE_CFG_t int_s : 2;
    LPS22HB_BIT_t drdy             : 1;
    LPS22HB_BIT_t f_ovr            : 1;
    LPS22HB_BIT_t f_fth            : 1;
    LPS22HB_BIT_t f_fss5           : 1;
    LPS22HB_BIT_t pp_od            : 1;
    LPS22HB_BIT_t int_h_l          : 1;
} LPS22HB_CTRL_REG3_BYTE_t;

typedef struct {
    LPS22HB_WTM_t wtm          : 5;
    LPS22HB_FIFO_MODE_t f_mode : 3;
} LPS22HB_FIFO_CTRL_BYTE_t;

typedef struct {
    LPS22HB_FSS_t fss      : 6;
    LPS22HB_BIT_t ovr      : 1;
    LPS22HB_BIT_t fth_fifo : 1;
} LPS22HB_FIFO_STATUS_BYTE_t;

typedef struct {
    LPS22HB_BIT_t p_da : 1;
    LPS22HB_BIT_t t_da : 1;
    LPS22HB_BIT_t nodata1 : 2;
    LPS22HB_BIT_t p_or : 1;
    LPS22HB_BIT_t t_or : 1;
    LPS22HB_BIT_t nodata2 : 2;
} LPS22HB_STATUS_BYTE_t;

typedef union {
    struct{
        LPS22HB_DATA_t press_out_XL;
        LPS22HB_DATA_t press_out_L;
        LPS22HB_DATA_t press_out_H;
    };
    uint32_t press : 24;
} LPS22HB_PRESS_ROW_t;

typedef union {
    struct{
        LPS22HB_DATA_t temp_out_L;
        LPS22HB_DATA_t temp_out_H;
    };
    uint16_t temp;
} LPS22HB_TEMP_ROW_t;

typedef struct {
    LPS22HB_PRESS_ROW_t press_row;
    LPS22HB_TEMP_ROW_t temp_row;
} LPS22HB_DATA_CONTEINER_t;

typedef struct {
    LPS22HB_INTERRUPT_CFG_BYTE_t interrupt_cfg;
    LPS22HB_CTRL_REG1_BYTE_t ctrl_reg1;
    LPS22HB_CTRL_REG2_BYTE_t ctrl_reg2;
    LPS22HB_CTRL_REG3_BYTE_t ctrl_reg3;
    LPS22HB_FIFO_CTRL_BYTE_t fifo_ctrl;
} LPS22HB_CONFIG_t;

typedef enum {
    READ = 0,
    WRITE = 1
} SPI_DIRECTION;

void LPS22HBStart();
LPS22HB_CONFIG_t LPS22HBInitializeConfig();
LPS22HB_BIT_t LPS22HBWhoAmI();
LPS22HB_BIT_t LPS22HBSoftwareReset();
LPS22HB_BIT_t LPS22HBSetODR(LPS22HB_ODR_t odr,LPS22HB_CONFIG_t *config);
LPS22HB_BIT_t LPS22HBSetDRDY(LPS22HB_BIT_t drdy,LPS22HB_CONFIG_t *config);
LPS22HB_BIT_t LPS22HBSetLPFP(LPS22HB_LPFP_CFG_t lpfp_cfg,LPS22HB_CONFIG_t *config);
LPS22HB_DATA_CONTEINER_t LPS22HBUpdateData();
uint32_t LPS22HBGetPressRow(LPS22HB_DATA_CONTEINER_t lps22hb_data);
uint16_t LPS22HBGetTempRow(LPS22HB_DATA_CONTEINER_t lps22hb_data);
float LPS22HBGetPress(LPS22HB_DATA_CONTEINER_t lps22hb_data);
float LPS22HBGetTemp(LPS22HB_DATA_CONTEINER_t lps22hb_data);

#endif