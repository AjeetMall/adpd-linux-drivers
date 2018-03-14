/**
 *ADPD143 driver source
 * 
 * Based on ADPD142RI driver
 * Added ADPD143 specific by Joshua Yoon(joshua.yoon@analog.com)
 * 25th/Feb/2016 : Added TIA_ADC/float mode support for ALS by Joshua Yoon(joshua.yoon@analog.com)
 * 25th/Feb/2016 : Added DC normalization support by Joshua Yoon(joshua.yoon@analog.com)
 * 23rd/Mar/2016 : Added AGC(Automatic Gain Control) support by Joshua Yoon(joshua.yoon@analog.com)
 * 25th/Mar/2016 : Added EOL test support by Joshua Yoon(joshua.yoon@analog.com)
 * 30th/Mar/2016 : Enhanced EOL clock calibration by Joshua Yoon(joshua.yoon@analog.com)
 * 30th/Mar/2016 : Added Object Proximity Detection with the dynamic AGC trigger by Joshua Yoon(joshua.yoon@analog.com)
 * 1st/Apr/2016 : Enhanced AGC by Joshua Yoon(joshua.yoon@analog.com)
 * 12nd/Apr/2016 : Enhanced AGC/Proximity integration and some run-time bug fix by Joshua Yoon(joshua.yoon@analog.com)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/


#include <linux/init.h>     /* module initialization api */
#include <linux/module.h>   /* module functionality */
#include <linux/i2c.h>      /* i2c related functionality */
#include <linux/slab.h>
#include <linux/pm.h>       /* device Power Management functionality */
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/average.h>
	
#include <linux/uaccess.h>
	
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
	
#include <linux/moduleparam.h>
#include <linux/input/adpd143.h>

void ewma_init2(struct ewma *avg, unsigned long factor, unsigned long weight)
{
	WARN_ON(!is_power_of_2(weight) || !is_power_of_2(factor));

	avg->weight = ilog2(weight);
	avg->factor = ilog2(factor);
	avg->internal = 0;
}

/**
 * ewma_add() - Exponentially weighted moving average (EWMA)
 * @avg: Average structure
 * @val: Current value
 *
 * Add a sample to the average.
 */
struct ewma *ewma_add2(struct ewma *avg, unsigned long val)
{
	avg->internal = avg->internal  ?
		(((avg->internal << avg->weight) - avg->internal) +
			(val << avg->factor)) >> avg->weight :
		(val << avg->factor);
	return avg;
}

#define GPIO_NUM			59

static int gpio_num = GPIO_NUM;
static int dbg_enable;

module_param(dbg_enable, int, S_IRUGO | S_IWUSR);

/* ADPD143 driver version*/
#define ADPD143_VERSION			"1.1"
/* ADPD143 release date*/
#define ADPD143_RELEASE_DATE		"Tue Feb 16 12:00:00 KST 2016"


#define ADPD_DEV_NAME			"adpd143"

/*INPUT DEVICE NAME LIST*/
#define MODULE_NAME_HRM			"HRM_ALS_sensor"
#define CHIP_NAME				"ADPD143RI"
#define VENDOR				"ADI"

#define MAX_EOL_RESULT			132

/*I2C RELATED*/
#define I2C_RETRY_DELAY			5
#define I2C_RETRIES			5

/*ADPD143 DRIVER RELATED*/
#define MAX_BUFFER			128
#define GLOBAL_OP_MODE_OFF		0
#define GLOBAL_OP_MODE_IDLE		1
#define GLOBAL_OP_MODE_GO		2

#define SET_GLOBAL_OP_MODE(val, cmd)	((val & 0xFC) | cmd)

#define FRM_FILE				0
#define FRM_ARR				1
#define EN_FIFO_CLK			0x3B5B
#define DS_FIFO_CLK			0x3050

/******************************************************************/
/**************** ADPD143 USER OPERATING MODE ********************/
/*
Operating mode defined to user
*/
#define IDLE_MODE			0x00
#define SAMPLE_XY_A_MODE		0x30
#define SAMPLE_XY_AB_MODE		0x31
#define SAMPLE_XY_B_MODE		0x32
#define GET_USR_MODE(usr_val)	((usr_val & 0xF0) >> 4)
#define GET_USR_SUB_MODE(usr_val)	(usr_val & 0xF)
/******************************************************************/
/**************** ADPD143 OPERATING MODE & DATA MODE *************/
#define R_RDATA_FLAG_EN		 (0x1 << 15)
#define R_PACK_START_EN		 (0x0 << 14)
#define	R_RDOUT_MODE			(0x0 << 13)
#define R_FIFO_PREVENT_EN	   (0x1 << 12)
#define	R_SAMP_OUT_MODE			(0x0 << 9)
#define DEFAULT_OP_MODE_CFG_VAL(cfg_x)  (R_RDATA_FLAG_EN + R_PACK_START_EN +\
					R_RDOUT_MODE + R_FIFO_PREVENT_EN+\
					R_SAMP_OUT_MODE)

#define SET_R_OP_MODE_A(val)		((val & 0xF000) >> 12)
#define SET_R_DATAMODE_A(val)	   ((val & 0x0F00) >> 8)
#define SET_R_OP_MODE_B(val)		((val & 0x00F0) >> 4)
#define SET_R_DATAMODE_B(val)	   ((val & 0x000F))
/**
@brief Used to get the value need to be set for slot-A, B operating mode
and data out mode.
*/
#define SET_MODE_VALUE(val)	 (SET_R_OP_MODE_A(val) |\
					(SET_R_DATAMODE_A(val) << 2) |\
					(SET_R_OP_MODE_B(val) << 5) |\
					(SET_R_DATAMODE_B(val) << 6))
/******************************************************************/
/**************** ADPD143 INTERRUPT MASK *************************/
#define INTR_MASK_IDLE_USR		0x0000
#define INTR_MASK_SAMPLE_USR		0x0020
#define INTR_MASK_S_SAMP_XY_A_USR	0x0020
#define INTR_MASK_S_SAMP_XY_AB_USR	0x0040
#define	INTR_MASK_S_SAMP_XY_B_USR	0x0040
#define SET_INTR_MASK(usr_val)		((~(INTR_MASK_##usr_val)) & 0x1FF)
/******************************************************************/
/***************** ADPD143 PRINT *********************************/
#ifdef ADPD_DBG
#define ADPD143_dbg(format, arg...)	\
				printk(KERN_DEBUG "ADPD143 : "format, ##arg);
#else
#define ADPD143_dbg(format, arg...)    {if (dbg_enable)\
				printk(KERN_DEBUG "ADPD143 : "format, ##arg);\
					}
#endif

#ifdef ADPD_INFO
#define ADPD143_info(format, arg...)	\
				printk(KERN_INFO "ADPD143 : "format, ##arg);
#else
#define ADPD143_info(format, arg...)   {if (0)\
					; }
#endif
/******************************************************************/
/***************** ADPD143 DIFFERENT MODE ************************/
/*
Main mode
*/
#define IDLE_USR			0
#define SAMPLE_USR			3
#define TIA_ADC_USR			5

#ifdef __EOL_SUPPORT__
#define EOL_USR				7
#endif
/*
Sub mode
*/
#define S_SAMP_XY_A			0
#define S_SAMP_XY_B			2
#define S_SAMP_XY_AB		1
/*
OPERATING MODE ==>      MAIN_mode[20:16] |
			OP_mode_A[15:12] |
			DATA_mode_A[11:8]|
			OP_mode_B[7:4]   |
			DATA_mode_B[3:0]
*/
#define IDLE_OFF                        ((IDLE_USR << 16) | (0x0000))

#define SAMPLE_XY_A                     ((SAMPLE_USR << 16) | (0x1400))
#define SAMPLE_XY_AB			((SAMPLE_USR << 16) | (0x1414))
#define SAMPLE_XY_B			((SAMPLE_USR << 16) | (0x0014))

#ifdef __EOL_SUPPORT__
#define MAX_MODE                        8
#else
#define MAX_MODE                        6
#endif

#define MAX_IDLE_MODE                   1
#define MAX_SAMPLE_MODE                 3

#define MODE(val)                       __arr_mode_##val
#define MODE_SIZE(val)                  MAX_##val##_MODE


#define MAX_LIB_VER		20
#define OSC_TRIM_ADDR26_REG		0x26
#define OSC_TRIM_ADDR28_REG		0x28
#define OSC_TRIM_ADDR29_REG		0x29
#define OSC_TRIM_32K_REG		0x4B
#define OSC_TRIM_32M_REG		0x4D
#define OSC_DEFAULT_32K_SET		0x2695
#define OSC_DEFAULT_32M_SET		0x4272
#define OSCILLATOR_TRIM_FILE_PATH	"/efs/osc_trim"


/**
mode mapping structure used to hold the number mode actually supported by
driver
*/
struct mode_mapping {
	char *name;
	int *mode_code;
	unsigned int size;
};
int __arr_mode_IDLE[MAX_IDLE_MODE] = { IDLE_OFF };
int __arr_mode_SAMPLE[MAX_SAMPLE_MODE]	= {	SAMPLE_XY_A,
						SAMPLE_XY_AB,
						SAMPLE_XY_B };
int __arr_mode_TIA_ADC_SAMPLE[MAX_SAMPLE_MODE]	= {	SAMPLE_XY_A,
													SAMPLE_XY_AB,
													SAMPLE_XY_B };
#ifdef __EOL_SUPPORT__
int __arr_mode_EOL_SAMPLE[MAX_SAMPLE_MODE]	= {	SAMPLE_XY_A,
												SAMPLE_XY_AB,
												SAMPLE_XY_B };
int __arr_mode_MELANIN_SAMPLE[MAX_SAMPLE_MODE]	= {	SAMPLE_XY_A,
													SAMPLE_XY_AB,
													SAMPLE_XY_B };
#endif


struct mode_mapping __mode_recv_frm_usr[MAX_MODE] = {
	{
		.name = "IDLE",
		.mode_code = MODE(IDLE),
		.size = MODE_SIZE(IDLE)
	},
	{	.name = "UNSUPPORTED1",
		.mode_code = 0,
		.size = 0,
	},
	{	.name = "UNSUPPORTED2",
		.mode_code = 0,
		.size = 0,
	},
	{
		.name = "SAMPLE",
		.mode_code = MODE(SAMPLE),
		.size = MODE_SIZE(SAMPLE)
	}
	,
	{	.name = "UNSUPPORTED3",
		.mode_code = 0,
		.size = 0,
	},	
#ifdef __TIA_ADC_SUPPORT__
	{
		.name = "TIA_ADC_SAMPLE",
		.mode_code = MODE(TIA_ADC_SAMPLE),
		.size = MODE_SIZE(SAMPLE)
	},	
#endif
#ifdef __EOL_SUPPORT__
	{	.name = "MELANIN_SAMPLE",
		.mode_code = MODE(MELANIN_SAMPLE),
		.size = MODE_SIZE(SAMPLE),
	},	

	{
		.name = "EOL_SAMPLE",
		.mode_code = MODE(EOL_SAMPLE),
		.size = MODE_SIZE(SAMPLE)
	},
#endif
};

/**
general data buffer that is required to store the recieved buffer
*/
unsigned short data_buffer[MAX_BUFFER] = { 0 };

struct wrk_q_timestamp {
	struct timeval interrupt_trigger;
};

struct adpd143_stats {
	atomic_t interrupts;
	atomic_t fifo_requires_sync;
	atomic_t fifo_bytes[4];
	atomic_t wq_pending;
	atomic_t wq_schedule_time_peak_usec;
	atomic_t wq_schedule_time_last_usec;
	struct ewma wq_schedule_time_avg_usec;
	atomic_t data_process_time_peak_usec;
	atomic_t data_process_time_last_usec;
	struct ewma data_process_time_avg_usec;
	struct wrk_q_timestamp stamp;
};

#ifdef __EOL_SUPPORT__
typedef enum __EOL_state{
	ST_EOL_IDLE = 0,

	ST_EOL_CLOCK_CAL_INIT,
	ST_EOL_CLOCK_CAL_RUNNING,
	ST_EOL_CLOCK_CAL_DONE,

	ST_EOL_LOW_DC_INIT,
	ST_EOL_LOW_DC_RUNNING,
	ST_EOL_LOW_DC_DONE,

	ST_EOL_MED_DC_INIT,
	ST_EOL_MED_DC_RUNNING,
	ST_EOL_MED_DC_DONE,

	ST_EOL_HIGH_DC_INIT,
	ST_EOL_HIGH_DC_RUNNING,
	ST_EOL_HIGH_DC_DONE,
	
	ST_EOL_AC_INIT,
	ST_EOL_AC_RUNNING,
	ST_EOL_AC_DONE,
	
	ST_EOL_DONE,
} EOL_state;

unsigned short g_eol_regNumbers[32] = {  	0x1,
	                                        0x2,
	                                        0x6,
	                                        0x11,
	                                        0x12,
	                                        0x14,
	                                        0x15,
	                                        0x18,
	                                        0x19,
	                                        0x1a,
	                                        0x1b,
	                                        0x1e,
	                                        0x1f,
	                                        0x20,
	                                        0x21,
	                                        0x23,
	                                        0x24,
	                                        0x25,
	                                        0x27,
	                                        0x30,
	                                        0x31,
	                                        0x34,
	                                        0x35,
	                                        0x36,
	                                        0x39,
	                                        0x3b,
	                                        0x42,
	                                        0x43,
	                                        0x44,
	                                        0x45,
	                                        0x4b,
	                                        0x52};
unsigned short g_org_regValues[33];
unsigned g_eol_ADCOffset[8]; 
unsigned g_dc_buffA[100];
unsigned g_dc_buffB[100];
unsigned g_ac_buffA[100];
unsigned g_ac_buffB[100];


#define EOL_ODR_TARGET 				200
#define EOL_ALLOWABLE_ODR_ERROR 	4

#endif

#ifdef __EOL_SUPPORT__
struct timeval prev_interrupt_trigger_ts;	
struct timeval curr_interrupt_trigger_ts;	
#endif

#ifdef __PROXIMITY_SUPPORT__
typedef enum __OBJ_PROXIMITY_STATUS__{
	OBJ_DETACHED = 0,
	OBJ_DETECTED,
	OBJ_PROXIMITY_RED_ON,
	OBJ_PROXIMITY_ON,
	OBJ_HARD_PRESSURE,
	DEVICE_ON_TABLE,
} OBJ_PROXIMITY_STATUS;
#endif


/**
structure hold adpd143 chip structure, and parameter used
to control the adpd143 software part.
*/
struct adpd143_data {
	struct i2c_client *client;
	struct mutex mutex;/*for chip structure*/
	struct device *dev;
	struct input_dev *ptr_sample_inputdev;
	struct adpd_platform_data *ptr_config;
	struct class *adpd_class;
	struct device *adpd_dev;

	struct work_struct work;
	struct workqueue_struct *ptr_adpd143_wq_st;
	int irq;
	int hrm_int;
	int led_en;
	const char *vdd_1p8;
	struct regulator *leda;
	unsigned short *ptr_data_buffer;

	struct adpd143_stats stats;

	unsigned short intr_mask_val;
	unsigned short intr_status_val;
	unsigned short fifo_size;

	/*sysfs register read write */
	unsigned short sysfs_I2C_regaddr;
	unsigned short sysfs_I2C_regval;
	unsigned short sysfslastreadval;

	atomic_t sample_enabled;
	atomic_t adpd_mode;

	/*Efuse read for DC normalization */
	unsigned short efuseIrSlope;
	unsigned short efuseRedSlope;
	unsigned short efuseIrIntrcpt;
	unsigned short efuseRedIntrcpt;	
	unsigned short efuse32KfreqOffset;
#define SF_RED_CH 	100
#define SF_IR_CH  	100
#ifdef __AGC_SUPPORT__
	unsigned char bOnOffAGC;
#endif

#define OBJ_DETACH_FROM_DETECTION_DC_PERCENT      	80
#define OBJ_DETACH_FROM_PROXIMITY_DC_PERCENT      	60
#define OBJ_DETECTION_CNT_DC_PERCENT    			90
#define OBJ_PROXIMITY_CNT_DC_PERCENT     			70
#define CNT_THRESHOLD_OBJ_PROXIMITY_ON   			2
#define CNT_THRESHOLD_DC_SATURATION					100

#if 1//Proximity Joshua
	OBJ_PROXIMITY_STATUS st_obj_proximity; 
	unsigned obj_proximity_theshld;
	unsigned cur_proximity_dc_level;	
	unsigned char cnt_proximity_on;
	unsigned char cnt_dc_saturation;
	unsigned char cnt_proximity_buf;
	unsigned char begin_proximity_buf;
#endif

#ifdef __EOL_SUPPORT__
	//for HRM mode
	EOL_state eol_state;
	unsigned char eol_counter;

#define __USE_EOL_US_INT_SPACE__
#define __USE_EOL_ADC_OFFSET__
#define CNT_SAMPLE_PER_EOL_CYCLE   	50
#define CNT_EOL_SKIP_SAMPLE   		5

#ifndef __USE_EOL_US_INT_SPACE__
	int msec_eol_int_space;
#else
	int usec_eol_int_space;
#endif
	long eol_measured_period;

	unsigned char eol_res_odr;
	
	unsigned eol_res_ir_high_dc;
	unsigned eol_res_ir_med_dc;
	unsigned eol_res_ir_low_dc;
	unsigned short eol_res_ir_low_noise;
	unsigned short eol_res_ir_med_noise;
	unsigned short eol_res_ir_high_noise;
	unsigned short eol_res_ir_ac;
	
	unsigned eol_res_red_high_dc;
	unsigned eol_res_red_med_dc;
	unsigned eol_res_red_low_dc;
	unsigned short eol_res_red_low_noise;
	unsigned short eol_res_red_med_noise;
	unsigned short eol_res_red_high_noise;
	unsigned short eol_res_red_ac;
#endif

	int (*read)(struct adpd143_data *, u8 reg_addr, int len, u16 *buf);
	int (*write)(struct adpd143_data *, u8 reg_addr, int len, u16 data);
};

typedef struct _adpd_eol_result_{
unsigned char eol_res_odr;

unsigned eol_res_red_low_dc;
unsigned eol_res_ir_low_dc;
unsigned short eol_res_red_low_noise;
unsigned short eol_res_ir_low_noise;

unsigned eol_res_red_med_dc;
unsigned eol_res_ir_med_dc;
unsigned short eol_res_red_med_noise;
unsigned short eol_res_ir_med_noise;

unsigned eol_res_red_high_dc;
unsigned eol_res_ir_high_dc;
unsigned short eol_res_red_high_noise;
unsigned short eol_res_ir_high_noise;

unsigned short eol_res_red_ac;
unsigned short eol_res_ir_ac;
}adpd_eol_result;


typedef union __ADPD_EOL_RESULT__{
	adpd_eol_result st_eol_res;
	char buf[41]; 
} ADPD_EOL_RESULT;

/**
structure hold adpd143 configuration data to be written during probe.
*/

static struct adpd_platform_data adpd143_config_data = {
	.config_size = 58,
	.config_data = {
		0x00100001,
		0x000100ff,
		0x00020004,
		0x00060100,
		0x00111011,
		0x0012000A,
		0x00130320,
		0x00140449,
		0x00150330,
		0x00168080,
		0x00178080,
		0x00181ff0,
		0x00191ff0,
		0x001a1ff0,
		0x001b1ff0,
		0x001c8080,
		0x001d8080,
		0x001e1ff0,
		0x001f1ff0,
		0x00201ff0,
		0x00211ff0,
		0x00233072, //0420
		0x00243075, //0420
		0x00250000,
		0x00260000,
		0x00270800,
		0x00280000,
		0x00294003,
		0x00300330,
		0x00310813,
		0x00320320,
		0x00330113,
		0x00340000,
		0x00350330,
		0x00360813,
		0x003924D2,
		0x003a22d4,
		0x003b24D2,
		0x003c0006,
		0x00401010,
		0x0041004c,
		0x00421c37,
		0x0043ada5,
		0x00441c37,
		0x0045ada5,
		0x004b2694,
		0x004c0004,
		0x004e0040,
		0x004f2090,
		//0x00500000,
		0x005f0001,
		0x00520040,
		0x00540020,
		0x00580000,
		0x00590808,
		0x005a0000,
		0x005b1831,
		0x005d1b31,
		0x005e2808,
	},
};

#ifdef __TIA_ADC_SUPPORT__
static struct adpd_platform_data adpd143_tia_adc_config_data = {
      .config_size = 66,
      .config_data = {
                 0x00100001, 0x000100ff, 0x00020004, 0x00060000,
                 0x00111011, 0x00120020, 0x00130320, 
			     0x00140000,
                 0x00150000, 
                 0x00233034, 0x0024303A, 0x002502CC,
                 0x00340100,
                 0x00260000, 0x00270800, 0x00280000,
			     0x00300220, 0x00350220,
			     0x00310440, 0x00360244,
                 0x00294003, 
                 0x003924D2, 0x003b1a70,
			     0x00421d34, 0x00441d26,
                 0x0043b065, 0x0045ae65,
			     0x005e0808, 0x00597a08,
                 0x00168080, 0x00178080, 0x001C8080, 0x001D8080,                         

                 0x005b0000,  
                 0x00370000, 0x003d0000, 
                 0x00460000, 0x00470080, 0x00480000,
                 0x00490000, 0x004a0000, 0x005d0000, 
                 0x00320320, 0x00330113, 0x003a22d4,
                 0x003c3206,
                 0x00401010, 0x0041004c, 0x004c0004,
                 0x004d4272, 0x004f2090, 0x00500000, 0x00523000,
                 0x0053e400, 0x004e0040, 
                 0x005a0000, 
                 0x00580040,
                 0x00180000,
                 0x00190000,
                 0x001a0000,
                 0x001b0000,
                 0x001e0000,
                 0x001f0000,
                 0x00200000,
                 0x00210000,
	},
};
#endif

static int adpd143_configuration(struct adpd143_data *pst_adpd, unsigned char config);

#ifdef __TIA_ADC_SUPPORT__
static int adpd143_TIA_ADC_configuration(struct adpd143_data *pst_adpd, unsigned char config);

static unsigned short Is_TIA_ADC_Dark_Calibrated = 0;
static unsigned short Is_Float_Dark_Calibrated = 0;
static unsigned rawDarkCh[8];
static unsigned rawDataCh[8];

#define CH1_APPROXIMATE_FLOAT_SATURATION_VALUE_IN_TIA_ADC   1200
#define CH2_APPROXIMATE_FLOAT_SATURATION_VALUE_IN_TIA_ADC   1600
#define CH3_APPROXIMATE_FLOAT_SATURATION_VALUE_IN_TIA_ADC   2200
#define CH4_APPROXIMATE_FLOAT_SATURATION_VALUE_IN_TIA_ADC   4500

static unsigned ch1FloatSat = CH1_APPROXIMATE_FLOAT_SATURATION_VALUE_IN_TIA_ADC;
static unsigned ch2FloatSat = CH2_APPROXIMATE_FLOAT_SATURATION_VALUE_IN_TIA_ADC;
static unsigned ch3FloatSat = CH3_APPROXIMATE_FLOAT_SATURATION_VALUE_IN_TIA_ADC;
static unsigned ch4FloatSat = CH4_APPROXIMATE_FLOAT_SATURATION_VALUE_IN_TIA_ADC;
static unsigned short Is_Prev_TIA_ADC = 0;
#endif

/**
This function is a inline function mapped to i2c read functionality
@param pst_adpd the ADPD143 data structure
@param reg is the address from which the value need to be read out
@return u16 value present in the register.
*/
static inline u16
reg_read(struct adpd143_data *pst_adpd, u16 reg)
{
	u16 value;
	if (!pst_adpd->read(pst_adpd, reg, 1, &value))
		return value;
	return 0xFFFF;
}

/**
This function is a inline function mapped to i2c read functionality
@param pst_adpd the ADPD143 data structure
@param reg is the address from which the value need to be read out
@param len number of data need to be readout
@param buf is pointer store the value present in the register.
@return u16 status of i2c read.
*/
static inline u16
multi_reg_read(struct adpd143_data *pst_adpd, u16 reg, u16 len, u16 *buf)
{
	return pst_adpd->read(pst_adpd, reg, len, buf);
}

/**
This function is a inline function mapped to i2c write functionality
@param pst_adpd the ADPD143 data structure
@param reg is the address to which the data need to be written
@param value is the data need to be write.
@return u16 status of i2c write.
*/
static inline u16
reg_write(struct adpd143_data *pst_adpd, u16 reg, u16 value)
{
	return pst_adpd->write(pst_adpd, reg, 1, value);
}



#ifdef __AGC_SUPPORT__
static struct adpd143_data *gp_adpd_data;

typedef enum {
    MwErrPass = 0x0000,
    MwErrFail,
    MwErrProcessing,

    // LED Error Code
    MwErrLedAOutofRange = 0x0100,
    MwErrLedATrimOutofRange = 0x0200,
    MwErrLedBOutofRange = 0x0400,
    MwErrLedBTrimOutofRange = 0x0800,
} AGC_ErrCode_t;

#define DOC_INIT_VALUE      0x1FFF

//#define CH_RAW_32BIT_DATA_SIZE    
#define MAX_CODE_AFTER_ADC_OFFSET_PER_PULSE  0x1FFF     
#define MAX_CODE_16BIT  0xffff     
#define MAX_CODE_32BIT  0xffffffff
#define MAX_CODE_PERCENTAGE_A  80
#define MAX_CODE_PERCENTAGE_B  80
#define AGC_STOP_MARGIN_PERCENTAGE_A 10
#define AGC_STOP_MARGIN_PERCENTAGE_B 10

//LED driver current - 25~250mA
//if prefMinPower = 1
#define POWER_PREF_MAX_LED_DRV_CURRENT_A 250
#define POWER_PREF_MIN_LED_DRV_CURRENT_A 25
#define POWER_PREF_MAX_LED_DRV_CURRENT_B 250
#define POWER_PREF_MIN_LED_DRV_CURRENT_B 25
//if prefMinPower = 0
#define PERF_PREF_MAX_LED_DRV_CURRENT_A 250
#define PERF_PREF_MIN_LED_DRV_CURRENT_A 25
#define PERF_PREF_MAX_LED_DRV_CURRENT_B 250
#define PERF_PREF_MIN_LED_DRV_CURRENT_B 25

//TIA gain - 200k : x8, 100k :x 4, 50k : x2, 25k : x1
#define MAX_TIA_GAIN_A        8
#define MIN_TIA_GAIN_A        1
#define MAX_TIA_GAIN_B        8
#define MIN_TIA_GAIN_B        1
//AFE gain - x1~x4
#define MAX_AFE_GAIN_A        4
#define MIN_AFE_GAIN_A        1
#define MAX_AFE_GAIN_B        4
#define MIN_AFE_GAIN_B        1
//# of samples to skip
#define NO_OF_SAMPLES_TO_SKIP  2

/* Public function prototypes -----------------------------------------------*/
AGC_ErrCode_t initAGC(unsigned char  param_prefMinPower);
//AGC_ErrCode_t confAGC();
AGC_ErrCode_t stepAGC(unsigned *rawDataA, unsigned *rawDataB);
//AGC_ErrCode_t runAGC();


/* Private function prototypes ----------------------------------------------*/
static unsigned maxValue(unsigned *testData, unsigned short size);
static AGC_ErrCode_t stepAgcLedA(unsigned *rawData);
static AGC_ErrCode_t stepAgcLedB(unsigned *rawData);

/* static variable related with AGC*/
static unsigned char RegLedTrimBitPosA, RegLedTrimBitPosB;
static unsigned RegAddrLedDrvA, RegAddrLedDrvB;
//static unsigned RegAddrAfeTiaA, RegAddrAfeTiaB;
static unsigned CurrLedDrvValA, CurrLedDrvValB;
//static unsigned CurrLedTrimValA, CurrLedTrimValB;
static unsigned CurrLedScaleA, CurrLedScaleB;
static unsigned CurrLedCoarseA, CurrLedCoarseB;
static unsigned CurrTIAgainA, CurrTIAgainB;
static unsigned CurrAFEgainA, CurrAFEgainB;

static unsigned PrevLedScaleA, PrevLedScaleB;
static unsigned PrevLedCoarseA, PrevLedCoarseB;
static unsigned PrevTIAgainA, PrevTIAgainB;
static unsigned PrevAFEgainA, PrevAFEgainB;
static unsigned char cntStepAgcA, cntStepAgcB;
static unsigned prevStepDiffAgcA, prevStepDiffAgcB;

//static unsigned CurrPulseWidthA, CurrPulseWidthB;
//static unsigned short AgcStopMarginA, AgcStopMarginB;
static unsigned TargetDcLevelA, TargetDcLevelB;
static unsigned char  pulseA, pulseB;
static unsigned char  sumA, sumB;

static unsigned short maxLedCurrentRegValA;
static unsigned short minLedCurrentRegValA;
static unsigned short maxTiaGainRegValA;
static unsigned short minTiaGainRegValA;
static unsigned short maxAfeGainRegValA;
static unsigned short minAfeGainRegValA;
static unsigned short maxLedCurrentRegValB;
static unsigned short minLedCurrentRegValB; 
static unsigned short maxTiaGainRegValB;
static unsigned short minTiaGainRegValB;
static unsigned short maxAfeGainRegValB;
static unsigned short minAfeGainRegValB; 

static unsigned char  prefMinPower = 0;
static unsigned char  NoSkipSamples = 0;//# of samples to skip
static unsigned char  maxLEDcurrDrvA, minLEDcurrDrvA;
static unsigned char  maxLEDcurrDrvB, minLEDcurrDrvB;

typedef enum
{
    AgcStage_LedA_Init=0,
    AgcStage_LedA,
    AgcStage_LedA_Trim_Init,
    AgcStage_LedA_Trim,
    AgcStage_LedB_Init,
    AgcStage_LedB,
    AgcStage_LedB_Trim_Init,
    AgcStage_LedB_Trim,
    AgcStage_Done,
} AgcStage_t;

static AgcStage_t AgcStage = AgcStage_LedA_Init;
AgcStage_t getAGCstate(void){ return AgcStage; }


#define INT_STATUS              0x00
#define INT_MASK                0x01
#define PAD_IO_CTRL             0x02
#define I2CS_CTL_MATCH          0x06
#define CHIP_ID                 0x08
#define OP_MODE                 0x10
#define OP_MODE_CFG             0x11
#define SAMPLING_FREQ           0x12
#define PD_SELECT               0x14
#define DEC_MODE                0x15
#define CH1_OFFSET_A            0x18
#define CH2_OFFSET_A            0x19
#define CH3_OFFSET_A            0x1A
#define CH4_OFFSET_A            0x1B
#define CH1_OFFSET_B            0x1E
#define CH2_OFFSET_B            0x1F
#define CH3_OFFSET_B            0x20
#define CH4_OFFSET_B            0x21
#define LED3_DRV                0x22
#define LED1_DRV                0x23
#define LED2_DRV                0x24
#define LED_TRIM                0x25
#define PULSE_OFFSET_A          0x30
#define PULSE_PERIOD_A          0x31
#define PULSE_MASK              0x34
#define PULSE_OFFSET_B          0x35
#define PULSE_PERIOD_B          0x36
#define AFE_CTRL_A              0x39
#define AFE_CTRL_B              0x3B
#define AFE_TRIM_A              0x42
#define AFE_TEST_A              0x43
#define AFE_TRIM_B              0x44
#define AFE_TEST_B              0x45
#define OSC32K                  0x4B
#define TEST_PD                 0x52
#define EFUSE_CTRL              0x57
#define FIFO_CLK_SET            0x5f
#define DATA_BUFFER             0x60
#define EFUSE_STATUS0           0x67

//#define CH_RAW_32BIT_DATA_SIZE
//unsinged uAgcRawDataA[4];
//unsinged uAgcRawDataB[4];

AGC_ErrCode_t initAGC(unsigned char  param_prefMinPower)
{
#ifdef CH_RAW_32BIT_DATA_SIZE//20150730 ADI Joshua Yoon for Gear4 AGC
	unsigned temp32;
#endif
    unsigned short temp16, slotA_led, slotB_led;
    unsigned reg_value, targetDcLevelA, targetDcLevelB;

	if( !gp_adpd_data )
	{
		ADPD143_dbg("ADPD should be initialized first before initAGC()!!!!\r\n");
		return MwErrFail;
	}

#if 0	
	if( AgcStage == AgcStage_LedA || AgcStage == AgcStage_LedB )
	{
		ADPD143_dbg("initAGC() was already done!!!!\r\n");
		return MwErrFail;
	}
#endif

	prefMinPower = param_prefMinPower;

    // read reg14
	reg_value = reg_read(gp_adpd_data, PD_SELECT);
    slotA_led=reg_value&0x03;
    slotB_led=(reg_value&0x0C)>>2;

	ADPD143_dbg("slotA_led : %d , slotB_led : %d",slotA_led,slotB_led);
	//Find the current LED allocation to each slotA/B
    if (slotA_led == 1) {
        RegAddrLedDrvA = LED1_DRV;
        RegLedTrimBitPosA = 0;
        //LedTrimMaskA = 0xFFC0;
    } else if (slotA_led == 2) {
        RegAddrLedDrvA = LED2_DRV;
        RegLedTrimBitPosA = 6;
        //LedTrimMaskA = 0xF81F;
    } else if (slotA_led == 3) {
        RegAddrLedDrvA = LED3_DRV;
        RegLedTrimBitPosA = 11;
        //LedTrimMaskA = 0x07DF;
    } else {
        RegAddrLedDrvA = 0;
    }
    
    
    if (slotB_led == 1) {
        RegAddrLedDrvB = LED1_DRV;
        RegLedTrimBitPosB = 0;
        //LedTrimMaskB = 0xFFC0;
    } else if (slotB_led == 2) {
        RegAddrLedDrvB = LED2_DRV;
        RegLedTrimBitPosB = 6;
        //LedTrimMaskB = 0xF81F;
    } else if (slotB_led == 3) {
        RegAddrLedDrvB = LED3_DRV;
        RegLedTrimBitPosB = 11;
        //LedTrimMaskB = 0x07DF;
    } else {
        RegAddrLedDrvB = 0;        
    }

	if( prefMinPower )
	{
		maxLEDcurrDrvA = POWER_PREF_MAX_LED_DRV_CURRENT_A;
		minLEDcurrDrvA = POWER_PREF_MIN_LED_DRV_CURRENT_A;
		maxLedCurrentRegValA = (POWER_PREF_MAX_LED_DRV_CURRENT_A-25)/15;
		minLedCurrentRegValA = (POWER_PREF_MIN_LED_DRV_CURRENT_A-25)/15;
		maxTiaGainRegValA = (MAX_TIA_GAIN_A>=4)?(8-MAX_TIA_GAIN_A)>>2:(MAX_TIA_GAIN_A>=2)?MAX_TIA_GAIN_A:(8-MAX_TIA_GAIN_A)>>1;
		minTiaGainRegValA = (MIN_TIA_GAIN_A>=4)?(8-MIN_TIA_GAIN_A)>>2:(MIN_TIA_GAIN_A>=2)?MIN_TIA_GAIN_A:(8-MIN_TIA_GAIN_A)>>1; 
		maxAfeGainRegValA = (MAX_AFE_GAIN_A==1)?0:MAX_AFE_GAIN_A>>1;
		minAfeGainRegValA = (MIN_AFE_GAIN_A==1)?0:MIN_AFE_GAIN_A>>1;

		maxLEDcurrDrvB = POWER_PREF_MAX_LED_DRV_CURRENT_B;
		minLEDcurrDrvB = POWER_PREF_MIN_LED_DRV_CURRENT_B;
		maxLedCurrentRegValB = (POWER_PREF_MAX_LED_DRV_CURRENT_B-25)/15;
		minLedCurrentRegValB = (POWER_PREF_MIN_LED_DRV_CURRENT_B-25)/15;
		maxTiaGainRegValB = (MAX_TIA_GAIN_B>=4)?(8-MAX_TIA_GAIN_B)>>2:(MAX_TIA_GAIN_B>=2)?MAX_TIA_GAIN_B:(8-MAX_TIA_GAIN_B)>>1;
		minTiaGainRegValB = (MIN_TIA_GAIN_B>=4)?(8-MIN_TIA_GAIN_B)>>2:(MIN_TIA_GAIN_B>=2)?MIN_TIA_GAIN_B:(8-MIN_TIA_GAIN_B)>>1;
		maxAfeGainRegValB = (MAX_AFE_GAIN_B==1)?0:MAX_AFE_GAIN_B>>1;
		minAfeGainRegValB = (MIN_AFE_GAIN_B==1)?0:MIN_AFE_GAIN_B>>1;
	}
	else
	{
		maxLEDcurrDrvA = PERF_PREF_MAX_LED_DRV_CURRENT_A;
		minLEDcurrDrvA = PERF_PREF_MIN_LED_DRV_CURRENT_A;
		maxLedCurrentRegValA = (PERF_PREF_MAX_LED_DRV_CURRENT_A-25)/15;
		minLedCurrentRegValA = (PERF_PREF_MIN_LED_DRV_CURRENT_A-25)/15;
		maxTiaGainRegValA = (MAX_TIA_GAIN_A>=4)?(8-MAX_TIA_GAIN_A)>>2:(MAX_TIA_GAIN_A>=2)?MAX_TIA_GAIN_A:(8-MAX_TIA_GAIN_A)>>1;
		minTiaGainRegValA = (MIN_TIA_GAIN_A>=4)?(8-MIN_TIA_GAIN_A)>>2:(MIN_TIA_GAIN_A>=2)?MIN_TIA_GAIN_A:(8-MIN_TIA_GAIN_A)>>1; 
		maxAfeGainRegValA = (MAX_AFE_GAIN_A==1)?0:MAX_AFE_GAIN_A>>1;
		minAfeGainRegValA = (MIN_AFE_GAIN_A==1)?0:MIN_AFE_GAIN_A>>1;

		maxLEDcurrDrvB = PERF_PREF_MAX_LED_DRV_CURRENT_B;
		minLEDcurrDrvB = PERF_PREF_MIN_LED_DRV_CURRENT_B;
		maxLedCurrentRegValB = (PERF_PREF_MAX_LED_DRV_CURRENT_B-25)/15;
		minLedCurrentRegValB = (PERF_PREF_MIN_LED_DRV_CURRENT_B-25)/15;
		maxTiaGainRegValB = (MAX_TIA_GAIN_B>=4)?(8-MAX_TIA_GAIN_B)>>2:(MAX_TIA_GAIN_B>=2)?MAX_TIA_GAIN_B:(8-MAX_TIA_GAIN_B)>>1;
		minTiaGainRegValB = (MIN_TIA_GAIN_B>=4)?(8-MIN_TIA_GAIN_B)>>2:(MIN_TIA_GAIN_B>=2)?MIN_TIA_GAIN_B:(8-MIN_TIA_GAIN_B)>>1;
		maxAfeGainRegValB = (MAX_AFE_GAIN_B==1)?0:MAX_AFE_GAIN_B>>1;
		minAfeGainRegValB = (MIN_AFE_GAIN_B==1)?0:MIN_AFE_GAIN_B>>1;
	}
    
    // find number of pulses gp_adpd_data
	reg_value = reg_read(gp_adpd_data, PULSE_PERIOD_A);
    pulseA=(reg_value>>8)& 0xFF;
	reg_value = reg_read(gp_adpd_data, DEC_MODE);
    sumA=(reg_value>>4)& 0x7;
	
#ifndef CH_RAW_32BIT_DATA_SIZE//for 16bit raw data	
	targetDcLevelA = MAX_CODE_AFTER_ADC_OFFSET_PER_PULSE*MAX_CODE_PERCENTAGE_A*pulseA/100;
	temp16 = MAX_CODE_16BIT/100*MAX_CODE_PERCENTAGE_A;
    if (targetDcLevelA > temp16)
        targetDcLevelA = temp16;
#else//for 32bit raw data		
	targetDcLevelA = MAX_CODE_AFTER_ADC_OFFSET_PER_PULSE*MAX_CODE_PERCENTAGE_A*pulseA/100;
    targetDcLevelA <<= pulseA;
	temp32 = MAX_CODE_32BIT/100*MAX_CODE_PERCENTAGE_A;
    if (targetDcLevelA > temp32)
        targetDcLevelA = temp32;	
#endif	

	reg_value = reg_read(gp_adpd_data, PULSE_PERIOD_B);
    pulseB=(reg_value>>8)& 0xFF;
	reg_value = reg_read(gp_adpd_data, DEC_MODE);
    sumB=(reg_value>>8)& 0x7;
	
#ifndef CH_RAW_32BIT_DATA_SIZE//for 16bit raw data	
	targetDcLevelB = MAX_CODE_AFTER_ADC_OFFSET_PER_PULSE*MAX_CODE_PERCENTAGE_B*pulseB/100; 
	temp16 = MAX_CODE_16BIT/100*MAX_CODE_PERCENTAGE_B;
    if (targetDcLevelB > temp16)
        targetDcLevelB = temp16;	
#else//for 32bit raw data	
	targetDcLevelB = MAX_CODE_AFTER_ADC_OFFSET_PER_PULSE*MAX_CODE_PERCENTAGE_B*pulseB/100; 
    targetDcLevelB <<= pulseB;
	temp32 = MAX_CODE_32BIT/100*MAX_CODE_PERCENTAGE_B;
    if (targetDcLevelB > temp32)
        targetDcLevelB = temp32;
#endif

    TargetDcLevelA = targetDcLevelA;
    TargetDcLevelB = targetDcLevelB;
	ADPD143_dbg("LED PulseA = %d PulseB = %d", pulseA, pulseB);
    ADPD143_dbg("LED TargetA = %d TargetB = %d", TargetDcLevelA, TargetDcLevelB);

	
    // Set DOC initial value
    reg_write(gp_adpd_data, 0x10, 0x1);
#if 0	
    regWrite(0x18, DOC_INIT_VALUE);
    regWrite(0x19, DOC_INIT_VALUE);
    regWrite(0x1A, DOC_INIT_VALUE);
    regWrite(0x1B, DOC_INIT_VALUE);
    regWrite(0x1E, DOC_INIT_VALUE);
    regWrite(0x1F, DOC_INIT_VALUE);
    regWrite(0x20, DOC_INIT_VALUE);
    regWrite(0x21, DOC_INIT_VALUE);
#endif

#if 0
	//start from LED trim x1.0 
	reg_value = 0xC | 0xC<<6 | 0xC<<11; 
    reg_write(gp_adpd_data, LED_TRIM, reg_value);
#endif
	//set slotB pusle count
    //regRead(PULSE_PERIOD_B, &reg_value);
    //pulseB=(reg_value>>8)&0xFF;
	//regWrite(PULSE_PERIOD_B, 0x1013);

	if( prefMinPower )
	{ // if power preference, start from 100% LED scale,  min LED power & max TIA gain
		reg_value = reg_read(gp_adpd_data, 0x42);
		reg_write(gp_adpd_data, 0x42, (reg_value & 0xfffc) | maxTiaGainRegValA);
		reg_value = reg_read(gp_adpd_data, 0x44);
		reg_write(gp_adpd_data, 0x44, (reg_value & 0xfffc) | maxTiaGainRegValB);
		if (RegAddrLedDrvA != 0){
			reg_write(gp_adpd_data, RegAddrLedDrvA, 0x3030|minLedCurrentRegValA);
		}
		if (RegAddrLedDrvB != 0){
			reg_write(gp_adpd_data, RegAddrLedDrvB, 0x3030|minLedCurrentRegValB);
		}	
	}
	else// if no power preference, start from 100% LED scale, max LED power & min TIA gain
	{
		reg_value = reg_read(gp_adpd_data, 0x42);
		reg_write(gp_adpd_data, 0x42, (reg_value & 0xfffc) | minTiaGainRegValA);
		reg_value = reg_read(gp_adpd_data, 0x44);
		reg_write(gp_adpd_data, 0x44, (reg_value & 0xfffc) | minTiaGainRegValB);
		if (RegAddrLedDrvA != 0){
			reg_write(gp_adpd_data, RegAddrLedDrvA, 0x1070|maxLedCurrentRegValA);
		}
		if (RegAddrLedDrvB != 0){
			reg_write(gp_adpd_data, RegAddrLedDrvB, 0x1070|maxLedCurrentRegValB);
		}	
		
	}
    reg_write(gp_adpd_data, 0x52, 0x0040);
	
    //reg_write(gp_adpd_data, 0x12, 100);
    reg_write(gp_adpd_data, 0x10, 0x2);
	NoSkipSamples = 0;
	cntStepAgcA = cntStepAgcB = 0;
	prevStepDiffAgcA = prevStepDiffAgcB = 0xffffffff;
		
	if (RegAddrLedDrvA != 0)
    	AgcStage = AgcStage_LedA;
	else if(RegAddrLedDrvB != 0) 
    	AgcStage = AgcStage_LedB;
	else{
		AgcStage = AgcStage_Done;
		return MwErrFail;
	}

    return MwErrPass;
}

AGC_ErrCode_t stepAGC(unsigned *rawDataA, unsigned *rawDataB) {
    unsigned char retVal;

	if( NoSkipSamples++ < NO_OF_SAMPLES_TO_SKIP )
	{
		ADPD143_dbg("$$$$$$$$$$$$$$$$$$$$$$$ stepAGC = %d", NoSkipSamples);	
		return MwErrProcessing;
	}

    reg_write(gp_adpd_data, 0x10, 0x1);
    if (AgcStage == AgcStage_LedA) {
		if( rawDataA != NULL  )
		{
			retVal = stepAgcLedA(rawDataA);
			if( retVal == MwErrLedAOutofRange || retVal == MwErrPass )
			{
				if(RegAddrLedDrvB != 0) 
					AgcStage = AgcStage_LedB;
				else
					AgcStage = AgcStage_Done;
			}
		}
		else
			AgcStage = AgcStage_LedB;
    }
    if (AgcStage == AgcStage_LedB ) {
		if( rawDataB != NULL  )
		{
	        retVal = stepAgcLedB(rawDataB);
			if( retVal == MwErrLedAOutofRange || retVal == MwErrPass )
				AgcStage = AgcStage_Done;
		}
		else{
			AgcStage = AgcStage_Done;
			//reg_write(gp_adpd_data, 0x10, 0x1);
			//reg_write(gp_adpd_data, 0x12, 10);
			//reg_write(gp_adpd_data, 0x10, 0x2);
		}
    }
    reg_write(gp_adpd_data, 0x10, 0x2);

    return retVal;
}

static AGC_ErrCode_t stepAgcLedA(unsigned *rawData){
    unsigned raw_max, diff;
    unsigned char toInc = 0;
	unsigned reg_val;
	unsigned short currLedCoarseCurrent;
	unsigned short tgtLedCoarseCurrent;	

    if (RegAddrLedDrvA != 0){
		CurrLedDrvValA = reg_read(gp_adpd_data, RegAddrLedDrvA);
		CurrLedScaleA = (CurrLedDrvValA & 0x2000)>>12;
		CurrLedCoarseA = CurrLedDrvValA & 0x000f;
    }
	reg_val = reg_read(gp_adpd_data, 0x42);
	CurrTIAgainA = reg_val & 0x3; 
	CurrAFEgainA = (reg_val & 0x0300) >> 8;	

	reg_val = reg_read(gp_adpd_data, PULSE_PERIOD_A);
    pulseA=(reg_val>>8)&0xFF;	
	reg_val = reg_read(gp_adpd_data, PULSE_PERIOD_B);
    pulseB=(reg_val>>8)&0xFF;
	
    raw_max =  maxValue(rawData, 4);

	ADPD143_dbg("------------------------stepAgcLedA [start : %u]", cntStepAgcA++);
	ADPD143_dbg("maxLedCurrentRegValA : %u minLedCurrentRegValA : %u maxTiaGainRegValA : %u minTiaGainRegValA : %u", 
				maxLedCurrentRegValA, minLedCurrentRegValA, maxTiaGainRegValA, maxTiaGainRegValA);
	ADPD143_dbg("LED PulseA = %d PulseB = %d", pulseA, pulseB);
	ADPD143_dbg("current sample : %d target : %d ",raw_max,TargetDcLevelA);
	
    if (raw_max > TargetDcLevelA) {
        diff = raw_max - TargetDcLevelA;
        toInc = 0;
    } else {
        diff = TargetDcLevelA - raw_max;
        toInc = 1;
    }
	ADPD143_dbg("CurrLedScaleA : %d,  CurrLedCoarseA : %d, CurrTIAgainA : %d, CurrAFEgainA : %d ",
		  		CurrLedScaleA, CurrLedCoarseA, CurrTIAgainA, CurrAFEgainA);
	ADPD143_dbg("%s - PrevLedScaleA : %d,  PrevLedCoarseA : %d, PrevTIAgainA : %d, PrevAFEgainA : %d\n",
		__func__, PrevLedScaleA, PrevLedCoarseA, PrevTIAgainA, PrevAFEgainA);
	ADPD143_dbg("current diff : %d,  prev diff : %d,  toInc : %d ", diff, prevStepDiffAgcA, toInc);

#if 1
	if( prevStepDiffAgcA < diff )
	{
#if 1
		if( (PrevTIAgainA - CurrTIAgainA) == 1 || (CurrTIAgainA - PrevTIAgainA) == 1 )
			reg_write(gp_adpd_data, 0x42, (reg_val & 0xfffc) | PrevTIAgainA);
		else if( CurrTIAgainA > PrevTIAgainA )
			reg_write(gp_adpd_data, RegAddrLedDrvA, (reg_val & 0xfff0) | (CurrTIAgainA-1));
		else if( CurrTIAgainA < PrevTIAgainA )
			reg_write(gp_adpd_data, RegAddrLedDrvA, (reg_val & 0xfff0) | (CurrTIAgainA+1));
		
		if( (PrevLedCoarseA - CurrLedCoarseA) == 1 || (CurrLedCoarseA - PrevLedCoarseA) == 1 )
			reg_write(gp_adpd_data, RegAddrLedDrvA, (reg_val & 0xfff0) | PrevLedCoarseA);
		else if( CurrLedCoarseA > PrevLedCoarseA )
			reg_write(gp_adpd_data, RegAddrLedDrvA, (reg_val & 0xfff0) | (CurrLedCoarseA-1));
		else if( CurrLedCoarseA < PrevLedCoarseA )
			reg_write(gp_adpd_data, RegAddrLedDrvA, (reg_val & 0xfff0) | (CurrLedCoarseA+1));
#else	
		reg_write(gp_adpd_data, 0x42, (reg_val & 0xfffc) | PrevTIAgainA );
		reg_write(gp_adpd_data, RegAddrLedDrvA, (reg_val & 0xfff0) | PrevLedCoarseA );	
#endif
		ADPD143_dbg("------------------------stepAgcLedA stopped at best!!!!![End]");
		NoSkipSamples = 0;
		return MwErrLedAOutofRange;
	}
#endif
	PrevLedScaleA = CurrLedScaleA; PrevLedCoarseA = CurrLedCoarseA; PrevTIAgainA = CurrTIAgainA; PrevAFEgainA = CurrAFEgainA;
	prevStepDiffAgcA = diff;

	if( !toInc )
	{
		if( diff >= raw_max>>1 )
		{ 
			if( !prefMinPower )// min TIA gain, max LED current
			{
				if( CurrTIAgainA < minTiaGainRegValA )
				{
					reg_val = reg_read(gp_adpd_data, 0x42);
					reg_write(gp_adpd_data, 0x42, (reg_val & 0xfffc) | (CurrTIAgainA+1));
					NoSkipSamples = 0;
					ADPD143_dbg("------------------------stepAgcLedA MwErrProcessing[End]");
					return MwErrProcessing;
				}
				else
				{
					if( CurrLedCoarseA > minLedCurrentRegValA )
					{
						currLedCoarseCurrent = CurrLedCoarseA*15+25;
						tgtLedCoarseCurrent = currLedCoarseCurrent>>1;
						if( tgtLedCoarseCurrent < minLEDcurrDrvA )
							tgtLedCoarseCurrent = minLEDcurrDrvA;
						reg_val = reg_read(gp_adpd_data, RegAddrLedDrvA);
						reg_write(gp_adpd_data, RegAddrLedDrvA, (reg_val & 0xfff0) | ((tgtLedCoarseCurrent-25)/15));
						NoSkipSamples = 0;
						ADPD143_dbg("------------------------stepAgcLedA MwErrProcessing[End]");
						return MwErrProcessing; 	
					}
					else{						
						NoSkipSamples = 0;					
						ADPD143_dbg("------------------------stepAgcLedA MwErrLedAOutofRange1[End]");
						return MwErrLedAOutofRange;
					}
				}
			}
			else// max TIA gain, min LED current
			{
				if( CurrLedCoarseA > minLedCurrentRegValA )
				{
					currLedCoarseCurrent = CurrLedCoarseA*15+25;
					tgtLedCoarseCurrent = currLedCoarseCurrent>>1;
					if( tgtLedCoarseCurrent < minLEDcurrDrvA )
						tgtLedCoarseCurrent = minLEDcurrDrvA;
					reg_val = reg_read(gp_adpd_data, RegAddrLedDrvA);
					reg_write(gp_adpd_data, RegAddrLedDrvA, (reg_val & 0xfff0) | ((tgtLedCoarseCurrent-25)/15));
					NoSkipSamples = 0;					
					ADPD143_dbg("------------------------stepAgcLedA MwErrProcessing[End]");
					return MwErrProcessing;
				}
				else
				{
					if( CurrTIAgainA < minTiaGainRegValA )
					{
						reg_val = reg_read(gp_adpd_data, 0x42);
						reg_write(gp_adpd_data, 0x42, (reg_val & 0xfffc) | (CurrTIAgainA+1));
						NoSkipSamples = 0;						
						ADPD143_dbg("------------------------stepAgcLedA MwErrProcessing[End]");
						return MwErrProcessing;
					}
					else{
						NoSkipSamples = 0;					
						ADPD143_dbg("------------------------stepAgcLedA MwErrLedAOutofRange2[End]");						
						return MwErrLedAOutofRange;
					}
				}	
			}	
		}
		else if( diff > TargetDcLevelA*AGC_STOP_MARGIN_PERCENTAGE_A/100 )
		{
			if( !prefMinPower )// min TIA gain, max LED current
			{
#if 0//if want to smaller code gap from target			
				if( CurrTIAgainA < minTiaGainRegValA && CurrLedCoarseA < maxLedCurrentRegValA )//return to smaller TIA gain@max LED current
				{
					currLedCoarseCurrent = (CurrLedCoarseA+1)*15+25;
					tgtLedCoarseCurrent = currLedCoarseCurrent<<1;
					if( tgtLedCoarseCurrent > maxLEDcurrDrvA )
						tgtLedCoarseCurrent = maxLEDcurrDrvA;
					reg_val = reg_read(gp_adpd_data, RegAddrLedDrvA);
					reg_write(gp_adpd_data, RegAddrLedDrvA, (reg_val & 0xfff0) | ((tgtLedCoarseCurrent-25)/15));
					reg_val = reg_read(gp_adpd_data, 0x42);
					reg_write(gp_adpd_data, 0x42, (reg_val & 0xfffc) | (CurrTIAgainA+1));
					NoSkipSamples = 0;
				
					ADPD143_dbg("------------------------stepAgcLedA MwErrLedAOutofRange3[End]");
					return MwErrLedAOutofRange;
				}
				else
#endif
				{
					if( CurrLedCoarseA > minLedCurrentRegValA )
					{
						reg_val = reg_read(gp_adpd_data, RegAddrLedDrvA);
						reg_write(gp_adpd_data, RegAddrLedDrvA, (reg_val & 0xfff0) | (CurrLedCoarseA-1));
						NoSkipSamples = 0;						
						ADPD143_dbg("------------------------stepAgcLedA MwErrProcessing[End]");
						return MwErrProcessing; 	
					}
					else{
						NoSkipSamples = 0;					
						ADPD143_dbg("------------------------stepAgcLedA MwErrLedAOutofRange[End]");						
						return MwErrLedAOutofRange;
					}
				}
			}
			else// max TIA gain, min LED current
			{
		
				if( CurrLedCoarseA > minLedCurrentRegValA )
				{
					reg_val = reg_read(gp_adpd_data, RegAddrLedDrvA);
					reg_write(gp_adpd_data, RegAddrLedDrvA, (reg_val & 0xfff0) | (CurrLedCoarseA-1));
					NoSkipSamples = 0;					
					ADPD143_dbg("------------------------stepAgcLedA MwErrProcessing[End]");
					return MwErrProcessing;		
				}
				else
				{
					if( CurrTIAgainA < minTiaGainRegValA )
					{
						reg_val = reg_read(gp_adpd_data, 0x42);
						reg_write(gp_adpd_data, 0x42, (reg_val & 0xfffc) | (CurrTIAgainA+1));
						NoSkipSamples = 0;					
					}
					NoSkipSamples = 0;					
					ADPD143_dbg("------------------------stepAgcLedA MwErrLedAOutofRange4[End]");
					return MwErrLedAOutofRange;
				}
			}
		}
		else
		{
			NoSkipSamples = 0;					
			ADPD143_dbg("------------------------stepAgcLedA MwErrPass!!!!![End]");
			return MwErrPass;
		}
	}
	else//toInc
	{
		if( diff >= raw_max>>1 )
		{ 
			if( !prefMinPower )// min TIA gain, max LED current
			{
				if( CurrLedCoarseA < maxLedCurrentRegValA )
				{
					currLedCoarseCurrent = CurrLedCoarseA*15+25;
					tgtLedCoarseCurrent = currLedCoarseCurrent<<1;
					if( tgtLedCoarseCurrent > maxLEDcurrDrvA )
						tgtLedCoarseCurrent = maxLEDcurrDrvA;
					reg_val = reg_read(gp_adpd_data, RegAddrLedDrvA);
					reg_write(gp_adpd_data, RegAddrLedDrvA, (reg_val & 0xfff0) | ((tgtLedCoarseCurrent-25)/15));
					NoSkipSamples = 0;					
					ADPD143_dbg("------------------------stepAgcLedA MwErrProcessing[End]");
					return MwErrProcessing;
				}
				else
				{
					if( CurrTIAgainA > maxTiaGainRegValA )
					{
						reg_val = reg_read(gp_adpd_data, 0x42);
						reg_write(gp_adpd_data, 0x42, (reg_val & 0xfffc) | (CurrTIAgainA-1));
						NoSkipSamples = 0;						
						ADPD143_dbg("------------------------stepAgcLedA MwErrProcessing[End]");
						return MwErrProcessing;
					}
					else{
						NoSkipSamples = 0;					
						ADPD143_dbg("------------------------stepAgcLedA MwErrLedAOutofRange5[End]");						
						return MwErrLedAOutofRange;
					}
				}
			}	
			else// max TIA gain, min LED current
			{
				if( CurrTIAgainA > maxTiaGainRegValA )
				{
					reg_val = reg_read(gp_adpd_data, 0x42);
					reg_write(gp_adpd_data, 0x42, (reg_val & 0xfffc) | (CurrTIAgainA-1));
					NoSkipSamples = 0;					
					ADPD143_dbg("------------------------stepAgcLedA MwErrProcessing[End]");
					return MwErrProcessing;
				}
				else
				{
					if( CurrLedCoarseA < maxLedCurrentRegValA-5 )
					{
						reg_val = reg_read(gp_adpd_data, RegAddrLedDrvA);
						reg_write(gp_adpd_data, RegAddrLedDrvA, (reg_val & 0xfff0) | (CurrLedCoarseA+2));
						NoSkipSamples = 0;
						ADPD143_dbg("------------------------stepAgcLedA MwErrProcessing[End]");						
						return MwErrProcessing; 	
					}
					else{
						NoSkipSamples = 0;					
						ADPD143_dbg("------------------------stepAgcLedA MwErrLedAOutofRange6[End]");						
						return MwErrLedAOutofRange;
					}
				}
			}
		}
		else if( diff > TargetDcLevelA*AGC_STOP_MARGIN_PERCENTAGE_A/100 )
		{
			if( !prefMinPower )// min TIA gain, max LED current
			{
				if( CurrLedCoarseA < maxLedCurrentRegValA )
				{
					reg_val = reg_read(gp_adpd_data, RegAddrLedDrvA);
					reg_write(gp_adpd_data, RegAddrLedDrvA, (reg_val & 0xfff0) | (CurrLedCoarseA+1));
					NoSkipSamples = 0;
					ADPD143_dbg("------------------------stepAgcLedA MwErrProcessing[End]");					
					return MwErrProcessing;		
				}
				else
				{
					NoSkipSamples = 0;					
					ADPD143_dbg("------------------------stepAgcLedA MwErrLedAOutofRange7[End]");						
					return MwErrLedAOutofRange;				
				}
			}
			else// max TIA gain, min LED current
			{
#if 0//if want to smaller code gap from target			
				if( CurrTIAgainA > maxTiaGainRegValA && CurrLedCoarseA > minLedCurrentRegValA )
				{
					currLedCoarseCurrent = (CurrLedCoarseA-1)*15+25;

					tgtLedCoarseCurrent = currLedCoarseCurrent>>1;
					if( tgtLedCoarseCurrent < minLEDcurrDrvA )
					{
						ADPD143_dbg("------------------------stepAgcLedA MwErrLedAOutofRange8[End]");						
						return MwErrLedAOutofRange; 			
					}
					if( tgtLedCoarseCurrent >= 40 )  tgtLedCoarseCurrent -= 15;  
					reg_val = reg_read(gp_adpd_data, RegAddrLedDrvA);
					reg_write(gp_adpd_data, RegAddrLedDrvA, (reg_val & 0xfff0) | ((tgtLedCoarseCurrent-25)/15));
					
					reg_val = reg_read(gp_adpd_data, 0x42);
					reg_write(gp_adpd_data, 0x42, (reg_val & 0xfffc) | (CurrTIAgainA-1));
					NoSkipSamples = 0;					
					ADPD143_dbg("------------------------stepAgcLedA MwErrLedAOutofRange9[End]");						
					return MwErrLedAOutofRange; 			
				}
				else
#endif
				{
					if( CurrLedCoarseA < maxLedCurrentRegValA )
					{
						reg_val = reg_read(gp_adpd_data, RegAddrLedDrvA);
						reg_write(gp_adpd_data, RegAddrLedDrvA, (reg_val & 0xfff0) | (CurrLedCoarseA+1));
						NoSkipSamples = 0;
						ADPD143_dbg("------------------------stepAgcLedA MwErrProcessing[End]");					
						return MwErrProcessing; 	
					}
					else
					{
						NoSkipSamples = 0;					
						ADPD143_dbg("------------------------stepAgcLedA MwErrLedAOutofRange10[End]");						
						return MwErrLedAOutofRange; 			
					}
				}
			}
		}
		else
		{
			NoSkipSamples = 0;					
			ADPD143_dbg("------------------------stepAgcLedA MwErrPass!!!!![End]");
			return MwErrPass;
		}
	}	
}


static AGC_ErrCode_t stepAgcLedB(unsigned *rawData){
    unsigned raw_max, diff;
    unsigned char toInc = 0;
	unsigned reg_val;
	unsigned short currLedCoarseCurrent;
	unsigned short tgtLedCoarseCurrent;	

    if (RegAddrLedDrvB != 0){
		CurrLedDrvValB = reg_read(gp_adpd_data, RegAddrLedDrvB);
		CurrLedScaleB = (CurrLedDrvValB & 0x2000)>>12;
		CurrLedCoarseB = CurrLedDrvValB & 0x000f;
    }
	reg_val = reg_read(gp_adpd_data, 0x44);
	CurrTIAgainB = reg_val & 0x3; 
	CurrAFEgainB = (reg_val & 0x0300)>>8;	

	reg_val = reg_read(gp_adpd_data, PULSE_PERIOD_A);
    pulseA=(reg_val>>8)&0xFF;	
	reg_val = reg_read(gp_adpd_data, PULSE_PERIOD_B);
    pulseB=(reg_val>>8)&0xFF;

	
    raw_max =  maxValue(rawData, 4);

	ADPD143_dbg("------------------------stepAgcLedB [start : %u]", cntStepAgcB++);
	ADPD143_dbg("maxLedCurrentRegValB : %u minLedCurrentRegValB : %u maxTiaGainRegValB : %u minTiaGainRegValB : %u", 
				maxLedCurrentRegValB, minLedCurrentRegValB, maxTiaGainRegValB, minTiaGainRegValB);
	ADPD143_dbg("LED PulseA = %d PulseB = %d", pulseA, pulseB);
	ADPD143_dbg("current sample : %d target : %d ",raw_max,TargetDcLevelB);
	
    if (raw_max > TargetDcLevelB) {
        diff = raw_max - TargetDcLevelB;
        toInc = 0;
    } else {
        diff = TargetDcLevelB - raw_max;
        toInc = 1;
    }
	ADPD143_dbg("CurrLedScaleB : %d,  CurrLedCoarseB : %d, CurrTIAgainB : %d, CurrAFEgainB : %d ",
		  		CurrLedScaleB, CurrLedCoarseB, CurrTIAgainB, CurrAFEgainB);
	ADPD143_dbg("%s - PrevLedScaleB : %d,  PrevLedCoarseB : %d, PrevTIAgainB : %d, PrevAFEgainB : %d\n",
		__func__, PrevLedScaleB, PrevLedCoarseB, PrevTIAgainB, PrevAFEgainB);
	ADPD143_dbg("current diff : %d,  prev diff : %d,  toInc : %d ", diff, prevStepDiffAgcB, toInc);

#if 1
	if( prevStepDiffAgcB < diff )
	{
#if 1
		if( (PrevTIAgainB - CurrTIAgainB) == 1 || (CurrTIAgainB - PrevTIAgainB) == 1 )
			reg_write(gp_adpd_data, 0x44, (reg_val & 0xfffc) | PrevTIAgainB);
		else if( CurrTIAgainB > PrevTIAgainB )
			reg_write(gp_adpd_data, RegAddrLedDrvB, (reg_val & 0xfff0) | (CurrTIAgainB-1));
		else if( CurrTIAgainB < PrevTIAgainB )
			reg_write(gp_adpd_data, RegAddrLedDrvB, (reg_val & 0xfff0) | (CurrTIAgainB+1));
		
		if( (PrevLedCoarseB - CurrLedCoarseB) == 1 || (CurrLedCoarseB - PrevLedCoarseB) == 1 )
			reg_write(gp_adpd_data, RegAddrLedDrvB, (reg_val & 0xfff0) | PrevLedCoarseB);
		else if( CurrLedCoarseB > PrevLedCoarseB )
			reg_write(gp_adpd_data, RegAddrLedDrvB, (reg_val & 0xfff0) | (CurrLedCoarseB-1));
		else if( CurrLedCoarseB < PrevLedCoarseB )
			reg_write(gp_adpd_data, RegAddrLedDrvB, (reg_val & 0xfff0) | (CurrLedCoarseB+1));
#else	
		reg_write(gp_adpd_data, 0x44, (reg_val & 0xfffc) | PrevTIAgainB );
		reg_write(gp_adpd_data, RegAddrLedDrvB, (reg_val & 0xfff0) | PrevLedCoarseB );	
#endif
		ADPD143_dbg("------------------------stepAgcLedB stopped at best!!!!![End]");
		NoSkipSamples = 0;
		return MwErrLedAOutofRange;
	}
#endif
	PrevLedScaleB = CurrLedScaleB; PrevLedCoarseB = CurrLedCoarseB; PrevTIAgainB = CurrTIAgainB; PrevAFEgainB = CurrAFEgainB;
	prevStepDiffAgcB = diff;
	

	if( !toInc )
	{
		if( diff >= raw_max>>1 )
		{ 

			if( !prefMinPower )// min TIA gain, max LED current
			{
				if( CurrTIAgainB < minTiaGainRegValB )
				{
					reg_val = reg_read(gp_adpd_data, 0x44);
					reg_write(gp_adpd_data, 0x44, (reg_val & 0xfffc) | (CurrTIAgainB+1));
					NoSkipSamples = 0;
					ADPD143_dbg("------------------------stepAgcLedB MwErrProcessing[End]");
					return MwErrProcessing;
				}
				else
				{
					if( CurrLedCoarseB > minLedCurrentRegValB )
					{
						currLedCoarseCurrent = CurrLedCoarseB*15+25;
						tgtLedCoarseCurrent = currLedCoarseCurrent>>1;
						if( tgtLedCoarseCurrent < minLEDcurrDrvB )
							tgtLedCoarseCurrent = minLEDcurrDrvB;
						reg_val = reg_read(gp_adpd_data, RegAddrLedDrvB);
						reg_write(gp_adpd_data, RegAddrLedDrvB, (reg_val & 0xfff0) | ((tgtLedCoarseCurrent-25)/15));
						NoSkipSamples = 0;
						ADPD143_dbg("------------------------stepAgcLedB MwErrProcessing[End]");
						return MwErrProcessing; 	
					}
					else{
						NoSkipSamples = 0;					
						ADPD143_dbg("------------------------stepAgcLedB MwErrLedAOutofRange1[End]");						
						return MwErrLedAOutofRange;
					}
				}
			}
			else// max TIA gain, min LED current
			{
				if( CurrLedCoarseB > minLedCurrentRegValB )
				{
					currLedCoarseCurrent = CurrLedCoarseB*15+25;
					tgtLedCoarseCurrent = currLedCoarseCurrent>>1;
					if( tgtLedCoarseCurrent < minLEDcurrDrvB )
						tgtLedCoarseCurrent = minLEDcurrDrvB;
					reg_val = reg_read(gp_adpd_data, RegAddrLedDrvB);
					reg_write(gp_adpd_data, RegAddrLedDrvB, (reg_val & 0xfff0) | ((tgtLedCoarseCurrent-25)/15));
					NoSkipSamples = 0;
					ADPD143_dbg("------------------------stepAgcLedB MwErrProcessing[End]");
					return MwErrProcessing;
				}
				else
				{
					if( CurrTIAgainB < minTiaGainRegValB )
					{
						reg_val = reg_read(gp_adpd_data, 0x44);
						reg_write(gp_adpd_data, 0x44, (reg_val & 0xfffc) | (CurrTIAgainB+1));
						NoSkipSamples = 0;
						ADPD143_dbg("------------------------stepAgcLedB MwErrProcessing[End]");
						return MwErrProcessing;
					}
					else{
						NoSkipSamples = 0;					
						ADPD143_dbg("------------------------stepAgcLedB MwErrLedAOutofRange2[End]");						
						return MwErrLedAOutofRange;
					}
				}	
			}				
		}
		
		else if( diff > TargetDcLevelB*AGC_STOP_MARGIN_PERCENTAGE_B/100 )
		{
			if( !prefMinPower )// min TIA gain, max LED current
			{
#if 0//if want to smaller code gap from target			
				if( CurrTIAgainB < minTiaGainRegValB && CurrLedCoarseB < maxLedCurrentRegValB )//return to smaller TIA gain@max LED current
				{
					currLedCoarseCurrent = (CurrLedCoarseB+1)*15+25;
					tgtLedCoarseCurrent = currLedCoarseCurrent<<1;
					if( tgtLedCoarseCurrent > maxLEDcurrDrvB )
						tgtLedCoarseCurrent = maxLEDcurrDrvB;
					reg_val = reg_read(gp_adpd_data, RegAddrLedDrvB);
					reg_write(gp_adpd_data, RegAddrLedDrvB, (reg_val & 0xfff0) | ((tgtLedCoarseCurrent-25)/15));
					reg_val = reg_read(gp_adpd_data, 0x44);
					reg_write(gp_adpd_data, 0x44, (reg_val & 0xfffc) | (CurrTIAgainB+1));
					NoSkipSamples = 0;
				
					ADPD143_dbg("------------------------stepAgcLedB MwErrLedAOutofRange[End]");
					return MwErrLedAOutofRange;
				}
				else
#endif
				{
					if( CurrLedCoarseB > minLedCurrentRegValB )
					{
						reg_val = reg_read(gp_adpd_data, RegAddrLedDrvB);
						reg_write(gp_adpd_data, RegAddrLedDrvB, (reg_val & 0xfff0) | (CurrLedCoarseB-1));
						NoSkipSamples = 0;
						ADPD143_dbg("------------------------stepAgcLedB MwErrProcessing[End]");						
						return MwErrProcessing; 	
					}
					else{
						NoSkipSamples = 0;					
						ADPD143_dbg("------------------------stepAgcLedB MwErrLedAOutofRange3[End]");						
						return MwErrLedAOutofRange;
					}
				}
			}
			else// max TIA gain, min LED current
			{
		
				if( CurrLedCoarseB > minLedCurrentRegValB )
				{
					reg_val = reg_read(gp_adpd_data, RegAddrLedDrvB);
					reg_write(gp_adpd_data, RegAddrLedDrvB, (reg_val & 0xfff0) | (CurrLedCoarseB-1));
					NoSkipSamples = 0;
					ADPD143_dbg("------------------------stepAgcLedB MwErrProcessing[End]");
					return MwErrProcessing; 	
				}
				else
				{
					if( CurrTIAgainB < minTiaGainRegValB )
					{
						reg_val = reg_read(gp_adpd_data, 0x44);
						reg_write(gp_adpd_data, 0x44, (reg_val & 0xfffc) | (CurrTIAgainB+1));
						NoSkipSamples = 0;
					}
					NoSkipSamples = 0;					
					ADPD143_dbg("------------------------stepAgcLedB MwErrLedAOutofRange4[End]");
					return MwErrLedAOutofRange;
				}
			}
		}
		else
		{
			NoSkipSamples = 0;					
			ADPD143_dbg("------------------------stepAgcLedB MwErrPass[End]");
			return MwErrPass;
		}
	}
	else//toInc
	{
		if( diff >= raw_max>>1 )
		{ 
			if( !prefMinPower )// min TIA gain, max LED current
			{
				if( CurrLedCoarseB < maxLedCurrentRegValB )
				{
					currLedCoarseCurrent = CurrLedCoarseB*15+25;
					tgtLedCoarseCurrent = currLedCoarseCurrent<<1;
					if( tgtLedCoarseCurrent > maxLEDcurrDrvB )
						tgtLedCoarseCurrent = maxLEDcurrDrvB;
					reg_val = reg_read(gp_adpd_data, RegAddrLedDrvB);
					reg_write(gp_adpd_data, RegAddrLedDrvB, (reg_val & 0xfff0) | ((tgtLedCoarseCurrent-25)/15));
					NoSkipSamples = 0;
					ADPD143_dbg("------------------------stepAgcLedB MwErrProcessing5[End]");
					return MwErrProcessing;
				}
				else
				{
					if( CurrTIAgainB > maxTiaGainRegValB )
					{
						reg_val = reg_read(gp_adpd_data, 0x44);
						reg_write(gp_adpd_data, 0x44, (reg_val & 0xfffc) | (CurrTIAgainB-1));
						NoSkipSamples = 0;
						ADPD143_dbg("------------------------stepAgcLedB MwErrProcessing[End]");
						return MwErrProcessing;
					}
					else{
						NoSkipSamples = 0;					
						ADPD143_dbg("------------------------stepAgcLedB MwErrLedAOutofRange6[End]");						
						return MwErrLedAOutofRange;				
					}
				}
			}	
			else// max TIA gain, min LED current
			{
				if( CurrTIAgainB > maxTiaGainRegValB )
				{
					reg_val = reg_read(gp_adpd_data, 0x44);
					reg_write(gp_adpd_data, 0x44, (reg_val & 0xfffc) | (CurrTIAgainB-1));
					NoSkipSamples = 0;
					ADPD143_dbg("------------------------stepAgcLedB MwErrProcessing[End]");
					return MwErrProcessing;
				}
				else
					{
					if( CurrLedCoarseB < maxLedCurrentRegValB-5 )
					{
						reg_val = reg_read(gp_adpd_data, RegAddrLedDrvB);
						reg_write(gp_adpd_data, RegAddrLedDrvB, (reg_val & 0xfff0) | (CurrLedCoarseB+2));
						NoSkipSamples = 0;
						ADPD143_dbg("------------------------stepAgcLedB MwErrProcessing[End]");
						return MwErrProcessing; 	
					}
					else{
						NoSkipSamples = 0;					
						ADPD143_dbg("------------------------stepAgcLedB MwErrLedAOutofRange7[End]");						
						return MwErrLedAOutofRange;
					}
				}
			}
		}
		else if( diff > TargetDcLevelB*AGC_STOP_MARGIN_PERCENTAGE_B/100 )
		{
		
			if( !prefMinPower )// min TIA gain, max LED current
			{
				if( CurrLedCoarseB < maxLedCurrentRegValB )
				{
					reg_val = reg_read(gp_adpd_data, RegAddrLedDrvB);
					reg_write(gp_adpd_data, RegAddrLedDrvB, (reg_val & 0xfff0) | (CurrLedCoarseB+1));
					NoSkipSamples = 0;
					ADPD143_dbg("------------------------stepAgcLedB MwErrProcessing[End]");
					return MwErrProcessing;		
				}
				else
				{
					NoSkipSamples = 0;					
					ADPD143_dbg("------------------------stepAgcLedA MwErrLedAOutofRange8[End]");						
					return MwErrLedAOutofRange;								
				}
			}
			else// max TIA gain, min LED current
			{
#if 0//if want to smaller code gap from target			
				if( CurrTIAgainB > maxTiaGainRegValB && CurrLedCoarseB > minLedCurrentRegValB )
				{
					currLedCoarseCurrent = (CurrLedCoarseB-1)*15+25;

					tgtLedCoarseCurrent = currLedCoarseCurrent>>1;
					if( tgtLedCoarseCurrent < minLEDcurrDrvB )
					{
						ADPD143_dbg("------------------------stepAgcLedB MwErrLedAOutofRange[End]");						
						return MwErrLedAOutofRange; 			
					}
					if( tgtLedCoarseCurrent >= 40 )  tgtLedCoarseCurrent -= 15;  
					reg_val = reg_read(gp_adpd_data, RegAddrLedDrvB);
					reg_write(gp_adpd_data, RegAddrLedDrvB, (reg_val & 0xfff0) | ((tgtLedCoarseCurrent-25)/15));
					
					reg_val = reg_read(gp_adpd_data, 0x44);
					reg_write(gp_adpd_data, 0x44, (reg_val & 0xfffc) | (CurrTIAgainB-1));
					NoSkipSamples = 0;					
					ADPD143_dbg("------------------------stepAgcLedB MwErrLedAOutofRange[End]");						
					return MwErrLedAOutofRange; 			
				}
				else
#endif
				{
					if( CurrLedCoarseB < maxLedCurrentRegValB )
					{
						reg_val = reg_read(gp_adpd_data, RegAddrLedDrvB);
						reg_write(gp_adpd_data, RegAddrLedDrvB, (reg_val & 0xfff0) | (CurrLedCoarseB+1));
						NoSkipSamples = 0;
						ADPD143_dbg("------------------------stepAgcLedB MwErrProcessing[End]");					
						return MwErrProcessing; 	
					}
					else
					{
						NoSkipSamples = 0;					
						ADPD143_dbg("------------------------stepAgcLedB MwErrLedAOutofRange9[End]");						
						return MwErrLedAOutofRange; 			
					}
				}
			}
		}
		else
		{
			NoSkipSamples = 0;					
			ADPD143_dbg("------------------------stepAgcLedB MwErrPass[End]");
			return MwErrPass;
		}
	}	
}


static unsigned maxValue(unsigned *testData, unsigned short size) 
{
    unsigned tempData = *testData;
    for (;size>1; size--) {
        testData++;
        if (tempData<*testData)
            tempData = *testData;
    }
    return tempData;
}

#endif

#ifdef __PROXIMITY_SUPPORT__
void setObjProximityThreshold(unsigned thresh)
{
	gp_adpd_data->obj_proximity_theshld = thresh;
}

OBJ_PROXIMITY_STATUS getObjProximity(void)
{
	return gp_adpd_data->st_obj_proximity;
}

static void
adpd143_mode_switching(struct adpd143_data *pst_adpd, u32 usr_mode);

OBJ_PROXIMITY_STATUS checkObjProximity(unsigned* dataInA, unsigned* dataIn)
{
	unsigned dataInMax = 0; 
	unsigned dataInAMax = 0;
	if( !dataIn ) return gp_adpd_data->st_obj_proximity;
	dataInMax =  maxValue(dataIn, 4);
	if( dataInA ) dataInAMax = maxValue(dataInA, 4);

	switch( gp_adpd_data->st_obj_proximity )
	{
		case OBJ_DETACHED:
			if( dataIn[0]+dataIn[1]+dataIn[2]+dataIn[3] > gp_adpd_data->obj_proximity_theshld )
			{
				ADPD143_dbg("******************checkObjProximity() : OBJ_DETECTED ( %u %u )*****************\n", 
					dataIn[0]+dataIn[1]+dataIn[2]+dataIn[3], gp_adpd_data->obj_proximity_theshld);			
				gp_adpd_data->st_obj_proximity = OBJ_DETECTED;
				gp_adpd_data->cnt_proximity_on = 0;
			}
			else
			{
				ADPD143_dbg("******************checkObjProximity() : OBJ_DETACHED ( %u %u )*****************\n", 
					dataIn[0]+dataIn[1]+dataIn[2]+dataIn[3], gp_adpd_data->obj_proximity_theshld);			
			}
			break;
		case OBJ_DETECTED:
			ADPD143_dbg("******************checkObjProximity() : OBJ_DETECTED ( %u %u )*****************\n", 
				dataIn[0]+dataIn[1]+dataIn[2]+dataIn[3], gp_adpd_data->obj_proximity_theshld);			
			if( dataIn[0]+dataIn[1]+dataIn[2]+dataIn[3] < gp_adpd_data->obj_proximity_theshld*OBJ_DETACH_FROM_DETECTION_DC_PERCENT/100 )
				gp_adpd_data->st_obj_proximity = OBJ_DETACHED;
			else if( dataIn[0]+dataIn[1]+dataIn[2]+dataIn[3] < gp_adpd_data->obj_proximity_theshld*OBJ_DETECTION_CNT_DC_PERCENT/100 )
				gp_adpd_data->cnt_proximity_on = (gp_adpd_data->cnt_proximity_on > 0)?gp_adpd_data->cnt_proximity_on-1:0;
			else
				gp_adpd_data->cnt_proximity_on = (gp_adpd_data->cnt_proximity_on > 100)?100:gp_adpd_data->cnt_proximity_on+1;
				//gp_adpd_data->cnt_proximity_on++;
			if( gp_adpd_data->cnt_proximity_on >= CNT_THRESHOLD_OBJ_PROXIMITY_ON )
			{
				ADPD143_dbg("******************checkObjProximity() : OBJ_PROXIMITY_RED_ON ( %u )*****************\n", 
							gp_adpd_data->cnt_proximity_on);
				gp_adpd_data->st_obj_proximity = OBJ_PROXIMITY_RED_ON;
				gp_adpd_data->cnt_dc_saturation = 0;
				gp_adpd_data->cnt_proximity_on = 0;
				gp_adpd_data->cur_proximity_dc_level = 0;
#if 0			
				mutex_unlock(&gp_adpd_data->mutex);
				adpd143_mode_switching(gp_adpd_data, 0x31);
				mutex_lock(&gp_adpd_data->mutex);				
#else
				reg_write(gp_adpd_data, 0x10, 0x1);
				atomic_set(&gp_adpd_data->adpd_mode, 0x31);
				reg_write(gp_adpd_data, 0x1, 0x1ff);
				reg_write(gp_adpd_data, 0x0, 0x80ff);
				reg_write(gp_adpd_data, 0x1, ~(0x0040) & 0x1ff);
				//reg_write(gp_adpd_data, 0x1, 0x0ff);
				reg_write(gp_adpd_data, 0x11, 0x1131);
				reg_write(gp_adpd_data, 0x34, 0x0);
				reg_write(gp_adpd_data, 0x10, 0x2);
				if( gp_adpd_data->bOnOffAGC ) initAGC(0);
#endif
			}
			break;
		case OBJ_PROXIMITY_RED_ON:
			if( gp_adpd_data->cur_proximity_dc_level == 0 )
				gp_adpd_data->cur_proximity_dc_level = dataIn[0]+dataIn[1]+dataIn[2]+dataIn[3];
			
			if( dataIn[0]+dataIn[1]+dataIn[2]+dataIn[3] < gp_adpd_data->cur_proximity_dc_level*OBJ_DETACH_FROM_PROXIMITY_DC_PERCENT/100 )
			{
				ADPD143_dbg("******************checkObjProximity() : OBJ_PROXIMITY_RED_ON ( %u < %u )*****************\n", 
					dataIn[0]+dataIn[1]+dataIn[2]+dataIn[3], gp_adpd_data->cur_proximity_dc_level*OBJ_DETACH_FROM_PROXIMITY_DC_PERCENT/100 );			
				gp_adpd_data->st_obj_proximity = OBJ_DETACHED;
				gp_adpd_data->cnt_dc_saturation = 0;				
#if 0				
				mutex_unlock(&gp_adpd_data->mutex);
				adpd143_mode_switching(gp_adpd_data, 0x32);
				mutex_lock(&gp_adpd_data->mutex);				
#else
				atomic_set(&gp_adpd_data->adpd_mode, 0x32);
				reg_write(gp_adpd_data, 0x10, 0x1);
				reg_write(gp_adpd_data, 0x1, 0x1ff);
				reg_write(gp_adpd_data, 0x0, 0x80ff);
				//reg_write(gp_adpd_data, 0x1, 0x0ff);
				reg_write(gp_adpd_data, 0x1, ~(0x0040) & 0x1ff);
				reg_write(gp_adpd_data, 0x24, 0x3075);
				reg_write(gp_adpd_data, 0x11, 0x1130);
				reg_write(gp_adpd_data, 0x34, 0x0100);
				reg_write(gp_adpd_data, 0x10, 0x2);				
#endif
				break;
			}
			else if( dataIn[0]+dataIn[1]+dataIn[2]+dataIn[3] < gp_adpd_data->cur_proximity_dc_level*OBJ_PROXIMITY_CNT_DC_PERCENT/100 )
			{
				ADPD143_dbg("******************checkObjProximity() : OBJ_PROXIMITY_RED_ON( %u < %u )*****************\n", 
					dataIn[0]+dataIn[1]+dataIn[2]+dataIn[3], gp_adpd_data->cur_proximity_dc_level ); 
				gp_adpd_data->cnt_dc_saturation = 0;
				gp_adpd_data->cnt_proximity_on = (gp_adpd_data->cnt_proximity_on > 0)?gp_adpd_data->cnt_proximity_on-1:0;
			}
			else{
				ADPD143_dbg("******************checkObjProximity() : OBJ_PROXIMITY_RED_ON ( %u %u )*****************\n", 
					dataIn[0]+dataIn[1]+dataIn[2]+dataIn[3], gp_adpd_data->obj_proximity_theshld);
				//gp_adpd_data->cnt_proximity_on++;
				gp_adpd_data->cnt_proximity_on = (gp_adpd_data->cnt_proximity_on > 100)?100:gp_adpd_data->cnt_proximity_on+1;
			}
			if( gp_adpd_data->bOnOffAGC && getAGCstate() != AgcStage_Done )
			{
				stepAGC( dataInA, dataIn );
				ADPD143_dbg("******************checkObjProximity() : OBJ_PROXIMITY_RED_ON - in AGC*****************\n");
				reg_write(gp_adpd_data, 0x0, 0x80ff);
				break;
			}
			
			if( gp_adpd_data->cnt_proximity_on >= CNT_THRESHOLD_OBJ_PROXIMITY_ON )
			{
				ADPD143_dbg("******************checkObjProximity() : OBJ_PROXIMITY_RED_ON ( %u )*****************\n", 
							gp_adpd_data->cnt_proximity_on);
				gp_adpd_data->st_obj_proximity = OBJ_PROXIMITY_ON;
				gp_adpd_data->cnt_proximity_on = 0;
				gp_adpd_data->cur_proximity_dc_level = dataIn[0]+dataIn[1]+dataIn[2]+dataIn[3];
			}
			break;
		case OBJ_PROXIMITY_ON:
			if( dataIn[0]+dataIn[1]+dataIn[2]+dataIn[3] < gp_adpd_data->cur_proximity_dc_level*OBJ_DETACH_FROM_PROXIMITY_DC_PERCENT/100 )
			{
				ADPD143_dbg("******************checkObjProximity() : OBJ_DETACHED ( %u < %u )*****************\n", 
					dataIn[0]+dataIn[1]+dataIn[2]+dataIn[3], gp_adpd_data->cur_proximity_dc_level*OBJ_DETACH_FROM_PROXIMITY_DC_PERCENT/100 );			
				gp_adpd_data->st_obj_proximity = OBJ_DETACHED;
				gp_adpd_data->cnt_dc_saturation = 0;				
#if 0				
				mutex_unlock(&gp_adpd_data->mutex);
				adpd143_mode_switching(gp_adpd_data, 0x32);
				mutex_lock(&gp_adpd_data->mutex);				
#else
				atomic_set(&gp_adpd_data->adpd_mode, 0x32);
				reg_write(gp_adpd_data, 0x10, 0x1);
				reg_write(gp_adpd_data, 0x1, 0x1ff);
				reg_write(gp_adpd_data, 0x0, 0x80ff);
				//reg_write(gp_adpd_data, 0x1, 0x0ff);
				reg_write(gp_adpd_data, 0x1, ~(0x0040) & 0x1ff);
				reg_write(gp_adpd_data, 0x11, 0x1130);
				reg_write(gp_adpd_data, 0x24, 0x3075);
				reg_write(gp_adpd_data, 0x34, 0x0100);
				reg_write(gp_adpd_data, 0x10, 0x2);				
#endif
				break;
			}
			else if( dataIn[0]+dataIn[1]+dataIn[2]+dataIn[3] < gp_adpd_data->cur_proximity_dc_level*OBJ_PROXIMITY_CNT_DC_PERCENT/100 )
			{
				ADPD143_dbg("******************checkObjProximity() : OBJ_PROXIMITY_ON( %u < %u )*****************\n", 
					dataIn[0]+dataIn[1]+dataIn[2]+dataIn[3], gp_adpd_data->cur_proximity_dc_level );	
				gp_adpd_data->cnt_dc_saturation = 0;
				gp_adpd_data->cnt_proximity_on = (gp_adpd_data->cnt_proximity_on > 0)?gp_adpd_data->cnt_proximity_on-1:0;
			}
			else{
				ADPD143_dbg("******************checkObjProximity() : OBJ_PROXIMITY_ON ( %u %u )*****************\n", 
					dataIn[0]+dataIn[1]+dataIn[2]+dataIn[3], gp_adpd_data->cur_proximity_dc_level);
				//gp_adpd_data->cnt_proximity_on++;				
				gp_adpd_data->cnt_proximity_on = (gp_adpd_data->cnt_proximity_on > 100)?100:gp_adpd_data->cnt_proximity_on+1;
			}
			if( dataInMax >= MAX_CODE_AFTER_ADC_OFFSET_PER_PULSE*pulseB )
			{
				//gp_adpd_data->cnt_dc_saturation++;
				gp_adpd_data->cnt_dc_saturation = (gp_adpd_data->cnt_dc_saturation > 100)?100:gp_adpd_data->cnt_dc_saturation+1;//<--here	
				if( gp_adpd_data->cnt_dc_saturation > CNT_THRESHOLD_DC_SATURATION )
				{
					ADPD143_dbg("******************checkObjProximity() : initAGC (ChB %u > %u )*****************\n", 
						dataInMax, MAX_CODE_AFTER_ADC_OFFSET_PER_PULSE*pulseB );
					if( gp_adpd_data->bOnOffAGC )
					{ 
						initAGC(0);
						gp_adpd_data->st_obj_proximity = OBJ_DETACHED;//OBJ_PROXIMITY_RED_ON;
					}
					gp_adpd_data->cnt_dc_saturation = 0;
				}
				break;
			}
			else
				gp_adpd_data->cnt_dc_saturation = (gp_adpd_data->cnt_dc_saturation > 0)?gp_adpd_data->cnt_dc_saturation-1:0;//<--here 
				
			if( gp_adpd_data->cnt_proximity_on == 0 )
			{
				ADPD143_dbg("******************checkObjProximity() : OBJ_DETACHED ( gp_adpd_data->cnt_proximity_on = %u )*****************\n", 
					gp_adpd_data->cnt_proximity_on );			
				gp_adpd_data->st_obj_proximity = OBJ_DETACHED;
				gp_adpd_data->cnt_dc_saturation = 0;				
#if 0				
				mutex_unlock(&gp_adpd_data->mutex);
				adpd143_mode_switching(gp_adpd_data, 0x32);
				mutex_lock(&gp_adpd_data->mutex);				
#else
				atomic_set(&gp_adpd_data->adpd_mode, 0x32);
				reg_write(gp_adpd_data, 0x10, 0x1);
				reg_write(gp_adpd_data, 0x1, 0x1ff);
				reg_write(gp_adpd_data, 0x0, 0x80ff);
				//reg_write(gp_adpd_data, 0x1, 0x0ff);
				reg_write(gp_adpd_data, 0x1, ~(0x0040) & 0x1ff);
				reg_write(gp_adpd_data, 0x11, 0x1130);
				reg_write(gp_adpd_data, 0x24, 0x3075);
				reg_write(gp_adpd_data, 0x34, 0x0100);
				reg_write(gp_adpd_data, 0x10, 0x2);
#endif
			}
			if( dataInAMax >= MAX_CODE_AFTER_ADC_OFFSET_PER_PULSE*pulseA )
			{
				//gp_adpd_data->cnt_dc_saturation++;
				gp_adpd_data->cnt_dc_saturation = (gp_adpd_data->cnt_dc_saturation > 100)?100:gp_adpd_data->cnt_dc_saturation+1;//<--here	
				if( gp_adpd_data->cnt_dc_saturation > CNT_THRESHOLD_DC_SATURATION )
				{
					ADPD143_dbg("******************checkObjProximity() : initAGC (ChA %u > %u )*****************\n", 
						dataInAMax, MAX_CODE_AFTER_ADC_OFFSET_PER_PULSE*pulseA );
					if( gp_adpd_data->bOnOffAGC )
					{
						initAGC(0);
						gp_adpd_data->st_obj_proximity = OBJ_DETACHED;//OBJ_PROXIMITY_RED_ON;					
					}
					gp_adpd_data->cnt_dc_saturation = 0;
				}
				break;
			}
			else
				gp_adpd_data->cnt_dc_saturation = (gp_adpd_data->cnt_dc_saturation > 0)?gp_adpd_data->cnt_dc_saturation-1:0;//<--here 
			break;
		case OBJ_HARD_PRESSURE: 
			break;
		case DEVICE_ON_TABLE: 
			break;
		default:
			break;
	}
	return gp_adpd_data->st_obj_proximity;
}
#endif

/**
This function is used to parse data from the string "0xXXX 1 0xYYY"
@param buf is pointer point constant address
@param cnt store number value need to parse
@param data parse data are stored in it
@return void
*/
static void
cmd_parsing(const char *buf, unsigned short cnt, unsigned short *data)
{

	char **bp = (char **)&buf;
	char *token, minus, parsing_cnt = 0;
	int val;
	int pos;

	while ((token = strsep(bp, " "))) {
		pos = 0;
		minus = false;
		if ((char)token[pos] == '-') {
			minus = true;
			pos++;
		}
		if ((token[pos] == '0') &&
			(token[pos + 1] == 'x' || token[pos + 1] == 'X')) {
			if (kstrtoul(&token[pos + 2], 16,
				(long unsigned int *)&val))
				val = 0;
			if (minus)
				val *= (-1);
		} else {
			if (kstrtoul(&token[pos], 10,
				(long unsigned int *)&val))
				val = 0;
			if (minus)
				val *= (-1);
		}

		if (parsing_cnt < cnt)
			*(data + parsing_cnt) = val;
		else
			break;
		parsing_cnt++;
	}
}


/**
This function is used for I2C read from the sysfs
file system of the ADPD143 Chip
@param pst_adpd the ADPD143 data structure
@return u16 status of i2c read.
*/
static u16
adpd143_sysfs_I2C_read(struct adpd143_data *pst_adpd)
{
	return reg_read(pst_adpd, pst_adpd->sysfs_I2C_regaddr);
}

/**
This function is used for I2C write from the sysfs
file system of the ADPD143 Chip
@param pst_adpd the ADPD143 data structure
@return u16 the bytes of written data.
*/
static u16
adpd143_sysfs_I2C_write(struct adpd143_data *pst_adpd)
{
	u16 err;
	err = reg_write(pst_adpd, pst_adpd->sysfs_I2C_regaddr,
			pst_adpd->sysfs_I2C_regval);
	if (err)
		return 0xFFFF;
	else
		return pst_adpd->sysfs_I2C_regval;
}

/**
This function is used to parse string
@param recv_buf pointer point a string
@return int
*/
static int
parse_data(char *recv_buf)
{
	char **bp = (char **)&recv_buf;
	char *token, parsing_cnt = 0;
	long val;
	int test_data;
	unsigned int data = 0;
	int ret = 0;
	char test[10] = {'\0'};

	while ((token = strsep(bp, " \t"))) {
		memset(test, '\0', 10);
		memcpy(test, token, strlen(token));
		memmove(test+2, test, strlen(test));
		test[0] = '0';
		test[1] = 'x';
		ret = kstrtol(test, 0, &val);
		if (ret) {
			if (ret  == -ERANGE) {
				ADPD143_info("out of range\n");
				val = 0;
			}
			if (ret ==  -EINVAL) {
				ADPD143_info("parsing error\n");
				sscanf(test, "%x", &test_data);
				val = test_data;
			}
		}

		if (parsing_cnt == 0) {
			data = (int) (0xFF & val);
			if (data == 0)
				return -1;
		}
		if (parsing_cnt == 1) {
			data = data << 16;
			data |= (0xFFFF & val);
		}
		parsing_cnt++;
		if (parsing_cnt > 2)
			break;
	}
	return data;
}

/**
This function is used for reading configuration file
@param pst_adpd the ADPD143 data structure
@return int status of the configuration file.
*/
static int
adpd143_read_config_file(struct adpd143_data *pst_adpd)
{
	mm_segment_t old_fs;
	struct file *fpt_adpd = NULL;
	struct adpd_platform_data *ptr_config = NULL;
	int line_cnt = 0;
	int start = 0;
	int cnt = 0;
	int ret = 0;
	int i = 0;
	char get_char;
	char *recieved_buf = NULL;
	loff_t pos = 0;
	ptr_config = pst_adpd->ptr_config;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	fpt_adpd = filp_open("/data/misc/adpd143_config.dcfg", O_RDONLY, 0666);
	if (IS_ERR(fpt_adpd)) {
		ADPD143_dbg("unable to find dcfg file %ld\n", PTR_ERR(fpt_adpd));
		set_fs(old_fs);
		ret = /*PTR_ERR(fpt_adpd_filp); */ -1;
		ptr_config->config_size = 0;
		goto err_filp_open;
	}

	recieved_buf = kzalloc(sizeof(char) * 15, GFP_KERNEL);
	memset(recieved_buf, '\0', sizeof(char) * 15);

	while (vfs_read(fpt_adpd, &get_char, sizeof(char), &pos)) {
		if (get_char == '\n') {
			line_cnt++;
			if (cnt > 1) {
				ret = parse_data(recieved_buf);
				if (ret == -1) {
					ADPD143_dbg("Error in reading dcfg\n");
					break;
				}
				ADPD143_info("0x%08x\n", ret);
				ptr_config->config_data[i] = ret;
				i++;
			}
			memset(recieved_buf, '\0', sizeof(char) * 15);
			start = 0;
			cnt = 0;
			ret = 0;
		} else {
			if (get_char != '#') {
				if (start != 0xF) {
					recieved_buf[cnt] = get_char;
					cnt++;
				}
			} else {
				start = 0xF;
			}
		}
	}

	filp_close(fpt_adpd, NULL);

	set_fs(old_fs);
	kfree(recieved_buf);
	ptr_config->config_size = i;
	return 0;
err_filp_open:
	return -1;
}

/**
This function is used for reading interrupt and FIFO status
@param pst_adpd the ADPD143 data structure
@return unsigned short interrupt status value
*/
static unsigned short
adpd143_rd_intr_fifo(struct adpd143_data *pst_adpd)
{
	unsigned short regval = 0;
	unsigned short fifo_size = 0;
	unsigned short usr_mode = 0;
	regval = reg_read(pst_adpd, ADPD_INT_STATUS_ADDR);
	fifo_size = ((regval & 0xFF00) >> 8);
	pst_adpd->fifo_size = ((fifo_size / 2) + (fifo_size & 1));
	//pst_adpd->fifo_size = fifo_size>>1;
	pst_adpd->intr_status_val = (regval & 0xFF);

	ADPD143_dbg("Intr_status 0x%x, FIFO_SIZE 0x%x, regval = 0x%x\n",
			  pst_adpd->intr_status_val, pst_adpd->fifo_size, regval );

	if (fifo_size <= 16)
		atomic_inc(&pst_adpd->stats.fifo_bytes[0]);
	else if (fifo_size <= 32)
		atomic_inc(&pst_adpd->stats.fifo_bytes[1]);
	else if (fifo_size <= 64)
		atomic_inc(&pst_adpd->stats.fifo_bytes[2]);
	else
		atomic_inc(&pst_adpd->stats.fifo_bytes[3]);

	usr_mode = atomic_read(&pst_adpd->adpd_mode);
	if (0 != usr_mode) {
		unsigned int mask = 0;
		unsigned int mod = 0;

		switch (GET_USR_SUB_MODE(usr_mode)) {
		case S_SAMP_XY_AB:
			mask = 0x07;
			//pst_adpd->fifo_size /= 8;
			break;
		default:
			//pst_adpd->fifo_size /= 4;
			mask = 0x03;
			break;
		};

		mod = pst_adpd->fifo_size & mask;
		if (mod) {
			ADPD143_dbg("Keeping %d samples in FIFO from %d\n",
				     mod, pst_adpd->fifo_size);
			pst_adpd->fifo_size &= ~mask;
			atomic_inc(&pst_adpd->stats.fifo_requires_sync);
		}
	}

	return pst_adpd->intr_status_val;
}

/**
This function is used for clearing the Interrupt as well as FIFO
@param pst_adpd the ADPD143 data structure
@return void
*/
static void
adpd143_clr_intr_fifo(struct adpd143_data *pst_adpd)
{
	/*below code is added for verification */
	unsigned short regval;
	regval = reg_read(pst_adpd, ADPD_INT_STATUS_ADDR);
	ADPD143_info("fifo_size: 0x%04x, Status: 0x%x\n",
			  ((regval & 0xFF00) >> 8), (regval & 0xFF));

	reg_write(pst_adpd, ADPD_INT_STATUS_ADDR, 0x80FF);
	regval = reg_read(pst_adpd, ADPD_INT_STATUS_ADDR);
	ADPD143_info("After clear - fifo_size: 0x%04x, Status: 0x%x\n",
			  ((regval & 0xFF00) >> 8), (regval & 0xFF));
}

/**
This function is used for clearing the Interrupt status
@param pst_adpd the ADPD143 data structure
@param mode operating mode of ADPD143
@return void
*/
static void
adpd143_clr_intr_status(struct adpd143_data *pst_adpd, unsigned short mode)
{
	reg_write(pst_adpd, ADPD_INT_STATUS_ADDR,
		  pst_adpd->intr_status_val);
}

/**
This function is used to read out FIFO data from FIFO
@param pst_adpd the ADPD143 data structure
@return unsigned int FIFO size
*/
static unsigned int
adpd143_rd_fifo_data(struct adpd143_data *pst_adpd)
{
	int loop_cnt = 0;

	if (!pst_adpd->fifo_size)
		goto err_rd_fifo_data;

	if (0 != atomic_read(&pst_adpd->adpd_mode) &&
		pst_adpd->fifo_size & 0x3) {
			pr_err("Unexpected FIFO_SIZE=%d,\
			should be multiple of four (channels)!\n",
			pst_adpd->fifo_size);
	}

	reg_write(pst_adpd, ADPD_TEST_PD_ADDR, EN_FIFO_CLK);
	reg_write(pst_adpd, ADPD_TEST_PD_ADDR, EN_FIFO_CLK);
	multi_reg_read(pst_adpd, ADPD_DATA_BUFFER_ADDR,
			   pst_adpd->fifo_size, pst_adpd->ptr_data_buffer);
	//reg_write(pst_adpd, ADPD_TEST_PD_ADDR, DS_FIFO_CLK);

	for (; loop_cnt < pst_adpd->fifo_size; loop_cnt++) {
		ADPD143_info("[0x%04x] 0x%04x\n", loop_cnt,
				  pst_adpd->ptr_data_buffer[loop_cnt]);
	}
	return pst_adpd->fifo_size;
err_rd_fifo_data:
	return 0;
}

/**
This function is a thing need to configure before changing the register
@param pst_adpd the ADPD143 data structure
@param global_mode OFF, IDLE, GO are the three mode
@return void
*/
static void
adpd143_config_prerequisite(struct adpd143_data *pst_adpd, u32 global_mode)
{
	unsigned short regval = 0;
	regval = reg_read(pst_adpd, ADPD_OP_MODE_ADDR);
	regval = SET_GLOBAL_OP_MODE(regval, global_mode);
	ADPD143_info("reg 0x%04x,\tafter set 0x%04x\n",
			  regval, SET_GLOBAL_OP_MODE(regval, global_mode));
	reg_write(pst_adpd, ADPD_OP_MODE_ADDR, regval);
}

/**
This function is used for switching the adpd143 mode
@param pst_adpd the ADPD143 data structure
@param usr_mode user mode
@return void
*/
static void
adpd143_mode_switching(struct adpd143_data *pst_adpd, u32 usr_mode)
{
	unsigned int opmode_val = 0;
	unsigned short mode_val = 0;
	unsigned short intr_mask_val = 0;
	unsigned short i;

	unsigned short mode = GET_USR_MODE(usr_mode);
	unsigned short sub_mode = GET_USR_SUB_MODE(usr_mode);
	/*disabling further avoid further interrupt to trigger */
	disable_irq(pst_adpd->irq);

	/*stop the pending work */
	/*this function Gurantee that wrk is not pending or executing on CPU */
	cancel_work_sync(&pst_adpd->work);

	atomic_set(&pst_adpd->adpd_mode, 0);
	mutex_lock(&pst_adpd->mutex);

	/*Depending upon the mode get the value need to write Operatin mode*/
	opmode_val = *(__mode_recv_frm_usr[mode].mode_code + (sub_mode));

	adpd143_config_prerequisite(pst_adpd, GLOBAL_OP_MODE_IDLE);

	/*switch to IDLE mode */
	mode_val = DEFAULT_OP_MODE_CFG_VAL(pst_adpd);
	mode_val += SET_MODE_VALUE(IDLE_OFF);
	intr_mask_val = SET_INTR_MASK(IDLE_USR);

	reg_write(pst_adpd, ADPD_OP_MODE_CFG_ADDR, mode_val);
	reg_write(pst_adpd, ADPD_INT_MASK_ADDR, intr_mask_val);

	/*clear FIFO and flush buffer */
	adpd143_clr_intr_fifo(pst_adpd);
	adpd143_rd_intr_fifo(pst_adpd);
	if (pst_adpd->fifo_size != 0) {
		adpd143_clr_intr_status(pst_adpd, IDLE_USR);
		ADPD143_info("Flushing FIFO\n");
		adpd143_rd_fifo_data(pst_adpd);
	}


	adpd143_config_prerequisite(pst_adpd, GLOBAL_OP_MODE_GO);
	msleep(20);
	adpd143_config_prerequisite(pst_adpd, GLOBAL_OP_MODE_IDLE);

	/*Find Interrupt mask value */
	switch (mode) {
	case IDLE_USR:
		ADPD143_info("IDLE MODE\n");
		intr_mask_val = SET_INTR_MASK(IDLE_USR);
		atomic_set(&pst_adpd->sample_enabled, 0);
		break;
	case SAMPLE_USR:
		ADPD143_info("SAMPLE MODE\n");
		/*enable interrupt only when data written to FIFO */
		switch (sub_mode) {
		case S_SAMP_XY_A:
			intr_mask_val = SET_INTR_MASK(S_SAMP_XY_A_USR);
			break;
		case S_SAMP_XY_AB:
			intr_mask_val = SET_INTR_MASK(S_SAMP_XY_AB_USR);
			break;
		case S_SAMP_XY_B:
			intr_mask_val = SET_INTR_MASK(S_SAMP_XY_B_USR);
			break;
		default:
			intr_mask_val = SET_INTR_MASK(SAMPLE_USR);
			break;
		};
		intr_mask_val |= 0xC000;
		atomic_set(&pst_adpd->sample_enabled, 1);
		if( Is_Prev_TIA_ADC ){
			adpd143_configuration(pst_adpd, 1);
			Is_Prev_TIA_ADC = 0;
		}
#ifdef __AGC_SUPPORT__		
		AgcStage = AgcStage_LedA_Init;
#endif
#ifdef __PROXIMITY_SUPPORT__
		pst_adpd->st_obj_proximity = 0;
#endif	
		break;
#ifdef __TIA_ADC_SUPPORT__
	case TIA_ADC_USR:
		ADPD143_info("TIA_ADC_USR SAMPLE MODE\n");
		ch1FloatSat = CH1_APPROXIMATE_FLOAT_SATURATION_VALUE_IN_TIA_ADC;
		ch2FloatSat = CH2_APPROXIMATE_FLOAT_SATURATION_VALUE_IN_TIA_ADC;
		ch3FloatSat = CH3_APPROXIMATE_FLOAT_SATURATION_VALUE_IN_TIA_ADC;
		ch4FloatSat = CH4_APPROXIMATE_FLOAT_SATURATION_VALUE_IN_TIA_ADC;
		Is_TIA_ADC_Dark_Calibrated = Is_Float_Dark_Calibrated = 0;		
		for(i=0; i<8; i++) rawDarkCh[i] = rawDataCh[i] = 0;
		/*enable interrupt only when data written to FIFO */
		switch (sub_mode) {
		case S_SAMP_XY_A:
			intr_mask_val = SET_INTR_MASK(S_SAMP_XY_A_USR);
			break;
		case S_SAMP_XY_AB:
			intr_mask_val = SET_INTR_MASK(S_SAMP_XY_AB_USR);
			break;
		case S_SAMP_XY_B:
			intr_mask_val = SET_INTR_MASK(S_SAMP_XY_B_USR);
			break;
		default:
			intr_mask_val = SET_INTR_MASK(SAMPLE_USR);
			break;
		};
		intr_mask_val |= 0xC000;
		atomic_set(&pst_adpd->sample_enabled, 1);
		if( !Is_Prev_TIA_ADC ){
			pr_err("adpd143_TIA_ADC_configuration #######################################");
			adpd143_TIA_ADC_configuration(pst_adpd, 1);
			Is_Prev_TIA_ADC = 1;
		}
		break;
#endif

#ifdef __EOL_SUPPORT__
	case EOL_USR:
		ADPD143_dbg("EOL SAMPLE MODE*********************************\n");
		pst_adpd->eol_state = ST_EOL_IDLE;
		gp_adpd_data->eol_res_odr = 0;
		prev_interrupt_trigger_ts.tv_sec = -1;
		switch (sub_mode) {
		case S_SAMP_XY_A:
			intr_mask_val = SET_INTR_MASK(S_SAMP_XY_A_USR);
			break;
		case S_SAMP_XY_AB:
			intr_mask_val = SET_INTR_MASK(S_SAMP_XY_AB_USR);
			break;
		case S_SAMP_XY_B:
			intr_mask_val = SET_INTR_MASK(S_SAMP_XY_B_USR);
			break;
		default:
			intr_mask_val = SET_INTR_MASK(SAMPLE_USR);
			break;
		};
		intr_mask_val |= 0xC000;
		atomic_set(&pst_adpd->sample_enabled, 1);
		if( Is_Prev_TIA_ADC ){
			adpd143_configuration(pst_adpd, 1);
			Is_Prev_TIA_ADC = 0;
		}
		AgcStage = AgcStage_LedA_Init;
		pst_adpd->st_obj_proximity = 0;
		break;
#endif	
	default:
		/*This case won't occur */
		ADPD143_dbg("invalid mode\n");
		break;
	};

	ADPD143_info("INT_MASK_ADDR[0x%04x] = 0x%04x\n", ADPD_INT_MASK_ADDR,
			  intr_mask_val);
	pst_adpd->intr_mask_val = intr_mask_val;

	/*Fetch Default opmode configuration other than OP mode
	   and DATA_OUT mode */
	mode_val = DEFAULT_OP_MODE_CFG_VAL(pst_adpd);
	/*Get Mode value from the opmode value,
	to hardocde the macro, change SET_MODE_VALUE macro and concern
	mode macro
	*/
	mode_val += SET_MODE_VALUE(opmode_val);

	ADPD143_info("OP_MODE_CFG[0x%04x] = 0x%04x\n",
		ADPD_OP_MODE_CFG_ADDR, mode_val);

	atomic_set(&pst_adpd->adpd_mode, usr_mode);

	reg_write(pst_adpd, ADPD_INT_MASK_ADDR, pst_adpd->intr_mask_val);
	reg_write(pst_adpd, ADPD_OP_MODE_CFG_ADDR, mode_val);

	mutex_unlock(&pst_adpd->mutex);

	enable_irq(pst_adpd->irq);

	if (mode != IDLE_USR)
		adpd143_config_prerequisite(pst_adpd, GLOBAL_OP_MODE_GO);
	else
		adpd143_config_prerequisite(pst_adpd, GLOBAL_OP_MODE_OFF);

}


#ifdef __EOL_SUPPORT__
void adpd_set_eol_enable(unsigned char enable)
{	
	if( gp_adpd_data )
	{
		if( enable ) 
			adpd143_mode_switching(gp_adpd_data, 0x71);
		else
			adpd143_mode_switching(gp_adpd_data, 0x0);			
	}		
}

void adpd_get_eol_result(char* result)
{
	if( gp_adpd_data && result )
	{
		ADPD_EOL_RESULT temp;
		temp.st_eol_res.eol_res_odr = gp_adpd_data->eol_res_odr;
		
		temp.st_eol_res.eol_res_red_low_dc = gp_adpd_data->eol_res_red_low_dc;
		temp.st_eol_res.eol_res_ir_low_dc = gp_adpd_data->eol_res_ir_low_dc;
		temp.st_eol_res.eol_res_red_low_noise = gp_adpd_data->eol_res_red_low_noise;
		temp.st_eol_res.eol_res_ir_low_noise = gp_adpd_data->eol_res_ir_low_noise;
		
		temp.st_eol_res.eol_res_red_med_dc = gp_adpd_data->eol_res_red_med_dc;
		temp.st_eol_res.eol_res_ir_med_dc = gp_adpd_data->eol_res_ir_med_dc;
		temp.st_eol_res.eol_res_red_med_noise = gp_adpd_data->eol_res_red_med_noise;
		temp.st_eol_res.eol_res_ir_med_noise = gp_adpd_data->eol_res_ir_med_noise;
		
		temp.st_eol_res.eol_res_red_high_dc = gp_adpd_data->eol_res_red_high_dc;
		temp.st_eol_res.eol_res_ir_high_dc = gp_adpd_data->eol_res_ir_high_dc;
		temp.st_eol_res.eol_res_red_high_noise = gp_adpd_data->eol_res_red_high_noise;
		temp.st_eol_res.eol_res_ir_high_noise = gp_adpd_data->eol_res_ir_high_noise;
		
		temp.st_eol_res.eol_res_red_ac = gp_adpd_data->eol_res_red_ac;
		temp.st_eol_res.eol_res_ir_ac = gp_adpd_data->eol_res_ir_ac;

		memcpy(result, temp.buf, sizeof(temp.buf));		
	}
}

unsigned char adpd_get_eol_status(void)
{
	if( gp_adpd_data->eol_state != ST_EOL_DONE ) return 0;
	else return 1;
}

unsigned short ADPD143_32khzTrim(void)	
{
	unsigned short oscTrimValue;
	int error;
	unsigned short oscRegValue;

	oscRegValue = reg_read(gp_adpd_data, 0x4B);
	oscTrimValue = oscRegValue & 0x3F;

	if(EOL_ODR_TARGET - gp_adpd_data->eol_res_odr > 0 )
		error = 100*EOL_ODR_TARGET - 100*gp_adpd_data->eol_res_odr; 	 
	else
		error = 100*gp_adpd_data->eol_res_odr - 100*EOL_ODR_TARGET; 

	if( EOL_ODR_TARGET - gp_adpd_data->eol_res_odr > 0 )
		oscTrimValue -= (error/625 > 0)?((error-(error/625)*625 > 320)?(error/625)+1:(error/625)):((error > 320)?1:0);
	else
		oscTrimValue += (error/625 > 0)?((error-(error/625)*625 > 320)?(error/625)+1:(error/625)):((error > 320)?1:0);
	
	oscRegValue &= 0xFFC0;
	oscRegValue |= oscTrimValue;

	ADPD143_dbg("[ADPD143_32khzTrim] oscRegValue : 0x%04x, error : %d\n",
				oscRegValue, error);
	
	reg_write(gp_adpd_data, 0x4B, oscRegValue); 
	return oscRegValue; //Joshua, for saving Clock Calibration result   	
}

static EOL_state step_EOL_clock_calibration(unsigned* dataInA, unsigned* dataInB)//pst_adpd->mutex is necessary for clock cal.
{
	unsigned short i = 0;
	
	if( gp_adpd_data->eol_state == ST_EOL_IDLE )
	{
		gp_adpd_data->eol_state = ST_EOL_CLOCK_CAL_INIT;
		
		for (i=0; i<8; i++)
			g_eol_ADCOffset[i] = 0;
		for (i=0; i<32; i++)
		  g_org_regValues[i] = reg_read(gp_adpd_data, g_eol_regNumbers[i]);
		
		reg_write(gp_adpd_data, 0x10, 0x1); 					
		reg_write(gp_adpd_data, 0x12, 8000/EOL_ODR_TARGET); 									 
		reg_write(gp_adpd_data, 0x15, 0x0); 									 

		if( dataInA ) reg_write(gp_adpd_data, 0x31, 0x0113);
		if( dataInB ) reg_write(gp_adpd_data, 0x36, 0x0113);
		reg_write(gp_adpd_data, 0x34, 0x0300);	

		if( dataInA )
		{
			reg_write(gp_adpd_data, 0x18, 0);
			reg_write(gp_adpd_data, 0x19, 0);
			reg_write(gp_adpd_data, 0x1A, 0);
			reg_write(gp_adpd_data, 0x1B, 0);
		}
		if( dataInB )
		{
			reg_write(gp_adpd_data, 0x1E, 0);
			reg_write(gp_adpd_data, 0x1F, 0);
			reg_write(gp_adpd_data, 0x20, 0);
			reg_write(gp_adpd_data, 0x21, 0);
		}
		
		gp_adpd_data->eol_counter = 0;
		reg_write(gp_adpd_data, 0x10, 0x2);
	}
	else if( gp_adpd_data->eol_state == ST_EOL_CLOCK_CAL_INIT )
	{
		if( gp_adpd_data->eol_counter++ > 20 )
		{
			gp_adpd_data->eol_state = ST_EOL_CLOCK_CAL_RUNNING;
			gp_adpd_data->eol_counter = 0;
		}
	}	
	else if( gp_adpd_data->eol_state == ST_EOL_CLOCK_CAL_RUNNING )
	{
		gp_adpd_data->eol_counter++;
#ifndef __USE_EOL_US_INT_SPACE__
		gp_adpd_data->eol_measured_period += gp_adpd_data->msec_eol_int_space;
		ADPD143_dbg("[step_EOL_clock_calibration] eol_counter : %d, msec_eol_int_space : %dms, eol_measured_period : %ldms, gp_adpd_data->eol_res_odr : %dhz\n",
					gp_adpd_data->eol_counter, gp_adpd_data->msec_eol_int_space, gp_adpd_data->eol_measured_period, gp_adpd_data->eol_res_odr);
#else
		gp_adpd_data->eol_measured_period += gp_adpd_data->usec_eol_int_space;
		ADPD143_dbg("[step_EOL_clock_calibration] eol_counter : %d, usec_eol_int_space : %dus, eol_measured_period : %ldms, gp_adpd_data->eol_res_odr : %dhz\n",
					gp_adpd_data->eol_counter, gp_adpd_data->usec_eol_int_space, gp_adpd_data->eol_measured_period, gp_adpd_data->eol_res_odr);
#endif
		if( gp_adpd_data->eol_counter <= 16 )
		{
			for(i=0; i<4; i++)
			{
				if( dataInA )
					g_eol_ADCOffset[i] += dataInA[i];
				if( dataInB )
					g_eol_ADCOffset[i+4] += dataInB[i];
			}
		}
		else if( gp_adpd_data->eol_counter == 17 )
		{
			for (i=0; i<8; i++)
			{
			  g_eol_ADCOffset[i] >>= 4;			
			  ADPD143_dbg("g_eol_ADCOffset[i] : 0x%04x\n",
						g_eol_ADCOffset[i]);
			}
			
		}
#ifndef __USE_EOL_US_INT_SPACE__
		if(  gp_adpd_data->eol_measured_period >= 498 )
		{
			gp_adpd_data->eol_res_odr = (gp_adpd_data->eol_counter * 1000 / gp_adpd_data->eol_measured_period);
#else
		if(  gp_adpd_data->eol_measured_period >= 498000 )
		{
			gp_adpd_data->eol_res_odr = (gp_adpd_data->eol_counter * 1000*1000 / gp_adpd_data->eol_measured_period);
#endif
			if( (gp_adpd_data->eol_res_odr - EOL_ODR_TARGET) > EOL_ALLOWABLE_ODR_ERROR || (EOL_ODR_TARGET - gp_adpd_data->eol_res_odr) > EOL_ALLOWABLE_ODR_ERROR )
			{
				ADPD143_32khzTrim();   
				gp_adpd_data->eol_counter = 0;
				gp_adpd_data->eol_measured_period = 0;
				gp_adpd_data->eol_state = ST_EOL_CLOCK_CAL_INIT;	
			}
			else
			{
				gp_adpd_data->eol_counter = 0;
				gp_adpd_data->eol_measured_period = 0;
				gp_adpd_data->eol_state = ST_EOL_CLOCK_CAL_DONE;	
			}
		}	
	}

	return gp_adpd_data->eol_state;
}

static EOL_state step_EOL_low_DC(unsigned* dataInA, unsigned* dataInB)
{
	unsigned short i = 0;
	
	if ( gp_adpd_data->eol_state == ST_EOL_CLOCK_CAL_DONE )
	{

		gp_adpd_data->eol_state = ST_EOL_LOW_DC_INIT; 
		reg_write(gp_adpd_data, 0x10, 0x1); 					

		reg_write(gp_adpd_data, 0x12, 8000/(EOL_ODR_TARGET>>1<<3)); 									 
		reg_write(gp_adpd_data, 0x15, 0x333); 									 

		if( dataInA ) reg_write(gp_adpd_data, 0x31, 0x0813);
		if( dataInB ) reg_write(gp_adpd_data, 0x36, 0x0813);
		if( dataInA ) reg_write(gp_adpd_data, 0x23, 0x3031);
		if( dataInB ) reg_write(gp_adpd_data, 0x24, 0x3031);
		reg_write(gp_adpd_data, 0x34, 0x0000);	

#ifndef __USE_EOL_ADC_OFFSET__
		reg_write(gp_adpd_data, 0x18, 0x1fff);
		reg_write(gp_adpd_data, 0x19, 0x1fff);
		reg_write(gp_adpd_data, 0x1A, 0x1fff);
		reg_write(gp_adpd_data, 0x1B, 0x1fff);

		reg_write(gp_adpd_data, 0x1E, 0x1fff);
		reg_write(gp_adpd_data, 0x1F, 0x1fff);
		reg_write(gp_adpd_data, 0x20, 0x1fff);
		reg_write(gp_adpd_data, 0x21, 0x1fff);
#else
		reg_write(gp_adpd_data, 0x18, g_eol_ADCOffset[0]);
		reg_write(gp_adpd_data, 0x19, g_eol_ADCOffset[1]);
		reg_write(gp_adpd_data, 0x1A, g_eol_ADCOffset[2]);
		reg_write(gp_adpd_data, 0x1B, g_eol_ADCOffset[3]);

		reg_write(gp_adpd_data, 0x1E, g_eol_ADCOffset[4]);
		reg_write(gp_adpd_data, 0x1F, g_eol_ADCOffset[5]);
		reg_write(gp_adpd_data, 0x20, g_eol_ADCOffset[6]);
		reg_write(gp_adpd_data, 0x21, g_eol_ADCOffset[7]);
#endif

		reg_write(gp_adpd_data, 0x39,0x24D4);
		reg_write(gp_adpd_data, 0x3B,0x24D4);
		reg_write(gp_adpd_data, 0x42,0x1C37);
		reg_write(gp_adpd_data, 0x43,0xADA5);
		reg_write(gp_adpd_data, 0x52,0x0040);
		reg_write(gp_adpd_data, 0x44,0x1C37);
		reg_write(gp_adpd_data, 0x45,0xADA5);
		reg_write(gp_adpd_data, 0x52,0x0040);

		gp_adpd_data->eol_counter = 0;
		reg_write(gp_adpd_data, 0x10, 0x2);
	}
	else if( gp_adpd_data->eol_state == ST_EOL_LOW_DC_INIT )
	{
		if( gp_adpd_data->eol_counter++ > CNT_EOL_SKIP_SAMPLE )
		{
			gp_adpd_data->eol_state = ST_EOL_LOW_DC_RUNNING;
			gp_adpd_data->eol_counter = 0;
		}
	}	
	else if( gp_adpd_data->eol_state == ST_EOL_LOW_DC_RUNNING )
	{	
		if( dataInA ) g_dc_buffA[gp_adpd_data->eol_counter] = *(dataInA) + *(dataInA+1) + *(dataInA+2) + *(dataInA+3);
		if( dataInB ) g_dc_buffB[gp_adpd_data->eol_counter] = *(dataInB) + *(dataInB+1) + *(dataInB+2) + *(dataInB+3);
		if( ++gp_adpd_data->eol_counter == CNT_SAMPLE_PER_EOL_CYCLE )
		{
			if( dataInA )
			{
				gp_adpd_data->eol_res_red_low_dc = 0;
				gp_adpd_data->eol_res_red_low_noise = 0;				
				for (i=0; i<100; i++)
					gp_adpd_data->eol_res_red_low_dc += g_dc_buffA[i];
				gp_adpd_data->eol_res_red_low_dc /= CNT_SAMPLE_PER_EOL_CYCLE;
				for (i=0; i<100; i++)
					gp_adpd_data->eol_res_red_low_noise += (g_dc_buffA[i] - gp_adpd_data->eol_res_red_low_dc)*(g_dc_buffA[i] - gp_adpd_data->eol_res_red_low_dc);
				gp_adpd_data->eol_res_red_low_noise /= CNT_SAMPLE_PER_EOL_CYCLE;
			}
			if( dataInB )
			{
				gp_adpd_data->eol_res_ir_low_dc = 0;
				gp_adpd_data->eol_res_ir_low_noise = 0;				
				for (i=0; i<100; i++)
					gp_adpd_data->eol_res_ir_low_dc += g_dc_buffB[i];
				gp_adpd_data->eol_res_ir_low_dc /= CNT_SAMPLE_PER_EOL_CYCLE;
				for (i=0; i<100; i++)
					gp_adpd_data->eol_res_ir_low_noise += (g_dc_buffB[i] - gp_adpd_data->eol_res_ir_low_dc)*(g_dc_buffB[i] - gp_adpd_data->eol_res_ir_low_dc);
				gp_adpd_data->eol_res_ir_low_noise /= CNT_SAMPLE_PER_EOL_CYCLE;
			}
			gp_adpd_data->eol_state = ST_EOL_LOW_DC_DONE;
			gp_adpd_data->eol_counter = 0;
		}
	}
		
	return gp_adpd_data->eol_state;

}

static EOL_state step_EOL_med_DC(unsigned* dataInA, unsigned* dataInB)
{
	unsigned short i = 0;
	
	if ( gp_adpd_data->eol_state == ST_EOL_LOW_DC_DONE )
	{
		gp_adpd_data->eol_state = ST_EOL_MED_DC_INIT; 

		reg_write(gp_adpd_data, 0x10, 0x1); 					

		if( dataInA ) reg_write(gp_adpd_data, 0x23, 0x3034);
		if( dataInB ) reg_write(gp_adpd_data, 0x24, 0x3034);

		gp_adpd_data->eol_counter = 0;
		reg_write(gp_adpd_data, 0x10, 0x2);
	}
	else if( gp_adpd_data->eol_state == ST_EOL_MED_DC_INIT )
	{
		if( gp_adpd_data->eol_counter++ > CNT_EOL_SKIP_SAMPLE )
		{
			gp_adpd_data->eol_state = ST_EOL_MED_DC_RUNNING;
			gp_adpd_data->eol_counter = 0;
		}
	}	
	else if( gp_adpd_data->eol_state == ST_EOL_MED_DC_RUNNING )
	{	
		if( dataInA ) g_dc_buffA[gp_adpd_data->eol_counter] = *(dataInA) + *(dataInA+1) + *(dataInA+2) + *(dataInA+3);
		if( dataInB ) g_dc_buffB[gp_adpd_data->eol_counter] = *(dataInB) + *(dataInB+1) + *(dataInB+2) + *(dataInB+3);
		if( ++gp_adpd_data->eol_counter == CNT_SAMPLE_PER_EOL_CYCLE )
		{
			if( dataInA )
			{
				gp_adpd_data->eol_res_red_med_dc = 0;
				gp_adpd_data->eol_res_red_med_noise = 0;				
				for (i=0; i<100; i++)
					gp_adpd_data->eol_res_red_med_dc += g_dc_buffA[i];
				gp_adpd_data->eol_res_red_med_dc /= CNT_SAMPLE_PER_EOL_CYCLE;
				for (i=0; i<100; i++)
					gp_adpd_data->eol_res_red_med_noise += (g_dc_buffA[i] - gp_adpd_data->eol_res_red_med_dc)*(g_dc_buffA[i] - gp_adpd_data->eol_res_red_med_dc);
				gp_adpd_data->eol_res_red_med_noise /= CNT_SAMPLE_PER_EOL_CYCLE;
			}
			if( dataInB )
			{
				gp_adpd_data->eol_res_ir_med_dc = 0;
				gp_adpd_data->eol_res_ir_med_noise = 0;				
				for (i=0; i<100; i++)
					gp_adpd_data->eol_res_ir_med_dc += g_dc_buffB[i];
				gp_adpd_data->eol_res_ir_med_dc /= CNT_SAMPLE_PER_EOL_CYCLE;
				for (i=0; i<100; i++)
					gp_adpd_data->eol_res_ir_med_noise += (g_dc_buffB[i] - gp_adpd_data->eol_res_ir_med_dc)*(g_dc_buffB[i] - gp_adpd_data->eol_res_ir_med_dc);
				gp_adpd_data->eol_res_ir_med_noise /= CNT_SAMPLE_PER_EOL_CYCLE;
			}
			gp_adpd_data->eol_state = ST_EOL_MED_DC_DONE;
			gp_adpd_data->eol_counter = 0;
		}
	}
		
	return gp_adpd_data->eol_state;

}

static EOL_state step_EOL_high_DC(unsigned* dataInA, unsigned* dataInB)
{
	unsigned short i = 0;
	
	if ( gp_adpd_data->eol_state == ST_EOL_MED_DC_DONE )
	{
		gp_adpd_data->eol_state = ST_EOL_HIGH_DC_INIT; 

		reg_write(gp_adpd_data, 0x10, 0x1); 					

		if( dataInA ) reg_write(gp_adpd_data, 0x23, 0x3037);
		if( dataInB ) reg_write(gp_adpd_data, 0x24, 0x3037);

		gp_adpd_data->eol_counter = 0;
		reg_write(gp_adpd_data, 0x10, 0x2);
	}
	else if( gp_adpd_data->eol_state == ST_EOL_HIGH_DC_INIT )
	{
		if( gp_adpd_data->eol_counter++ > CNT_EOL_SKIP_SAMPLE )
		{
			gp_adpd_data->eol_state = ST_EOL_HIGH_DC_RUNNING;
			gp_adpd_data->eol_counter = 0;
		}
	}	
	else if( gp_adpd_data->eol_state == ST_EOL_HIGH_DC_RUNNING )
	{	
		if( dataInA ) g_dc_buffA[gp_adpd_data->eol_counter] = *(dataInA) + *(dataInA+1) + *(dataInA+2) + *(dataInA+3);
		if( dataInB ) g_dc_buffB[gp_adpd_data->eol_counter] = *(dataInB) + *(dataInB+1) + *(dataInB+2) + *(dataInB+3);
		if( ++gp_adpd_data->eol_counter == CNT_SAMPLE_PER_EOL_CYCLE )
		{
			if( dataInA )
			{
				gp_adpd_data->eol_res_red_high_dc = 0;
				gp_adpd_data->eol_res_red_high_noise = 0;				
				for (i=0; i<100; i++)
					gp_adpd_data->eol_res_red_high_dc += g_dc_buffA[i];
				gp_adpd_data->eol_res_red_high_dc /= CNT_SAMPLE_PER_EOL_CYCLE;
				for (i=0; i<100; i++)
					gp_adpd_data->eol_res_red_high_noise += (g_dc_buffA[i] - gp_adpd_data->eol_res_red_high_dc)*(g_dc_buffA[i] - gp_adpd_data->eol_res_red_high_dc);
				gp_adpd_data->eol_res_red_high_noise /= CNT_SAMPLE_PER_EOL_CYCLE;
			}
			if( dataInB )
			{
				gp_adpd_data->eol_res_ir_high_dc = 0;
				gp_adpd_data->eol_res_ir_high_noise = 0;				
				for (i=0; i<100; i++)
					gp_adpd_data->eol_res_ir_high_dc += g_dc_buffB[i];
				gp_adpd_data->eol_res_ir_high_dc /= CNT_SAMPLE_PER_EOL_CYCLE;
				for (i=0; i<100; i++)
					gp_adpd_data->eol_res_ir_high_noise += (g_dc_buffB[i] - gp_adpd_data->eol_res_ir_high_dc)*(g_dc_buffB[i] - gp_adpd_data->eol_res_ir_high_dc);
				gp_adpd_data->eol_res_ir_high_noise /= CNT_SAMPLE_PER_EOL_CYCLE;
			}
			gp_adpd_data->eol_state = ST_EOL_HIGH_DC_DONE;
			gp_adpd_data->eol_counter = 0;
		}
	}
		
	return gp_adpd_data->eol_state;

}

static EOL_state step_EOL_AC(unsigned* dataInA, unsigned* dataInB)
{
	unsigned short i = 0;
	
	if ( gp_adpd_data->eol_state == ST_EOL_HIGH_DC_DONE )
	{
		gp_adpd_data->eol_state = ST_EOL_AC_INIT; 

		reg_write(gp_adpd_data, 0x10, 0x1); 					

		if( dataInA ) reg_write(gp_adpd_data, 0x23, 0x3035);
		if( dataInB ) reg_write(gp_adpd_data, 0x24, 0x3035);

		gp_adpd_data->eol_counter = 0;
		reg_write(gp_adpd_data, 0x10, 0x2);
	}
	else if( gp_adpd_data->eol_state == ST_EOL_AC_INIT )
	{
		if( gp_adpd_data->eol_counter++ > CNT_EOL_SKIP_SAMPLE )
		{
			gp_adpd_data->eol_state = ST_EOL_AC_RUNNING;
			gp_adpd_data->eol_counter = 0;
		}
	}	
	else if( gp_adpd_data->eol_state == ST_EOL_AC_RUNNING )
	{	
		if( dataInA ) g_ac_buffA[gp_adpd_data->eol_counter] = *(dataInA) + *(dataInA+1) + *(dataInA+2) + *(dataInA+3);
		if( dataInB ) g_ac_buffB[gp_adpd_data->eol_counter] = *(dataInB) + *(dataInB+1) + *(dataInB+2) + *(dataInB+3);
		if( ++gp_adpd_data->eol_counter == CNT_SAMPLE_PER_EOL_CYCLE )
		{
			if( dataInA )
			{
				gp_adpd_data->eol_res_red_ac = 0;
				for (i=0; i<100; i++)
					gp_adpd_data->eol_res_red_ac += (g_dc_buffA[i]-g_ac_buffA[i]);
				gp_adpd_data->eol_res_red_ac /= CNT_SAMPLE_PER_EOL_CYCLE;
			}
			if( dataInB )
			{
				gp_adpd_data->eol_res_ir_ac = 0;
				for (i=0; i<100; i++)
					gp_adpd_data->eol_res_ir_ac += (g_dc_buffB[i]-g_ac_buffB[i]);
				gp_adpd_data->eol_res_ir_ac /= CNT_SAMPLE_PER_EOL_CYCLE;
			}
			gp_adpd_data->eol_state = ST_EOL_AC_DONE;
			gp_adpd_data->eol_counter = 0;
		}
	}
		
	return gp_adpd_data->eol_state;

}


static EOL_state stepEOL(unsigned* dataInA, unsigned* dataInB)
{
	unsigned short i = 0;
	unsigned short oscRegValue;
	
	switch (gp_adpd_data->eol_state){
	case ST_EOL_IDLE:
		ADPD143_dbg("******************EOL started*****************\n");
	case ST_EOL_CLOCK_CAL_INIT:
		ADPD143_dbg("******************Clock Calibration Init*****************\n");
	case ST_EOL_CLOCK_CAL_RUNNING:
		ADPD143_dbg("******************Clock Calibration Running*****************\n");
		step_EOL_clock_calibration(dataInA, dataInB);
		break;
		
	case ST_EOL_CLOCK_CAL_DONE:		
		oscRegValue = reg_read(gp_adpd_data, 0x4B);
		ADPD143_dbg("******************Clock Calibration Finished : %dhz, 0x%04x*****************\n", gp_adpd_data->eol_res_odr, oscRegValue);
	case ST_EOL_LOW_DC_INIT:
		ADPD143_dbg("******************Low DC Statistics Init : %d*****************\n", gp_adpd_data->eol_counter);
	case ST_EOL_LOW_DC_RUNNING:
		ADPD143_dbg("******************Low DC Statistics Running : %d*****************\n", gp_adpd_data->eol_counter);
		step_EOL_low_DC(dataInA, dataInB); 		
		break;
		
	case ST_EOL_LOW_DC_DONE:
		ADPD143_dbg("****************** [Low DC statistics] Red DC : %d, Red Noise : %d, IR DC : %d, IR noise : %d *****************\n", 
			gp_adpd_data->eol_res_red_low_dc, gp_adpd_data->eol_res_red_low_noise, gp_adpd_data->eol_res_ir_low_dc, gp_adpd_data->eol_res_ir_low_noise);
	case ST_EOL_MED_DC_INIT:
		ADPD143_dbg("******************Medium DC Statistics Init : %d*****************\n", gp_adpd_data->eol_counter);
	case ST_EOL_MED_DC_RUNNING:
		ADPD143_dbg("******************Medium DC Statistics Running : %d*****************\n", gp_adpd_data->eol_counter);
		step_EOL_med_DC(dataInA, dataInB); 		
		break;
		
	case ST_EOL_MED_DC_DONE:
		ADPD143_dbg("****************** [Medium DC statistics] Red DC : %d, Red Noise : %d, IR DC : %d, IR noise : %d *****************\n", 
			gp_adpd_data->eol_res_red_med_dc, gp_adpd_data->eol_res_red_med_noise, gp_adpd_data->eol_res_ir_med_dc, gp_adpd_data->eol_res_ir_med_noise);
	case ST_EOL_HIGH_DC_INIT:
		ADPD143_dbg("******************High DC Statistics Init : %d*****************\n", gp_adpd_data->eol_counter);
	case ST_EOL_HIGH_DC_RUNNING:
		ADPD143_dbg("******************High DC Statistics Running : %d*****************\n", gp_adpd_data->eol_counter);
		step_EOL_high_DC(dataInA, dataInB); 		
		break;
		
	case ST_EOL_HIGH_DC_DONE:
		ADPD143_dbg("****************** [High DC statistics] Red DC : %d, Red Noise : %d, IR DC : %d, IR noise : %d *****************\n", 
			gp_adpd_data->eol_res_red_high_dc, gp_adpd_data->eol_res_red_high_noise, gp_adpd_data->eol_res_ir_high_dc, gp_adpd_data->eol_res_ir_high_noise);
	case ST_EOL_AC_INIT:
		ADPD143_dbg("******************AC Statistics Init : %d*****************\n", gp_adpd_data->eol_counter);
	case ST_EOL_AC_RUNNING:
		ADPD143_dbg("******************AC Statistics Running : %d*****************\n", gp_adpd_data->eol_counter);
		step_EOL_AC(dataInA, dataInB); 		
		break;
		
	case ST_EOL_AC_DONE:
		ADPD143_dbg("****************** [AC statistics] Red AC : %d, IR AC : %d *****************\n", 
			gp_adpd_data->eol_res_red_ac, gp_adpd_data->eol_res_ir_ac);
		gp_adpd_data->eol_state = ST_EOL_DONE;
		break;

	case ST_EOL_DONE:
		ADPD143_dbg("******************EOL Completed*****************\n");
		for (i=0; i<32; i++)
			if( ( g_eol_regNumbers[i] < 0x18 || g_eol_regNumbers[i] > 0x21 ) && ( g_eol_regNumbers[i] != 0x4B ) )
				reg_write(gp_adpd_data, g_eol_regNumbers[i],g_org_regValues[i]);
		break;
	default:
		break;
	}
	return gp_adpd_data->eol_state;
}

static EOL_state getEOLstate(void){ return 	gp_adpd_data->eol_state; }


#endif

/**
This function is used for sending Sample event depend upon
the Sample mode
@param pst_adpd the ADPD143 data structure
@return void
*/

#define FLOAT_MODE_SATURATION_CONTROL_CODE 7200
#define TIA_ADC_MODE_DARK_CALIBRATION_CNT 5
#define FLOAT_MODE_DARK_CALIBRATION_CNT 5

static void
adpd143_sample_event(struct adpd143_data *pst_adpd)
{
	unsigned short usr_mode = 0;
	unsigned short sub_mode = 0;
	unsigned short cnt = 0;
	unsigned short uncoated_ch3 = 0;
	unsigned short coated_ch4 = 0;
	unsigned short k = 0;

#ifdef __AGC_SUPPORT__
	unsigned uAgcRawDataA[4];
	unsigned uAgcRawDataB[4];
#endif

	ADPD143_info("%s\n", __func__);

	usr_mode = atomic_read(&pst_adpd->adpd_mode);
	sub_mode = GET_USR_SUB_MODE(usr_mode);

	ADPD143_dbg("#########[%s] user mode : %d, sub mode : %d\n", __func__, GET_USR_MODE(usr_mode),GET_USR_SUB_MODE(usr_mode));

	switch (sub_mode) {
	case S_SAMP_XY_A:
		if (pst_adpd->fifo_size < 4 || (pst_adpd->fifo_size & 0x3)) {
			ADPD143_dbg("Unexpected FIFO_SIZE=%d\n",
				pst_adpd->fifo_size);
			break;
		}
#ifdef __TIA_ADC_SUPPORT__		
		if( GET_USR_MODE(usr_mode) != TIA_ADC_USR )
#endif
		{
#ifdef __EOL_SUPPORT__
			if( GET_USR_MODE(usr_mode) == EOL_USR )
			{
				if( getEOLstate() != ST_EOL_DONE )
				{	
					for (cnt = 0; cnt < pst_adpd->fifo_size; cnt += 4)
					{
						for(k=0; k<4; k++)
							uAgcRawDataA[k] = (unsigned)pst_adpd->ptr_data_buffer[cnt+k];
#ifndef __USE_EOL_US_INT_SPACE__
						if( cnt > 0 ) gp_adpd_data->msec_eol_int_space = 0;
#else
						if( cnt > 0 ) gp_adpd_data->usec_eol_int_space = 0;
#endif
						stepEOL(uAgcRawDataA, NULL);
					}
					break;
				}
			}
#endif
			
			for (cnt = 0; cnt < pst_adpd->fifo_size; cnt += 4) {
				input_event(pst_adpd->ptr_sample_inputdev, EV_REL,
					REL_Z, sub_mode + 1);

				{
					ADPD143_dbg("adpd143 SLOT A(HRM mode) : rawDataCh1 : %d, normDataCh1 : %d\n",
						pst_adpd->ptr_data_buffer[cnt], 
						(pst_adpd->ptr_data_buffer[cnt]*256*SF_RED_CH/(pst_adpd->efuseRedSlope+128)/100)); 								
					ADPD143_dbg("adpd143 SLOT A(HRM mode) : rawDataCh2 : %d, normDataCh2 : %d\n",
						pst_adpd->ptr_data_buffer[cnt+1], 
						(pst_adpd->ptr_data_buffer[cnt+1]*256*SF_RED_CH/(pst_adpd->efuseRedSlope+128)/100)); 								
					ADPD143_dbg("adpd143 SLOT A(HRM mode) : rawDataCh3 : %d, normDataCh3 : %d\n",
						pst_adpd->ptr_data_buffer[cnt+2], 
						(pst_adpd->ptr_data_buffer[cnt+2]*256*SF_RED_CH/(pst_adpd->efuseRedSlope+128)/100)); 								
					ADPD143_dbg("adpd143 SLOT A(HRM mode) : rawDataCh4 : %d, normDataCh4 : %d\n",
						pst_adpd->ptr_data_buffer[cnt+3], 
						(pst_adpd->ptr_data_buffer[cnt+4]*256*SF_RED_CH/(pst_adpd->efuseRedSlope+128)/100)); 								

					input_event(pst_adpd->ptr_sample_inputdev, EV_MSC,
						MSC_RAW, (pst_adpd->ptr_data_buffer[cnt]*256*SF_RED_CH/(pst_adpd->efuseRedSlope+128)/100));
					input_event(pst_adpd->ptr_sample_inputdev, EV_MSC,
						MSC_RAW, (pst_adpd->ptr_data_buffer[cnt+1]*256*SF_RED_CH/(pst_adpd->efuseRedSlope+128)/100));
					input_event(pst_adpd->ptr_sample_inputdev, EV_MSC,
						MSC_RAW, (pst_adpd->ptr_data_buffer[cnt+2]*256*SF_RED_CH/(pst_adpd->efuseRedSlope+128)/100));
					input_event(pst_adpd->ptr_sample_inputdev, EV_MSC,
						MSC_RAW, (pst_adpd->ptr_data_buffer[cnt+3]*256*SF_RED_CH/(pst_adpd->efuseRedSlope+128)/100));

					input_sync(pst_adpd->ptr_sample_inputdev);
				}
			}
#ifdef __AGC_SUPPORT__
			{
				cnt -= 4;
				for(k=0; k<4; k++)
					uAgcRawDataA[k] = (unsigned)pst_adpd->ptr_data_buffer[cnt+k];
#ifdef __PROXIMITY_SUPPORT__
				checkObjProximity(NULL, uAgcRawDataA);
#endif
			}
#endif
			
			ADPD143_dbg("**********************************\n\n");
		}
#ifdef __TIA_ADC_SUPPORT__		
		else
		{

			for (cnt = 0; cnt < pst_adpd->fifo_size; cnt += 4) {
				unsigned int channel;
				
				input_event(pst_adpd->ptr_sample_inputdev, EV_REL,
					REL_Z, sub_mode + 1);

				if( Is_TIA_ADC_Dark_Calibrated < TIA_ADC_MODE_DARK_CALIBRATION_CNT && Is_TIA_ADC_Dark_Calibrated > 1){
					for(k=0; k<4; k++){
						if( rawDarkCh[k] < pst_adpd->ptr_data_buffer[cnt+k] || !rawDarkCh[k] )
							rawDarkCh[k] = pst_adpd->ptr_data_buffer[cnt+k];
					}
					Is_TIA_ADC_Dark_Calibrated++;
					continue;
				}
				else if( Is_TIA_ADC_Dark_Calibrated >= TIA_ADC_MODE_DARK_CALIBRATION_CNT && Is_TIA_ADC_Dark_Calibrated < 100 ){
					
						rawDarkCh[0] -= 1470;
						rawDarkCh[1] -= 205;
						rawDarkCh[2] -= 210;
						rawDarkCh[3] -= 230;
		
						Is_TIA_ADC_Dark_Calibrated = 100;						
						continue;
				}
				else if( Is_TIA_ADC_Dark_Calibrated < TIA_ADC_MODE_DARK_CALIBRATION_CNT ){
					Is_TIA_ADC_Dark_Calibrated++;
					continue;
				}
				else if( Is_TIA_ADC_Dark_Calibrated == 100 ){
					reg_write(pst_adpd, 0x10,0x0001);
					reg_write(pst_adpd, 0x14,0x0441);
					reg_write(pst_adpd, 0x10,0x0002);
					Is_TIA_ADC_Dark_Calibrated++;
					continue;
				}
				for(channel = 0; channel < 4; ++channel){
					ADPD143_dbg("adpd143 SLOT A(TIA mode) : rawDataCh%d : %u, rawDarkCh%d : %u\n",
						channel, pst_adpd->ptr_data_buffer[cnt+channel], channel, rawDarkCh[channel]);									
					rawDataCh[channel] = rawDarkCh[channel] - pst_adpd->ptr_data_buffer[cnt+channel];
					rawDataCh[channel] = ((int)rawDataCh[channel] < 0)? 0 : rawDataCh[channel];
				}
			
				input_event(pst_adpd->ptr_sample_inputdev, EV_MSC,
					MSC_RAW, rawDataCh[0] );
				input_event(pst_adpd->ptr_sample_inputdev, EV_MSC,
					MSC_RAW, rawDataCh[1] );
				input_event(pst_adpd->ptr_sample_inputdev, EV_MSC,
					MSC_RAW, rawDataCh[2] );
				uncoated_ch3 = rawDataCh[2];
				input_event(pst_adpd->ptr_sample_inputdev, EV_MSC,
					MSC_RAW, rawDataCh[3] );			
				coated_ch4 = rawDataCh[3];
				ADPD143_dbg("adpd143 SLOT A(TIA mode) uncoated_ch3:%d, coated_ch4=%d\n",
					uncoated_ch3, coated_ch4);				
				
				input_sync(pst_adpd->ptr_sample_inputdev);
			}
		}
#endif
		break;

	case S_SAMP_XY_AB:
		if (!pst_adpd->fifo_size)	
		{
			pr_err("Unexpected FIFO_SIZE=%d\n",
				pst_adpd->fifo_size);
			break;
		}
#ifdef __TIA_ADC_SUPPORT__		
		if( GET_USR_MODE(usr_mode) != TIA_ADC_USR )
#endif
		{

#ifdef __EOL_SUPPORT__
			if( GET_USR_MODE(usr_mode) == EOL_USR )
			{
				if( getEOLstate() != ST_EOL_DONE )
				{	
					for (cnt = 0; cnt < pst_adpd->fifo_size; cnt += 8)
					{
						for(k=0; k<4; k++)
							uAgcRawDataA[k] = (unsigned)pst_adpd->ptr_data_buffer[cnt+k];
						for(k=4; k<8; k++)
							uAgcRawDataB[k-4] = (unsigned)pst_adpd->ptr_data_buffer[cnt+k];
#ifndef __USE_EOL_US_INT_SPACE__
						if( cnt > 0 ) gp_adpd_data->msec_eol_int_space = 0;
#else
						if( cnt > 0 ) gp_adpd_data->usec_eol_int_space = 0;
#endif
						stepEOL(uAgcRawDataA, uAgcRawDataB);
					}
					break;
				}
			}
#endif



			for (cnt = 0; cnt < pst_adpd->fifo_size; cnt += 8) {
				unsigned int channel;
				unsigned int sum_slot_a = 0;
				unsigned int sum_slot_b = 0;
				unsigned int sum_slot_norm_a = 0;
				unsigned int sum_slot_norm_b = 0;

				input_event(pst_adpd->ptr_sample_inputdev,
					EV_REL, REL_Z, sub_mode + 1);

				for (channel = 0; channel < 4; ++channel) {
#ifdef __AGC_SUPPORT__
					if( pst_adpd->bOnOffAGC )
						uAgcRawDataA[channel] = (unsigned)pst_adpd->ptr_data_buffer[cnt+channel];
#endif
					sum_slot_a +=
					pst_adpd->ptr_data_buffer[cnt+channel];
					sum_slot_norm_a +=
					(pst_adpd->ptr_data_buffer[cnt+channel]);
					input_event(pst_adpd->ptr_sample_inputdev,
					EV_MSC, MSC_RAW, (pst_adpd->ptr_data_buffer[cnt+channel]));
					ADPD143_dbg("adpd143 SLOT A(HRM mode) : rawDataCh%d : %d, normDataCh%d : %d\n",
						channel+1, pst_adpd->ptr_data_buffer[cnt+channel], 
						channel+1, (pst_adpd->ptr_data_buffer[cnt+channel]));					
				}
				ADPD143_dbg("\n");
				input_event(pst_adpd->ptr_sample_inputdev,
					EV_REL, REL_X, sum_slot_a + 1);

				for (channel = 4; channel < 8; ++channel) {
#ifdef __AGC_SUPPORT__
					if( pst_adpd->bOnOffAGC )
						uAgcRawDataB[channel-4] = (unsigned)pst_adpd->ptr_data_buffer[cnt+channel];
#endif
					sum_slot_b +=
					pst_adpd->ptr_data_buffer[cnt+channel];
					sum_slot_norm_b +=
					(pst_adpd->ptr_data_buffer[cnt+channel]);
					input_event(pst_adpd->ptr_sample_inputdev,		
					EV_MSC, MSC_RAW, (pst_adpd->ptr_data_buffer[cnt+channel]));
					ADPD143_dbg("adpd143 SLOT B(HRM mode) : rawDataCh%d : %d, normDataCh%d : %d\n",
						channel-3, pst_adpd->ptr_data_buffer[cnt+channel], 
						channel-3, (pst_adpd->ptr_data_buffer[cnt+channel]));	
				}
				input_event(pst_adpd->ptr_sample_inputdev,
					EV_REL, REL_Y, sum_slot_b + 1);
				input_sync(pst_adpd->ptr_sample_inputdev);
				ADPD143_dbg("** sum_slot_a : %d  sum_slot_b : %d  sum_slot_norm_a : %d, sum_slot_norm_b : %d\n", 
					sum_slot_a, sum_slot_b, sum_slot_norm_a, sum_slot_norm_b);
			}
#ifdef __PROXIMITY_SUPPORT__
			checkObjProximity(uAgcRawDataA, uAgcRawDataB);
#endif			
			pr_err("**********************************\n\n");
		}
#ifdef __TIA_ADC_SUPPORT__		
		else
		{
		

			for (cnt = 0; cnt < pst_adpd->fifo_size; cnt += 8) {
				unsigned int channel;

				input_event(pst_adpd->ptr_sample_inputdev,
					EV_REL, REL_Z, sub_mode + 1);


				if( Is_TIA_ADC_Dark_Calibrated < TIA_ADC_MODE_DARK_CALIBRATION_CNT && Is_TIA_ADC_Dark_Calibrated > 1){
					for(k=0; k<4; k++){
						if( rawDarkCh[k] < pst_adpd->ptr_data_buffer[cnt+k] || !rawDarkCh[k] )
							rawDarkCh[k] = pst_adpd->ptr_data_buffer[cnt+k];
						ADPD143_dbg("adpd143 SLOT A(TIA mode) : rawDataCh%d : %d, rawDarkCh%d : %d\n",
							k, pst_adpd->ptr_data_buffer[cnt+k], k, rawDarkCh[k]);									
					}
					Is_TIA_ADC_Dark_Calibrated++;
				}
				else if( Is_TIA_ADC_Dark_Calibrated >= TIA_ADC_MODE_DARK_CALIBRATION_CNT && Is_TIA_ADC_Dark_Calibrated < 100 ){
					
						rawDarkCh[0] -= 1470;
						rawDarkCh[1] -= 205;
						rawDarkCh[2] -= 210;
						rawDarkCh[3] -= 230;
		
						Is_TIA_ADC_Dark_Calibrated = 100;
				}
				else if( Is_TIA_ADC_Dark_Calibrated < TIA_ADC_MODE_DARK_CALIBRATION_CNT ){
					Is_TIA_ADC_Dark_Calibrated++;
				}
				if( Is_Float_Dark_Calibrated < TIA_ADC_MODE_DARK_CALIBRATION_CNT && Is_Float_Dark_Calibrated > 1){
					for(k=4; k<8; k++){
						if( rawDarkCh[k] < pst_adpd->ptr_data_buffer[cnt+k] || !rawDarkCh[k] )
							rawDarkCh[k] = pst_adpd->ptr_data_buffer[cnt+k];
						ADPD143_dbg("adpd143 SLOT B(Float mode) : rawDataCh%d : %d, rawDarkCh%d : %d\n",
							k, pst_adpd->ptr_data_buffer[cnt+k], k, rawDarkCh[k]);									
					}
					Is_Float_Dark_Calibrated++;
					continue;
				}
				else if( Is_Float_Dark_Calibrated >= TIA_ADC_MODE_DARK_CALIBRATION_CNT && Is_Float_Dark_Calibrated < 100 ){
						Is_Float_Dark_Calibrated = 100;
						continue;
				}
				else if( Is_Float_Dark_Calibrated < TIA_ADC_MODE_DARK_CALIBRATION_CNT ){
					Is_Float_Dark_Calibrated++;
					continue;
				}
				else if( Is_Float_Dark_Calibrated == 100 ){
					reg_write(pst_adpd, 0x10,0x0001);
					reg_write(pst_adpd, 0x14,0x0441);
					reg_write(pst_adpd, 0x10,0x0002);
					Is_Float_Dark_Calibrated++;
					continue;
				}
				
				for(channel = 0; channel < 4; ++channel){
					ADPD143_dbg("adpd143 SLOT A(TIA mode) : rawDataCh%d : %d, rawDarkCh%d : %d\n",
						channel, pst_adpd->ptr_data_buffer[cnt+channel], channel, rawDarkCh[channel]);									
					rawDataCh[channel] = rawDarkCh[channel] - pst_adpd->ptr_data_buffer[cnt+channel];
					rawDataCh[channel] = ((int)rawDataCh[channel] < 0)? 0 : rawDataCh[channel];
				}			

				for (channel = 0; channel < 4; ++channel) {
					if( channel == 2 ) uncoated_ch3 = rawDataCh[2];
					if( channel == 3 ) coated_ch4 = rawDataCh[3];					
					if( channel == 3 )
						input_event(pst_adpd->ptr_sample_inputdev,
						EV_MSC, MSC_RAW, rawDataCh[channel]*2 );
					else
						input_event(pst_adpd->ptr_sample_inputdev,
						EV_MSC, MSC_RAW, rawDataCh[channel] );
					ADPD143_dbg("(After Dark Cal) adpd143 SLOT A %d:%d\n",
						channel, rawDataCh[channel]);	
				}
				if( uncoated_ch3 == 0 ){
					uncoated_ch3++;
					coated_ch4++;
				}
				input_event(pst_adpd->ptr_sample_inputdev,
				EV_REL, REL_X, (((unsigned)coated_ch4)*100)/uncoated_ch3);	
				
				for(channel = 4; channel < 8; ++channel){
					ADPD143_dbg("adpd143 SLOT B(float mode) : rawDataCh%d : %d, rawDarkCh%d : %d\n",
						channel, pst_adpd->ptr_data_buffer[cnt+channel], channel, rawDarkCh[channel]);									
					
					rawDataCh[channel] = pst_adpd->ptr_data_buffer[cnt+channel] - rawDarkCh[channel];
					rawDataCh[channel] = ((int)rawDataCh[channel] < 0)? 0 : rawDataCh[channel];
					
					if( (int)rawDataCh[channel] >= (FLOAT_MODE_SATURATION_CONTROL_CODE-rawDarkCh[channel]) ){
						rawDataCh[channel] = FLOAT_MODE_SATURATION_CONTROL_CODE-rawDarkCh[channel];
						if( channel == 4 && (((rawDataCh[0] < ch1FloatSat) && (rawDataCh[0] > ch1FloatSat-20)) || (ch1FloatSat == CH1_APPROXIMATE_FLOAT_SATURATION_VALUE_IN_TIA_ADC )) )
							ch1FloatSat = rawDataCh[0];
						if( channel == 5 && (((rawDataCh[1] < ch2FloatSat) && (rawDataCh[1] > ch2FloatSat-20)) || (ch2FloatSat == CH2_APPROXIMATE_FLOAT_SATURATION_VALUE_IN_TIA_ADC )) )
							ch2FloatSat = rawDataCh[1];
						if( channel == 6 && (((rawDataCh[2] < ch3FloatSat) && (rawDataCh[2] > ch3FloatSat-20)) || (ch3FloatSat == CH3_APPROXIMATE_FLOAT_SATURATION_VALUE_IN_TIA_ADC )) )
							ch3FloatSat = rawDataCh[2];
						if( channel == 7 && (((rawDataCh[3] < ch4FloatSat) && (rawDataCh[3] > ch4FloatSat-10)) || (ch4FloatSat == CH4_APPROXIMATE_FLOAT_SATURATION_VALUE_IN_TIA_ADC )) )
							ch4FloatSat = rawDataCh[3];
					}
				}
				
				if( (int)rawDataCh[0] >= ch1FloatSat ){
					rawDataCh[4] = FLOAT_MODE_SATURATION_CONTROL_CODE-rawDarkCh[4];
				}
				if( (int)rawDataCh[1] >= ch2FloatSat ){
					rawDataCh[5] = FLOAT_MODE_SATURATION_CONTROL_CODE-rawDarkCh[5];
				}
				if( (int)rawDataCh[2] >= ch3FloatSat ){
					rawDataCh[6] = FLOAT_MODE_SATURATION_CONTROL_CODE-rawDarkCh[6];
				}
				if( (int)rawDataCh[3] >= ch4FloatSat ){
					rawDataCh[7] = FLOAT_MODE_SATURATION_CONTROL_CODE;
				}				

				for (channel = 4; channel < 8; ++channel) {
					if( GET_USR_MODE(usr_mode) == TIA_ADC_USR ){

						if( channel == 6 ) uncoated_ch3 = rawDataCh[6];
						if( channel == 7 ) coated_ch4 = rawDataCh[7];				
						input_event(pst_adpd->ptr_sample_inputdev,
						EV_MSC, MSC_RAW, rawDataCh[channel]);
						ADPD143_dbg("(After Dark Cal) adpd143 SLOT B %d:%d\n",
							channel, rawDataCh[channel] );					
					}
				}
				if( GET_USR_MODE(usr_mode) == TIA_ADC_USR ){
					if( uncoated_ch3 == 0 ){
						uncoated_ch3++;
						coated_ch4++;
					}
					input_event(pst_adpd->ptr_sample_inputdev,
						EV_REL, REL_Y, (((unsigned)coated_ch4)*100)/uncoated_ch3);				
				}
				
				input_sync(pst_adpd->ptr_sample_inputdev);
			}
		}
#endif
		break;
	case S_SAMP_XY_B:
		if (pst_adpd->fifo_size < 4 || (pst_adpd->fifo_size & 0x3)) {
			pr_err("Unexpected FIFO_SIZE=%d\n",
				pst_adpd->fifo_size);
			break;
		}
#ifdef __TIA_ADC_SUPPORT__		
		if( GET_USR_MODE(usr_mode) != TIA_ADC_USR )
#endif
		{
#ifdef __EOL_SUPPORT__
			if( GET_USR_MODE(usr_mode) == EOL_USR )
			{
				if( getEOLstate() != ST_EOL_DONE )
				{	
					for (cnt = 0; cnt < pst_adpd->fifo_size; cnt += 4)
					{
						for(k=0; k<4; k++)
							uAgcRawDataB[k] = (unsigned)pst_adpd->ptr_data_buffer[cnt+k];
#ifndef __USE_EOL_US_INT_SPACE__
						if( cnt > 0 ) gp_adpd_data->msec_eol_int_space = 0;
#else
						if( cnt > 0 ) gp_adpd_data->usec_eol_int_space = 0;
#endif
						stepEOL(NULL, uAgcRawDataB);
					}
					break;
				}
				else{
					//adpd143_mode_switching(pst_adpd, 0);
				}
			}
#endif


			for (cnt = 0; cnt < pst_adpd->fifo_size; cnt += 4) {


				input_event(pst_adpd->ptr_sample_inputdev, EV_REL,
					REL_Z,
					sub_mode + 1);
				ADPD143_dbg("adpd143 SLOT B(HRM mode) : rawDataCh1 : %d, normDataCh1 : %d\n",
					pst_adpd->ptr_data_buffer[cnt], 
					(pst_adpd->ptr_data_buffer[cnt]*256*SF_IR_CH/(pst_adpd->efuseIrSlope+128)/100)); 								
				ADPD143_dbg("adpd143 SLOT B(HRM mode) : rawDataCh2 : %d, normDataCh2 : %d\n",
					pst_adpd->ptr_data_buffer[cnt+1], 
					(pst_adpd->ptr_data_buffer[cnt+1]*256*SF_IR_CH/(pst_adpd->efuseIrSlope+128)/100)); 								
				ADPD143_dbg("adpd143 SLOT B(HRM mode) : rawDataCh3 : %d, normDataCh3 : %d\n",
					pst_adpd->ptr_data_buffer[cnt+2], 
					(pst_adpd->ptr_data_buffer[cnt+2]*256*SF_IR_CH/(pst_adpd->efuseIrSlope+128)/100)); 								
				ADPD143_dbg("adpd143 SLOT B(HRM mode) : rawDataCh4 : %d, normDataCh4 : %d\n",
					pst_adpd->ptr_data_buffer[cnt+3], 
					(pst_adpd->ptr_data_buffer[cnt+3]*256*SF_IR_CH/(pst_adpd->efuseIrSlope+128)/100)); 								

				input_event(pst_adpd->ptr_sample_inputdev, EV_MSC,
					MSC_RAW,
					(pst_adpd->ptr_data_buffer[cnt]*256*SF_IR_CH/(pst_adpd->efuseIrSlope+128)/100));
				input_event(pst_adpd->ptr_sample_inputdev, EV_MSC,
					MSC_RAW,
					(pst_adpd->ptr_data_buffer[cnt+1]*256*SF_IR_CH/(pst_adpd->efuseIrSlope+128)/100));
				input_event(pst_adpd->ptr_sample_inputdev, EV_MSC,
					MSC_RAW,
					(pst_adpd->ptr_data_buffer[cnt+2]*256*SF_IR_CH/(pst_adpd->efuseIrSlope+128)/100));
				input_event(pst_adpd->ptr_sample_inputdev, EV_MSC,
					MSC_RAW,
					(pst_adpd->ptr_data_buffer[cnt+3]*256*SF_IR_CH/(pst_adpd->efuseIrSlope+128)/100));

				input_sync(pst_adpd->ptr_sample_inputdev);
			}			
#ifdef __AGC_SUPPORT__
			if( pst_adpd->bOnOffAGC )
			{
				cnt -= 4;
				for(k=0; k<4; k++)
					uAgcRawDataB[k] = (unsigned)pst_adpd->ptr_data_buffer[cnt+k];
#ifdef __PROXIMITY_SUPPORT__
				checkObjProximity(NULL, uAgcRawDataB);
#endif
			}
#endif
			ADPD143_dbg("**********************************\n\n");
		}
#ifdef __TIA_ADC_SUPPORT__		
		else
		{

			for (cnt = 0; cnt < pst_adpd->fifo_size; cnt += 4) {
				unsigned int channel;
				
				input_event(pst_adpd->ptr_sample_inputdev, EV_REL,
					REL_Z,
					sub_mode + 1);

				if( Is_Float_Dark_Calibrated < FLOAT_MODE_DARK_CALIBRATION_CNT && Is_Float_Dark_Calibrated > 1){
					for(k=4; k<8; k++){
						if( rawDarkCh[k] < pst_adpd->ptr_data_buffer[cnt+k-4] || !rawDarkCh[k] )
							rawDarkCh[k] = pst_adpd->ptr_data_buffer[cnt+k-4];
					}
					Is_Float_Dark_Calibrated++;
					continue;
				}
				else if( Is_Float_Dark_Calibrated >= FLOAT_MODE_DARK_CALIBRATION_CNT && Is_Float_Dark_Calibrated < 100 ){ 			
						Is_Float_Dark_Calibrated = 100;
						continue;
				}
				else if( Is_Float_Dark_Calibrated < FLOAT_MODE_DARK_CALIBRATION_CNT ){
					Is_Float_Dark_Calibrated++;
					continue;
				}
				else if( Is_Float_Dark_Calibrated == 100 ){
					reg_write(pst_adpd, 0x10,0x0001);
					reg_write(pst_adpd, 0x14,0x0441);
					reg_write(pst_adpd, 0x10,0x0002);
					Is_Float_Dark_Calibrated++;
				}

				for(channel = 4; channel < 8; ++channel){
					rawDataCh[channel] = pst_adpd->ptr_data_buffer[cnt+channel] - rawDarkCh[channel];
					rawDataCh[channel] = ((int)rawDataCh[channel] < 0)? 0 : rawDataCh[channel];
					
					if( (int)rawDataCh[channel] >= (FLOAT_MODE_SATURATION_CONTROL_CODE-rawDarkCh[channel]) )
						rawDataCh[channel] = FLOAT_MODE_SATURATION_CONTROL_CODE-rawDarkCh[channel];
				}

				input_event(pst_adpd->ptr_sample_inputdev, EV_MSC,
					MSC_RAW, rawDataCh[4] );
				input_event(pst_adpd->ptr_sample_inputdev, EV_MSC,
					MSC_RAW, rawDataCh[5] );
				input_event(pst_adpd->ptr_sample_inputdev, EV_MSC,
					MSC_RAW, rawDataCh[6] );
				uncoated_ch3 = rawDataCh[6];
				input_event(pst_adpd->ptr_sample_inputdev, EV_MSC,
					MSC_RAW, rawDataCh[7]);
				coated_ch4 = rawDataCh[7];
				ADPD143_dbg("adpd143 SLOT B(floating mode) uncoated_ch3:%d, coated_ch4=%d\n",
						uncoated_ch3, coated_ch4);			
				
				input_sync(pst_adpd->ptr_sample_inputdev);
			}
		}
#endif
		break;
	default:
		break;
	};
}

/**
This function is used for handling FIFO when interrupt occured
@param pst_adpd the ADPD143 data structure
@return void
*/
static void
adpd143_data_handler(struct adpd143_data *pst_adpd)
{
	unsigned short usr_mode = 0;
	unsigned short mode = 0;
	unsigned short sub_mode = 0;
	mutex_lock(&pst_adpd->mutex);
	usr_mode = atomic_read(&pst_adpd->adpd_mode);
	mode = GET_USR_MODE(usr_mode);
	sub_mode = GET_USR_SUB_MODE(usr_mode);

	ADPD143_info("mode - 0x%x,\t sub_mode - 0x%x\n", mode, sub_mode);
	adpd143_rd_intr_fifo(pst_adpd);

	adpd143_clr_intr_status(pst_adpd, mode);
#ifdef __EOL_SUPPORT__
	do_gettimeofday(&curr_interrupt_trigger_ts);
	if( prev_interrupt_trigger_ts.tv_sec > 0 )
	{
#ifndef __USE_EOL_US_INT_SPACE__
		pst_adpd->msec_eol_int_space = timeval_compare(&curr_interrupt_trigger_ts, &prev_interrupt_trigger_ts);
		if( pst_adpd->msec_eol_int_space - ((int)(pst_adpd->msec_eol_int_space/1000))*1000 >= 500 )
			pst_adpd->msec_eol_int_space = ((int)(pst_adpd->msec_eol_int_space/1000))+1;
		else
			pst_adpd->msec_eol_int_space = ((int)(pst_adpd->msec_eol_int_space/1000));
#else
		pst_adpd->usec_eol_int_space = timeval_compare(&curr_interrupt_trigger_ts, &prev_interrupt_trigger_ts);
#endif
	}
	else
#ifndef __USE_EOL_US_INT_SPACE__	
		pst_adpd->msec_eol_int_space = -1;
#else
		pst_adpd->usec_eol_int_space = -1;
#endif
	memcpy(&prev_interrupt_trigger_ts, &curr_interrupt_trigger_ts, sizeof(curr_interrupt_trigger_ts));
#endif

	switch (mode) {
	case IDLE_USR:
		ADPD143_dbg("IDLE_MODE\n");
		adpd143_rd_fifo_data(pst_adpd);
		break;
	case SAMPLE_USR:
		ADPD143_info("SAMPLE MODE\n");
		adpd143_rd_fifo_data(pst_adpd);
		adpd143_sample_event(pst_adpd);
		break;
#ifdef __TIA_ADC_SUPPORT__
	case TIA_ADC_USR:
		ADPD143_info("SAMPLE TIA ADC MODE\n");
		adpd143_rd_fifo_data(pst_adpd);
		adpd143_sample_event(pst_adpd);
		break;
#endif
#ifdef __EOL_SUPPORT__
	case EOL_USR:
		ADPD143_info("EOL SAMPLE MODE\n");
		adpd143_rd_fifo_data(pst_adpd);
		adpd143_sample_event(pst_adpd);
		break;
#endif
	default:
		ADPD143_info("DEFAULT MODE\n");
		adpd143_rd_fifo_data(pst_adpd);
		break;
	};

	mutex_unlock(&pst_adpd->mutex);
}

/**
This function is a handler for WorkQueue
@param ptr_work linux work structure
@return void
*/
static void
adpd143_wq_handler(struct work_struct *ptr_work)
{

	struct adpd143_data *pst_adpd = container_of(ptr_work,
					struct adpd143_data, work);

	struct timeval wkq_start;
	struct timeval wkq_comp;
	int diff_usec = 0;

	do_gettimeofday(&wkq_start);

	diff_usec = timeval_compare(&wkq_start,
			&pst_adpd->stats.stamp.interrupt_trigger);

	if (diff_usec > 1) {
		if (diff_usec >
			atomic_read(
				&pst_adpd->stats.wq_schedule_time_peak_usec))
			atomic_set(
				&pst_adpd->stats.wq_schedule_time_peak_usec,
				diff_usec);
		atomic_set(&pst_adpd->stats.wq_schedule_time_last_usec,
			   diff_usec);
		ewma_add2(&pst_adpd->stats.wq_schedule_time_avg_usec, diff_usec);
	}

	adpd143_data_handler(pst_adpd);

	do_gettimeofday(&wkq_comp);

	diff_usec = timeval_compare(&wkq_comp, &wkq_start);

	if (diff_usec > 1) {
		if (diff_usec >
		atomic_read(&pst_adpd->stats.data_process_time_peak_usec))
			atomic_set(
				&pst_adpd->stats.data_process_time_peak_usec,
				diff_usec);
		atomic_set(&pst_adpd->stats.data_process_time_last_usec,
			   diff_usec);
		ewma_add2(&pst_adpd->stats.data_process_time_avg_usec,
			 diff_usec);
	}

}

/**
This function is used for handling Interrupt from ADPD143
@param irq is Interrupt number
@param dev_id is pointer point to ADPD143 data structure
@return irqreturn_t is a Interrupt flag
*/
static irqreturn_t
adpd143_isr_handler(int irq, void *dev_id)
{
	struct adpd143_data *pst_adpd = dev_id;

	atomic_inc(&pst_adpd->stats.interrupts);

	if (!work_pending(&pst_adpd->work)) {
		do_gettimeofday(&pst_adpd->stats.stamp.interrupt_trigger);
		ADPD143_info("%s\n", __func__);
		if (!queue_work(pst_adpd->ptr_adpd143_wq_st, &pst_adpd->work))
			atomic_inc(&pst_adpd->stats.wq_pending);
	} else {
		atomic_inc(&pst_adpd->stats.wq_pending);
		ADPD143_info("work_pending !!\n");
	}
	return IRQ_HANDLED;
}

/**
This function is used for updating the ADPD143 structure after configuration
@param pst_adpd the ADPD143 data structure
@return void
*/
static void
adpd143_update_config(struct adpd143_data *pst_adpd)
{
	return;
}

/**
This function is used for loading the configuration data to ADPD143 chip
0 - From file "/data/misc/adpd143_configuration.dcfg"
1 - From Static defined Array
@param pst_adpd the ADPD143 data structure
@param config configuration command
@return int status
*/
static int
adpd143_configuration(struct adpd143_data *pst_adpd, unsigned char config)
{
	struct adpd_platform_data *ptr_config;
	unsigned short addr;
	unsigned short data;
	unsigned short cnt = 0;
	int ret = 0;
	if (config == FRM_FILE) {
		ret = adpd143_read_config_file(pst_adpd);
		/*	 ADPD143_info("ARRAY_SIZE - %d\n", size);*/
	} else {
		ret = FRM_ARR;
	}
	if (ret == 0)
		ptr_config = pst_adpd->ptr_config;
	else
		ptr_config = &adpd143_config_data;

	for (cnt = 0; cnt < ptr_config->config_size; cnt++) {
		addr = (unsigned short) ((0xFFFF0000 &
					  ptr_config->config_data[cnt]) >> 16);
		data = (unsigned short) (0x0000FFFF &
					 ptr_config->config_data[cnt]);

		ADPD143_dbg("addr[0x%04x] = 0x%04x\n", addr, data);
		reg_write(pst_adpd, addr, data);
	}

	adpd143_update_config(pst_adpd);

	return 0;
}

#ifdef __TIA_ADC_SUPPORT__
static int
adpd143_TIA_ADC_configuration(struct adpd143_data *pst_adpd, unsigned char config)
{
	struct adpd_platform_data *ptr_config;
	unsigned short addr;
	unsigned short data;
	unsigned short cnt = 0;
	int ret = 0;
	if (config == FRM_FILE) {
		ret = adpd143_read_config_file(pst_adpd);
		/*	 ADPD143_info("ARRAY_SIZE - %d\n", size);*/
	} else {
		ret = FRM_ARR;
	}
	if (ret == 0)
		ptr_config = pst_adpd->ptr_config;
	else
		ptr_config = &adpd143_tia_adc_config_data;

	for (cnt = 0; cnt < ptr_config->config_size; cnt++) {
		addr = (unsigned short) ((0xFFFF0000 &
					  ptr_config->config_data[cnt]) >> 16);
		data = (unsigned short) (0x0000FFFF &
					 ptr_config->config_data[cnt]);

		ADPD143_dbg("addr[0x%04x] = 0x%04x\n", addr, data);
		reg_write(pst_adpd, addr, data);
	}

	adpd143_update_config(pst_adpd);

	return 0;
}
#endif


/**
This function clears all the statistic counters.
@param pst_adpd the ADPD143 data structure
@return void
*/
static void
adpd_stat_reset(struct adpd143_data *pst_adpd)
{
	atomic_set(&pst_adpd->stats.interrupts, 0);
	atomic_set(&pst_adpd->stats.wq_pending, 0);
	atomic_set(&pst_adpd->stats.wq_schedule_time_peak_usec, 0);
	atomic_set(&pst_adpd->stats.wq_schedule_time_last_usec, 0);
	atomic_set(&pst_adpd->stats.data_process_time_peak_usec, 0);
	atomic_set(&pst_adpd->stats.data_process_time_last_usec, 0);
	atomic_set(&pst_adpd->stats.fifo_requires_sync, 0);
	atomic_set(&pst_adpd->stats.fifo_bytes[0], 0);
	atomic_set(&pst_adpd->stats.fifo_bytes[1], 0);
	atomic_set(&pst_adpd->stats.fifo_bytes[2], 0);
	atomic_set(&pst_adpd->stats.fifo_bytes[3], 0);
	ewma_init2(&pst_adpd->stats.wq_schedule_time_avg_usec, 2048, 128);
	ewma_init2(&pst_adpd->stats.data_process_time_avg_usec, 2048, 128);
}

/* SAMPLE - SYSFS ATTRIBUTE*/
/**
This function is used for getting the status of the sample enable bit
@param dev linux device structure
@param attr pointer point linux device_attribute structure
@param buf pointer point the buffer
@return ssize_t size of data presnt in the buffer
*/
static ssize_t
sample_attr_get_enable(struct device *dev,
			   struct device_attribute *attr, char *buf)
{
	struct adpd143_data *pst_adpd = dev_get_drvdata(dev);
	int val = atomic_read(&pst_adpd->sample_enabled);
	return sprintf(buf, "%d\n", val);
}

/**
This function is used for enabling the Sample mode
@param dev linux device structure
@param attr pointer point linux device_attribute structure
@param buf pointer point the buffer
@param size buffer size
@return ssize_t size of data written to the buffer
*/
static ssize_t
sample_attr_set_enable(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t size)
{
	struct adpd143_data *pst_adpd = dev_get_drvdata(dev);
	unsigned short parse_data[2];
	unsigned short mode = 0;

	int val;
	int n = sscanf(buf, "%d", &val);
	(void)n;
	memset(parse_data, 0, sizeof(parse_data));

	if (val == 1) {
		pr_info("adpd143_%s_enable HRM.\n", __func__);
		cmd_parsing("0x31", 1, parse_data);
		atomic_set(&pst_adpd->sample_enabled, 1);
#ifdef __TIA_ADC_SUPPORT__
	} else if (val == 2) {
		pr_info("adpd143_%s_enable Ambient Light Measurement.\n", __func__);
		cmd_parsing("0x51", 1, parse_data);
		atomic_set(&pst_adpd->sample_enabled, 0);
#endif			
	} else {
		pr_info("adpd143_%s_disable.\n", __func__);
		cmd_parsing("0x0", 1, parse_data);
		atomic_set(&pst_adpd->sample_enabled, 0);
	}
	mode = GET_USR_MODE(parse_data[0]);

	if (GET_USR_MODE(parse_data[0]) < MAX_MODE) {
		if ((GET_USR_SUB_MODE(parse_data[0])) <
			__mode_recv_frm_usr[mode].size) {
			adpd143_mode_switching(pst_adpd, parse_data[0]);
		} else {
			ADPD143_dbg("Sub mode Out of bound\n");
			adpd143_mode_switching(pst_adpd, 0);
		}
	} else {
		ADPD143_dbg("Mode out of bound\n");
		adpd143_mode_switching(pst_adpd, 0);
	}
	return size;
}

/* GENERAL - SYSFS ATTRIBUTE*/
/**
This function is used for getting the status of the adpd_mode
@param dev linux device structure
@param attr pointer point linux device_attribute structure
@param buf pointer point the buffer
@return ssize_t size of data presnt in the buffer
*/
static ssize_t
attr_get_mode(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct adpd143_data *pst_adpd = dev_get_drvdata(dev);
	int val = atomic_read(&pst_adpd->adpd_mode);
	return sprintf(buf, "%d\n", val);
}

/**
This function is used for switching ADPD143 mode
@param dev linux device structure
@param attr pointer point linux device_attribute structure
@param buf pointer point the buffer
@param size buffer size
@return ssize_t size of data written to the buffer
*/
static ssize_t
attr_set_mode(struct device *dev, struct device_attribute *attr,
		  const char *buf, size_t size)
{
	struct adpd143_data *pst_adpd = dev_get_drvdata(dev);
	unsigned short parse_data[2];
	unsigned short mode = 0;

	memset(parse_data, 0, sizeof(parse_data));
	cmd_parsing(buf, 1, parse_data);

	ADPD143_info("Mode requested 0x%02x\n", parse_data[0]);

	mode = GET_USR_MODE(parse_data[0]);

	if (GET_USR_MODE(parse_data[0]) < MAX_MODE) {
		if ((GET_USR_SUB_MODE(parse_data[0])) <
				__mode_recv_frm_usr[mode].size) {
			adpd143_mode_switching(pst_adpd, parse_data[0]);
		} else {
			ADPD143_dbg("Sub mode Out of bound\n");
			adpd143_mode_switching(pst_adpd, 0);
		}
	} else {
		ADPD143_dbg("Mode out of bound\n");
		adpd143_mode_switching(pst_adpd, 0);
	}

	return size;
}

/**
This function is used for reading the register value
@param dev linux device structure
@param attr pointer point linux device_attribute structure
@param buf pointer point the buffer
@return ssize_t size of data presnt in the buffer
*/
static ssize_t
attr_reg_read_get(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct adpd143_data *pst_adpd = dev_get_drvdata(dev);
	ADPD143_dbg("Regval: 0x%4x\n", pst_adpd->sysfslastreadval);
	return sprintf(buf, "0x%04x\n", pst_adpd->sysfslastreadval);
}

/**
This function is used for writing the register for reading back
@param dev linux device structure
@param attr pointer point linux device_attribute structure
@param buf pointer point the buffer
@param size buffer size
@return ssize_t size of data written to the buffer
*/
static ssize_t
attr_reg_read_set(struct device *dev, struct device_attribute *attr,
		  const char *buf, size_t size)
{
	struct adpd143_data *pst_adpd = dev_get_drvdata(dev);
	unsigned short addr, cnt;
	unsigned short parse_data[4];
	unsigned short ret;

	memset(parse_data, 0, sizeof(unsigned short) * 4);
	cmd_parsing(buf, 2, parse_data);
	addr = parse_data[0];
	cnt = parse_data[1];

	mutex_lock(&pst_adpd->mutex);

	pst_adpd->sysfs_I2C_regaddr = addr;

	ret = adpd143_sysfs_I2C_read(pst_adpd);
	if (ret != (-1)) {
		ADPD143_dbg("RegRead_Store: addr = 0x%04X,value = 0x%04X\n",
				addr, ret);
		pst_adpd->sysfslastreadval = ret;
	} else {
		ADPD143_dbg("%s Error\n", __func__);
		pst_adpd->sysfslastreadval = (unsigned short) -1;
	}

	mutex_unlock(&pst_adpd->mutex);
	return size;
}

/**
This function is used for writing a particular data to the register
@param dev linux device structure
@param attr pointer point linux device_attribute structure
@param buf pointer point the buffer
@param size buffer size
@return ssize_t size of data written to the buffer
*/
static ssize_t
attr_reg_write_set(struct device *dev, struct device_attribute *attr,
		   const char *buf, size_t size)
{
	struct adpd143_data *pst_adpd = dev_get_drvdata(dev);
	unsigned short cnt;
	unsigned short parse_data[4];
	unsigned short ret;

	memset(parse_data, 0, sizeof(unsigned short) * 4);
	cmd_parsing(buf, 3, parse_data);
	if (parse_data[1] != 1) {
		ADPD143_dbg("few many argument!!\n");
		goto err_reg_write_argument;
	}

	pst_adpd->sysfs_I2C_regaddr = parse_data[0];
	cnt = parse_data[1];
	pst_adpd->sysfs_I2C_regval = parse_data[2];
	mutex_lock(&pst_adpd->mutex);
	ret = adpd143_sysfs_I2C_write(pst_adpd);
	if (ret == pst_adpd->sysfs_I2C_regval) {
		ADPD143_dbg("Reg[0x%04x] = 0x%04x\n",
				pst_adpd->sysfs_I2C_regaddr,
				pst_adpd->sysfs_I2C_regval);
	} else {
		ADPD143_dbg("Reg write error!!\n");
	}

	adpd143_update_config(pst_adpd);

	mutex_unlock(&pst_adpd->mutex);
err_reg_write_argument:
	return size;
}

/**
This function is used for getting the status of configuration
TBD
@param dev linux device structure
@param attr pointer point linux device_attribute structure
@param buf pointer point the buffer
@return ssize_t size of data presnt in the buffer
*/
static ssize_t
attr_config_get(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "status of config thread\n");
}

/**
This function is used for wrting the configuration value to the register
0 - write the configuration data present in file to ADPD143
1 - write the configuration data present in Array to ADPD143
@param dev linux device structure
@param attr pointer point linux device_attribute structure
@param buf pointer point the buffer
@param size buffer size
@return ssize_t size of data written to the buffer
*/
static ssize_t
attr_config_set(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct adpd143_data *pst_adpd = dev_get_drvdata(dev);
	unsigned short parse_data[1];
	memset(parse_data, 0, sizeof(unsigned short) * 1);
	cmd_parsing(buf, 1, parse_data);

	if (parse_data[0] == FRM_ARR)
		adpd143_configuration(pst_adpd, FRM_ARR);
	else if (parse_data[0] == FRM_FILE)
		adpd143_configuration(pst_adpd, FRM_FILE);
	else
		ADPD143_dbg("set 1 to config\n");
	return size;
}


/**
This function is used for getting the status of adpd143 and driver
@param dev linux device structure
@param attr pointer point linux device_attribute structure
@param buf pointer point the buffer
@return ssize_t size of data presnt in the buffer
*/
static ssize_t
attr_stat_get(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct adpd143_data *pst_adpd = dev_get_drvdata(dev);
	unsigned int interrupts = atomic_read(&pst_adpd->stats.interrupts);
	unsigned int wq_pending = atomic_read(&pst_adpd->stats.wq_pending);
	unsigned int wq_schedule_time_peak_usec =
		atomic_read(&pst_adpd->stats.wq_schedule_time_peak_usec);
	unsigned int wq_schedule_time_last_usec =
		atomic_read(&pst_adpd->stats.wq_schedule_time_last_usec);
	unsigned int data_process_time_peak_usec =
		atomic_read(&pst_adpd->stats.data_process_time_peak_usec);
	unsigned int data_process_time_last_usec =
		atomic_read(&pst_adpd->stats.data_process_time_last_usec);

	return sprintf(buf, "\
		interrupts					: %d\n\
		wq_pending					: %d\n\
		wq_schedule_time_peak_usec	: %d\n\
		wq_schedule_time_avg_usec	: %d\n\
		wq_schedule_time_last_usec	: %d\n\
		data_process_time_peak_usec : %d\n\
		data_process_time_avg_usec	: %d\n\
		data_process_time_last_usec : %d\n\
		fifo_requires_sync			: %d\n\
		fifo bytes history			: [%d %d %d %d]\n\
		ADPD143 driver version		: %s\n\
		ADPD143 Release date		: %s\n",
		interrupts, wq_pending,
		wq_schedule_time_peak_usec,
		(int)ewma_read(&pst_adpd->stats.wq_schedule_time_avg_usec),
		wq_schedule_time_last_usec,
		data_process_time_peak_usec,
		(int)ewma_read(&pst_adpd->stats.data_process_time_avg_usec),
		data_process_time_last_usec,
		atomic_read(&pst_adpd->stats.fifo_requires_sync),
		atomic_read(&pst_adpd->stats.fifo_bytes[0]),
		atomic_read(&pst_adpd->stats.fifo_bytes[1]),
		atomic_read(&pst_adpd->stats.fifo_bytes[2]),
		atomic_read(&pst_adpd->stats.fifo_bytes[3]),
		ADPD143_VERSION,
		ADPD143_RELEASE_DATE);
}

#define ADPD143_STAT_RESET	1

/**
This function is used for wrting the adpd stat value to zero
@param dev linux device structure
@param attr pointer point linux device_attribute structure
@param buf pointer point the buffer
@param size buffer size
@return ssize_t size of data written to the buffer
*/
static ssize_t
attr_stat_set(struct device *dev, struct device_attribute *attr,
		  const char *buf, size_t size)
{
	struct adpd143_data *pst_adpd = dev_get_drvdata(dev);
	unsigned short parse_data[1];

	memset(parse_data, 0, sizeof(unsigned short) * 1);
	cmd_parsing(buf, 1, parse_data);

	if (parse_data[0] == ADPD143_STAT_RESET) {
		ADPD143_dbg("Resetting statistics\n");
		adpd_stat_reset(pst_adpd);
	}

	return size;
}


/**
array of sample attributes
*/
static struct device_attribute sample_attributes[] = {
	__ATTR(enable, 0777, sample_attr_get_enable, sample_attr_set_enable),
};

/**
array of attributes
*/
static struct device_attribute attributes[] = {
	__ATTR(mode,
		   0777, attr_get_mode, attr_set_mode),
	__ATTR(reg_read,
		   0777, attr_reg_read_get, attr_reg_read_set),
	__ATTR(reg_write,
		   0777, NULL, attr_reg_write_set),
	__ATTR(configuration,
		   0777, attr_config_get, attr_config_set),
	__ATTR(stat,
		   0777, attr_stat_get, attr_stat_set),
};


/**
This function is used for creating sysfs attribute for sample
@param dev linux device structure
@return int status of attribute creation
*/
static int
create_sysfs_interfaces_sample(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(sample_attributes); i++)
		if (device_create_file(dev, sample_attributes + i))
			goto err_sysfs_interface_sample;
	return 0;
err_sysfs_interface_sample:
	for (; i >= 0; i--)
		device_remove_file(dev, sample_attributes + i);
	dev_err(dev, "%s:Unable to create interface\n", __func__);
	printk("%s:Unable to create interface\n", __func__);
	return -1;
}

/**
This function is used for removing sysfs attribute for sample
@param dev linux device structure
@return void
*/
static void
remove_sysfs_interfaces_sample(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(sample_attributes); i++)
		device_remove_file(dev, sample_attributes + i);
}

/**
This function is used for creating sysfs attribute
@param dev linux device structure
@return int status of attribute creation
*/
static int
create_sysfs_interfaces(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		if (device_create_file(dev, attributes + i))
			goto err_sysfs_interface;
	return 0;
err_sysfs_interface:
	for (; i >= 0; i--)
		device_remove_file(dev, attributes + i);
	dev_err(dev, "%s:Unable to create interface\n", __func__);
	printk("%s:Unable to create interface\n", __func__);
	return -1;
}

/**
This function is used for removing sysfs attribute
@param dev linux device structure
@return void
*/
static void
remove_sysfs_interfaces(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		device_remove_file(dev, attributes + i);
}

/*INPUT DEVICE NAME LIST*/
#define ADPD_INPUT_DEV_NAME_3   "adpd143_sample"

/**
This function is used for registering input event for Sample
@param pst_adpd pointer point ADPD143 data structure
@return s32 status of this function
*/
static s32
adpd143_input_init_sample(struct adpd143_data *pst_adpd)
{
	int err;
	pst_adpd->ptr_sample_inputdev = input_allocate_device();
	if (!pst_adpd->ptr_sample_inputdev) {
		err = -ENOMEM;
		dev_err(&pst_adpd->client->dev, "input dev allocation fail\n");
		goto err_sample_allocate;
	}

	pst_adpd->ptr_sample_inputdev->name = ADPD_INPUT_DEV_NAME_3;
	pst_adpd->ptr_sample_inputdev->id.bustype = BUS_I2C;
	pst_adpd->ptr_sample_inputdev->dev.parent = &pst_adpd->client->dev;
	input_set_drvdata(pst_adpd->ptr_sample_inputdev, pst_adpd);

	__set_bit(EV_MSC, pst_adpd->ptr_sample_inputdev->evbit);
	__set_bit(EV_REL, pst_adpd->ptr_sample_inputdev->evbit);
	__set_bit(MSC_RAW, pst_adpd->ptr_sample_inputdev->mscbit);
	__set_bit(REL_X, pst_adpd->ptr_sample_inputdev->relbit);
	__set_bit(REL_Y, pst_adpd->ptr_sample_inputdev->relbit);

	err = input_register_device(pst_adpd->ptr_sample_inputdev);
	if (err) {
		dev_err(&pst_adpd->client->dev,
			"unable to register input dev %s\n",
			pst_adpd->ptr_sample_inputdev->name);
		goto err_sample_register_failed;
	}
	return 0;
err_sample_register_failed:
	input_free_device(pst_adpd->ptr_sample_inputdev);
err_sample_allocate:
	return err;
}

/**
This function is used for unregistering input event for Sample
@param pst_adpd pointer point ADPD143 data structure
@return void
*/
static void
adpd143_input_cleanup_sample(struct adpd143_data *pst_adpd)
{
	input_set_drvdata(pst_adpd->ptr_sample_inputdev, NULL);
	input_unregister_device(pst_adpd->ptr_sample_inputdev);
	input_free_device(pst_adpd->ptr_sample_inputdev);
}

/**
This function is used for registering input event for ADPD143
@param pst_adpd pointer point ADPD143 data structure
@return s32 status of the function
*/
static s32
adpd143_input_init(struct adpd143_data *pst_adpd)
{
	return adpd143_input_init_sample(pst_adpd);
}

/**
This function is used for unregistering input event done for ADPD143
@param pst_adpd pointer point ADPD143 data structure
@return void
*/
static void
adpd143_input_cleanup(struct adpd143_data *pst_adpd)
{
	adpd143_input_cleanup_sample(pst_adpd);
}

/**
This function is used for registering sysfs attribute for ADPD143
@param pst_adpd pointer point ADPD143 data structure
@return s32 status of the called function
*/
static s32
adpd143_sysfs_init(struct adpd143_data *pst_adpd)
{
	/*ADPD_SENSOR class is created*/
	if (!pst_adpd->adpd_class) {
		pst_adpd->adpd_class = class_create(THIS_MODULE, "sensors");
		if (IS_ERR(pst_adpd->adpd_class)) {
			pst_adpd->adpd_class = NULL;
			return -1;
		}
	}

	 /*sensor attribute is created*/
	if (!pst_adpd->adpd_dev) {
		pst_adpd->adpd_dev = device_create(pst_adpd->adpd_class,
						   NULL,
						   0,
						   pst_adpd,
						   "%s",
						   "adpd143");
	}

	if (IS_ERR(pst_adpd->adpd_dev)) {
		/*return PTR_ERR(pst_adpd->adpd_dev);*/
		pst_adpd->adpd_dev = NULL;
		dev_err("Error in adpd143_sysfs_init()!!! sysfs device creation failed......\n");
		return -1;
	}

	if (create_sysfs_interfaces(pst_adpd->adpd_dev))
		goto err_sysfs_create_gen;
	
	if (create_sysfs_interfaces_sample(pst_adpd->adpd_dev))
		goto err_sysfs_create_sample;

	dev_set_drvdata(pst_adpd->adpd_dev, pst_adpd);

	return 0;
err_sysfs_create_sample:
	dev_err("Error in adpd143_sysfs_init()!!! create_sysfs_interfaces_sample() failed......\n");	
	remove_sysfs_interfaces_sample(pst_adpd->adpd_dev);
err_sysfs_create_gen:
	remove_sysfs_interfaces(pst_adpd->adpd_dev);
	dev_err("Error in adpd143_sysfs_init()!!! create_sysfs_interfaces() failed......\n");	
	return -1;
}

/**
This function is used for unregistering sysfs attribute for ADPD143
@param pst_adpd pointer point ADPD143 data structure
@return void
*/
static void
adpd143_sysfs_cleanup(struct adpd143_data *pst_adpd)
{
	remove_sysfs_interfaces(pst_adpd->adpd_dev);
	remove_sysfs_interfaces_sample(pst_adpd->adpd_dev);
	if (pst_adpd->adpd_class!= NULL) {
		dev_err("destroying device......\n");
		device_destroy(pst_adpd->adpd_class, 0);
		dev_err("destroying class......\n");
		class_destroy(pst_adpd->adpd_class);
		pst_adpd->adpd_class = NULL;
	}
}

/**
This function is used for assigning initial assignment value to
ADPD143 data structure
@param pst_adpd pointer point ADPD143 data structure
@return void
*/
static void
adpd143_struct_assign(struct adpd143_data *pst_adpd)
{
	pst_adpd->ptr_data_buffer = data_buffer;
}

/**
This function is used for initializing ADPD143
@param pst_adpd pointer point ADPD143 data structure
@param id pointer point i2c device id
@return s32 status of the called function
*/
static s32
adpd143_initialization(struct adpd143_data *pst_adpd,
			   const struct i2c_device_id *id)
{
	int err = 0;
	if (adpd143_input_init(pst_adpd)) {
		err = -1;
		goto err_input_init;
	}
	if (adpd143_sysfs_init(pst_adpd)) {
		err = -1;
		goto err_sysfs_init;
	}

	adpd143_struct_assign(pst_adpd);

	memset(&pst_adpd->stats, 0, sizeof(pst_adpd->stats));
	adpd_stat_reset(pst_adpd);

	INIT_WORK(&pst_adpd->work, adpd143_wq_handler);
	pst_adpd->ptr_adpd143_wq_st =
		create_workqueue("adpd143_wq");
/*		create_singlethread_workqueue("adpd143_wq");*/
	if (!pst_adpd->ptr_adpd143_wq_st) {
		err = -ENOMEM;
		goto err_wq_creation_init;
	}

	if (!pst_adpd->client->irq) {
		if (!pst_adpd->irq)
			goto err_work_queue_init;

	} else {
		pst_adpd->irq = pst_adpd->client->irq;
	}
	/* XXX,REVISIT:
	 * are we matching Samsung "K" platform's interrupt
	 * edge/level triggering?
	 */
	irq_set_irq_type(pst_adpd->irq, IRQ_TYPE_EDGE_RISING);
	err = request_irq(pst_adpd->irq, adpd143_isr_handler,
			IRQF_TRIGGER_RISING, dev_name(&pst_adpd->client->dev),
			pst_adpd);
	if (err) {
		ADPD143_dbg("irq %d busy?\n", pst_adpd->irq);
		goto err_work_queue_init;
	}
	disable_irq_nosync(pst_adpd->irq);

	pst_adpd->ptr_config = kzalloc(sizeof(struct adpd_platform_data),
					GFP_KERNEL);
	if (pst_adpd->ptr_config == NULL) {
		err = -ENOMEM;
		goto err_work_queue_init;
	}

	enable_irq(pst_adpd->irq);

	adpd143_configuration(pst_adpd, 1);

#ifdef __AGC_SUPPORT__
	pst_adpd->bOnOffAGC = 1;
#endif
#ifdef __PROXIMITY_SUPPORT__
	pst_adpd->st_obj_proximity = 0;
	setObjProximityThreshold(30000);
#endif

	return err;
err_work_queue_init:
	destroy_workqueue(pst_adpd->ptr_adpd143_wq_st);
err_wq_creation_init:

err_sysfs_init:
	adpd143_input_cleanup(pst_adpd);
err_input_init:
	return err;
}

/**
This function is used for cleanup ADPD143
@param pst_adpd pointer point ADPD143 data structure
@return void
*/
static void
adpd143_initialization_cleanup(struct adpd143_data *pst_adpd)
{
	adpd143_mode_switching(pst_adpd, 0);

	free_irq(pst_adpd->irq, pst_adpd);

	destroy_workqueue(pst_adpd->ptr_adpd143_wq_st);
	adpd143_sysfs_cleanup(pst_adpd);
	adpd143_input_cleanup(pst_adpd);
	kobject_uevent(&pst_adpd->client->dev.kobj, KOBJ_OFFLINE);
}

#ifdef CONFIG_PM
static s32
adpd143_i2c_suspend(struct device *dev)
{
	return 0;
}

static s32
adpd143_i2c_resume(struct device *dev)
{
	return 0;
}
#else
#define adpd143_i2c_resume  NULL
#define adpd143_i2c_suspend NULL
#endif              /* CONFIG_PM */


/**
This function is used for i2c read communication between ADPD143 and AP
@param pst_adpd pointer point ADPD143 data structure
@param reg_addr address need to be fetch
@param len number byte to be read
@param buf pointer point the read out data.
@return s32 status of the called function
*/
static int
adpd143_i2c_read(struct adpd143_data *pst_adpd, u8 reg_addr, int len,
		 u16 *buf)
{
	int err;
	int tries = 0;
	int icnt = 0;

	struct i2c_msg msgs[] = {
		{
			.addr = pst_adpd->client->addr,
			.flags = pst_adpd->client->flags & I2C_M_TEN,
			.len = 1,
			.buf = (s8 *)&reg_addr,
		},
		{
			.addr = pst_adpd->client->addr,
			.flags = (pst_adpd->client->flags & I2C_M_TEN) |
				I2C_M_RD,
			.len = len * sizeof(unsigned short),
			.buf = (s8 *)buf,
		},
	};

	do {
		err = i2c_transfer(pst_adpd->client->adapter, msgs, 2);
		if (err != 2)
			msleep_interruptible(I2C_RETRY_DELAY);
	} while ((err != 2) && (++tries < I2C_RETRIES));

	if (err != 2) {
		dev_err(&pst_adpd->client->dev, "read transfer error\n");
		err = -EIO;
	} else {
		err = 0;
	}

	for (icnt = 0; icnt < len; icnt++) {
		/*convert big endian to CPU format*/
		buf[icnt] = be16_to_cpu(buf[icnt]);
	}

	return err;
}

/**
This function is used for i2c write communication between ADPD143 and AP
@param pst_adpd pointer point ADPD143 data structure
@param reg_addr address need to be fetch
@param len number byte to be read
@param data value to be written on the register.
@return s32 status of the called function
*/
static int
adpd143_i2c_write(struct adpd143_data *pst_adpd, u8 reg_addr, int len,
		  u16 data)
{
	struct i2c_msg msgs[1];
	int err;
	int tries = 0;
	unsigned short data_to_write = cpu_to_be16(data);
	char buf[4];

	buf[0] = (s8) reg_addr;
	memcpy(buf + 1, &data_to_write, sizeof(unsigned short));
	msgs[0].addr = pst_adpd->client->addr;
	msgs[0].flags = pst_adpd->client->flags & I2C_M_TEN;
	msgs[0].len = 1 + (1 * sizeof(unsigned short));
	msgs[0].buf = buf;

	do {
		err = i2c_transfer(pst_adpd->client->adapter, msgs, 1);
		if (err != 1)
			msleep_interruptible(I2C_RETRY_DELAY);
	} while ((err != 1) && (++tries < I2C_RETRIES));

	if (err != 1) {
		dev_err(&pst_adpd->client->dev, "write transfer error\n");
		err = -EIO;
	} else {
		err = 0;
	}

	return err;
}

/**
This function is used for ADPD143 probe function
@param client pointer point to the linux i2c client structure
@param id pointer point to the linux i2c device id
@return s32 status of the probe function
*/
static s32
adpd_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct adpd143_data *pst_adpd = NULL;
	int err;
	unsigned short u16_regval = 0;
	int i=0;
	u16 for_p2p_reg_value, for_p2p_clk32K, for_p2p_clkfifo;

	ADPD143_dbg("%s\n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "client not i2c capable\n");
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}

#ifdef __AGC_SUPPORT__
	gp_adpd_data = pst_adpd = kzalloc(sizeof(struct adpd143_data), GFP_KERNEL);
#endif
	if (pst_adpd == NULL) {
		err = -ENOMEM;
		goto exit_mem_allocate_failed;
	}
	mutex_init(&pst_adpd->mutex);
	mutex_lock(&pst_adpd->mutex);

	pst_adpd->client = client;
	pst_adpd->read = adpd143_i2c_read;
	pst_adpd->write = adpd143_i2c_write;
	/*Need to allocate and assign data then use the below function */
	i2c_set_clientdata(client, (struct adpd143_data *)pst_adpd);
	/*chip ID verification */
	u16_regval = reg_read(pst_adpd, ADPD_CHIPID_ADDR);

	switch (u16_regval) {
	case ADPD_CHIPID(0):
	case ADPD_CHIPID(1):
	case ADPD_CHIPID(2):
	case ADPD_CHIPID(3):
	case ADPD_CHIPID(4):
		err = 0;
		ADPD143_dbg("chipID value = 0x%x\n", u16_regval);		
		for(i=0; i<8; i++) rawDarkCh[i] = rawDataCh[i] = 0;
	break;
	default:
		err = 1;
	break;
	};
	if (err) {
		ADPD143_dbg("chipID value = 0x%x\n", u16_regval);
		goto exit_chipid_verification;
	}
	ADPD143_info("chipID value = 0x%x\n", u16_regval);

	if (!client->irq) {
		dev_err(&client->dev, "no IRQ?\n");
		if (gpio_is_valid(gpio_num) && gpio_num) {
			if (gpio_request(gpio_num, "adpd_gpio"))
				goto exit_chipid_verification;
			if (gpio_direction_input(gpio_num))
				goto exit_gpio_irq_initialization;
			gpio_export(gpio_num, false);
			pst_adpd->irq = gpio_to_irq(gpio_num);
			ADPD143_dbg("GPIO - %d, IRQ - %d\n", gpio_num,
					pst_adpd->irq);
		} else {
			ADPD143_dbg("Failed Assigned GPIO no %d\n", gpio_num);
			/*goto error messgae */
		}
	} else {

	}

	/*Efuse read for DC normalization */	
	for_p2p_clk32K = reg_read(pst_adpd, 0x4B);
	reg_write(pst_adpd, 0x4B, for_p2p_clk32K | 0x80);
	for_p2p_clkfifo = reg_read(pst_adpd, 0x5F);
	reg_write(pst_adpd, 0x5F, for_p2p_clkfifo|1);
	reg_write(pst_adpd, 0x57, 0x7);
	do{
		for_p2p_reg_value = reg_read(pst_adpd, 0x67);
		ADPD143_dbg("******************* Reg0x67 : 0x%x ********************", for_p2p_reg_value);
	}while( (for_p2p_reg_value & 0x7) != 0x4 ); 
	if ((for_p2p_reg_value & 0x7) == 0x4)
	{
		pst_adpd->efuseRedSlope = reg_read(pst_adpd, 0x71);
		pst_adpd->efuseIrSlope = reg_read(pst_adpd, 0x72);
		pst_adpd->efuseRedIntrcpt = reg_read(pst_adpd, 0x73);
		pst_adpd->efuseIrIntrcpt = reg_read(pst_adpd, 0x74);
		pst_adpd->efuse32KfreqOffset = reg_read(pst_adpd, 0x77);//use efuse clock offset
		ADPD143_dbg("******************* efuseRedSlope : %d ********************", pst_adpd->efuseRedSlope);	
		ADPD143_dbg("******************* efuseIrSlope : %d ********************", pst_adpd->efuseIrSlope);	
		ADPD143_dbg("******************* efuseRedIntrcpt : %d ********************", pst_adpd->efuseRedIntrcpt);	
		ADPD143_dbg("******************* efuseIrIntrcpt : %d ********************", pst_adpd->efuseIrIntrcpt);	
		ADPD143_dbg("******************* efuse32KfreqOffset : %d ********************", pst_adpd->efuse32KfreqOffset);//use efuse clock offset	
	}
	reg_write(pst_adpd, 0x57, 0x0);
#if 1//use efuse clock offset
	for_p2p_clkfifo &= 0xFFC0;
	for_p2p_clkfifo |=((34 +(pst_adpd->efuse32KfreqOffset-0x80)/30) & 0x3F);
	reg_write(pst_adpd, 0x4B, for_p2p_clkfifo); 
#else
	reg_write(pst_adpd, 0x5F, for_p2p_clkfifo);
#endif

	mutex_unlock(&pst_adpd->mutex);

	adpd143_initialization(pst_adpd, id);

	kobject_uevent(&pst_adpd->client->dev.kobj, KOBJ_ONLINE);

	return 0;

exit_gpio_irq_initialization:
	if (gpio_is_valid(gpio_num) && gpio_num)
		gpio_free(gpio_num);
exit_chipid_verification:
	mutex_unlock(&pst_adpd->mutex);
	kfree(pst_adpd);
	i2c_set_clientdata(client, NULL);
exit_mem_allocate_failed:
	return -1;
exit_check_functionality_failed:
	dev_err(&client->dev, "%s: Driver Init failed\n", ADPD_DEV_NAME);
	return -1;
}

/**
This function is used for ADPD143 remove function
@param client pointer point to the linux i2c client structure
@return s32 status of the remove function
*/
static s32
adpd_i2c_remove(struct i2c_client *client)
{
	struct adpd143_data *pst_adpd = i2c_get_clientdata(client);
	ADPD143_dbg("%s\n", __func__);
	adpd143_initialization_cleanup(pst_adpd);

	cancel_work_sync(&pst_adpd->work);
	kfree(pst_adpd->ptr_config);
	kfree(pst_adpd);
	pst_adpd = NULL;

	if (!client->irq)
		gpio_free(gpio_num);

	i2c_set_clientdata(client, NULL);
	return 0;
}

#ifdef CONFIG_PM
/**
	device power management operation structure
*/
static const struct dev_pm_ops adpd_pm_ops = {
	.resume = adpd143_i2c_resume,
	.suspend = adpd143_i2c_suspend,
};
#endif

/**
This table tell which framework it supported
@brief the name has to get matched to the board configuration file setup
 */
static struct i2c_device_id adpd_id[] = {	{ADPD_DEV_NAME, 0},
						{} };

#ifdef CONFIG_OF
static struct of_device_id adpd_match_table[] = {
	{ .compatible = "ad,adpd143",},
	{ },
};
#else
#define adpd_match_table NULL
#endif

MODULE_DEVICE_TABLE(i2c, adpd_id);

/**
  i2c operation structure
*/
struct i2c_driver adpd143_i2c_driver = {
	.driver = {
		.name = ADPD_DEV_NAME,
		.owner = THIS_MODULE,
		 .of_match_table = adpd_match_table,
#ifdef CONFIG_PM
		.pm = &adpd_pm_ops,
#endif
	},
	.probe = adpd_i2c_probe,
	.remove = adpd_i2c_remove,
	.id_table = adpd_id,
};

#ifdef ADPD_AUTO_PROBE
/**
i2c_client structure, which hold the attached client detail,
*/
static struct i2c_client *i2c_client;
/**
This function is used to search i2c slave device connected to the
i2c-adapter for specific bus,
@param slave_name pointer hold the slave name,
@param slave_addrs pointer hold the slave address,
@brief cnt hold the number of Slave address to check
@return s32 status of the remove function

*/
static int
i2c_check_dev_attach(char *slave_name, unsigned short *slave_addrs,
			 unsigned short cnt)
{
	struct i2c_board_info info;
	struct i2c_adapter *i2c_adapter = NULL;
	int ret = 0;
	unsigned short *scan_device = NULL;
	unsigned short count = 0;

	/*need to check whether we need to free the memory */
	scan_device = kzalloc(sizeof(unsigned short) * (cnt + 2), GFP_KERNEL);
	if (IS_ERR(scan_device)) {
		ret = -ENOMEM;	/* out of memory */
		goto i2c_check_attach_mem_fail;
	}

	memset(scan_device, '\0', sizeof(unsigned short) * (cnt + 2));
	for (count = 0; count < cnt; count++) {
		*(scan_device + count) = *(slave_addrs + count);
		ADPD143_info("list of slave addr = 0x%x\n",
				 *(scan_device + count));
	}
	*(scan_device + count) = I2C_CLIENT_END;

	count = 0;

	do {
		i2c_adapter = i2c_get_adapter(count);
		if (i2c_adapter != NULL) {
			memset(&info, 0, sizeof(struct i2c_board_info));
			strlcpy(info.type, slave_name /*"adpd143" */ ,
				I2C_NAME_SIZE);
			/*need to check i2c_new_device instead of
			i2c_new_probed_device*/
			i2c_client =
				i2c_new_probed_device(i2c_adapter, &info,
							  (const unsigned short *)
							  scan_device, NULL);
			if (i2c_client != NULL) {
				ADPD143_dbg("I2C busnum - %d\n", count);
				ADPD143_dbg("dev attach to bus i2c-%d\n",
						count);
			} else {

			}
			i2c_put_adapter(i2c_adapter);
		} else {
			ADPD143_info("Not valid adapter\n");
		}
		count++;
	} while (i2c_client == NULL && count < 20);

	kfree(scan_device);

	if (i2c_client == NULL) {
		/*No such device or address */
		return -ENXIO;
	} else {
		return 0;
	}

i2c_check_attach_mem_fail:
	return ret;
}
#endif


/**
This function is get called when the module is inserted
@return inti status of the adpd143_multisensor_init
*/
static int __init
adpd143_multisensor_init(void)
{
#ifdef ADPD_AUTO_PROBE
	unsigned short addr[] = { ADPD143_SLAVE_ADDR };
	ADPD143_dbg("%s\n", __func__);
	if (!i2c_check_dev_attach(ADPD_DEV_NAME, addr, 1)) {
		return i2c_add_driver(&adpd143_i2c_driver);
	} else {
		pr_err("i2c bus connect error\n");
		return -1;
	}
#else
	ADPD143_dbg("%s\n", __func__);
	return i2c_add_driver(&adpd143_i2c_driver);
#endif
}

/**
This function is get called when the module is removed
@return void
*/
static void __exit
adpd143_multisensor_exit(void)
{
	ADPD143_dbg("%s\n", __func__);
	i2c_del_driver(&adpd143_i2c_driver);
#ifdef ADPD_AUTO_PROBE
	if (i2c_client)
		i2c_unregister_device(i2c_client);
#else
#endif
}

module_init(adpd143_multisensor_init);
module_exit(adpd143_multisensor_exit);

MODULE_DESCRIPTION("ADI ADPD143 HRM/ALM module device driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Joshua Yoon<joshua.yoon@analog.com>");
