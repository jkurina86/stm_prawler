/**
  ******************************************************************************
  * @file    rtc.h
  * @brief   SPI Driver for the AB-RTCMC-32.768kHz-EOA9-S3 RTC
  * @note    This driver uses the STM32 HAL library for SPI communication.
  * @note    See the AB-RTCMC-32.768kHz-EOA9-S3 Application Manual for details.
  ******************************************************************************
  */

#ifndef INC_AB_RTCMC_RTC_H_
#define INC_AB_RTCMC_RTC_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "spi.h"
#include <stdint.h>
#include <stdbool.h>

/* Private defines -----------------------------------------------------------*/
/* GPIO pins for RTC control */
#define RTC_CLKOE_PIN      GPIO_PIN_6
#define RTC_CLKOE_PORT     GPIOC
#define RTC_CLKOUT_PIN     GPIO_PIN_7
#define RTC_CLKOUT_PORT    GPIOC
#define RTC_CS_PIN         GPIO_PIN_12
#define RTC_CS_PORT        GPIOB

/* RTC Register addresses (Memory Pages) */
/* Control Page 00000 */
#define RTC_REG_CONTROL_1           0x00
#define RTC_REG_CONTROL_INT         0x01
#define RTC_REG_CONTROL_INT_FLAG    0x02
#define RTC_REG_CONTROL_STATUS      0x03
#define RTC_REG_CONTROL_RESET       0x04

/* Clock Page 00001 */
#define RTC_REG_SECONDS             0x08
#define RTC_REG_MINUTES             0x09
#define RTC_REG_HOURS               0x0A
#define RTC_REG_DAYS                0x0B
#define RTC_REG_WEEKDAYS            0x0C
#define RTC_REG_MONTHS              0x0D
#define RTC_REG_YEARS               0x0E

/* Alarm Page 00010 */
#define RTC_REG_ALARM_SECONDS       0x10
#define RTC_REG_ALARM_MINUTES       0x11
#define RTC_REG_ALARM_HOURS         0x12
#define RTC_REG_ALARM_DAYS          0x13
#define RTC_REG_ALARM_WEEKDAYS      0x14
#define RTC_REG_ALARM_MONTHS        0x15
#define RTC_REG_ALARM_YEARS         0x16

/* Timer Page 00011 */
#define RTC_REG_TIMER_LOW           0x18
#define RTC_REG_TIMER_HIGH          0x19

/* Temperature Page 00100 */
#define RTC_REG_TEMPERATURE         0x20

/* EEPROM User 00101 */
#define RTC_REG_EEPROM_USER1        0x28
#define RTC_REG_EEPROM_USER2        0x29

/* EEPROM Control Page 00110 */
#define RTC_REG_EEPROM_CONTROL      0x30
#define RTC_REG_XTAL_OFFSET         0x31
#define RTC_REG_XTAL_COEF           0x32
#define RTC_REG_XTAL_T0             0x33

/* RAM Page 00111 */
#define RTC_REG_RAM_START           0x38
#define RTC_REG_RAM_END             0x3F

/* SPI Commands (typical for SPI RTC - may need adjustment for AB-RTCMC) */
#define RTC_CMD_READ                0x80  /* Standard read command */
#define RTC_CMD_WRITE               0x00  /* Standard write command */

/* Alternative command formats to test */
#define RTC_CMD_READ_ALT1           0x00  /* Direct address read */
#define RTC_CMD_WRITE_ALT1          0x80  /* Alternative write command */

/* Control_1 register bits (0x00) */
#define RTC_CTRL1_WE                (1 << 0)  /* Write Enable 			00000001 */
#define RTC_CTRL1_TE                (1 << 1)  /* Timer Enable 			00000010 */
#define RTC_CTRL1_TAR               (1 << 2)  /* Timer Auto Reload 		00000100 */
#define RTC_CTRL1_EERE              (1 << 3)  /* EEPROM Refresh Enable	00001000 */
#define RTC_CTRL1_SRON              (1 << 4)  /* Self Recovery On		00010000 */
#define RTC_CTRL1_TD0               (1 << 5)  /* Timer Division 0		00100000 */
#define RTC_CTRL1_TD1               (1 << 6)  /* Timer Division 1		01000000 */
#define RTC_CTRL1_CLK_INT           (1 << 7)  /* Clock/Interrupt		10000000 */

/* Control_INT register bits (0x01) */
#define RTC_CTRL_INT_AIE            (1 << 0)  /* Alarm Interrupt Enable */
#define RTC_CTRL_INT_TIE            (1 << 1)  /* Timer Interrupt Enable */
#define RTC_CTRL_INT_V1IE           (1 << 2)  /* Voltage 1 Interrupt Enable */
#define RTC_CTRL_INT_V2IE           (1 << 3)  /* Voltage 2 Interrupt Enable */
#define RTC_CTRL_INT_SRIE           (1 << 4)  /* Self Recovery Interrupt Enable */

/* Control_INT_Flag register bits (0x02) */
#define RTC_CTRL_INT_FLAG_AF        (1 << 0)  /* Alarm Flag */
#define RTC_CTRL_INT_FLAG_TF        (1 << 1)  /* Timer Flag */
#define RTC_CTRL_INT_FLAG_V1IF      (1 << 2)  /* Voltage 1 Interrupt Flag */
#define RTC_CTRL_INT_FLAG_V2IF      (1 << 3)  /* Voltage 2 Interrupt Flag */
#define RTC_CTRL_INT_FLAG_SRF       (1 << 4)  /* Self Recovery Flag */

/* Control_Status register bits (0x03) */
#define RTC_CTRL_STATUS_V1F         (1 << 2)  /* Voltage 1 Flag */
#define RTC_CTRL_STATUS_V2F         (1 << 3)  /* Voltage 2 Flag */
#define RTC_CTRL_STATUS_SR          (1 << 4)  /* Self Recovery */
#define RTC_CTRL_STATUS_PON         (1 << 5)  /* Power On */
#define RTC_CTRL_STATUS_EEBUSY      (1 << 7)  /* EEPROM Busy */

/* Control_Reset register bits (0x04) */
#define RTC_CTRL_RESET_SYSR         (1 << 4)  /* System Reset */

/* Hours register bits (0x0A) */
#define RTC_HOURS_12_24             (1 << 6)  /* 12/24 hour format */
#define RTC_HOURS_PM                (1 << 5)  /* PM indicator (in 12h mode) */

/* Alarm Enable bits (bit 7 of each alarm register) */
#define RTC_ALARM_ENABLE            (1 << 7)  /* Alarm Enable bitmask */

/* Timer Division values */
#define RTC_TIMER_DIV_4096HZ        0x00      /* 4096 Hz */
#define RTC_TIMER_DIV_64HZ          0x20      /* 64 Hz (TD0=1) */
#define RTC_TIMER_DIV_1HZ           0x40      /* 1 Hz (TD1=1) */
#define RTC_TIMER_DIV_1_60HZ        0x60      /* 1/60 Hz (TD1=1, TD0=1) */

/* EEPROM Control register bits (0x30) */
#define RTC_EEPROM_CTRL_THP         (1 << 0)  /* Temperature High/Positive */
#define RTC_EEPROM_CTRL_THE         (1 << 1)  /* Temperature High Enable */
#define RTC_EEPROM_CTRL_FD0         (1 << 2)  /* Frequency Deviation 0 */
#define RTC_EEPROM_CTRL_FD1         (1 << 3)  /* Frequency Deviation 1 */
#define RTC_EEPROM_CTRL_R1K         (1 << 4)  /* 1k resistor */
#define RTC_EEPROM_CTRL_R5K         (1 << 5)  /* 5k resistor */
#define RTC_EEPROM_CTRL_R20K        (1 << 6)  /* 20k resistor */
#define RTC_EEPROM_CTRL_R80K        (1 << 7)  /* 80k resistor */

/* Data structures -----------------------------------------------------------*/
typedef struct {
    uint8_t seconds;    /* 0-59 */
    uint8_t minutes;    /* 0-59 */
    uint8_t hours;      /* 0-23 (24h) or 1-12 (12h) */
    uint8_t days;       /* 1-31 (day of month) */
    uint8_t weekdays;   /* 1-7 (1=Sunday, stored in BCD format) */
    uint8_t months;     /* 1-12 */
    uint8_t years;      /* 0-99 (20xx) */
    bool is_12h_format; /* true for 12h, false for 24h */
    bool is_pm;         /* true for PM in 12h format */
} RTC_DateTime_t;

typedef struct {
    uint8_t seconds;     /* 0-59 */
    uint8_t minutes;     /* 0-59 */
    uint8_t hours;       /* 0-23 (24h) or 1-12 (12h) */
    uint8_t days;        /* 1-31 (day of month) */
    uint8_t weekdays;    /* 1=Sunday to 7=Saturday (BCD format) */
    uint8_t months;      /* 1-12 */
    uint8_t years;       /* 0-99 (20xx) */
    bool seconds_enable; /* Enable seconds alarm */
    bool minutes_enable; /* Enable minutes alarm */
    bool hours_enable;   /* Enable hours alarm */
    bool days_enable;    /* Enable days alarm */
    bool weekdays_enable;/* Enable weekdays alarm */
    bool months_enable;  /* Enable months alarm */
    bool years_enable;   /* Enable years alarm */
} RTC_ExtendedAlarm_t;

typedef struct {
    uint8_t seconds;    /* 0-59 */
    uint8_t minutes;    /* 0-59 */
    uint8_t hours;      /* 0-23 (24h) or 1-12 (12h) */
    uint8_t date;       /* 1-31 */
    bool enabled;       /* Alarm enable/disable */
} RTC_Alarm_t;

typedef struct {
    uint16_t timer_value;  /* 16-bit timer value */
    uint8_t division;      /* Timer division setting */
    bool auto_reload;      /* Auto reload enable */
    bool enabled;          /* Timer enable */
} RTC_Timer_t;

typedef enum {
    RTC_OK = 0,            /* Given a value of zero for error accumulation */
    RTC_ERROR,
    RTC_TIMEOUT,
    RTC_INVALID_PARAM,
    RTC_EEPROM_BUSY
} RTC_Status_t;

/* Function prototypes -------------------------------------------------------*/
RTC_Status_t RTC_Init(void);
RTC_Status_t RTC_DeInit(void);

/* Basic time/date operations */
RTC_Status_t RTC_SetDateTime(RTC_DateTime_t* datetime);
RTC_Status_t RTC_GetDateTime(RTC_DateTime_t* datetime);

/* Alarm operations (basic) */
RTC_Status_t RTC_SetAlarm(RTC_Alarm_t* alarm);
RTC_Status_t RTC_GetAlarm(RTC_Alarm_t* alarm);
RTC_Status_t RTC_EnableAlarm(bool enable);

/* Extended alarm operations (full) */
RTC_Status_t RTC_SetExtendedAlarm(RTC_ExtendedAlarm_t* alarm);
RTC_Status_t RTC_GetExtendedAlarm(RTC_ExtendedAlarm_t* alarm);
RTC_Status_t RTC_EnableAlarmComponent(uint8_t component_mask);

/* Timer operations */
RTC_Status_t RTC_SetTimer(RTC_Timer_t* timer);
RTC_Status_t RTC_GetTimer(RTC_Timer_t* timer);
RTC_Status_t RTC_EnableTimer(bool enable);
RTC_Status_t RTC_EnableTimerInterrupt(bool enable);
RTC_Status_t RTC_SetTimerDivision(uint8_t division);

/* Interrupt and flag operations */
bool RTC_IsAlarmTriggered(void);
bool RTC_IsTimerTriggered(void);
RTC_Status_t RTC_ClearAlarmFlag(void);
RTC_Status_t RTC_ClearTimerFlag(void);
RTC_Status_t RTC_ClearAllFlags(void);
RTC_Status_t RTC_EnableInterrupt(uint8_t interrupt_mask);
RTC_Status_t RTC_DisableInterrupt(uint8_t interrupt_mask);

/* Temperature operations */
RTC_Status_t RTC_GetTemperature(int8_t* temperature);

/* EEPROM operations */
RTC_Status_t RTC_WriteEEPROM(uint8_t address, uint8_t data);
RTC_Status_t RTC_ReadEEPROM(uint8_t address, uint8_t* data);
bool RTC_IsEEPROMBusy(void);

/* Crystal calibration */
RTC_Status_t RTC_SetCrystalOffset(int8_t offset);
RTC_Status_t RTC_GetCrystalOffset(int8_t* offset);
RTC_Status_t RTC_SetCrystalCoef(uint8_t coef);
RTC_Status_t RTC_SetCrystalT0(uint8_t t0);

/* RAM operations */
RTC_Status_t RTC_WriteRAM(uint8_t address, uint8_t data);
RTC_Status_t RTC_ReadRAM(uint8_t address, uint8_t* data);
RTC_Status_t RTC_WriteRAMBuffer(uint8_t start_address, uint8_t* buffer, uint8_t length);
RTC_Status_t RTC_ReadRAMBuffer(uint8_t start_address, uint8_t* buffer, uint8_t length);

/* Clock output control (using CLK/INT functionality) */
RTC_Status_t RTC_EnableClockOutput(bool enable);

/* Status and control */
bool RTC_IsVoltageLow(void);
bool RTC_IsPowerOn(void);
RTC_Status_t RTC_ClearFlags(void);
RTC_Status_t RTC_SystemReset(void);

/* Register access */
RTC_Status_t RTC_WriteRegister(uint8_t reg, uint8_t data);
RTC_Status_t RTC_ReadRegister(uint8_t reg, uint8_t* data);
RTC_Status_t RTC_ReadMultipleRegisters(uint8_t start_reg, uint8_t* buffer, uint8_t count);

/* Utility functions */
uint8_t RTC_BCD2Bin(uint8_t bcd);
uint8_t RTC_Bin2BCD(uint8_t bin);
bool RTC_IsLeapYear(uint16_t year);
uint8_t RTC_GetDaysInMonth(uint8_t month, uint16_t year);

#ifdef __cplusplus
}
#endif

#endif /* INC_AB_RTCMC_RTC_H_ */
