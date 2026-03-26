/**
  ******************************************************************************
  * @file    ab-rtcmc-rtc.c
  * @brief   SPI Driver for the AB-RTCMC-32.768kHz-EOA9-S3 RTC
  * @note    This driver uses the STM32 HAL library for SPI communication.
  * @note    See the AB-RTCMC-32.768kHz-EOA9-S3 Application Manual for details.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "ab-rtcmc-rtc.h"

#include <string.h>

/* Private defines -----------------------------------------------------------*/
#define RTC_SPI_TIMEOUT     1000  /* Timeout for SPI operations in ms */
#define RTC_CS_SELECT()     HAL_GPIO_WritePin(RTC_CS_PORT, RTC_CS_PIN, GPIO_PIN_SET)
#define RTC_CS_DESELECT()   HAL_GPIO_WritePin(RTC_CS_PORT, RTC_CS_PIN, GPIO_PIN_RESET)

/* Private variables ---------------------------------------------------------*/
extern SPI_HandleTypeDef hspi2;

/* Private function prototypes -----------------------------------------------*/
static RTC_Status_t RTC_SPITransmit(uint8_t* data, uint16_t size);
static RTC_Status_t RTC_SPITransmitReceive(uint8_t* tx_data, uint8_t* rx_data, uint16_t size);

static RTC_Status_t RTC_WaitForEEPROMReady(uint32_t timeout_ms);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  RTCSPI transmit
  * @param  data: pointer to data buffer
  * @param  size: number of bytes to transmit
  * @retval RTC status
  */
static RTC_Status_t RTC_SPITransmit(uint8_t* data, uint16_t size) {
    if (HAL_SPI_Transmit(&hspi2, data, size, RTC_SPI_TIMEOUT) != HAL_OK) {
        return RTC_TIMEOUT;
    }
    return RTC_OK;
}

/**
  * @brief  RTC SPI transmit and receive
  * @param  tx_data: pointer to transmit data buffer
  * @param  rx_data: pointer to receive data buffer
  * @param  size: number of bytes to transfer
  * @retval RTC status
  */
static RTC_Status_t RTC_SPITransmitReceive(uint8_t* tx_data, uint8_t* rx_data, uint16_t size) {
    if (HAL_SPI_TransmitReceive(&hspi2, tx_data, rx_data, size, RTC_SPI_TIMEOUT) != HAL_OK) {
        return RTC_TIMEOUT;
    }
    return RTC_OK;
}

static RTC_Status_t RTC_WaitForEEPROMReady(uint32_t timeout_ms) {
    const uint32_t start_tick = HAL_GetTick();
    uint8_t status;

    while (1) {
        RTC_Status_t read_status = RTC_ReadRegister(RTC_REG_CONTROL_STATUS, &status);
        if (read_status != RTC_OK) {
            return read_status;
        }

        if ((status & RTC_CTRL_STATUS_EEBUSY) == 0) {
            return RTC_OK;
        }

        if ((HAL_GetTick() - start_tick) >= timeout_ms) {
            return RTC_EEPROM_BUSY;
        }

        HAL_Delay(1);
    }
}

/* Public functions ----------------------------------------------------------*/

/**
  * @brief  Initialize the RTC
  * @retval RTC status
  */
RTC_Status_t RTC_Init(void) {
    /* Ensure CS pin is Low */
    RTC_CS_DESELECT();
    
    HAL_Delay(10);

    /* Enable write operations */
    uint8_t ctrl1 = RTC_CTRL1_WE;
    if (RTC_WriteRegister(RTC_REG_CONTROL_1, ctrl1) != RTC_OK) {
        return RTC_ERROR;
    }

    /* Clear any existing flags */
    RTC_ClearAllFlags();

    /* Configure interrupt control register (disable all interrupts initially) */
    uint8_t ctrl_int = 0x00;
    if (RTC_WriteRegister(RTC_REG_CONTROL_INT, ctrl_int) != RTC_OK) {
        return RTC_ERROR;
    }

    /* Set CLKOUT frequency to 1Hz using EEPROM Control register */
    uint8_t eeprom_ctrl;
    RTC_Status_t status = RTC_ReadEEPROM(RTC_REG_EEPROM_CONTROL, &eeprom_ctrl);
    if (status == RTC_OK) {
        /* Clear existing FD0 and FD1 bits */
        eeprom_ctrl &= ~(RTC_EEPROM_CTRL_FD0 | RTC_EEPROM_CTRL_FD1);
        /* Set FD1=1, FD0=1 for 1Hz */
        eeprom_ctrl |= RTC_EEPROM_CTRL_FD0;
        eeprom_ctrl |= RTC_EEPROM_CTRL_FD1;
        /* Enable temperature compensation for accuracy */
        eeprom_ctrl &= ~RTC_EEPROM_CTRL_THP; // ThP=0 for 1s scanning interval
        eeprom_ctrl |= RTC_EEPROM_CTRL_THE;  // ThE=1 to enable thermometer

        status = RTC_WriteEEPROM(RTC_REG_EEPROM_CONTROL, eeprom_ctrl);
        if (status != RTC_OK) {
            return status;
        }

        status = RTC_WaitForEEPROMReady(200);
        if (status != RTC_OK) {
            return status;
        }

        /* Enable clock output */
        status = RTC_EnableClockOutput(true);
        if (status != RTC_OK) {
            return status;
        }

        HAL_Delay(100); /* Wait for CLKOUT to stabilize with new frequency */
    } else {
        return status;
    }

    return RTC_OK;
}

/**
  * @brief  Deinitialize the RTC
  * @retval RTC status
  */
RTC_Status_t RTC_DeInit(void) {
    /* Disable clock output */
    RTC_EnableClockOutput(false);

    /* Disable alarms */
    RTC_EnableAlarm(false);

    /* Deselect CS pin */
    RTC_CS_DESELECT();
    
    return RTC_OK;
}

/**
  * @brief  Write to RTC register
  * @param  reg: register address
  * @param  data: data to write
  * @retval RTC status
  */
RTC_Status_t RTC_WriteRegister(uint8_t reg, uint8_t data) {
    uint8_t tx_data[2];

    tx_data[0] = RTC_CMD_WRITE | reg; /* Mask the register address with write command: 0000 0000 */
    tx_data[1] = data;                /* Data byte to write */

    RTC_CS_SELECT();
    RTC_Status_t status = RTC_SPITransmit(tx_data, 2);
    RTC_CS_DESELECT();
    
    return status;
}

/**
  * @brief  Read from RTC register
  * @param  reg: register address
  * @param  data: pointer to store read data
  * @retval RTC status
  */
RTC_Status_t RTC_ReadRegister(uint8_t reg, uint8_t* data) {
    uint8_t tx_data[2];
    uint8_t rx_data[2];
    
    if (data == NULL) {
        return RTC_INVALID_PARAM;       /* Invalid RAM address */
    }

    tx_data[0] = RTC_CMD_READ | reg;    /* Mask the register address with read command: 1000 0000 */
    tx_data[1] = 0x00;                  /* Dummy byte */
    
    RTC_CS_SELECT();
    RTC_Status_t status = RTC_SPITransmitReceive(tx_data, rx_data, 2);
    RTC_CS_DESELECT();
    
    if (status == RTC_OK) {
        *data = rx_data[1];
    }
    
    return status;
}

/**
  * @brief  Read multiple consecutive registers
  * @param  start_reg: starting register address
  * @param  buffer: buffer to store read data
  * @param  count: number of registers to read
  * @retval RTC status
  * @note   This function relies on the buffer being large enough to hold 'count' bytes.
  */
RTC_Status_t RTC_ReadMultipleRegisters(uint8_t start_reg, uint8_t* buffer, uint8_t count) {
    if (buffer == NULL || count == 0) {
        return RTC_INVALID_PARAM;
    }
    
    RTC_Status_t status = RTC_OK;
    
    for (uint8_t i = 0; i < count; i++) {
        status = RTC_ReadRegister(start_reg + i, &buffer[i]);
        if (status != RTC_OK) {
            break;
        }
    }
    
    return status;
}

/**
  * @brief  Set date and time
  * @param  datetime: pointer to datetime struct
  * @retval RTC status
  */
RTC_Status_t RTC_SetDateTime(RTC_DateTime_t* datetime) {
    /* NULL check */
    if (datetime == NULL) {
        return RTC_INVALID_PARAM;
    }

    /* Validate input parameters */
    if (datetime->seconds > 59 || datetime->minutes > 59 ||
        datetime->hours > 23 || datetime->weekdays > 7 || datetime->weekdays == 0 ||
        datetime->days > 31 || datetime->days == 0 ||
        datetime->months > 12 || datetime->months == 0 ||
        datetime->years > 99) {
        return RTC_INVALID_PARAM;
    }

    /* Sets the status to zero, use OR-Equals for the operations to accumulate any errors */
    RTC_Status_t status = RTC_OK;

    /* Enable write operations while preserving CLK_INT bit (for clock output) */
    uint8_t ctrl1;
    status = RTC_ReadRegister(RTC_REG_CONTROL_1, &ctrl1);
    if (status != RTC_OK) {
        return status;
    }
    ctrl1 |= RTC_CTRL1_WE;  /* Set WE bit, preserve other bits including CLK_INT */

    status = RTC_WriteRegister(RTC_REG_CONTROL_1, ctrl1);

    if (status != RTC_OK) {
        return status;
    }

    /* Set time registers */
    status |= RTC_WriteRegister(RTC_REG_SECONDS, RTC_Bin2BCD(datetime->seconds));
    status |= RTC_WriteRegister(RTC_REG_MINUTES, RTC_Bin2BCD(datetime->minutes));
    
    uint8_t hours = RTC_Bin2BCD(datetime->hours);

    status |= RTC_WriteRegister(RTC_REG_HOURS, hours);
    status |= RTC_WriteRegister(RTC_REG_DAYS, RTC_Bin2BCD(datetime->days));
    status |= RTC_WriteRegister(RTC_REG_WEEKDAYS, RTC_Bin2BCD(datetime->weekdays));
    status |= RTC_WriteRegister(RTC_REG_MONTHS, RTC_Bin2BCD(datetime->months));
    status |= RTC_WriteRegister(RTC_REG_YEARS, RTC_Bin2BCD(datetime->years));

    return status; /* Return accumulated status, non-zero (!= RTC_OK) indicates failure */
}

/**
  * @brief  Get date and time
  * @param  datetime: pointer to datetime struct
  * @retval RTC status
  */
RTC_Status_t RTC_GetDateTime(RTC_DateTime_t* datetime) {
if (datetime == NULL) {
    return RTC_INVALID_PARAM;
}

    /* byte buffer for register values */
    uint8_t regs[7];

    /* Read all the Clock Page registers */
    if (RTC_ReadMultipleRegisters(RTC_REG_SECONDS, regs, 7) != RTC_OK) {
        return RTC_ERROR;
    }

    /* Seconds and Minutes BCD encoding only uses bits 4-6 for the tens (0-5). 
        Filter out bit 7 with bitwise AND 0111 1111 */
    datetime->seconds = RTC_BCD2Bin(regs[0] & 0x7F);
    datetime->minutes = RTC_BCD2Bin(regs[1] & 0x7F);

    /* Handle hours with 12/24h format */
    uint8_t hours_reg = regs[2];

    /* TODO: Implement 12/24 hour format detection. For now, assume 24h format */
    datetime->is_12h_format = false;
    datetime->is_pm = false;
    datetime->hours = RTC_BCD2Bin(hours_reg & 0x3F);    /* 24-hour format, mask with 0011 1111 */

    datetime->days = RTC_BCD2Bin(regs[3] & 0x3F);        /* Day of month, mask with 0011 1111 (tens: 0-3) and (ones: 0-9)*/
    datetime->weekdays = RTC_BCD2Bin(regs[4] & 0x07);    /* Day of week,  mask with 0000 0111 (ones: 1-7) */
    datetime->months = RTC_BCD2Bin(regs[5] & 0x1F);      /* Month, mask with 0001 1111 (tens: 0-1) and (ones: 0-9) */
    datetime->years = RTC_BCD2Bin(regs[6] & 0x7F);       /* Year, mask with 0111 1111 (tens: 0-9) and (ones: 0-9) */
    
    return RTC_OK;
}

/**
  * @brief  Set alarm
  * @param  alarm: pointer to alarm struct
  * @retval RTC status
  */
RTC_Status_t RTC_SetAlarm(RTC_Alarm_t* alarm) {
    if (alarm == NULL) {
        return RTC_INVALID_PARAM;
    }

    /* Validate parameters */
    if (alarm->seconds > 59 || alarm->minutes > 59 || 
        alarm->hours > 23 || alarm->date > 31 || alarm->date == 0) {
        return RTC_INVALID_PARAM;
    }
    
    RTC_Status_t status = RTC_OK;

    /* Set basic alarm registers (seconds, minutes, hours, days) */
    /* Enable alarms by default */
    status |= RTC_WriteRegister(RTC_REG_ALARM_SECONDS, RTC_Bin2BCD(alarm->seconds) | RTC_ALARM_ENABLE);
    status |= RTC_WriteRegister(RTC_REG_ALARM_MINUTES, RTC_Bin2BCD(alarm->minutes) | RTC_ALARM_ENABLE);
    status |= RTC_WriteRegister(RTC_REG_ALARM_HOURS, RTC_Bin2BCD(alarm->hours) | RTC_ALARM_ENABLE);
    status |= RTC_WriteRegister(RTC_REG_ALARM_DAYS, RTC_Bin2BCD(alarm->date) | RTC_ALARM_ENABLE);
    
    return status;
}

/**
  * @brief  Get alarm settings
  * @param  alarm: pointer to alarm structure
  * @retval RTC status
  */
RTC_Status_t RTC_GetAlarm(RTC_Alarm_t* alarm) {
    if (alarm == NULL) {
        return RTC_INVALID_PARAM;
    }
    
    uint8_t regs[4];
    
    if (RTC_ReadMultipleRegisters(RTC_REG_ALARM_SECONDS, regs, 4) != RTC_OK) {
        return RTC_ERROR;
    }
    
    alarm->seconds = RTC_BCD2Bin(regs[0] & 0x7F);
    alarm->minutes = RTC_BCD2Bin(regs[1] & 0x7F);
    alarm->hours = RTC_BCD2Bin(regs[2] & 0x3F);
    alarm->date = RTC_BCD2Bin(regs[3] & 0x3F);

    /* Check if alarm interrupt is enabled */
    uint8_t ctrl_int;
    if (RTC_ReadRegister(RTC_REG_CONTROL_INT, &ctrl_int) == RTC_OK) {
        if (ctrl_int & RTC_CTRL_INT_AIE) {
            alarm->enabled = true;
        } else {
            alarm->enabled = false;
        }
    } else {
        alarm->enabled = false;
    }
    
    return RTC_OK;
}

/**
  * @brief  Enable or disable alarm
  * @param  enable: true to enable, false to disable
  * @retval RTC status
  */
RTC_Status_t RTC_EnableAlarm(bool enable) {
    uint8_t ctrl_int;
    
    if (RTC_ReadRegister(RTC_REG_CONTROL_INT, &ctrl_int) != RTC_OK) {
        return RTC_ERROR;
    }
    
    /* Configure alarm interrupt enable bit */
    if (enable) {
        /* Set the Alarm Interrupt Enable (AIE) bit */
        ctrl_int |= RTC_CTRL_INT_AIE;
    } else {
        /* Clear the Alarm Interrupt Enable (AIE) bit */
        ctrl_int &= (~RTC_CTRL_INT_AIE);
    }
    
    return RTC_WriteRegister(RTC_REG_CONTROL_INT, ctrl_int);
}

/**
  * @brief  Check if alarm was triggered
  * @retval true if alarm flag is set, false otherwise
  */
bool RTC_IsAlarmTriggered(void) {
    uint8_t ctrl_int_flag;
    
    if (RTC_ReadRegister(RTC_REG_CONTROL_INT_FLAG, &ctrl_int_flag) == RTC_OK) {
        if (ctrl_int_flag & RTC_CTRL_INT_FLAG_AF) {
            return true;
        } else {
            return false;
        }
    }
    
    return false;
}

/**
  * @brief  Check if timer was triggered
  * @retval true if timer flag is set, false otherwise
  */
bool RTC_IsTimerTriggered(void) {
    uint8_t ctrl_int_flag;
    
    if (RTC_ReadRegister(RTC_REG_CONTROL_INT_FLAG, &ctrl_int_flag) == RTC_OK) {
        if (ctrl_int_flag & RTC_CTRL_INT_FLAG_TF) {
            return true;
        } else {
            return false;
        }
    }
    
    return false;
}

/**
  * @brief  Clear alarm flag
  * @retval RTC status
  */
RTC_Status_t RTC_ClearAlarmFlag(void) {
    uint8_t ctrl_int_flag;
    
    if (RTC_ReadRegister(RTC_REG_CONTROL_INT_FLAG, &ctrl_int_flag) != RTC_OK) {
        return RTC_ERROR;
    }

    ctrl_int_flag &= ~RTC_CTRL_INT_FLAG_AF;  /* Clear alarm flag */

    return RTC_WriteRegister(RTC_REG_CONTROL_INT_FLAG, ctrl_int_flag);
}

/**
  * @brief  Clear timer flag
  * @retval RTC status
  */
RTC_Status_t RTC_ClearTimerFlag(void) {
    uint8_t ctrl_int_flag;
    
    if (RTC_ReadRegister(RTC_REG_CONTROL_INT_FLAG, &ctrl_int_flag) != RTC_OK) {
        return RTC_ERROR;
    }

    ctrl_int_flag &= ~RTC_CTRL_INT_FLAG_TF;  /* Clear timer flag */

    return RTC_WriteRegister(RTC_REG_CONTROL_INT_FLAG, ctrl_int_flag);
}

/**
  * @brief  Clear all interrupt flags
  * @retval RTC status
  */
RTC_Status_t RTC_ClearAllFlags(void) {
    /* Clear all interrupt flags */
    RTC_Status_t status = RTC_WriteRegister(RTC_REG_CONTROL_INT_FLAG, 0x00);
    
    return status;
}

/**
  * @brief  Enable or disable clock output
  * @param  enable: true to enable, false to disable
  * @retval RTC status
  */
RTC_Status_t RTC_EnableClockOutput(bool enable) {
    /* Control CLKOE pin (hardware enable) */
    if (enable) {
        HAL_GPIO_WritePin(RTC_CLKOE_PORT, RTC_CLKOE_PIN, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(RTC_CLKOE_PORT, RTC_CLKOE_PIN, GPIO_PIN_RESET);
    }
    
    /* Control CLK/INT bit in Control_1 register */
    uint8_t ctrl1;
    if (RTC_ReadRegister(RTC_REG_CONTROL_1, &ctrl1) != RTC_OK) {
        return RTC_ERROR;
    }
    
    if (enable) {
        ctrl1 |= RTC_CTRL1_CLK_INT;
    } else {
        ctrl1 &= ~RTC_CTRL1_CLK_INT;
    }
    
    return RTC_WriteRegister(RTC_REG_CONTROL_1, ctrl1);
}

/**
  * @brief  Clear status flags
  * @retval RTC status
  */
RTC_Status_t RTC_ClearFlags(void) {
    return RTC_ClearAllFlags();
}

/**
  * @brief  Set extended alarm with individual component enables
  * @param  alarm: pointer to extended alarm structure
  * @retval RTC status
  */
RTC_Status_t RTC_SetExtendedAlarm(RTC_ExtendedAlarm_t* alarm) {
    if (alarm == NULL) {
        return RTC_INVALID_PARAM;
    }

    /* Convert values to BCD and fill registers */
    uint8_t sec_reg = RTC_Bin2BCD(alarm->seconds);
    uint8_t min_reg = RTC_Bin2BCD(alarm->minutes);
    uint8_t hour_reg = RTC_Bin2BCD(alarm->hours);
    uint8_t day_reg = RTC_Bin2BCD(alarm->days);
    uint8_t month_reg = RTC_Bin2BCD(alarm->months);
    uint8_t weekday_reg = RTC_Bin2BCD(alarm->weekdays);
    uint8_t year_reg = RTC_Bin2BCD(alarm->years);

    RTC_Status_t status = RTC_OK;

    /* Set alarm-enable bits in the registers by OR= with masks */
    if (alarm->seconds_enable) { sec_reg |= RTC_ALARM_ENABLE; }
    if (alarm->minutes_enable) { min_reg |= RTC_ALARM_ENABLE; }
    if (alarm->hours_enable) { hour_reg |= RTC_ALARM_ENABLE; }
    if (alarm->days_enable) { day_reg |= RTC_ALARM_ENABLE; }
    if (alarm->weekdays_enable) { weekday_reg |= RTC_ALARM_ENABLE; }
    if (alarm->months_enable) { month_reg |= RTC_ALARM_ENABLE; }
    if (alarm->years_enable) { year_reg |= RTC_ALARM_ENABLE; }
    if (alarm->months_enable) { month_reg |= RTC_ALARM_ENABLE; }
    if (alarm->years_enable) { year_reg |= RTC_ALARM_ENABLE; }

    /* Write all alarm registers to the RTC */
    status |= RTC_WriteRegister(RTC_REG_ALARM_SECONDS, sec_reg);
    status |= RTC_WriteRegister(RTC_REG_ALARM_MINUTES, min_reg);
    status |= RTC_WriteRegister(RTC_REG_ALARM_HOURS, hour_reg);
    status |= RTC_WriteRegister(RTC_REG_ALARM_DAYS, day_reg);
    status |= RTC_WriteRegister(RTC_REG_ALARM_WEEKDAYS, weekday_reg);
    status |= RTC_WriteRegister(RTC_REG_ALARM_MONTHS, month_reg);
    status |= RTC_WriteRegister(RTC_REG_ALARM_YEARS, year_reg);
    
    return status;
}

/**
  * @brief  Get extended alarm settings
  * @param  alarm: pointer to extended alarm structure
  * @retval RTC status
  */
RTC_Status_t RTC_GetExtendedAlarm(RTC_ExtendedAlarm_t* alarm) {
    if (alarm == NULL) {
        return RTC_INVALID_PARAM;
    }
    
    uint8_t regs[7];
    
    /* Read all the alarm registers */
    if (RTC_ReadMultipleRegisters(RTC_REG_ALARM_SECONDS, regs, 7) != RTC_OK) {
        return RTC_ERROR;
    }
    
    /* Decode the BCD output and fill the alarm struct */
    alarm->seconds = RTC_BCD2Bin(regs[0] & 0x7F);
    alarm->minutes = RTC_BCD2Bin(regs[1] & 0x7F);
    alarm->hours = RTC_BCD2Bin(regs[2] & 0x3F);
    alarm->days = RTC_BCD2Bin(regs[3] & 0x3F);
    alarm->weekdays = RTC_BCD2Bin(regs[4] & 0x07);
    alarm->months = RTC_BCD2Bin(regs[5] & 0x1F);
    alarm->years = RTC_BCD2Bin(regs[6] & 0x7F);

    /* Check which components are enabled by checking the enable bit */
    alarm->seconds_enable = (regs[0] & RTC_ALARM_ENABLE) ? true : false;
    alarm->minutes_enable = (regs[1] & RTC_ALARM_ENABLE) ? true : false;
    alarm->hours_enable = (regs[2] & RTC_ALARM_ENABLE) ? true : false;
    alarm->days_enable = (regs[3] & RTC_ALARM_ENABLE) ? true : false;
    alarm->weekdays_enable = (regs[4] & RTC_ALARM_ENABLE) ? true : false;
    alarm->months_enable = (regs[5] & RTC_ALARM_ENABLE) ? true : false;
    alarm->years_enable = (regs[6] & RTC_ALARM_ENABLE) ? true : false;
    
    return RTC_OK;
}

/**
  * @brief  Set timer configuration
  * @param  timer: pointer to timer structure
  * @retval RTC status
  */
RTC_Status_t RTC_SetTimer(RTC_Timer_t* timer) {
    if (timer == NULL) {
        return RTC_INVALID_PARAM;
    }

    RTC_Status_t status = RTC_OK;

    /* Set timer value (16-bit split into low and high bytes) */
    status |= RTC_WriteRegister(RTC_REG_TIMER_LOW, timer->timer_value & 0xFF);
    status |= RTC_WriteRegister(RTC_REG_TIMER_HIGH, (timer->timer_value >> 8) & 0xFF);

    /* Read current control register to preserve CLK_INT bit */
    uint8_t ctrl1;
    status = RTC_ReadRegister(RTC_REG_CONTROL_1, &ctrl1);
    if (status != RTC_OK) {
        return status;
    }

    /* Clear timer-related bits, preserve CLK_INT */
    ctrl1 &= ~(RTC_CTRL1_TE | RTC_CTRL1_TAR | 0x60);  /* Clear TE, TAR, TD1, TD0 */
    ctrl1 |= RTC_CTRL1_WE;  /* Always enable write */

    if (timer->enabled) {
        ctrl1 |= RTC_CTRL1_TE;
    }
    if (timer->auto_reload) {
        ctrl1 |= RTC_CTRL1_TAR;
    }

    /* Set timer division */
    ctrl1 |= (timer->division & 0x60);  /* grab TD1 and TD0 bits with mask: 0110 0000 */

    status |= RTC_WriteRegister(RTC_REG_CONTROL_1, ctrl1);

    return status;
}

/**
  * @brief  Get timer configuration
  * @param  timer: pointer to timer structure
  * @retval RTC status
  */
RTC_Status_t RTC_GetTimer(RTC_Timer_t* timer) {
    if (timer == NULL) {
        return RTC_INVALID_PARAM;
    }
    
    uint8_t timer_low, timer_high, ctrl1;
    RTC_Status_t status = RTC_OK;
    
    status |= RTC_ReadRegister(RTC_REG_TIMER_LOW, &timer_low);
    status |= RTC_ReadRegister(RTC_REG_TIMER_HIGH, &timer_high);
    status |= RTC_ReadRegister(RTC_REG_CONTROL_1, &ctrl1);
    
    if (status != RTC_OK) {
        return status;
    }
    
    timer->timer_value = (timer_high << 8) | timer_low;
    timer->enabled = (ctrl1 & RTC_CTRL1_TE) ? true : false;
    timer->auto_reload = (ctrl1 & RTC_CTRL1_TAR) ? true : false;
    timer->division = ctrl1 & 0x60;  /* grab TD1 and TD0 bits with mask: 0110 0000 */

    return RTC_OK;
}

/**
  * @brief  Enable or disable timer
  * @param  enable: true to enable, false to disable
  * @retval RTC status
  */
RTC_Status_t RTC_EnableTimer(bool enable) {
    uint8_t ctrl1;
    
    if (RTC_ReadRegister(RTC_REG_CONTROL_1, &ctrl1) != RTC_OK) {
        return RTC_ERROR;
    }
    
    if (enable) {
        ctrl1 |= RTC_CTRL1_TE;
    } else {
        ctrl1 &= ~RTC_CTRL1_TE;
    }
    
    return RTC_WriteRegister(RTC_REG_CONTROL_1, ctrl1);
}

/**
  * @brief  Set timer division (frequency) for CLKOUT
  * @param  division: Timer division setting (RTC_TIMER_DIV_xxxx)
  * @retval RTC status
  */
RTC_Status_t RTC_SetTimerDivision(uint8_t division) {
    uint8_t ctrl1;
    
    /* Read current control register */
    if (RTC_ReadRegister(RTC_REG_CONTROL_1, &ctrl1) != RTC_OK) {
        return RTC_ERROR;
    }
    
    /* Clear existing TD1 and TD0 bits (bits 6:5) */
    ctrl1 &= ~0x60;  /* Clear bits 6:5 (TD1:TD0) */
    
    /* Set new timer division */
    ctrl1 |= (division & 0x60);  /* Set TD1 and TD0 bits */
    
    /* Ensure write enable is set for any register writes DEBUG */
    ctrl1 |= RTC_CTRL1_WE;
    
    /* Write back the modified control register */
    return RTC_WriteRegister(RTC_REG_CONTROL_1, ctrl1);
}

/**
  * @brief  Enable or disable timer interrupt
  * @param  enable: true to enable, false to disable
  * @retval RTC status
  */
RTC_Status_t RTC_EnableTimerInterrupt(bool enable) {
    uint8_t ctrl_int;
    
    if (RTC_ReadRegister(RTC_REG_CONTROL_INT, &ctrl_int) != RTC_OK) {
        return RTC_ERROR;
    }
    
    if (enable) {
        ctrl_int |= RTC_CTRL_INT_TIE;
    } else {
        ctrl_int &= ~RTC_CTRL_INT_TIE;
    }
    
    return RTC_WriteRegister(RTC_REG_CONTROL_INT, ctrl_int);
}

/**
  * @brief  Get temperature reading
  * @param  temperature: pointer to store temperature (signed 8-bit, in Celsius)
  * @retval RTC status
  */
RTC_Status_t RTC_GetTemperature(int8_t* temperature) {
    if (temperature == NULL) {
        return RTC_INVALID_PARAM;
    }
    
    uint8_t temp_reg;
    if (RTC_ReadRegister(RTC_REG_TEMPERATURE, &temp_reg) != RTC_OK) {
        return RTC_ERROR;
    }
    
    /* 60 degree C offset */
    *temperature = (int8_t)(temp_reg - 60);
    
    return RTC_OK;
}

/**
  * @brief  Write to EEPROM
  * @param  address: EEPROM address (0x28-0x29 for user bytes)
  * @param  data: data to write
  * @retval RTC status
  */
RTC_Status_t RTC_WriteEEPROM(uint8_t address, uint8_t data) {
    /* Validate address */
    if (address < RTC_REG_EEPROM_USER1 || address > RTC_REG_XTAL_T0) {
        return RTC_INVALID_PARAM;
    }

    uint8_t ctrl1;

    /* Enable EEPROM refresh before writing */
    if (RTC_ReadRegister(RTC_REG_CONTROL_1, &ctrl1) != RTC_OK) {
        return RTC_ERROR;
    }

    /* Set Write-Enable and EEPROM Refresh Enable bits in the control register */
    ctrl1 |= RTC_CTRL1_EERE | RTC_CTRL1_WE;
    if (RTC_WriteRegister(RTC_REG_CONTROL_1, ctrl1) != RTC_OK) {
        return RTC_ERROR;
    }
    
    return RTC_WriteRegister(address, data);
}

/**
  * @brief  Read from EEPROM
  * @param  address: EEPROM address
  * @param  data: pointer to store read data
  * @retval RTC status
  */
RTC_Status_t RTC_ReadEEPROM(uint8_t address, uint8_t* data) {
    /* Validate address */
    if (address < RTC_REG_EEPROM_USER1 || address > RTC_REG_XTAL_T0) {
        return RTC_INVALID_PARAM;
    }
    
    return RTC_ReadRegister(address, data);
}

/**
  * @brief  Check if EEPROM is busy
  * @retval true if EEPROM is busy, false otherwise
  */
bool RTC_IsEEPROMBusy(void) {
    uint8_t status;
    
    if (RTC_ReadRegister(RTC_REG_CONTROL_STATUS, &status) == RTC_OK) {
        return (status & RTC_CTRL_STATUS_EEBUSY) ? true : false;
    }
    
    return false;
}

/**
  * @brief  Write to RAM
  * @param  address: RAM address (0x38-0x3F)
  * @param  data: data to write
  * @retval RTC status
  */
RTC_Status_t RTC_WriteRAM(uint8_t address, uint8_t data) {
    /* Validate address */
    if (address < RTC_REG_RAM_START || address > RTC_REG_RAM_END) {
        return RTC_INVALID_PARAM;
    }
    
    return RTC_WriteRegister(address, data);
}

/**
  * @brief  Read from RAM
  * @param  address: RAM address
  * @param  data: pointer to store read data
  * @retval RTC status
  */
RTC_Status_t RTC_ReadRAM(uint8_t address, uint8_t* data) {
    /* Validate address */
    if (address < RTC_REG_RAM_START || address > RTC_REG_RAM_END) {
        return RTC_INVALID_PARAM;
    }
    
    return RTC_ReadRegister(address, data);
}

/**
  * @brief  Check if power-on occurred
  * @retval true if power-on flag is set, false otherwise
  */
bool RTC_IsPowerOn(void) {
    uint8_t status;
    
    if (RTC_ReadRegister(RTC_REG_CONTROL_STATUS, &status) == RTC_OK) {
        return (status & RTC_CTRL_STATUS_PON) ? true : false;
    }
    
    return false;
}

/**
  * @brief  Check if voltage is low
  * @retval true if voltage flags are set, false otherwise
  */
bool RTC_IsVoltageLow(void) {
    uint8_t status;
    
    if (RTC_ReadRegister(RTC_REG_CONTROL_STATUS, &status) == RTC_OK) {
        if (status & (RTC_CTRL_STATUS_V1F | RTC_CTRL_STATUS_V2F)) {
            return true;
        }
    }
    
    return false;
}

/* Utility functions ---------------------------------------------------------*/

/**
  * @brief  Convert BCD to binary
  * @param  bcd: BCD value
  * @retval Binary value
  */
uint8_t RTC_BCD2Bin(uint8_t bcd) {
    /**
     * BCD format: 1111 1111
     * Upper nibble: tens (0-9)
     * Lower nibble: ones (0-9)
     * Example: 0x25 = 0010 0101 = 2*10 + 5 = 25
     * Masks: 0xF0 for tens, 0x0F for ones
     */

    uint8_t tens_digit = (bcd & 0xF0) >> 4;  /* Extract upper tens by right shifting the 4 lower bits out */
    uint8_t ones_digit = (bcd & 0x0F);       /* Extract lower ones */

    return (tens_digit * 10) + ones_digit;
}

/**
  * @brief  Convert binary to BCD (Binary Coded Decimal)
  * @param  bin: Binary value (0-99)
  * @retval BCD value in format: upper nibble = tens, lower nibble = ones
  * @note   Example: 25 decimal -> 0x25 BCD (0010 0101)
  */
uint8_t RTC_Bin2BCD(uint8_t bin) {
    uint8_t tens_place = bin / 10; /* Extract the tens place digit (0-9) */
    uint8_t ones_place = bin % 10;  /* Extract the ones place digit (0-9) */

    /* Pack digits into BCD format: tens in upper nibble, ones in lower nibble */
    return (tens_place << 4) | ones_place;
}

/**
  * @brief  Check if year is leap year
  * @param  year: year to check (full year, e.g., 2024)
  * @retval true if leap year, false otherwise
  */
bool RTC_IsLeapYear(uint16_t year) {
    return ((year % 4 == 0) && (year % 100 != 0)) || (year % 400 == 0);
}

/**
  * @brief  Get number of days in month
  * @param  month: month (1-12)
  * @param  year: year (full year)
  * @retval Number of days in month
  */
uint8_t RTC_GetDaysInMonth(uint8_t month, uint16_t year) {
    const uint8_t days_in_month[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

    if (month < 1 || month > 12) {
        return 0;
    }

    if (month == 2 && RTC_IsLeapYear(year)) {
        return 29;
    }

    return days_in_month[month - 1];
}

/**
  * @brief  Convert RTC date/time to GPS epoch seconds
  * @param  dt: Pointer to RTC_DateTime_t (years field is 0-99, meaning 2000-2099)
  * @retval Seconds since GPS epoch (Jan 6, 1980 00:00:00 UTC)
  */
uint32_t RTC_ToGPSEpoch(const RTC_DateTime_t *dt)
{
    static const uint16_t days_before_month[] = {
        0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334
    };

    uint16_t year = 2000 + dt->years;

    /* Days from Jan 1 1980 to Jan 1 of target year */
    uint32_t days = 0;
    for (uint16_t y = 1980; y < year; y++) {
        days += RTC_IsLeapYear(y) ? 366 : 365;
    }

    /* Add days for completed months in target year */
    if (dt->months >= 1 && dt->months <= 12) {
        days += days_before_month[dt->months - 1];
    }

    /* Add leap day if past Feb in a leap year */
    if (dt->months > 2 && RTC_IsLeapYear(year)) {
        days += 1;
    }

    /* Add day of month (1-based) */
    days += dt->days - 1;

    /* Subtract 5 days to shift from Jan 1 to Jan 6 */
    days -= 5;

    return days * 86400UL + dt->hours * 3600UL + dt->minutes * 60UL + dt->seconds;
}