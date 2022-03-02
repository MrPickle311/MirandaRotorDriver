#ifndef INC_ROTOR_DRIVER_H_
#define INC_ROTOR_DRIVER_H_

#include "main.h"
#include "stm32h7xx_hal.h"

static I2C_HandleTypeDef* rotor_i2c;
static uint8_t rotor_write_addr = 0b01010000;
static uint8_t rotor_read_addr = 0b01010001;
static uint16_t max_request_delay = HAL_MAX_DELAY;


void ROTOR_Setup(I2C_HandleTypeDef* i2c_bus, uint16_t request_delay)
{
    rotor_i2c = i2c_bus;
    max_request_delay = request_delay;
}

HAL_StatusTypeDef ROTOR_Reset()
{
    static const uint8_t command_byte = 0x01;

    uint8_t data[] = {command_byte};

    return HAL_I2C_Master_Transmit(rotor_i2c,
                                   rotor_write_addr,
                                   data,
                                   sizeof(data) / sizeof(uint8_t),
                                   max_request_delay);
}

enum ROTOR_CalibrationStatus
{
    CalibrationInProgress = 0x00,
    CalibrationCompletedSuccessfully = 0x01,
    CalibrationFailed = 0x02
};

HAL_StatusTypeDef
ROTOR_GetCalibrationState(enum ROTOR_CalibrationStatus* calibration_state)
{
    static const uint8_t command_byte = 0x02;

    if (calibration_state == NULL)
    {
        return HAL_ERROR;
    }

    return HAL_I2C_Mem_Read(rotor_i2c,
                            rotor_read_addr,
                            command_byte,
                            1,
                            calibration_state,
                            1,
                            max_request_delay);
}

enum ROTOR_MovementStatus
{
    Stopped = 0x00,
    MovingAnticlockwise = 0xFF,
    MovingClockwise = 0x01
};

HAL_StatusTypeDef
ROTOR_GetMovementState(enum ROTOR_MovementStatus* movement_status)
{
    static const uint8_t command_byte = 0x03;

    if (movement_status == NULL)
    {
        return HAL_ERROR;
    }

    return HAL_I2C_Mem_Read(rotor_i2c,
                            rotor_read_addr,
                            command_byte,
                            1,
                            movement_status,
                            1,
                            max_request_delay);
}

HAL_StatusTypeDef ROTOR_GetCurrentLocation(uint16_t* current_position)
{
    static const uint8_t command_byte = 0x04;

    if (current_position == NULL)
    {
        return HAL_ERROR;
    }

    return HAL_I2C_Mem_Read(rotor_i2c,
                            rotor_read_addr,
                            command_byte,
                            1,
                            (uint8_t*)current_position,
                            2,
                            max_request_delay);
}


HAL_StatusTypeDef ROTOR_GoToAbsoluteLocation(uint16_t new_absolute_position)
{
    static const uint8_t command_byte = 0x05;

    uint8_t data[] = {command_byte,
                      ((uint8_t*)new_absolute_position)[0],
                      ((uint8_t*)new_absolute_position)[1]};

    return HAL_I2C_Master_Transmit(
        rotor_i2c, rotor_write_addr, data, 3, max_request_delay);
}


HAL_StatusTypeDef ROTOR_GoToRelativeLocation(uint16_t relative_position)
{
    static const uint8_t command_byte = 0x06;

    uint8_t data[] = {command_byte,
                      ((uint8_t*)relative_position)[0],
                      ((uint8_t*)relative_position)[1]};

    return HAL_I2C_Master_Transmit(
        rotor_i2c, rotor_write_addr, data, 3, max_request_delay);
}

#endif /* INC_ROTOR_DRIVER_H_ */
