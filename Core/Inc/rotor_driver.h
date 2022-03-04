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

    return HAL_I2C_Master_Transmit(
        rotor_i2c, 0x28 << 1, data, 1, max_request_delay);
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

void ROTOR_WaitForStop()
{
    enum ROTOR_MovementStatus status = MovingClockwise;
    while (status != Stopped)
    {
        ROTOR_GetMovementState(&status);
    }
}

HAL_StatusTypeDef ROTOR_GetCurrentLocation(uint8_t* current_position)
{
    static const uint8_t command_byte = 0x04;


    if (current_position == NULL)
    {
        return HAL_ERROR;
    }

    ROTOR_WaitForStop();

    return HAL_I2C_Mem_Read(rotor_i2c,
                            rotor_read_addr,
                            command_byte,
                            1,
                            current_position,
                            2,
                            max_request_delay);
}


HAL_StatusTypeDef ROTOR_GoToAbsoluteLocation(uint8_t msb_pos_byte,
                                             uint8_t lsb_pos_byte)
{
    static const uint8_t command_byte = 0x05;

    ROTOR_WaitForStop();

    uint8_t data[] = {command_byte, msb_pos_byte, lsb_pos_byte};

    return HAL_I2C_Master_Transmit(
        rotor_i2c, rotor_write_addr, data, 3, max_request_delay);
}

enum ROTOR_RotationDirection
{
    Clockwise = 0x00,
    Anticlockwise = 0x01
};

HAL_StatusTypeDef
ROTOR_GoToRelativeLocation(uint8_t msb_pos_byte,
                           uint8_t lsb_pos_byte,
						   enum ROTOR_RotationDirection direction)
{
    static const uint8_t command_byte = 0x40;

    ROTOR_WaitForStop();

    uint8_t data[] = {command_byte, direction, msb_pos_byte, lsb_pos_byte};

    return HAL_I2C_Master_Transmit(
        rotor_i2c, rotor_write_addr, data, 4, max_request_delay);
}

#endif /* INC_ROTOR_DRIVER_H_ */
