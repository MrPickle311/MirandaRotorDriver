#ifndef INC_SERIAL_DRIVER_H_
#define INC_SERIAL_DRIVER_H_

#include "main.h"
#include "rotor_driver.h"
#include "stm32h7xx_hal.h"

enum SERIAL_State
{
    WaitingForCommandIndicator,
    WaitingForIncomingCommand
};

enum SERIAL_CommandResponse
{
    SerialOk = 0x00,
    SerialError = 0x01
};

uint8_t serial_buffer[255];
uint8_t incoming_cmd_length = 0;
enum SERIAL_State serial_state = WaitingForCommandIndicator;

const uint8_t STX = 0x02;
const uint8_t ETX = 0x03;

static uint8_t ok_resp[] = {STX, (uint8_t)SerialOk, ETX};
static uint8_t err_resp[] = {STX, (uint8_t)SerialError, ETX};

// predefined responses

void SERIAL_Transmit(uint8_t* data, uint8_t size)
{
    HAL_UART_Transmit(&huart3, data, size, 300);
}

void SERIAL_WaitForBytes(uint8_t bytes_count)
{
    HAL_UART_Receive_IT(&huart3, serial_buffer, bytes_count);
}

void SERIAL_WaitForCommandIndicator()
{
    serial_state = WaitingForCommandIndicator;
    SERIAL_WaitForBytes(3);
}

void SERIAL_WaitForIncomingCommand()
{
    serial_state = WaitingForIncomingCommand;
    SERIAL_WaitForBytes(incoming_cmd_length);
}

void SERIAL_SendSimpleResponse(enum SERIAL_CommandResponse resp)
{
    if (resp == SerialOk)
    {
        SERIAL_Transmit(ok_resp, 3);
        return;
    }

    SERIAL_Transmit(err_resp, 3);
}

void SERIAL_ServiceCommandIndicator()
{
    uint8_t stx = serial_buffer[0];
    uint8_t ext = serial_buffer[2];

    if (stx == 0x02 && ext == 0x03)
    {
        incoming_cmd_length = serial_buffer[1];
        SERIAL_SendSimpleResponse(SerialOk);
        SERIAL_WaitForIncomingCommand();
    }
    else
    {
        SERIAL_SendSimpleResponse(SerialError);
        SERIAL_WaitForCommandIndicator();
    }
}

void SERIAL_TryResetRotor()
{
    HAL_StatusTypeDef status = HAL_OK; // ROTOR_Reset();

    if (status == HAL_OK)
    {
        SERIAL_SendSimpleResponse(SerialOk);
        return;
    }

    SERIAL_SendSimpleResponse(SerialError);
}

void SERIAL_TryGetCalibrationState()
{
    static uint8_t calibration_state_msg[] = {
        STX, (uint8_t)SerialOk, (uint8_t)CalibrationFailed, ETX};

    enum ROTOR_CalibrationStatus cal_status = CalibrationFailed;
    HAL_StatusTypeDef status = ROTOR_GetCalibrationState(&cal_status);

    calibration_state_msg[2] = (uint8_t)cal_status;

    if (status == HAL_OK)
    {
        SERIAL_Transmit(calibration_state_msg, 4);
        return;
    }

    SERIAL_SendSimpleResponse(SerialError);
}

void SERIAL_TryGetMovementState()
{

    static uint8_t move_state_msg[] = {
        STX, (uint8_t)SerialOk, (uint8_t)MovingClockwise, ETX};

    enum ROTOR_MovementStatus mov_status = MovingClockwise;
    HAL_StatusTypeDef status = ROTOR_GetMovementState(&mov_status);

    move_state_msg[2] = (uint8_t)mov_status;

    if (status == HAL_OK)
    {
        SERIAL_Transmit(move_state_msg, 4);
        return;
    }

    SERIAL_SendSimpleResponse(SerialError);
}

void SERIAL_TryGetCurrentLocation()
{
    static uint8_t curr_location_msg[] = {STX, (uint8_t)SerialOk, 0, 0, ETX};
    static uint8_t pos_bytes[] = {0, 0};

    HAL_StatusTypeDef status = ROTOR_GetCurrentLocation(pos_bytes);

    if (status == HAL_OK)
    {
        curr_location_msg[2] = pos_bytes[0];
        curr_location_msg[3] = pos_bytes[1];
        SERIAL_Transmit(curr_location_msg, 5);
        return;
    }

    SERIAL_SendSimpleResponse(SerialError);
}

void SERIAL_TrySendAbsolutePosition()
{
    HAL_StatusTypeDef status =
        ROTOR_GoToAbsoluteLocation(serial_buffer[2], serial_buffer[3]);

    if (status == HAL_OK)
    {
        SERIAL_SendSimpleResponse(SerialOk);
        return;
    }

    SERIAL_SendSimpleResponse(SerialError);
}

void SERIAL_TrySendRelativePosition()
{
    HAL_StatusTypeDef status = ROTOR_GoToRelativeLocation(
        serial_buffer[2], serial_buffer[3], serial_buffer[4]);

    if (status == HAL_OK)
    {
        SERIAL_SendSimpleResponse(SerialOk);
        return;
    }

    SERIAL_SendSimpleResponse(SerialError);
}

void SERIAL_ServiceIncomingCommand()
{
    uint8_t stx = serial_buffer[0];
    uint8_t ext = serial_buffer[incoming_cmd_length - 1];

    if (stx == 0x02 && ext == 0x03)
    {
        switch (serial_buffer[1])
        {
            case 0:
                SERIAL_TryResetRotor();
                break;
            case 1:
                SERIAL_TryGetCalibrationState();
                break;
            case 2:
                SERIAL_TryGetMovementState();
                break;
            case 3:
                SERIAL_TryGetCurrentLocation();
                break;
            case 4:
                SERIAL_TrySendAbsolutePosition();
                break;
            case 5:
                SERIAL_TrySendRelativePosition();
                break;
            default:
                HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET);
                SERIAL_SendSimpleResponse(SerialError);
                break;
        }
    }

    SERIAL_WaitForCommandIndicator();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
    if (serial_state == WaitingForCommandIndicator)
    {
        SERIAL_ServiceCommandIndicator();
    }
    else if (serial_state == WaitingForIncomingCommand)
    {
        SERIAL_ServiceIncomingCommand();
    }
}


#endif /* INC_SERIAL_DRIVER_H_ */
