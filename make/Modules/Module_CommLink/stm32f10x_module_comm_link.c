/*******************************************************************************
THIS PROGRAM IS FREE SOFTWARE. YOU CAN REDISTRIBUTE IT AND/OR MODIFY IT
UNDER THE TERMS OF THE GNU GPLV3 AS PUBLISHED BY THE FREE SOFTWARE FOUNDATION.

Copyright (C), 2016-2016, Team MicroDynamics <microdynamics@126.com>

Filename:    stm32f10x_module_comm_link.c
Author:      myyerrol
Version:     none
Create date: 2017.04.28
Description: Declare the communication operation function
Others:      none
Function List:
             1.  void  CommLink_AddBits8ToBuffer(u8 byte);
             2.  void  CommLink_AddBits16ToBuffer(s16 bytes);
             3.  void  CommLink_AddDataToBuffer(u8 *data, u8 length);
             4.  void  CommLink_ConvertEndian(u8 *data, u8 length);
             5.  void  CommLink_HandleCommand(void);
             6.  void  CommLink_HandleDebugDataA(void);
             7.  void  CommLink_HandleDebugDataB(void);
             8.  void  CommLink_HandleDebugDataC(void);
             9.  void  CommLink_ProcessDataFromNRF(void);
             10. void  CommLink_ReceiveDataFromNRF(void);
             11. void  CommLink_ReadPacket(u8 byte);
             12. void  CommLink_Test(void);
             13. void  CommLink_WriteBuffer(void);
             14. void  CommLink_WriteDebugData(void);
             15. void  CommLink_WritePacket(u8 command);
             16. void  CommLink_WritePID(u8 pid_type);
             17. float CommLink_CutDBScaleToLinear(float x_start, float x_end,
                                                   float deadband);
History:
<author>    <date>        <desc>
myyerrol    2017.04.28    Modify the module
*******************************************************************************/

#include "stm32f10x_driver_delay.h"
#include "stm32f10x_driver_eeprom.h"
#include "stm32f10x_driver_usart.h"
#include "stm32f10x_module_battery.h"
#include "stm32f10x_module_comm_link.h"
#include "stm32f10x_module_led.h"
#include "stm32f10x_module_ms5611.h"
#include "stm32f10x_module_nrf24l01.h"
#include "stm32f10x_algorithm_altitude.h"
#include "stm32f10x_algorithm_imu.h"

u8   comm_link_mcu_state   = COMM_LINK_STATE_DISEN_MCU;
u16  comm_link_rc_data[4]  = {1500, 1500, 1500, 1500};
u32  comm_link_last_rc_timestamp;
bool comm_link_fly_enable_flag = false;
bool comm_link_pc_cmd_flag     = false;

static u8 recv_state      = COMM_LINK_STATE_IDLE;
static u8 recv_command    = 0;
static u8 recv_length     = 0;
static u8 recv_count      = 0;
static u8 recv_checksum   = 0;
static u8 send_count      = 0;
static u8 send_checksum   = 0;
static u8 recv_buffer[32] = {0};
static u8 send_buffer[64] = {0XAA, 0XAA, 0X01, 0X14, 0, 100, 0, 200, 0, 130,
                             0, 0, 0, 100, 0, 0, 0, 200, 0, 0, 0, 30, 0, 10,
                             0X6B};
static u8 test_buffer[6]  = {0XAA, 0XAF, 0X02, 0X01, 0X01,
                            (u8)(0XAA + 0XAF + 0X02 + 1 + 1)};

CommLink_Data CommLink_DataStructure;
// Take notice of the address alignment, don't initialize 'sum'.
CommLink_DataPacketA CommLink_DataPacketAStructure = {{0XAA, 0XAA}, 0X01, 18};
CommLink_DataPacketB CommLink_DataPacketBStructure = {{0XAA, 0XAA}, 0X02, 30,
                                                      {0}};

void CommLink_AddBits8ToBuffer(u8 byte)
{
    send_buffer[send_count++] = byte;
    send_checksum += byte;
}

void CommLink_AddBits16ToBuffer(s16 bytes)
{
    CommLink_AddBits8ToBuffer((u8)(bytes >> 8));
    CommLink_AddBits8ToBuffer((u8)(bytes & 0XFF));
}

void CommLink_AddDataToBuffer(u8 *data, u8 length)
{
    u8 i;

    for (i = 0; i < length; i++)
    {
        CommLink_AddBits8ToBuffer(data[i]);
    }
}

void CommLink_ConvertEndian(u8 *data, u8 length)
{
    u8 temp[8], i;

    for (i = 0; i < length; i++)
    {
        temp[i] = data[i];
    }

    for (i = 0; i < length; i++)
    {
        data[length - 1 - i] = temp[i];
    }
}

void CommLink_HandleCommand(void)
{
    switch (recv_command)
    {
        case COMM_LINK_STATE_PC_REQ_PID:
        {
            if (recv_buffer[0] == 0X01)
            {
                CommLink_WritePacket(COMM_LINK_STATE_PC_PID_ROLL);
                CommLink_WritePacket(COMM_LINK_STATE_PC_PID_PITCH);
                CommLink_WritePacket(COMM_LINK_STATE_PC_PID_YAW);
                CommLink_WritePacket(COMM_LINK_STATE_PC_PID_ALT);
            }
            break;
        }
        case COMM_LINK_STATE_PC_PID_PITCH:
        {
            pitch_rate_PID.P  = (s16)(recv_buffer[0]  << 8 | recv_buffer[1])  *
                0.01F;
            pitch_rate_PID.I  = (s16)(recv_buffer[2]  << 8 | recv_buffer[3])  *
                0.01F;
            pitch_rate_PID.D  = (s16)(recv_buffer[4]  << 8 | recv_buffer[5])  *
                0.01F;
            pitch_angle_PID.P = (s16)(recv_buffer[6]  << 8 | recv_buffer[7])  *
                0.01F;
            pitch_angle_PID.I = (s16)(recv_buffer[8]  << 8 | recv_buffer[9])  *
                0.01F;
            pitch_angle_PID.D = (s16)(recv_buffer[10] << 8 | recv_buffer[11]) *
                0.01F;
            CommLink_WritePID(COMM_LINK_STATE_PC_PID_PITCH);
            eeprom_params_request_flag = true;
            break;
        }
        case COMM_LINK_STATE_PC_PID_ROLL:
        {
            roll_rate_PID.P  = (s16)(recv_buffer[0]  << 8 | recv_buffer[1])  *
                0.01F;
            roll_rate_PID.I  = (s16)(recv_buffer[2]  << 8 | recv_buffer[3])  *
                0.01F;
            roll_rate_PID.D  = (s16)(recv_buffer[4]  << 8 | recv_buffer[5])  *
                0.01F;
            roll_angle_PID.P = (s16)(recv_buffer[6]  << 8 | recv_buffer[7])  *
                0.01F;
            roll_angle_PID.I = (s16)(recv_buffer[8]  << 8 | recv_buffer[9])  *
                0.01F;
            roll_angle_PID.D = (s16)(recv_buffer[10] << 8 | recv_buffer[11]) *
                0.01F;
            CommLink_WritePID(COMM_LINK_STATE_PC_PID_ROLL);
            eeprom_params_request_flag = true;
            break;
        }
        case COMM_LINK_STATE_PC_PID_YAW:
        {
            yaw_rate_PID.P  = (s16)(recv_buffer[0]  << 8 | recv_buffer[1])  *
                0.01F;
            yaw_rate_PID.I  = (s16)(recv_buffer[2]  << 8 | recv_buffer[3])  *
                0.01F;
            yaw_rate_PID.D  = (s16)(recv_buffer[4]  << 8 | recv_buffer[5])  *
                0.01F;
            yaw_angle_PID.P = (s16)(recv_buffer[6]  << 8 | recv_buffer[7])  *
                0.01F;
            yaw_angle_PID.I = (s16)(recv_buffer[8]  << 8 | recv_buffer[9])  *
                0.01F;
            yaw_angle_PID.D = (s16)(recv_buffer[10] << 8 | recv_buffer[11]) *
                0.01F;
            CommLink_WritePID(COMM_LINK_STATE_PC_PID_YAW);
            eeprom_params_request_flag = true;
            break;
        }
        case COMM_LINK_STATE_PC_PID_ALT:
        {
            alt_vel_PID.P = (s16)(recv_buffer[0]  << 8 | recv_buffer[1])  *
                0.01F;
            alt_vel_PID.I = (s16)(recv_buffer[2]  << 8 | recv_buffer[3])  *
                0.01F;
            alt_vel_PID.D = (s16)(recv_buffer[4]  << 8 | recv_buffer[5])  *
                0.01F;
            alt_PID.P     = (s16)(recv_buffer[6]  << 8 | recv_buffer[7])  *
                0.01F;
            alt_PID.I     = (s16)(recv_buffer[8]  << 8 | recv_buffer[9])  *
                0.01F;
            alt_PID.D     = (s16)(recv_buffer[10] << 8 | recv_buffer[11]) *
                0.01F;
            CommLink_WritePID(COMM_LINK_STATE_PC_PID_ALT);
            eeprom_params_request_flag = true;
            break;
        }
    }
}

void CommLink_HandleDebugDataA(void)
{
    CommLink_DataPacketAStructure.roll.value  = imu.roll  * 100;
    CommLink_DataPacketAStructure.pitch.value = imu.pitch * 100;
    CommLink_DataPacketAStructure.yaw.value   = imu.yaw   * 100;
    CommLink_DataPacketAStructure.temp.value  = ms5611_temperature * 100;
    CommLink_DataPacketAStructure.speed.value =
        Altitude_NEDFrameStructure.vel_z * 100;
    CommLink_DataPacketAStructure.alt.value   =
        Altitude_NEDFrameStructure.pos_z * 100;
    CommLink_DataPacketAStructure.pres.value  = ms5611_pressure;

#if COMM_LINK_CONVERT_ENDIAN
    CommLink_ConvertEndian(CommLink_DataPacketAStructure.roll.bytes,  2);
    CommLink_ConvertEndian(CommLink_DataPacketAStructure.pitch.bytes, 2);
    CommLink_ConvertEndian(CommLink_DataPacketAStructure.yaw.bytes,   2);
    CommLink_ConvertEndian(CommLink_DataPacketAStructure.temp.bytes,  2);
    CommLink_ConvertEndian(CommLink_DataPacketAStructure.speed.bytes, 2);
    CommLink_ConvertEndian(CommLink_DataPacketAStructure.alt.bytes,   4);
    CommLink_ConvertEndian(CommLink_DataPacketAStructure.pres.bytes,  4);
#endif

    u8 i;

     // Take notice of the memory alignment.
    for (i = 0; i < 14; i++)
    {
        send_buffer[i] = *((u8 *)(&CommLink_DataPacketAStructure) + i);
    }

    for (i = 14; i < 22; i++)
    {
        send_buffer[i] = *((u8 *)(&CommLink_DataPacketAStructure) + 2 + i);
    }

    send_buffer[22] = 0;

    for (i = 0; i < 22; i++)
    {
        send_buffer[22] += send_buffer[i];
    }

    USART_SendBuffer(send_buffer, 23);
}

void CommLink_HandleDebugDataB(void)
{
    send_checksum = 0;

    CommLink_AddBits8ToBuffer(0XAA);
    CommLink_AddBits8ToBuffer(0XAA);
    CommLink_AddBits8ToBuffer(0X02);
    CommLink_AddBits8ToBuffer(30);

    CommLink_AddBits16ToBuffer(imu.accb[0] * 1000);
    CommLink_AddBits16ToBuffer(imu.accb[1] * 1000);
    CommLink_AddBits16ToBuffer(imu.accb[2] * 1000);

    CommLink_AddBits16ToBuffer(imu.gyro[0] * 180.0F / M_PI * 100);
    CommLink_AddBits16ToBuffer(imu.gyro[1] * 180.0F / M_PI * 100);
    CommLink_AddBits16ToBuffer(imu.gyro[2] * 180.0F / M_PI * 100);

    CommLink_AddBits16ToBuffer(0);
    CommLink_AddBits16ToBuffer(0);
    CommLink_AddBits16ToBuffer(0);

    CommLink_AddBits16ToBuffer(imu.accRaw[0] * 1000);
    CommLink_AddBits16ToBuffer(imu.accRaw[1] * 1000);
    CommLink_AddBits16ToBuffer(imu.accRaw[2] * 1000);

    CommLink_AddBits16ToBuffer(imu.gyroRaw[0] * 180.0F / M_PI * 100);
    CommLink_AddBits16ToBuffer(imu.gyroRaw[1] * 180.0F / M_PI * 100);
    CommLink_AddBits16ToBuffer(imu.gyroRaw[2] * 180.0F / M_PI * 100);

    CommLink_AddBits8ToBuffer(send_checksum);

    CommLink_WriteBuffer();
}

void CommLink_HandleDebubDataC(void)
{
    u8 i;
    CommLink_DataPacketBStructure.cmd      = 0X08;
    CommLink_DataPacketBStructure.len      = 6 * 2;
    CommLink_DataPacketBStructure.data[0]  = 0;
    CommLink_DataPacketBStructure.data[1]  = 0;
    CommLink_DataPacketBStructure.data[2]  = 0;
    CommLink_DataPacketBStructure.data[3]  = 0;
    CommLink_DataPacketBStructure.data[4]  =
        ((s16)(-CommLink_DataStructure.pitch * 100)) >> 8;
    CommLink_DataPacketBStructure.data[5]  =
        ((s16)(-CommLink_DataStructure.pitch * 100)) & 0XFF;
    CommLink_DataPacketBStructure.data[6]  =
        ((s16)(ms5611_altitude * 1000)) >> 8;
    CommLink_DataPacketBStructure.data[7]  =
        ((s16)(ms5611_altitude * 1000)) & 0XFF;
    CommLink_DataPacketBStructure.data[8]  =
        ((s16)(imu.accg[2] * 1000)) >> 8;
    CommLink_DataPacketBStructure.data[9]  =
        ((s16)(imu.accg[2] * 1000)) & 0XFF;
    CommLink_DataPacketBStructure.data[10] = 0;
    CommLink_DataPacketBStructure.data[11] = 0;
    CommLink_DataPacketBStructure.sum      = 0;

    for (i = 0; i < 4; i++)
    {
        CommLink_DataPacketBStructure.sum +=
            *((u8 *)(&CommLink_DataPacketBStructure) + i);
    }

    for (i = 0; i < CommLink_DataPacketBStructure.len; i++)
    {
        CommLink_DataPacketBStructure.sum +=
            CommLink_DataPacketBStructure.data[i];
    }

    USART_SendBuffer((u8 *)(&CommLink_DataPacketBStructure),
                     CommLink_DataPacketBStructure.len + 4);
    USART_SendBuffer(&(CommLink_DataPacketBStructure.sum), 1);
}

void CommLink_ProcessDataFromNRF(void)
{
    if (control_alt_control_mode == CONTROL_STATE_LANDING)
    {
        comm_link_rc_data[ROLL]     = 1500;
        comm_link_rc_data[PITCH]    = 1500;
        comm_link_rc_data[YAW]      = 1500;
        comm_link_rc_data[THROTTLE] = 1500;
    }

    COMM_LINK_CONSTRAIN(comm_link_rc_data[ROLL], 1000, 2000);
    COMM_LINK_CONSTRAIN(comm_link_rc_data[PITCH], 1000, 2000);
    COMM_LINK_CONSTRAIN(comm_link_rc_data[YAW], 1000, 2000);
    COMM_LINK_CONSTRAIN(comm_link_rc_data[THROTTLE], 1000, 2000);

    CommLink_DataStructure.roll  = CommLink_CutDBScaleToLinear(
        (comm_link_rc_data[ROLL] - 1500), 500, COMM_LINK_APP_DB_PR) *
        ANGLE_MAX;
    CommLink_DataStructure.pitch = CommLink_CutDBScaleToLinear(
        (comm_link_rc_data[PITCH] - 1500), 500, COMM_LINK_APP_DB_PR) *
        ANGLE_MAX;
    CommLink_DataStructure.yaw   = CommLink_CutDBScaleToLinear(
        (comm_link_rc_data[YAW] - 1500), 500, COMM_LINK_APP_DB_YAW) *
        YAW_RATE_MAX;
    CommLink_DataStructure.thr = comm_link_rc_data[THROTTLE] - 1000;

    switch (comm_link_mcu_state)
    {
        case COMM_LINK_STATE_REQ_EN_MCU:
        {
            if (IMU_Check() && !Battery_InformationStructure.flag_alarm)
            {
                comm_link_mcu_state       = COMM_LINK_STATE_EN_MCU;
                comm_link_fly_enable_flag = true;
                LED_C_ON;
            }
            else
            {
                comm_link_mcu_state       = COMM_LINK_STATE_DISEN_MCU;
                comm_link_fly_enable_flag = false;
            }
            break;
        }
        case COMM_LINK_STATE_REQ_DISEN_MCU:
        {
            comm_link_mcu_state          = COMM_LINK_STATE_DISEN_MCU;
            comm_link_fly_enable_flag    = false;
            control_alt_control_mode     = CONTROL_STATE_MANUAL;
            control_integral_reset_flag  = true;
            control_thrust_z_split_power = 0;
            control_thrust_z_integral    = Control_EstimateThrustRefHover();
            control_offland_flag         = false;
            break;
        }
        default:
        {
            break;
        }
    }
}

void CommLink_ReceiveDataFromNRF(void)
{
    if ((nrf24l01_rx_data[0] == '$') && (nrf24l01_rx_data[1] == 'M') &&
        (nrf24l01_rx_data[2] == '<'))
    {
        switch (nrf24l01_rx_data[4])
        {
            case COMM_LINK_MSP_SET_4CON:
            {
                comm_link_rc_data[ROLL]     = nrf24l01_rx_data[11] +
                    (nrf24l01_rx_data[12] << 8);
                comm_link_rc_data[PITCH]    = nrf24l01_rx_data[9]  +
                    (nrf24l01_rx_data[10] << 8);
                comm_link_rc_data[YAW]      = nrf24l01_rx_data[7]  +
                    (nrf24l01_rx_data[8]  << 8);
                comm_link_rc_data[THROTTLE] = nrf24l01_rx_data[5]  +
                    (nrf24l01_rx_data[6]  << 8);
                break;
            }
            case COMM_LINK_MSP_ARM_IT:
            {
                comm_link_mcu_state = COMM_LINK_STATE_REQ_EN_MCU;
                break;
            }
            case COMM_LINK_MSP_DISARM_IT:
            {
                comm_link_mcu_state = COMM_LINK_STATE_REQ_DISEN_MCU;
                break;
            }
            case COMM_LINK_MSP_ACC_CALI:
            {
                imuCaliFlag = 1;
                break;
            }
        }
    }

    comm_link_last_rc_timestamp = Delay_GetRuntimeMs();
}

void CommLink_ReadPacket(u8 byte)
{
    switch (recv_state)
    {
        case COMM_LINK_STATE_IDLE:
        {
            recv_checksum = 0;
            if (byte == 0XAA)
            {
                recv_state = COMM_LINK_STATE_HEADER1;
            }
            break;
        }
        case COMM_LINK_STATE_HEADER1:
        {
            if (byte == 0XAF)
            {
                recv_state = COMM_LINK_STATE_HEADER2;
            }
            else
            {
                recv_state = COMM_LINK_STATE_IDLE;
            }
            break;
        }
        case COMM_LINK_STATE_HEADER2:
        {
            recv_command = byte;
            recv_state   = COMM_LINK_STATE_COMMAND;
            break;
        }
        case COMM_LINK_STATE_COMMAND:
        {
            recv_length   = byte;
            recv_state    = COMM_LINK_STATE_DATA;
            recv_checksum = 0XAA + 0XAF + recv_length + recv_command;
            break;
        }
        case COMM_LINK_STATE_DATA:
        {
            if (recv_count < recv_length)
            {
                recv_buffer[recv_count++] = byte;
                recv_checksum            += byte;
            }
            if (recv_count == recv_length)
            {
                recv_state = COMM_LINK_STATE_CHECKSUM;
            }
            break;
        }
        case COMM_LINK_STATE_CHECKSUM:
        {
            if (recv_checksum == byte)
            {
                comm_link_pc_cmd_flag = true;
                recv_count = 0;
            }
            recv_state = COMM_LINK_STATE_IDLE;
            break;
        }
    }
}

void CommLink_Test(void)
{
    u8 i = 0;

    for (i = 0; i < 6; i++)
    {
        CommLink_ReadPacket(test_buffer[i]);
    }
}

void CommLink_WriteBuffer(void)
{
    USART_SendBuffer(send_buffer, send_count);
    send_count    = 0;
    send_checksum = 0;
}

void CommLink_WriteDebugData(void)
{
    static u8 count = 0;

    if (count > 2)
    {
        count = 0;
    }
    // Divide time to send different data packets to avoid use too much cpu at
    // the same time.
    if (count == 0)
    {
        CommLink_HandleDebugDataA();
    }
    else if (count == 1)
    {
        CommLink_HandleDebugDataB();
    }
    else if (count == 2)
    {
        CommLink_HandleDebugDataC();
    }

    count++;
}

void CommLink_WritePacket(u8 command)
{
    send_checksum = 0;

    CommLink_AddBits8ToBuffer(0XAA);
    CommLink_AddBits8ToBuffer(0XAA);
    CommLink_AddBits8ToBuffer(command);

    switch (command)
    {
        case COMM_LINK_STATE_PC_PID_PITCH:
        {
            CommLink_AddBits8ToBuffer(0X0C);
            CommLink_AddBits16ToBuffer((s16)((pitch_rate_PID.P  * 100)));
            CommLink_AddBits16ToBuffer((s16)((pitch_rate_PID.I  * 100)));
            CommLink_AddBits16ToBuffer((s16)((pitch_rate_PID.D  * 100)));
            CommLink_AddBits16ToBuffer((s16)((pitch_angle_PID.P * 100)));
            CommLink_AddBits16ToBuffer((s16)((pitch_angle_PID.I * 100)));
            CommLink_AddBits16ToBuffer((s16)((pitch_angle_PID.D * 100)));
            break;
        }
        case COMM_LINK_STATE_PC_PID_ROLL:
        {
            CommLink_AddBits8ToBuffer(0X0C);
            CommLink_AddBits16ToBuffer((s16)((roll_rate_PID.P  * 100)));
            CommLink_AddBits16ToBuffer((s16)((roll_rate_PID.I  * 100)));
            CommLink_AddBits16ToBuffer((s16)((roll_rate_PID.D  * 100)));
            CommLink_AddBits16ToBuffer((s16)((roll_angle_PID.P * 100)));
            CommLink_AddBits16ToBuffer((s16)((roll_angle_PID.I * 100)));
            CommLink_AddBits16ToBuffer((s16)((roll_angle_PID.D * 100)));
            break;
        }
        case COMM_LINK_STATE_PC_PID_YAW:
        {
            CommLink_AddBits8ToBuffer(0X0C);
            CommLink_AddBits16ToBuffer((s16)((yaw_rate_PID.P  * 100)));
            CommLink_AddBits16ToBuffer((s16)((yaw_rate_PID.I  * 100)));
            CommLink_AddBits16ToBuffer((s16)((yaw_rate_PID.D  * 100)));
            CommLink_AddBits16ToBuffer((s16)((yaw_angle_PID.P * 100)));
            CommLink_AddBits16ToBuffer((s16)((yaw_angle_PID.I * 100)));
            CommLink_AddBits16ToBuffer((s16)((yaw_angle_PID.D * 100)));
            break;
        }
        case COMM_LINK_STATE_PC_PID_ALT:
        {
            CommLink_AddBits8ToBuffer(0X0C);
            CommLink_AddBits16ToBuffer((s16)((alt_vel_PID.P * 100)));
            CommLink_AddBits16ToBuffer((s16)((alt_vel_PID.I * 100)));
            CommLink_AddBits16ToBuffer((s16)((alt_vel_PID.D * 100)));
            CommLink_AddBits16ToBuffer((s16)((alt_PID.P     * 100)));
            CommLink_AddBits16ToBuffer((s16)((alt_PID.I     * 100)));
            CommLink_AddBits16ToBuffer((s16)((alt_PID.D     * 100)));
            break;
        }
    }

    CommLink_AddBits8ToBuffer(send_checksum);
    CommLink_WriteBuffer();
}

void CommLink_WritePID(u8 pid_type)
{
    send_checksum = 0;
    send_count    = 0;

    CommLink_AddBits8ToBuffer(0XAA);
    CommLink_AddBits8ToBuffer(0XAA);
    CommLink_AddBits8ToBuffer(pid_type);
    CommLink_AddBits8ToBuffer(0X0C);

    CommLink_AddDataToBuffer(recv_buffer, 12);

    CommLink_AddBits8ToBuffer(send_checksum);
    CommLink_WriteBuffer();
}

// Cut deadband scale to move linear.
float CommLink_CutDBScaleToLinear(float x_start, float x_end, float deadband)
{
    if (x_start > deadband)
    {
        return (x_start - deadband) / (x_end - deadband);
    }
    else if (x_start < -deadband)
    {
        return (x_start + deadband) / (x_end - deadband);
    }
    else
    {
        return 0.0F;
    }
}
