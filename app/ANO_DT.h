#ifndef    ANO_DT_H
#define    ANO_DT_H

#include "stm32f4xx.h"
#include "FR_OS.h"


#define ANO_DT_MODE_IS_MASTER  (0) //1:Master, 0:Slaver

#ifndef ANO_DT_MODE_IS_MASTER
#define ANO_DT_MODE_IS_MASTER  (0)
#endif

#define ANO_DT_SUPPORT_FOC     (1)

#if (ANO_DT_MODE_IS_MASTER == 0)
#define ANO_DT_SLAVER_MODE_PASSIVE    (1)
#endif

#define ANO_DT_FRAME_PERIOD    (20) //20ms
#define ANO_DT_DATA_SIZE_MAX   (64)

typedef enum
{
    E_ANO_DT_HANDLE_TYPE_NULL           = 0x00,
    E_ANO_DT_HANDLE_TYPE_MASTER_SEND    = 0x01,
    E_ANO_DT_HANDLE_TYPE_MASTER_RECEIVE = 0x02,
    E_ANO_DT_HANDLE_TYPE_SLAVER_SEND    = 0x03,
    E_ANO_DT_HANDLE_TYPE_SLAVER_RECEIVE = 0x04,
    E_ANO_DT_HANDLE_TYPE_MAX
}EN_ANO_DT_HANDLE_TYPE;

typedef struct
{
    u8 u8FuncType;
    u8 u8DataSize;
    u8 pu8DataBuff[ANO_DT_DATA_SIZE_MAX];
}ST_ANO_DT_SEND_DATA;

typedef struct
{
    u8 *pu8Buff;
    u8 u8len;
}ST_ANO_DT_RECEIVE_DATA;

typedef struct
{
    u16 u16Head;
    u8  u8FuncType;
    u8  u8datalen;//data len, without head and checksum
    u8 *pu8Buff;
    u8  u8sum;
    u8  u8ErrCnt;
}ST_ANO_DT_FRAME_INFO;

typedef struct ST_ANO_DT_FrameListItem
{
    u8 eFuncType;
    u8 bActive;
    u8 bAutoRun;
    void *pParma;
    u8 u8ParmaSize;
    void *pvContainer;
    struct ST_ANO_DT_FrameListItem *pstNext;
}ST_ANO_DT_FRAME_LIST_ITEM;

typedef struct
{
    u8 u8ListItemNum;
    ST_ANO_DT_FRAME_LIST_ITEM *pstHead;
    ST_ANO_DT_FRAME_LIST_ITEM *pstIndex;
    ST_ANO_DT_FRAME_LIST_ITEM *pstEnd;
    ST_ANO_DT_FRAME_LIST_ITEM  stListItemStart;
}ST_ANO_DT_FRAME_LIST;

/*******************M2S type define********************/
typedef enum
{
    E_ANO_DT_FUNC_M2S_TYPE_NULL              = 0x00,
    E_ANO_DT_FUNC_M2S_TYPE_COMMAND           = 0x01,
    E_ANO_DT_FUNC_M2S_TYPE_ACK               = 0x02,
    E_ANO_DT_FUNC_M2S_TYPE_RCDATA            = 0x03,
    E_ANO_DT_FUNC_M2S_TYPE_READ_PARAMS       = 0x08,
    E_ANO_DT_FUNC_M2S_TYPE_SET_PARAMS        = 0x09,
    E_ANO_DT_FUNC_M2S_TYPE_FLY_MODE          = 0x0A,
    E_ANO_DT_FUNC_M2S_TYPE_PID1              = 0x10,
    E_ANO_DT_FUNC_M2S_TYPE_PID2              = 0x11,
    E_ANO_DT_FUNC_M2S_TYPE_PID3              = 0x12,
    E_ANO_DT_FUNC_M2S_TYPE_PID4              = 0x13,
    E_ANO_DT_FUNC_M2S_TYPE_PID5              = 0x14,
    E_ANO_DT_FUNC_M2S_TYPE_PID6              = 0x15,
    E_ANO_DT_FUNC_M2S_TYPE_READ_FP           = 0x20,
    E_ANO_DT_FUNC_M2S_TYPE_FP                = 0x21,
    E_ANO_DT_FUNC_M2S_TYPE_LOCATION_SET      = 0x3A,
    E_ANO_DT_FUNC_M2S_TYPE_LOCATION_SET2     = 0x3B,
    E_ANO_DT_FUNC_M2S_TYPE_RADIO_LINK_SET    = 0x40,
    E_ANO_DT_FUNC_M2S_TYPE_OPTICAL_FLOW_SET  = 0x50,
    E_ANO_DT_FUNC_M2S_TYPE_OPTICAL_FLOW_CMD  = 0x56,
#if (ANO_DT_SUPPORT_FOC == 1)
    E_ANO_DT_FUNC_M2S_TYPE_FOC_INPUT_CTRL    = 0x63,
#endif
    E_ANO_DT_FUNC_M2S_TYPE_ENTRY_IAP         = 0xF0,
    E_ANO_DT_FUNC_M2S_TYPE_MAX
}EN_ANO_DT_FUNC_M2S_TYPE;

typedef enum
{
    E_ANO_DT_M2S_TYPE_CMD_ACCEL         = 0x01,
    E_ANO_DT_M2S_TYPE_CMD_GYRO          = 0x02,
    E_ANO_DT_M2S_TYPE_CMD_ALT           = 0x03,
    E_ANO_DT_M2S_TYPE_CMD_MAG           = 0x04,
    E_ANO_DT_M2S_TYPE_CMD_BARO          = 0x05,
    E_ANO_DT_M2S_TYPE_CMD_INERT         = 0x10,
    E_ANO_DT_M2S_TYPE_CMD_SIX_FACE_EXIT = 0x20,
    E_ANO_DT_M2S_TYPE_CMD_SIX_FACE_1    = 0x21,
    E_ANO_DT_M2S_TYPE_CMD_SIX_FACE_2    = 0x22,
    E_ANO_DT_M2S_TYPE_CMD_SIX_FACE_3    = 0x23,
    E_ANO_DT_M2S_TYPE_CMD_SIX_FACE_4    = 0x24,
    E_ANO_DT_M2S_TYPE_CMD_SIX_FACE_5    = 0x25,
    E_ANO_DT_M2S_TYPE_CMD_SIX_FACE_6    = 0x26,
    E_ANO_DT_M2S_TYPE_CMD_LOCK          = 0xA0,
    E_ANO_DT_M2S_TYPE_CMD_UNLOCK        = 0xA1
}EN_ANO_DT_M2S_COMMAND_TYPE;

typedef enum
{
    E_ANO_DT_M2S_TYPE_ACK_READ_PID         = 0x01,
    E_ANO_DT_M2S_TYPE_ACK_READ_FLY_MODE    = 0x02,
    E_ANO_DT_M2S_TYPE_ACK_READ_RESERVE1    = 0x21,
    E_ANO_DT_M2S_TYPE_ACK_READ_RESERVE2    = 0x30,
    E_ANO_DT_M2S_TYPE_ACK_READ_RESERVE3    = 0x40,
    E_ANO_DT_M2S_TYPE_ACK_READ_RESERVE4    = 0x50,
    E_ANO_DT_M2S_TYPE_ACK_READ_VERSION     = 0xA0,
    E_ANO_DT_M2S_TYPE_ACK_RESET_REQ        = 0xA1
}EN_ANO_DT_M2S_ACK_TYPE;

typedef struct //__attribute__((packed))
{
    s16 s16THR;
    s16 s16YAW;
    s16 s16ROL;
    s16 s16PIT;
    s16 s16AUX1;
    s16 s16AUX2;
    s16 s16AUX3;
    s16 s16AUX4;
    s16 s16AUX5;
    s16 s16AUX6;
}ST_ANO_DT_DATA_M2S_RCDATA;

/*********************S2M type define********************************/
typedef enum
{
    E_ANO_DT_FUNC_S2M_TYPE_VER              = 0x00,
    E_ANO_DT_FUNC_S2M_TYPE_STATUS           = 0x01,
    E_ANO_DT_FUNC_S2M_TYPE_SENSER           = 0x02,
    E_ANO_DT_FUNC_S2M_TYPE_RCDATA           = 0x03,
    E_ANO_DT_FUNC_S2M_TYPE_GPSDATA          = 0x04,
    E_ANO_DT_FUNC_S2M_TYPE_POWER            = 0x05,
    E_ANO_DT_FUNC_S2M_TYPE_MOTO             = 0x06,
    E_ANO_DT_FUNC_S2M_TYPE_SENSER2          = 0x07,
    E_ANO_DT_FUNC_S2M_TYPE_PARAMS_SET       = 0x09,
    E_ANO_DT_FUNC_S2M_TYPE_FLY_MODE         = 0x0A,
    E_ANO_DT_FUNC_S2M_TYPE_PID1             = 0x10,
    E_ANO_DT_FUNC_S2M_TYPE_PID2             = 0x11,
    E_ANO_DT_FUNC_S2M_TYPE_PID3             = 0x12,
    E_ANO_DT_FUNC_S2M_TYPE_PID4             = 0x13,
    E_ANO_DT_FUNC_S2M_TYPE_PID5             = 0x14,
    E_ANO_DT_FUNC_S2M_TYPE_PID6             = 0x15,
    E_ANO_DT_FUNC_S2M_TYPE_FP_NUM           = 0x20,
    E_ANO_DT_FUNC_S2M_TYPE_FP               = 0x21,
    E_ANO_DT_FUNC_S2M_TYPE_DISTANCE         = 0x30,
    E_ANO_DT_FUNC_S2M_TYPE_LOCATION         = 0x32,
    E_ANO_DT_FUNC_S2M_TYPE_D_LOCATION       = 0x33,
    E_ANO_DT_FUNC_S2M_TYPE_LOCATION_SET     = 0x3A,
    E_ANO_DT_FUNC_S2M_TYPE_LOCATION_SET2    = 0x3B,
    E_ANO_DT_FUNC_S2M_TYPE_RADIO_LINK_SET   = 0x40,
    E_ANO_DT_FUNC_S2M_TYPE_OPTICAL_FLOW_SET = 0x50,
#if (ANO_DT_SUPPORT_FOC == 1)
    E_ANO_DT_FUNC_S2M_TYPE_FOC_STATUS       = 0x60,
    E_ANO_DT_FUNC_S2M_TYPE_FOC_INT_INFO1    = 0x61,
    E_ANO_DT_FUNC_S2M_TYPE_FOC_INT_INFO2    = 0x62,
    E_ANO_DT_FUNC_S2M_TYPE_FOC_INPUT_CTRL   = 0x63,
#endif
    E_ANO_DT_FUNC_S2M_TYPE_MSG              = 0xEE,
    E_ANO_DT_FUNC_S2M_TYPE_CHECK            = 0xEF,
    E_ANO_DT_FUNC_S2M_TYPE_USER_DATA1       = 0xF1,
    E_ANO_DT_FUNC_S2M_TYPE_USER_DATA2       = 0xF2,
    E_ANO_DT_FUNC_S2M_TYPE_USER_DATA3       = 0xF3,
    E_ANO_DT_FUNC_S2M_TYPE_USER_DATA4       = 0xF4,
    E_ANO_DT_FUNC_S2M_TYPE_USER_DATA5       = 0xF5,
    E_ANO_DT_FUNC_S2M_TYPE_USER_DATA6       = 0xF6,
    E_ANO_DT_FUNC_S2M_TYPE_USER_DATA7       = 0xF7,
    E_ANO_DT_FUNC_S2M_TYPE_USER_DATA8       = 0xF8,
    E_ANO_DT_FUNC_S2M_TYPE_USER_DATA9       = 0xF9,
    E_ANO_DT_FUNC_S2M_TYPE_USER_DATA10      = 0xFA,
    E_ANO_DT_FUNC_S2M_TYPE_MAX
}EN_FUNC_S2M_TYPE;

typedef enum
{
    E_ANO_DT_S2M_TYPE_MSG_ID_ACCEL      = 0x01,
    E_ANO_DT_S2M_TYPE_MSG_ID_GYRO       = 0x02,
    E_ANO_DT_S2M_TYPE_MSG_ID_MAG        = 0x03,
    E_ANO_DT_S2M_TYPE_MSG_ID_WIRELESS   = 0x30,
    E_ANO_DT_S2M_TYPE_MSG_ID_ANO_DATA   = 0x40
}EN_ANO_DT_S2M_MSG_ID;

typedef enum
{
    E_ANO_DT_S2M_TYPE_MSG_DATA_CALI_PASS    = 0x01,
    E_ANO_DT_S2M_TYPE_MSG_DATA_CALI_FAIL    = 0xE2,
    E_ANO_DT_S2M_TYPE_MSG_DATA_SET_DONE     = 0x31,
    E_ANO_DT_S2M_TYPE_MSG_DATA_SET_DONE2    = 0x32,
    E_ANO_DT_S2M_TYPE_MSG_DATA_RESET_DONE   = 0xA1
}EN_ANO_DT_S2M_MSG_DATA;

typedef struct //__attribute__((packed))
{
    u8  u8HardwareType;
    u16 u16HardwareVER;//*100
    u16 u16SoftwareVER;//*100
    u16 u16ProtocolVER;//*100
    u16 u16BootloaderVER;//*100
}ST_ANO_DT_DATA_S2M_VER;

typedef struct //__attribute__((packed))
{
    s16 s16ROL; //*100
    s16 s16PIT; //*100
    s16 s16YAW; //*100
    s32 s32ALT_USE; //unit cm
    u8  u8FLY_MODEL;
    u8  u8ARMED; //0加锁 1解锁;
}ST_ANO_DT_DATA_S2M_STATUS;

typedef struct //__attribute__((packed))
{
    s16 s16ACC_X;
    s16 s16ACC_Y;
    s16 s16ACC_Z;
    s16 s16GYRO_X;
    s16 s16GYRO_Y;
    s16 s16GYRO_Z;
    s16 s16MAG_X;
    s16 s16MAG_Y;
    s16 s16MAG_Z;
}ST_ANO_DT_DATA_S2M_SENSER;

typedef ST_ANO_DT_DATA_M2S_RCDATA ST_ANO_DT_DATA_S2M_RCDATA;

typedef struct //__attribute__((packed))
{
    u16 u16Votage; //*100
    u16 u16Current; //*100
}ST_ANO_DT_DATA_S2M_POWER;

typedef struct //__attribute__((packed))
{
    u16 u16PWM_MOTO1;
    u16 u16PWM_MOTO2;
    u16 u16PWM_MOTO3;
    u16 u16PWM_MOTO4;
    u16 u16PWM_MOTO5;
    u16 u16PWM_MOTO6;
    u16 u16PWM_MOTO7;
    u16 u16PWM_MOTO8;
}ST_ANO_DT_DATA_S2M_MOTO;

typedef struct //__attribute__((packed))
{
    s16 s16PID1_P;//*1000
    s16 s16PID1_I;//*1000
    s16 s16PID1_D;//*1000
    s16 s16PID2_P;//*1000
    s16 s16PID2_I;//*1000
    s16 s16PID2_D;//*1000
    s16 s16PID3_P;//*1000
    s16 s16PID3_I;//*1000
    s16 s16PID3_D;//*1000
}ST_ANO_DT_DATA_S2M_PID;

#if (ANO_DT_SUPPORT_FOC == 1)
typedef struct //__attribute__((packed))
{
    s16 s16Angle_M;//*100
    s16 s16Omega_M;//*100
    s16 s16Alpha_M;//*100
    s16 s16Angle_Sum;//*100
    u8 u8lock_mode;
    u8 u8IsActive;
    u8 u8status;
}ST_ANO_DT_DATA_S2M_FOC_STATUS;

typedef struct //__attribute__((packed))
{
    s16 s16U_d;//*100
    s16 s16U_q;//*100
    s16 s16theta;//*100
    s16 s16U_alpha;//*100
    s16 s16U_beta;//*100
    u8  u8N;
    u8  u8Sector;
    u16 u16T_a;
    u16 u16T_b;
    u16 u16T_c;
}ST_ANO_DT_DATA_S2M_FOC_INT_INFO1;

typedef struct //__attribute__((packed))
{
    s16 s16I_a;//*100
    s16 s16I_b;//*100
    s16 s16I_c;//*100
    s16 s16I_d;//*100
    s16 s16I_q;//*100
    s16 s16I_alpha;//*100
    s16 s16I_beta;//*100
    s16 s16Sin_theta;//*100
    s16 s16Cos_theta;//*100
    u16 u16PWMA;
    u16 u16PWMB;
    u16 u16PWMC;
}ST_ANO_DT_DATA_S2M_FOC_INT_INFO2;

typedef struct //__attribute__((packed))
{
    s16 s16Angle;//*100
    s16 s16Omega;//*100
    s16 s16I_d;//*100
    s16 s16I_q;//*100
    u8  u8lock_mode;
}ST_ANO_DT_DATA_S2M_FOC_INPUT_CTRL;

typedef ST_ANO_DT_DATA_S2M_FOC_INPUT_CTRL ST_ANO_DT_DATA_M2S_FOC_INPUT_CTRL;
#endif

typedef struct //__attribute__((packed))
{
    u8 u8MSG_ID;
    u8 u8MSG_DATA;
}ST_ANO_DT_DATA_S2M_MSG;

typedef struct //__attribute__((packed))
{
    u8 u8FREAM_HEAD; //帧头
    u8 u8CHECK_SUM; //和校验
}ST_ANO_DT_DATA_S2M_CHECK;

typedef enum
{
    E_ANO_DT_SLAVER_SEND_EVENT_MSG   = BIT(8),
    E_ANO_DT_SLAVER_SEND_EVENT_CHECK = BIT(9),
    E_ANO_DT_SLAVER_SEND_EVENT_PID   = BIT(10)
}EN_ANO_DT_SLAVER_SEND_EVENT;

/***********************************************************/
extern TaskHandle_t hAnoDtStartTask;
extern TaskHandle_t hAnoDtSendTask;
extern TaskHandle_t hAnoDtReceiveTask;
extern QueueHandle_t hAnoDtSend_Queue;
extern QueueHandle_t hAnoDtReceive_Queue;

extern ST_ANO_DT_DATA_M2S_RCDATA stAnoDtDataM2SRcdata;

extern ST_ANO_DT_DATA_S2M_VER    stAnoDtDataS2MVer;
extern ST_ANO_DT_DATA_S2M_STATUS stAnoDtDataS2MStatus;
extern ST_ANO_DT_DATA_S2M_SENSER stAnoDtDataS2MSenser;
extern ST_ANO_DT_DATA_S2M_RCDATA stAnoDtDataS2MRcdata;
extern ST_ANO_DT_DATA_S2M_POWER  stAnoDtDataS2MPower;
#if (ANO_DT_SUPPORT_FOC == 1)
extern ST_ANO_DT_DATA_M2S_FOC_INPUT_CTRL stAnoDtDataM2SFocInputCtrl;

extern ST_ANO_DT_DATA_S2M_FOC_STATUS     stAnoDtDataS2MFocStatus;
extern ST_ANO_DT_DATA_S2M_FOC_INT_INFO1  stAnoDtDataS2MFocIntInfo1;
extern ST_ANO_DT_DATA_S2M_FOC_INT_INFO2  stAnoDtDataS2MFocIntInfo2;
extern ST_ANO_DT_DATA_S2M_FOC_INPUT_CTRL stAnoDtDataS2MFocInputCtrl;
#endif
extern ST_ANO_DT_DATA_S2M_MSG    stAnoDtDataS2MMsg;
extern ST_ANO_DT_DATA_S2M_CHECK  stAnoDtDataS2MCheck;


#if (ANO_DT_MODE_IS_MASTER)
void ANO_DT_Master_Send_Task(void * pvParameters);
void ANO_DT_Master_Receive_Task(void * pvParameters);
u8   ANO_DT_Master_Set_Send_Msg(u8 u8MsgID, u8 u8MsgData);
u8   ANO_DT_Master_Set_Send_Check(u8 u8head, u8 u8check_Sum);
u8   ANO_DT_Master_Send_Enable(u8 bEn);
#else
#define ANO_DT_Master_Send_Task(...)
#define ANO_DT_Master_Receive_Task(...)
#define ANO_DT_Master_Set_Send_Msg(...)    (FALSE)
#define ANO_DT_Master_Set_Send_Check(...)  (FALSE)
#define ANO_DT_Master_Send_Enable(...)     (FALSE)
#endif

#if (!ANO_DT_MODE_IS_MASTER)
void ANO_DT_Slaver_Send_Task(void * pvParameters);
void ANO_DT_Slaver_Receive_Task(void * pvParameters);
u8   ANO_DT_Slaver_Set_Send_Msg(u8 u8MsgID, u8 u8MsgData);
u8   ANO_DT_Slaver_Set_Send_Check(u8 u8head, u8 u8check_Sum);
u8   ANO_DT_Slaver_Send_Enable(u8 bEn);
#else
#define ANO_DT_Slaver_Send_Task(...)
#define ANO_DT_Slaver_Receive_Task(...)
#define ANO_DT_Slaver_Set_Send_Msg(...)     (FALSE)
#define ANO_DT_Slaver_Set_Send_Check(...)   (FALSE)
#define ANO_DT_Slaver_Send_Enable(...)      (FALSE)
#endif


void ANO_DT_Start_Task(void * pvParameters);


#endif

