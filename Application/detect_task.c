/**
 ******************************************************************************
 * @file    detect_task.h
 * @author  Wang Hongxi
 * @version V1.0.0
 * @date    2020/1/30
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#include "detect_task.h"
#include "bsp_CAN.h"

uint32_t resetCount = 0;

Detect_t Detect_List[DETECT_LIST_LENGHT + 1];
BoardState_t BoardState;
uint32_t delta_time;


//uint32_t resetCount = 0;

Detect_t Detect_List[DETECT_LIST_LENGHT + 1];
uint32_t DetectSysTime;

static void Detect_Init(uint32_t time);
uint8_t is_reset = 0;

void Detect_Task_Init(void) {
    DetectSysTime = USER_GetTick();

    Detect_Init(DetectSysTime);

    USER_Delay_ms(60);
}

void Detect_Task(void)
{
    static uint8_t error_num_display = 0;

    // do
    // {
    //     const uint32_t a = __get_PRIMASK();
    //     __disable_irq();
    //     if (fuck_sem) {
    //         fuck_sem--;
    //     } else {
    //         __enable_irq();
    //         __set_PRIMASK(a);
    //         return;
    //     }
    //     __enable_irq();
    //     __set_PRIMASK(a);
    // } while (0);


    //DetectSysTime = USER_GetTick();



    error_num_display = DETECT_LIST_LENGHT;
    Detect_List[DETECT_LIST_LENGHT].is_Lost = 0;
    Detect_List[DETECT_LIST_LENGHT].Error_Exist = 0;

    for (int i = 0; i < DETECT_LIST_LENGHT; i++)
    {
        if (Detect_List[i].Enable == 0)
        {
            continue;
        }

        //TODO: 临时解决原子性缺失问题, 未处理标志位逻辑
        const volatile uint32_t new_time = Detect_List[i].New_Time;
        const volatile uint32_t last_time = Detect_List[i].Last_Time;
        const volatile uint32_t work_time = Detect_List[i].Work_Time;
        __DMB();
        DetectSysTime = USER_GetTick();
        // 该段代码勿动,禁止改变顺序

        Detect_List[i].delta = DetectSysTime - new_time;
        // �����ж�
        if (DetectSysTime - new_time > Detect_List[i].Offline_Time)
        {
            if (Detect_List[i].Error_Exist == 0)
            {
                // ��¼�����Լ�����ʱ��
                Detect_List[i].is_Lost = 1;
                Detect_List[i].Error_Exist = 1;
                Detect_List[i].Lost_Time = DetectSysTime;
            }

            // �������ȼ���ߵĴ�����
            if (Detect_List[i].Priority > Detect_List[error_num_display].Priority)
            {
                error_num_display = i;
            }

            Detect_List[DETECT_LIST_LENGHT].is_Lost = 1;
            Detect_List[DETECT_LIST_LENGHT].Error_Exist = 1;

            // ����ṩ������������н������
            if (Detect_List[i].f_Solve_Lost != NULL)
            {
                Detect_List[i].f_Solve_Lost();
            }
        }
        else if (DetectSysTime - work_time < Detect_List[i].Online_Time)
        {
            // �ո����ߣ����ܴ������ݲ��ȶ���ֻ��¼����ʧ��
            Detect_List[i].is_Lost = 0;
            Detect_List[i].Error_Exist = 1;
        }
        else
        {
            Detect_List[i].is_Lost = 0;
            // �ж��Ƿ�������ݴ���
            if (Detect_List[i].is_Data_Error != NULL)
            {
                Detect_List[i].Error_Exist = 1;
            }
            else
            {
                Detect_List[i].Error_Exist = 0;
            }

            // ����Ƶ��
            if (new_time > last_time)
            {
                Detect_List[i].frequency = 1000.0f / (float)(new_time - last_time);
            }
        }
    }

    // do
    // {
    //     const uint32_t a = __get_PRIMASK();
    //     __disable_irq();

    //     if (!fuck_sem) {
    //         fuck_sem++;
    //     }
    //     __enable_irq();
    //     __set_PRIMASK(a);
    // } while (0);
}

uint8_t is_TOE_Error(uint8_t toe)
{
    return (Detect_List[toe].Error_Exist == 1);
}

void Detect_Hook(uint8_t toe) {
    Detect_List[toe].Last_Time = Detect_List[toe].New_Time;
    Detect_List[toe].New_Time = USER_GetTick();

    if (Detect_List[toe].is_Lost)
    {
        Detect_List[toe].is_Lost = 0;
        Detect_List[toe].Work_Time = Detect_List[toe].New_Time;
    }

    if (Detect_List[toe].f_is_Data_Error != NULL)
    {
        if (Detect_List[toe].f_is_Data_Error())
        {
            Detect_List[toe].Error_Exist = 1;
            Detect_List[toe].is_Data_Error = 1;

            if (Detect_List[toe].f_Solve_Data_Error != NULL)
            {
                Detect_List[toe].f_Solve_Data_Error();
            }
        }
        else
        {
            Detect_List[toe].is_Data_Error = 0;
        }
    }
    else
    {
        Detect_List[toe].is_Data_Error = 0;
    }
}

static void Detect_Init(uint32_t time) {
    //��������ʱ�䣬�����ȶ�����ʱ�䣬���ȼ� offlineTime onlinetime Priority
    uint16_t set_item[DETECT_LIST_LENGHT][3] =
            {
                    {30,   3,    14},     // yaw
                    {30,   3,    13},     // pitch
                    {5,    10,   11},     // trigger
                    {50,   3,    12},     // RC device
                    {70,   4,    10},     // autoaim device
                    {1000, 1000, 9}, // refuree device
                    {500,  500,  12},  // VTM device
            };

    for (uint8_t i = 0; i < DETECT_LIST_LENGHT; i++)
    {
        Detect_List[i].Offline_Time = set_item[i][0];
        Detect_List[i].Online_Time = set_item[i][1];
        Detect_List[i].Priority = set_item[i][2];
        Detect_List[i].f_is_Data_Error = NULL;
        Detect_List[i].f_Solve_Lost = NULL;
        Detect_List[i].f_Solve_Data_Error = NULL;

        Detect_List[i].Enable = 1;
        Detect_List[i].Error_Exist = 1;
        Detect_List[i].is_Lost = 1;
        Detect_List[i].is_Data_Error = 1;
        Detect_List[i].frequency = 0.0f;
        Detect_List[i].New_Time = time;
        Detect_List[i].Last_Time = time;
        Detect_List[i].Lost_Time = time;
        Detect_List[i].Work_Time = time;
    }
    Detect_List[RC_TOE].f_is_Data_Error = RC_Data_is_Error;
    Detect_List[RC_TOE].f_Solve_Lost = Solve_RC_Lost;
    Detect_List[RC_TOE].f_Solve_Data_Error = Solve_RC_Data_Error;
}
