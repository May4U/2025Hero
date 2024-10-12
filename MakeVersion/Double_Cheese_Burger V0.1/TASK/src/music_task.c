#include "FreeRTOS.h"
#include "freertos.h"
#include "queue.h"
#include "semphr.h"

#include "bsp_buzzer.h"
#include "music_task.h"
#include "stm32f4xx_hal.h"

extern TIM_HandleTypeDef htim4;
    unsigned int Moon[]={
//SuperMario
//17,2,17,4,17,2,0,2,13,2,17,4,20,4,0,4,8,4,0,4,13,4,0,2,8,2,0,4,5,4,0,2,10,4,12,2,0,2,12,2,10,4,8,4,17,4,20,4,22,4,18,2,20,2,0,2,17,4,13,2,15,2,12,4,0,2,
//一路靠北
//13,4,13,4,17,4,24,4,24,4,25,8,25,4,25,4,27,4,25,4,24,4,24,4,25,8,25,4,25,4,27,4,25,4,27,4,29,4,27,8,25,4,25,4,27,4,25,4,27,4,29,4,30,4,29,4,27,4,27,4,20,4,20,4,27,4,27,4,0,4,25,2,25,4,29,4,29,4,27,4,24,4,22,4,22,4,0,4,20,2,20,8,20,4,20,4,20,4,25,4,25,4,18,8,25,4,24,4,24,4,25,4,27,4,27,4,29,2,29,2,29,8,29,4,20,4,20,4,27,4,27,4,0,4,25,2,25,4,29,4,29,4,27,4,24,4,22,4,22,4,20,4,20,4,17,2,27,2,27,2,25,4,0,4,25,4,20,4,20,4,22,2,13,2,13,4,0,4,15,2,18,4,17,4,15,4,0,4,13,2,13,8,
 //晴天//怪怪的
0,8,0,8,13,4,15,4,17,8,0,4,20,4,17,8,15,20,13,8,10,20,8,8,13,20,13,4,15,4,17,8,0,4,20,4,17,8,25,20,22,8,20,8,0,4,15,4,17,4,18,4,17,20,13,4,15,4,17,8,17,8,17,8,
        0xFF
};

float note_list[] =
    {
        1000000,
        NOTE_C4,
        NOTE_CS4,
        NOTE_D4,
        NOTE_DS4,
        NOTE_E4,
        NOTE_F4,
        NOTE_FS4,
        NOTE_G4,
        NOTE_GS4,
        NOTE_A4,
        NOTE_AS4,
        NOTE_B4,
        NOTE_C5,
        NOTE_CS5,
        NOTE_D5,
        NOTE_DS5,
        NOTE_E5,
        NOTE_F5,
        NOTE_FS5,
        NOTE_G5,
        NOTE_GS5,
        NOTE_A5,
        NOTE_AS5,
        NOTE_B5,
        NOTE_C6,
        NOTE_CS6,
        NOTE_D6,
        NOTE_DS6,
        NOTE_E6,
        NOTE_F6,
        NOTE_FS6,
        NOTE_G6,
        NOTE_GS6,
        NOTE_A6,
        NOTE_AS6,
        -1.0f,
};
    
void music()
{
//    static TickType_t last_note_systick = 0;
//    static uint64_t music_num = 0;
//    if (xTaskGetTickCount() - last_note_systick > Music[music_num + 1] * 73)
//        buzzer_off();
//    if (xTaskGetTickCount() - last_note_systick > Music[music_num + 1] * 80)
//    {
//        last_note_systick = xTaskGetTickCount();
//        music_num += 2;
//        if (Music[music_num] == 0xff)
//        {
//            music_num = 0;
////            vTaskSuspend(NULL);
//        }
//        buzzer_test(83, 1000000 / note_list[Music[music_num]]);
//    }
}

typedef enum
{
    Init_Music,
    Can_dis_Music
}Music_e;

Music_e Music;
uint64_t music_time = 0;
unsigned int *music_list_point;

void Music_choice(Music_e Music)
{
    switch (Music)
    {
        case Init_Music:
        {
            music_list_point = Moon;
        }break;
    }
    music_time = 0;
}

void MUSIC_TASK(void const * argument)
{  
    //HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    Music_choice(Init_Music);
    while (1)
    {
        static TickType_t last_note_systick = 0;
        if (xTaskGetTickCount() - last_note_systick > music_list_point[music_time + 1] * 73)
            buzzer_off();
        if (xTaskGetTickCount() - last_note_systick > music_list_point[music_time + 1] * 80)
        {
            last_note_systick = xTaskGetTickCount();
            music_time += 2;
            if (music_list_point[music_time] == 0xff)
            {
                music_time = 0;
                buzzer_off();
                vTaskSuspend(NULL);
            }
            buzzer_test(83, 1000000 / note_list[music_list_point[music_time]]);
        }
        vTaskDelay(1); // 绝对延时//vTaskDelay(2);
    }
}
