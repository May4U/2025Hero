#ifndef __BSP_NAVIGATION_H
#define __BSP_NAVIGATION_H
#include "struct_typedef.h"

#define Move_Speed_Factor   400
#define Rotate_Speed_Factor 11318
#define Gimbal_Speed_Factor 57.1
//Êý¾ÝËÀÇø
#define USB_Dead 0.02

typedef struct{
  uint32_t rxlen;
  uint32_t flag;
}VcpRx_t;

typedef struct{
    uint8_t head;
    union{
        fp32 speed;
        uint8_t byte[4];
    }x;
    
    union{
         fp32 speed;
        uint8_t byte[4];
    }y;
    
    union{
         fp32 speed;
        uint8_t byte[4];
    }z;
		
		uint8_t Flag_Auto;
		
    uint8_t end;
}navi_t;

typedef struct{
	 int16_t x;
	 int16_t y;
	 int16_t z;
}Chassis_Speed;

Chassis_Speed* Get_Navigation_Point(void); 
void Speed_Enlarge(void);
void USB_DataRecive(void);
#endif
