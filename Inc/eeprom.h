#ifndef EEPROM_H
#define EEPROM_H

#include "main.h"

//设备配置部分
#define PRO_SET_ADD								(0x800FC00)
#define PRO_SET_waterTemperature	(PRO_SET_ADD+0x00)
#define PRO_SET_windWarm					(PRO_SET_ADD+0x04)
#define PRO_SET_pjpos							(PRO_SET_ADD+0x08)
#define PRO_SET_FSFpos						(PRO_SET_ADD+0x0c)
#define PRO_SET_setTemperature		(PRO_SET_ADD+0x10)
#define PRO_SET_message						(PRO_SET_ADD+0x14)
#define PRO_SET_startHotWater			(PRO_SET_ADD+0x18)
#define PRO_SET_RC_ADD						(PRO_SET_ADD+0x1c)
#define PRO_SET_LED_ON						(PRO_SET_ADD+0x20)
//记录部分
//记录格式 uint32_t 	0x XX XX XX XX 	坐下时间
//					uint32_t	0x XX XX XX XX 	开始清洗时间
//					uint32_t  0x 00 00 00 XX 	清洗动作(女洗,臀洗)
//					uint32_t	0x XX XX XX XX	烘干启动时间
//					uint32_t  0x 00	00 00	XX 	是否烘干
//					uint32_t	0x XX XX XX XX 	离开时间
//用于记录部分区域

//健康记录部分
// 记录条数 	uint32_t 0x 00 00 00 xx 	数目
//	记录结构	0x00, dotype,		begin_tick,	do_tick,			hg_tick,			end_tick,		sys_begin_tick, 0x00
//						动作类型	开始时间		动作开始时间	烘干开始时间	离座时间		时间戳(秒 s为单位)
// 每个记录为 8*4 字节
// 一个page 记录 32条记录
//	最多6页记录,记录逆着来,从倒数第4页开始记录,第二页记录数据条数,第三页为交替用(暂时不用),
//
#define JK_NUM_ADD								(0x800f800)
#define JK_READ_NUM								(0x800f800)
#define JK_WRITE_NUM							(0x800f804)
#define JK_MAX_NUM								32*6
#define	JK_LIST_BEGIN_ADD					(0x800e000)		//开始地址

#endif
