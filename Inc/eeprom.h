#ifndef EEPROM_H
#define EEPROM_H

#include "main.h"

//�豸���ò���
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
//��¼����
//��¼��ʽ uint32_t 	0x XX XX XX XX 	����ʱ��
//					uint32_t	0x XX XX XX XX 	��ʼ��ϴʱ��
//					uint32_t  0x 00 00 00 XX 	��ϴ����(Ůϴ,��ϴ)
//					uint32_t	0x XX XX XX XX	�������ʱ��
//					uint32_t  0x 00	00 00	XX 	�Ƿ���
//					uint32_t	0x XX XX XX XX 	�뿪ʱ��
//���ڼ�¼��������

//������¼����
// ��¼���� 	uint32_t 0x 00 00 00 xx 	��Ŀ
//	��¼�ṹ	0x00, dotype,		begin_tick,	do_tick,			hg_tick,			end_tick,		sys_begin_tick, 0x00
//						��������	��ʼʱ��		������ʼʱ��	��ɿ�ʼʱ��	����ʱ��		ʱ���(�� sΪ��λ)
// ÿ����¼Ϊ 8*4 �ֽ�
// һ��page ��¼ 32����¼
//	���6ҳ��¼,��¼������,�ӵ�����4ҳ��ʼ��¼,�ڶ�ҳ��¼��������,����ҳΪ������(��ʱ����),
//
#define JK_NUM_ADD								(0x800f800)
#define JK_READ_NUM								(0x800f800)
#define JK_WRITE_NUM							(0x800f804)
#define JK_MAX_NUM								32*6
#define	JK_LIST_BEGIN_ADD					(0x800e000)		//��ʼ��ַ

#endif
