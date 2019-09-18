#ifndef USER_CONFIG_H
#define USER_CONFIG_H

///
///���巧��������
///
#define MCF_PULE_MS	50				//���巧 ����ms��
#define MCF_RB_MS		5000		//���巧 ���ms��

#define MCF_XC_U1_MS	8000		//С�� 1��
#define MCF_XC_D2_MS	5000		//
#define MCF_XC_U3_MS	5000		//

#define MCF_DC_U1_MS	8000		//��� 1��
#define MCF_DC_D2_MS	10000		//
#define MCF_DC_U3_MS	5000		//

#define PJ_STEP_MAX		2080
#define FSF_STEP_MAX	2000

///
///���첿��
///

#define PJ_MAX			2000	//4���߷�
#define PJ_MIN			1400	//��С��λλ��
#define PJ_JXF			40		//���ִָ���϶
#define PJ_LEVEL_		((PJ_MAX-PJ_MIN)/16)
#define PJ_LEVEL(N)	(PJ_MIN+N*PJ_LEVEL_)
#define PJ_MESSAGE	50		//��Ħʱ���첨����Χ
///
///��ˮ������
///
#define FSF_MAX				2000	//8���߷�
#define FSF_ZEROP_POS	1100	//�м䵲λ
#define FSF_MIN				400		//��С��λ
#define FSF_JXF				30		//��ˮ����϶
#define FSF_LEVEL_	((FSF_MAX-PJ_MIN-FSF_ZEROP_POS)/10)	//��һ����λ
#define FSF_LEVEL(N,ISW)	(ISW?(FSF_ZEROP_POS+FSF_MIN-N*FSF_LEVEL_):(FSF_ZEROP_POS-FSF_MIN+N*FSF_LEVEL_))

#endif
