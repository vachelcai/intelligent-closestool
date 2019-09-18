#ifndef USER_CONFIG_H
#define USER_CONFIG_H

///
///脉冲阀部分配置
///
#define MCF_PULE_MS	50				//脉冲阀 脉冲ms数
#define MCF_RB_MS		5000		//脉冲阀 润壁ms数

#define MCF_XC_U1_MS	8000		//小冲 1步
#define MCF_XC_D2_MS	5000		//
#define MCF_XC_U3_MS	5000		//

#define MCF_DC_U1_MS	8000		//大冲 1步
#define MCF_DC_D2_MS	10000		//
#define MCF_DC_U3_MS	5000		//

#define PJ_STEP_MAX		2080
#define FSF_STEP_MAX	2000

///
///喷嘴部分
///

#define PJ_MAX			2000	//4步走法
#define PJ_MIN			1400	//最小挡位位置
#define PJ_JXF			40		//齿轮钢带间隙
#define PJ_LEVEL_		((PJ_MAX-PJ_MIN)/16)
#define PJ_LEVEL(N)	(PJ_MIN+N*PJ_LEVEL_)
#define PJ_MESSAGE	50		//按摩时喷嘴波动范围
///
///分水阀部分
///
#define FSF_MAX				2000	//8步走法
#define FSF_ZEROP_POS	1100	//中间挡位
#define FSF_MIN				400		//最小挡位
#define FSF_JXF				30		//分水阀间隙
#define FSF_LEVEL_	((FSF_MAX-PJ_MIN-FSF_ZEROP_POS)/10)	//留一个空位
#define FSF_LEVEL(N,ISW)	(ISW?(FSF_ZEROP_POS+FSF_MIN-N*FSF_LEVEL_):(FSF_ZEROP_POS-FSF_MIN+N*FSF_LEVEL_))

#endif
