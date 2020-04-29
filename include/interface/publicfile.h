#ifndef publicfile_h__
#define publicfile_h__


enum DEVICE_TYPE__
{
	e_onboard = 1,//机载
	e_vehiclemounted,//车载
	e_onground,//地面
};

#ifdef M_RFANS_DEFINE
	#define _UI_RFans_PANEL
	#define _USE_MATH_DEFINES
	#define _UIEDIT
	#define _PRE
	#define _PRE_GPS
	#define _PRE_POS
	#define RFAN_DEVICE
	#define M_RFANS
	#define _TWS
#endif // UIRFans_DEFINE

#ifdef RFANS_DEFINE
	#define _UI_RFans_PANEL
	#define _USE_MATH_DEFINES
	#define _UIEDIT
	#define _PRE
	#define _PRE_GPS
	#define _PRE_POS
	#define RFAN_DEVICE
	#define _TWS

#endif // UIRFans_DEFINE




#endif // publicfile_h__
