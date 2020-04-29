/********************************************************************
 * $I
 * @Technic Support: <sdk@isurestar.com>
 * All right reserved, Sure-Star Coop.
 ********************************************************************/
#ifndef _ICD_USBCAMERA_H_
#define _ICD_USBCAMERA_H_


#pragma pack(2)

const int USBCmr_Brightness_Max = 15;
const int USBCmr_Brightness_Min = 0;
const int USBCmr_Contrast_Max = 15;
const int USBCmr_Contrast_Min = 0;
const int USBCmr_Hue_Max = 10;
const int USBCmr_Hue_Min = -10;
const int USBCmr_Saturation_Max = 15;
const int USBCmr_Saturation_Min = 0;
const int USBCmr_Sharpness_Max = 15;
const int USBCmr_Sharpness_Min = 0;
const int USBCmr_Gamma_Max = 10;
const int USBCmr_Gamma_Min = 1;
const int USBCmr_WhiteBalance_Max = 2800;
const int USBCmr_WhiteBalance_Min = 0;

const int USBCmr_Brightness_DFLT = 7;
const int USBCmr_Contrast_DFLT = 7;
const int USBCmr_Hue_DFLT = 0;
const int USBCmr_Saturation_DFLT = 6;
const int USBCmr_Sharpness_DFLT = 2;
const int USBCmr_Gamma_DFLT = 7;
const int USBCmr_WhiteBalance_DFLT = 1600;


typedef struct {
    int index;
    int brightness;     
    int contrast;
    int hue;
    int saturation;
    int sharpness;
    int gamma;
    int wbalance;
    bool trigger;
} USBCMR_CTRL_S;



typedef enum {
  eUsbCamAuto = 0 ,    //!< 自动模式
  eUsbCamHand ,        //!< 手动模式
  eUsbCamTorRed ,      //!< 红通道
  eUsbCamTorgreen ,    //!< 绿通道
  eUsbCamTorBlue ,     //!< 蓝通道
  eUsbCamTorAll ,      //!< 所有通道
  eUsbCamUnSingle ,    //!< 非触发，单帧模式
} USBCMR_M_E ;

//////////////////////////////////////////////////////////////////////////


#pragma pack()

#endif // _ICD_USBCAMERA_H_
