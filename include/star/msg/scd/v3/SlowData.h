/**
 * @author   lucb
 * @date     2019/12/12
 */

#ifndef __STAR_SDK_MSG_SCD_VER3_SLOW_DATA_H
#define __STAR_SDK_MSG_SCD_VER3_SLOW_DATA_H

#include <star/msg/Message.h>
#include <star/msg/scd/SCDMessage.h>
#include <star/msg/scd/SCDSlowData.h>

namespace ss {
namespace msg {
namespace scd {
namespace v3 {

#pragma pack(push, 2)

typedef struct {
	uint16_t dataId : 16; //8
	//uint16_t reserve       :12;
	uint16_t utc_HighL : 16;
	uint16_t utc_HighH : 16;
	uint16_t utc_LowL : 16;
	uint16_t utc_LowH : 16;
	uint16_t error_stamp : 16;
	uint16_t deviceID : 8;
	uint16_t error_statusH : 8;
	uint16_t error_statusL : 16;
} SCD_REPORT_S;

typedef struct {
	uint16_t dataId : 16; //8
	//uint16_t reserve       :12;
	uint32_t utc_High : 32;
	uint32_t utc_Low : 32;
	uint16_t config_stamp : 16;
	uint32_t config_content : 32;
	uint16_t DMI_selcount : 16;
} SCD_CONFIG_REPORT_S;

/***********Camera Data Structure*********/
typedef struct {
	uint16_t dataId : 16; //6
	uint16_t cmrID : 4;
	uint16_t reserve : 12;
	uint16_t t0stampL : 16;
	uint16_t t0stampH : 16;
	uint16_t TurnAngle : 16;
} SCD_CAMERA_S;

typedef struct {
	uint16_t cmrID : 4;
	uint16_t dataId : 12; //6   2
	// uint16_t reserve :12;
	uint32_t utcHigh : 32;      //8
	uint32_t utcLow : 32;
	uint16_t t0stamp : 16;     //2
	uint32_t flashstamp : 32;    //4
	uint16_t TurnAngleL : 16;  //2
	uint16_t TurnAngleH : 6;   //2
	uint16_t reserved : 10;
} SCD_CAMERASLW_S;

/***********AMCW Data Structure*********/
typedef struct {
	uint16_t dataId : 16;//10
	uint16_t reserved : 4;
	uint16_t t0stamp : 12;

	uint16_t hl : 4;
	uint16_t peak_ptim : 12;

	uint16_t ph1_reh : 12;
	uint16_t peak_ctimh : 4;

	uint16_t peak_ctimm : 4;
	uint16_t ph1_rel : 12;

	uint16_t ph1_imh : 16;

	uint16_t ch1_reh : 4;
	uint16_t peak_ctiml : 4;
	uint16_t ph1_iml : 8;

	uint16_t ch1_rem : 16;

	uint16_t cl1_imh : 12;
	uint16_t ch1_rel : 4;

	uint16_t cl1_iml : 16;

	uint16_t ph2_reh : 16;

	uint16_t ph2_imh : 4;
	uint16_t ph2_rel : 12;

	uint16_t ph2_imm : 16;

	uint16_t ch2_reh : 8;
	uint16_t ph2_iml : 8;

	uint16_t ch2_rem : 16;

	uint16_t cl2_imh : 12;
	uint16_t ch2_rel : 4;

	uint16_t cl2_iml : 16;

	uint16_t mirIntgr : 14;
	uint16_t mirDprdH : 2;

	uint16_t mirDprdL : 7;
	uint16_t mirDrsd : 9;
} SCD_AMCWECHO_S;

/***********InterFill Data Structure*********/

typedef struct {
	uint16_t dataId : 16; //6
	//uint16_t dump :12;
	uint16_t ppsStampL : 16;
	uint16_t ppsStampH : 16;
	uint16_t utcHighL16;
	uint16_t utcHighH : 16;
	uint16_t utcLowL : 16;
	uint16_t utcLowH : 16;
} SCD_INTERDUMP_S;

/********Fill Data Structure***********/
typedef struct {
	uint16_t dataId : 16;//5
	uint16_t reserve0 : 4;
	uint16_t reduNum : 12; //redundance
} SCD_DUMPING_S;

/*******************  Enviroment Data Structure ******/
typedef struct {
	uint16_t dataId : 16;//10
	// uint16_t t0stamp : 12;

	//uint16_t stampH :16;
	//uint16_t stampL :16;
	uint16_t heightH : 16;
	uint16_t heightL : 16;
	uint16_t speedH : 16;
	uint16_t speedL : 16;
	uint16_t tempAH : 16;
	uint16_t tempAL : 16;
	uint16_t tempBH : 16;
	uint16_t tempBL : 16;
	uint16_t tempCH : 16;
	uint16_t tempCL : 16;
} SCD_ENVIR_S;

typedef struct {
	uint16_t dataId : 16;//10
	//uint16_t reserve       :12;
	uint16_t t0stampL : 16;
	uint16_t t0stampH : 16;

	uint16_t X1 : 16;
	uint16_t X2 : 16;
	uint16_t X3 : 16;
	uint16_t X4 : 16;
	uint16_t X5 : 16;
	uint16_t X6 : 16;
	uint16_t X7 : 16;
	uint16_t X8 : 16;
	uint16_t X9 : 16;

	uint16_t Odo_velH : 16;
	uint16_t Odo_velL : 16;
	uint16_t Odo_countL : 16;
	uint16_t Odo_countH : 16;
} SCD_IMU_S;

//GPS_data
const uint32_t GPS_DATA_SIZE = 48;
typedef struct {
	uint16_t dataId : 16;//8
	uint8_t gpsData[GPS_DATA_SIZE];
} SCD_GPS_S;

typedef struct {
	uint32_t syncWord; // 0xA7A7A7A7 - zks: being embedded the turret identify within
	uint16_t packNo;
	uint16_t packSize;
} SCD_PACK_S;

typedef struct {
	uint32_t syncWord; // 0xB7B7B7B7 - zks: being embedded the turret identify within
	uint32_t count;
} SCD_SIMULATE_S;

typedef struct {
	uint32_t syncWord; // 0xE7E7E7E7 -
	uint32_t ppsStamp;
	uint32_t utcHigh;
	uint32_t utcLow;
	uint16_t utcCount;//FPGA根据 timeEnable的使能，若不使能，结构体大小- 4个字节，即不打包utcCount和ppsCount
	uint16_t ppsCount;
} SCD_SYNCHRON_S;

//POS2010_data
const uint32_t POS2010_DATA_SIZE = 82;
typedef struct {
	uint16_t dataId : 16;
	uint8_t posData[POS2010_DATA_SIZE];
} SCD_POS2010_S;

typedef struct {
	uint16_t dataId : 16; //6
	//uint16_t dump :12;
	uint16_t t0stampH : 16;
	uint16_t t0stampL : 16;
	uint16_t countH : 16;
	uint16_t countL : 16;
} SCD_DMI_S;

typedef struct {
	uint16_t DMI_ID : 4; //2
	uint16_t dataId : 12;
	//uint16_t reserved :12 ;
	uint32_t utcHigh;//8
	uint32_t utcLow;
	uint16_t t0stamp : 16; //2
	uint16_t countL : 16;   //2
	uint16_t countH : 16;   //2
} SCD_DMISLW_S;

typedef struct {
	uint16_t dataId : 16; //2
	uint32_t LCI_stamp : 32; //32
	uint32_t SYNC_Counter : 32;
	uint32_t X_angular_inc : 32;
	uint32_t Y_angular_inc : 32;
	uint32_t Z_angular_inc : 32;
	uint32_t X_velocity_inc : 32;
	uint32_t Y_velocity_inc : 32;
	uint32_t Z_velocity_inc : 32;
	uint16_t Gyro_Status : 8; //2
	uint16_t Accel_Status : 8;
} SCD_IMU_LC100_S;

const uint32_t DMI_DATA_SIZE = 20;
typedef struct {
	uint16_t dataId : 16; //2
	uint32_t DMI_stamp : 32; //32
	uint8_t dmiData[DMI_DATA_SIZE];
} SCD_DMI_ZT_S;

typedef struct {
	uint16_t dataId : 16;//0
	uint16_t dataIdcheck : 16;
	uint32_t motorStamp : 32;
	uint16_t motorLine : 16;
} SCD_MOTOR_S;

typedef struct {
	uint16_t dataId : 16; //6
	uint32_t zeroStamp : 32;
	uint32_t squareCounter : 32;
}SCD_CTMDATA_S;  //RW 零位信号脉冲计数

#pragma pack(pop)

class Report : public scd::Message<SCD_REPORT_S>,
	    public scd::ReportMessage {
public:
	inline uint64_t dataId() const override
	{
		return this->data().dataId;
	}

	fast_data get() const override;
	void set(const fast_data& data) override;

};

class ConfigReport : public scd::Message<SCD_CONFIG_REPORT_S>,
        public scd::ConfigReportMessage {
public:
	inline uint64_t dataId() const override
	{
		return this->data().dataId;
	}

    fast_data get() const override;
    void set(const fast_data& data) override;
};

class Environment : public scd::Message<SCD_ENVIR_S>,
        public scd::EnvironmentMessage {
public:
	inline uint64_t dataId() const override
	{
		return this->data().dataId;
	}

    fast_data get() const override;
    void set(const fast_data& data) override;
};

class PackageSign : public scd::Message<SCD_PACK_S>,
		public scd::PackageSignMessage {
public:
	inline uint64_t dataId() const override
	{
		return this->data().syncWord;
	};

	fast_data get() const override;
	void set(const fast_data& data) override;
};

class Simulation : public scd::Message<SCD_SIMULATE_S>,
		public scd::SimulationMessage {
public:
	inline uint64_t dataId() const override
	{
		return this->data().syncWord;
	}

	fast_data get() const override;
	void set(const fast_data& data) override;
};

class Synchron : public scd::Message<SCD_SYNCHRON_S>,
		public scd::SynchronMessage {
public:
	inline uint64_t dataId() const override
	{
		return this->data().syncWord;
	}

	fast_data get() const override;
	void set(const fast_data& data) override;
};

class DMI : public scd::Message<SCD_DMISLW_S>,
    public scd::DMIMessage {
public:
	inline uint64_t dataId() const override
	{
		return this->data().dataId;
	}

    fast_data get() const override;
    void set(const fast_data& data) override;
};

class Camera : public scd::Message<SCD_CAMERASLW_S> ,
        public scd::CameraMessage {
public:
	inline uint64_t dataId() const override
	{
		return this->data().dataId;
	}

    fast_data get() const override;
    void set(const fast_data& data) override;
};

//May be not used
class IMU_Pos2010 : public scd::Message<SCD_POS2010_S> ,
        public scd::IMUMessage {
public:
	inline uint64_t dataId() const override
	{
		return this->data().dataId;
	}

    fast_data get() const override;
    void set(const fast_data& data) override;
};

class GPS_R15 : public scd::Message<SCD_GPS_S>,
        public scd::GPSMessage {
public:
	inline uint64_t dataId() const override
	{
		return this->data().dataId;
	}

    fast_data get() const override;
    void set(const fast_data& data) override;
};

class IMU_LC100 : public scd::Message<SCD_IMU_LC100_S>,
        public scd::IMUMessage {
public:
	inline uint64_t dataId() const override
	{
		return this->data().dataId;
	}

    fast_data get() const override;
    void set(const fast_data& data) override;
};

class DMI_DIY : public scd::Message<SCD_DMI_ZT_S>,
        public scd::DMIMessage{
public:
	inline uint64_t dataId() const override
	{
		return this->data().dataId;
	}

    fast_data get() const override;
    void set(const fast_data& data) override;
};

class Motor : public scd::Message<SCD_MOTOR_S> {
public:
	inline uint64_t dataId() const override
	{
		return this->data().dataId;
	}
};

class DumpData : public Message<SCD_DUMPING_S> {
public:
	inline uint64_t dataId() const override
	{
		return this->data().dataId;
	}
};

class CustomerData : public Message<SCD_CTMDATA_S> {
public:
	inline uint64_t dataId() const override
	{
		return this->data().dataId;
	}
};

}
}
}
}

#endif //__STAR_SDK_MSG_SCD_VER3_SLOW_DATA_H
