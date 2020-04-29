/**
 * @author xiaoma 
 * @date 2020/2/3
 */

#ifndef __STAR_CONFIGURE_H
#define __STAR_CONFIGURE_H

#include <star/Star.h>

#include <star/Lidar.h>
#include <star/utils/variant.hpp>
#include <star/utils/factory.h>

#include <cstdint>
#include <string>
#include <vector>
#include <map>
#include <list>
#include <exception>
#include <atomic>

namespace ss {
namespace msg {
namespace scd {

namespace v1 {
    class Metadata;
}

namespace v3 {
	class Metadata;
}

}
}
}

namespace ss {
/**
 * 配置异常
 */
class __star_export configure_exception : public std::exception {
public:
    explicit configure_exception(const std::string& msg);
    const char* what() const noexcept override;
private:
    std::string _what;
};

/**
 * 未找到配置项异常
 */
class __star_export property_not_found_error : public configure_exception {
public:
    property_not_found_error(const std::string& path, const std::string& property);
};

/**
 *  配置项类型转换错误异常
 */
class __star_export property_convert_failed_error : public configure_exception {
public:
    property_convert_failed_error(const std::string& from, const std::string& to);
};

class Property;
class __star_export Settings {
public:
    using Array         = std::vector<Property>;
    using Value         = mpark::variant<int64_t, double, std::string, Array, Settings>;
    using name_list     = std::list<std::string>;
    using property_tree = std::map<std::string, Property>;

    Property& get(const std::string& path);
    const Property& get(const std::string& path) const;

    int8_t get(const std::string& path, int8_t defaultValue) const;
    uint8_t get(const std::string& path, uint8_t defaultValue) const;
    int16_t get(const std::string& path, int16_t defaultValue) const;
    uint16_t get(const std::string& path, uint16_t defaultValue) const;
    int get(const std::string& path, int defaultValue) const;
    uint32_t get(const std::string& path, uint32_t defaultValue) const;
    int64_t get(const std::string& path, int64_t defaultValue) const;
    uint64_t get(const std::string& path, uint64_t defaultValue) const;
    float get(const std::string& path, float defaultValue) const;
    double get(const std::string& path, double defaultValue) const;
    std::string get(const std::string& path, const std::string& defaultValue) const;

//    void set(const std::string& path, const Value& value);
    Property& set(const std::string& path, const Property& value);

    void merge(const Settings& other);

    void clear();

    const name_list& names() const;

private:
    name_list     _names;
    property_tree _values;
};

/**
 * 配置属性，属性可以是整型、浮点型、字符串、数组或者一组配置属性
 * 配置文件解析后存储为树状结构，Property为树的节点
 */
class __star_export Property {
public:
    using Array      = std::vector<Property>;
    using Value      = mpark::variant<int64_t, double, std::string, Array, Settings>;

    Property();
    Property(const Value& value);
    Property(int8_t value);
    Property(uint8_t value);
    Property(int16_t value);
    Property(uint16_t value);
    Property(int value);
    Property(uint32_t value);
    Property(int64_t value);
    Property(uint64_t value);
    Property(double value);
    Property(const char* value);
    Property(const std::string& value);
    Property(const Array& value);
    Property(const Settings& value);

    /**
     * 获取指定路径的配置
     * @param path 指定的路径，路径用.来隔开，如"configure.sample"
     * @return 获取到的配置属性，不存在则抛出
     */
    Property& get(const std::string& path);
    const Property& get(const std::string& path) const;

    bool isArray() const;
    bool isSettings() const;
    bool isString() const;
    bool isFloating() const;
    bool isInteger() const;

    int64_t getInt()const;
    double getFloating() const;
    std::string getString() const;
    Array& getArray();
    const Array& getArray() const;
    Settings& getSettings();
    const Settings& getSettings() const;

    std::string toString() const;

    //类型转换函数，允许隐士类型转换，转换失败抛出异常
    operator bool() const;
    operator int8_t() const;
    operator uint8_t() const;
    operator int16_t() const;
    operator uint16_t() const;
    operator int() const;
    operator uint32_t() const;
    operator int64_t() const;
    operator uint64_t() const;

    operator float() const;
    operator double() const;
    operator std::string() const;
    operator Array&();
    operator const Array&() const;
    operator Settings&();
    operator const Settings&() const;

    //赋值函数
    //void set(const std::string& path, const Value& value);
    void set(const std::string& path, const Property& value);

    Value& value();
    const Value& value() const;

private:
    Value                   _value;
};

class __star_export Register {
public:
    using Value = mpark::variant<int64_t, double>;
    Register();
    Register(int8_t value);
    Register(uint8_t value);
    Register(int16_t value);
    Register(uint16_t value);
    Register(int value);
    Register(uint32_t value);
    Register(int64_t value);
    Register(uint64_t value);
    Register(double value);

    int64_t getInt() const;
    double getDouble() const;
    std::string toString() const;

    operator bool() const;
    operator int8_t() const;
    operator uint8_t() const;
    operator int16_t() const;
    operator uint16_t() const;
    operator int() const;
    operator uint32_t() const;
    operator int64_t() const;
    operator uint64_t() const;

    operator float() const;
    operator double() const;

    bool valid() const;

private:
    bool  _initialized;
    Value _value;
};

namespace cfg {

template<typename _Type>
class Builder : public ss::utils::object_factory<int, _Type, std::allocator<_Type>> {
//private:
public:
    using parent_type = ss::utils::object_factory<int, _Type, std::allocator<_Type>>;
    _Type* get(int id) const
    {
        return this->make(id);
    }
};

#if defined(__GUNC__) || defined(__clang__)
#pragma GCC diagnostic pop
#endif

class Reader;
class Writer;
class Visitor;

using ReaderBuilder = Builder<Reader>;
using WriterBuilder = Builder<Writer>;
using VisitorBuilder = Builder<Visitor>;

    template <typename _Tp>
    struct data_range {
        using data_type = _Tp;
        data_type min = {0};
        data_type max = {0};
    };

    struct DeviceVersion {
        lidar::DeviceModel    type;
        uint32_t        date = {0};
        uint32_t        reversion = {0};
    };

    struct Environment {
        lidar::GpsType gpsType;
        uint32_t       utc = {0};
    };

    struct LaserControl {
        uint32_t freq = {0};
        uint32_t power = {0};
        int  temperature = {0};
    };

    struct ScannerControl {
        lidar::ScanMode     mode;
        float               angleSpeed = {0};
        uint32_t            speedCount = {0};   //扫描电机count
        lidar::ScanState    speedState;  //扫描电机转速状态
        int                 grade = {0};
    };

    struct TurretControl {
        lidar::TurretMode   mode;
        float               angleSpeed = {0};
        float               startAngle = {0};
        float               stopAngle = {0};
        float               angle = {0};
    };

    struct DeviceControl {
        LaserControl    laser;
        ScannerControl  scanner;
        TurretControl   turret;
    };

    struct LaserConfigure {
        int type = {0};
        data_range<int> freqLimit;
        data_range<int> elecLimit;
        data_range<int> userPowerLimit;
        data_range<int> rangeNearLimit;
        data_range<int> workPowerLimit;
    };

    struct ScannerConfigure {
        int                 type = {0};
        data_range<double>  speedLimit;
        int                 coder = {0};
        float               zeroAngle = {0};
        uint16_t            speedCheck = {0};
        uint32_t            period = {0};
        uint32_t            angleStart[4] = {0};
        uint32_t            angleStop[4] = {0};
    };

    struct TurretConfigure {
        int                 type = {0};
        int                 coder = {0};
        data_range<double>  speedLimit;
        float               zeroAngle = {0};
        uint32_t            angleStart[4] = {0};
        uint32_t            angleStop[4] = {0};
    };

    struct RulerConfigure {
        lidar::RangeMode    rangeMode;          /// 测距模式
        lidar::MpiaMode     mpiaMode;           /// 工作模式
        lidar::TZeorMode    tzeroMode;          /// 测量信号沿

        float               pulseWidthGpxA = {0};     /// GPXA 脉冲宽度 liyp  过时
        float               pulseWidthGpxB = {0};     /// GPXB 脉冲宽度 liyp  过时

        int                 peakSync = {0};           /// 高倍峰值同步
        int                 peakSyncLow = {0};        /// 低倍峰值同步L
        int                 simuRatio = {0};          /// 仿真数据分配
        int                 tdcaFact = {0};           /// TDC_A精度系数
        int                 tdcbFact = {0};           /// TDC_B精度系数
        int                 ef = {0};                 /// 单回波精度系数
        float               af = {0};                 /// 指南针磁偏角  liyp /*电子水泡精度系数*/
        uint32_t            packetEnable = {0};       /// 数据打包使能
        uint16_t            selectPeak = {0};
        uint16_t            scdVersion = {0};
        uint16_t            timeDelay = {0};          /// 时间窗口延时
        uint16_t            pulseCtrl = {0};
        uint16_t            pulseDelay = {0};
        uint16_t            selectPeakL = {0};
    };

    struct Program {
        lidar::EchoType     echoType;
        lidar::ScanType     scanType;
        lidar::ProgramMode  programMode ;
    };

    struct DeviceConfigure {
        lidar::DeviceModel     type;
        Program          program;
        LaserConfigure   laser;
        ScannerConfigure scanner;
        TurretConfigure  turret;
        RulerConfigure   ruler;
    };

    //解码输出
    struct DebugOutput {
        bool package_no = false;
        bool t0_pos = false;
        bool t0_stage = false;
        bool t0_inclt = false;
        bool angle_range_int = false;
        bool rise_fall = false;
        bool t0_camera = false;
        bool range_a = false;
        bool range_b = false;
        bool envir = false;
        bool amcw = false;
        bool pc = false;
        bool gps = false;
        bool imu = false;
        bool dmi = false;
        bool pos2010 = false;
        bool pack = false;
        bool pack_slow = false;
        bool simulate = false;
        bool simulate_slow = false;
        bool error = false;
        bool pack_config = false;
        bool imu_lc100 = false;
    };

	//有效距离
	struct RangeFilter {
		bool enable = false;
		data_range<double> range;
	};

	//距离灰度滤波
	struct RangeIntensityFilter {
		bool enable = false;
		data_range<uint32_t> range;
		data_range<uint32_t> intensity;
	};

	struct AngleFilter {
		bool enable = false;
		bool isInvert = false;
		data_range<double> angle;
	};

    ////距离灰度滤波
    //struct RangeIntensityFilter {
    //    bool enable;
    //    data_range<uint32_t> range[3][3];
    //    data_range<uint32_t> intensity[3][3];
    //};

	//0x3006
	//struct RangeByIntensityFilter {
	//	struct RangeByIntensity {
	//		double range;
	//		double intensity;
	//	};
	//	struct Filter {
	//		RangeByIntensity map[6]; //R0 I0 ~ R5 I5
	//		data_range<double> pointFreq;
	//	};
	//	Filter filter[3];
	//};
	struct RangeByIntensityFilter {
        bool enable = false;
		struct Filter {
			double range[4] = {0};
			double intensity[6] = {0};
			data_range<double> pointFreq;
		};
		Filter filter[3];
	};

	//	点频角度距离灰度滤波
	struct PointFreqAngleRangeFilter {
	    bool enable = false;
	    data_range<double> angle[5];
	    data_range<double> freq[3];
		data_range<double> range[3][3];
		data_range<double> intensity[3][3];
	};

    //高度滤波
    struct HeightFilter {
        bool enable = false;
        data_range<double> height;
    };

    //抽稀比
    struct Resample {
        bool enable = {0};
        uint32_t ratio = {0};
    };

	//距离常数---测绘设备
	struct RangeConstPara {
		struct MultiAddConst{
			double multiConstA = {0};
			double addConstA = {0};
			double multiConstB = {0};
			double addConstB = {0};
		};
		MultiAddConst multiAddConst[4];
	};

	//距离加常数---导航设备
	struct LaserRangeConstPara {
		double rangeconst[32] = {0};
	};

    struct RangeTemperPara {
        data_range<double> temperature;
        double rangeCoff[3] = {0};
    };

    const int INTENSE_MAX = 2048;
    struct IntensityTable {
        float riArrayA[INTENSE_MAX] = {0};  // gpxA灰度范围[0-2047]     //50k
        float riArrayB[INTENSE_MAX] = {0}; // gpxB灰度范围[0-2047]
        float riArrayC[INTENSE_MAX] = {0};  // gpxA灰度范围[0-2047]     //300k
        float riArrayD[INTENSE_MAX] = {0}; // gpxB灰度范围[0-2047]
        float riArrayE[INTENSE_MAX] = {0};  // gpxA灰度范围[0-2047]     //500k
        float riArray9[INTENSE_MAX] = {0}; // gpxB灰度范围[0-2047]
    };

    struct CommonCalibParams {
        double s = {0};
        double h = {0};
        double d[2] = {0};
        double b0 = {0};
        double q[4] = {0};
    };

    struct ApCalibParams {
        struct Parameters {
            double H = {0};		///<入射光在XOY平面内的偏转角
            double V = {0};		///<入射光在XOZ平面内的偏转角
            double d0 = {0};	    ///<测距修正值
            double db[3] = {0};		///<测角修正值
            double f = {0};		///<镜面法线在XOZ面内的安装偏角
            double ryy = {0};
            double rzz = {0};
        };
        Parameters parameters[4];
    };

    struct RaCalibParams {
        double H = {0};
        double V = {0};
        double d0 = {0};
        double db[3] = {0};
        double f[9] = {0};
        double w = {0};
    };

    struct AkCalibParams {
        double H = {0};
        double V = {0};
        double d0 = {0};
        double db[3] = {0};
        double f = {0};
        double q[3] = {0};
    };

	struct UaCalibParams {
		double k[8] = {0};
		double d = {0};
		double db[3] = {0};
		double f[3] = {0};
		double V = {0};
		double H = {0};
	};

	struct StageCalibParams {
		struct StagePara
		{
		  double angle = {0};
		  double dIx = {0};
		  double dIy = {0};
		};
		StagePara stagePara[8];
	};

	//interplation
	struct Interpolation {
		bool enable = false;
		//int gu_diff;
		int maxUtcGap = {0};
		int maxIncltGap = {0};
		int utcOffset = {0};
		int gridSize = {0};
	};

    //安装角安置角偏心量
    struct PosCmpCoor {
        bool     enable = false;
        double   utc2gps = {0};
		int8_t clcWise[3] = {0};
		double anlignAngle[3] = {0};
		double deltAngle[3] = {0};
		double deltXYZ[3] = {0};
		int8_t zDirection = {0};
    };

    struct ApCmpCoor {
        bool isSeparate = false;
        bool isOneMirror = false;
        bool isTwoMirror = false;
        bool isThrMirror = false;
        bool isFourMirror = false;
        double viewAngle = {0};
        double xSpeed = {0};
    };

    struct GeoProjection {
        bool        enable = false;
        uint32_t    UTM_GUSS = {0};
        uint32_t    ecllipse = {0};
        uint32_t    centralMeridian = {0};
        uint32_t    centralLatitude = {0};
        bool        isProjection = false;
        int         projectiontype = {0};
        int         ellipsolidtype = {0};
    };

    struct GpsUAPosition {
        bool enable = false;
        uint32_t centralMeridian = {0};
        uint32_t B = {0};
        uint32_t L = {0};
        uint32_t H = {0};
    };

    struct OrientUA {
        bool enable = false;
        uint32_t Hs = 0;
        uint32_t pointStart[3] = {0};
        uint32_t pointEnd[3] = {0};
        double turnAngle = 0;
        uint32_t deltAngle[3] = {0};
        uint32_t deltXYZ[3] = {0};
    };

	struct AnlignAngleRFans {
		double deltRoll[32] = {0};
		double deltPitch[32] = {0};
		double deltHeading[32] = {0};
	};

}

class __star_export Configure {
public:
    using Value = Property::Value;
    using Array = Property::Array;
    using Registers = std::vector<Register>;
//    using Revise = std::vector<Register>;

    /**
     * 配置文件存放的格式
     */
    enum Format {
        COMPAT_FORMAT_OLD,
        COMPAT_FORMAT,
        INI_FORMAT
    };

    Configure();
    explicit Configure(Format format);

    Configure(const Configure& configure) = delete;
    Configure& operator=(const Configure& configure) = delete;

    Configure(Configure&& configure) = delete;
    Configure& operator=(Configure&& configure) = delete;

    ~Configure();

    Format format() const;

    void changeFormat(Format format);

    // void enableMergeFromMetadata(bool enable);
    // bool isMergeFromMetadataEnabled() const;

    /**
     * 从文件中加载配置信息
     * @param path 文件路径
     */
    void loadFile(const std::string& path);

    /**
     * 从字符串中加载配置信息
     * @param source 包含配置信息的字符串
     */
    void loadSource(const std::string& source);

    bool loadMetadata(const msg::scd::v1::Metadata& metadata);
	bool loadMetadata(const msg::scd::v3::Metadata& metadata);

    bool storeToFile(const std::string& path) const;

    std::string storeToSource() const;

    Property& get(const std::string& path);
    const Property& get(const std::string& path) const;

    void merge(const Configure& other);

    int8_t get(const std::string& path, int8_t defaultValue) const;
    uint8_t get(const std::string& path, uint8_t defaultValue) const;
    int16_t get(const std::string& path, int16_t defaultValue) const;
    uint16_t get(const std::string& path, uint16_t defaultValue) const;
    int get(const std::string& path, int defaultValue) const;
    uint32_t get(const std::string& path, uint32_t defaultValue) const;
    int64_t get(const std::string& path, int64_t defaultValue) const;
    uint64_t get(const std::string& path, uint64_t defaultValue) const;
    float get(const std::string& path, float defaultValue) const;
    double get(const std::string& path, double defaultValue) const;
    std::string get(const std::string& path, const std::string& defaultValue) const;
	
    /**
     * 设置配置项的值
     * @param path 配置项的路径
     * @param value 配置项的值
     */
    void set(const std::string& path, const Property& value);

    /**
     * 设置寄存器的值
     * @param address 寄存器地址
     * @param value 设置的值，当该值可用时，覆盖旧值，不可用时，旧值保持不变
     */
    void setRegister(uint16_t address, const Register& value);

    /**
     * 清空所有配置项
     */
    void clear();

    /**
     * 获取寄存器
     * @return 返回所有的寄存器
     */
    const Registers& registers() const;

//    Registers& registers();

    Settings& settings();
    const Settings& settings() const;

    //配置项结构体获取
    cfg::DeviceVersion& deviceVersion();
    const cfg::DeviceVersion& deviceVersion() const;

    cfg::Environment& environment();
    const cfg::Environment& environment() const;

    cfg::DeviceConfigure& deviceConfigure();
    const cfg::DeviceConfigure& deviceConfigure() const;

    cfg::DeviceControl& deviceControl();
    const cfg::DeviceControl& deviceControl() const;

    cfg::DebugOutput &debugOutput();
    const cfg::DebugOutput &debugOutput() const;

    cfg::RangeIntensityFilter& rangeIntensityFilter();
    const cfg::RangeIntensityFilter& rangeIntensityFilter() const;

    cfg::RangeFilter& rangeFilter();
    const cfg::RangeFilter& rangeFilter() const;

    cfg::AngleFilter& angleFilter();
    const cfg::AngleFilter& angleFilter() const;

    cfg::HeightFilter& heightFilter();
    const cfg::HeightFilter& heightFilter() const;

    cfg::Resample& resample();
    const cfg::Resample& resample() const;

    cfg::RangeByIntensityFilter& rangeByIntensityFilter();
    const cfg::RangeByIntensityFilter& rangeByIntensityFilter() const;

    cfg::PointFreqAngleRangeFilter& pointFreqAngleRangeFilter();
    const cfg::PointFreqAngleRangeFilter& pointFreqAngleRangeFilter() const;

    cfg::RangeConstPara& rangeConstPara();
    const cfg::RangeConstPara& rangeConstPara() const;

    cfg::LaserRangeConstPara& laserRangeConstPara();
    const cfg::LaserRangeConstPara& laserRangeConstPara() const;

    cfg::RangeTemperPara& rangeTemperPara();
    const cfg::RangeTemperPara& rangeTemperPara() const;

    cfg::IntensityTable& intensityTable();
    const cfg::IntensityTable& intensityTable() const;

    cfg::CommonCalibParams& commonCalibParams();
    const cfg::CommonCalibParams& commonCalibParams() const;

    cfg::ApCalibParams& apCalibParams();
    const cfg::ApCalibParams& apCalibParams() const;

    cfg::RaCalibParams& raCalibParams();
    const cfg::RaCalibParams& raCalibParams() const;

    cfg::AkCalibParams& akCalibParams();
    const cfg::AkCalibParams& akCalibParams() const;

    cfg::UaCalibParams& uaCalibParams();
    const cfg::UaCalibParams& uaCalibParams() const;

    cfg::StageCalibParams& stageCalibParams();
    const cfg::StageCalibParams& stageCalibParams() const;

    cfg::Interpolation& interpolation();
    const cfg::Interpolation& interpolation() const;

    cfg::ApCmpCoor& apCmpCoor();
    const cfg::ApCmpCoor& apCmpCoor() const;

    cfg::PosCmpCoor& posCmpCoor();
    const cfg::PosCmpCoor& posCmpCoor() const;

    cfg::GeoProjection& geoProjection();
    const cfg::GeoProjection& geoProjection() const;

    cfg::GpsUAPosition& gpsUAPosition();
    const cfg::GpsUAPosition& gpsUAPosition() const;

    cfg::OrientUA& orientUA();
    const cfg::OrientUA& orientUA() const;

	cfg::AnlignAngleRFans& anlignAngleRFans();
	const cfg::AnlignAngleRFans& anlignAngleRFans()const;

    /**
     * 更新内容到结构体
     */
    void updateToStructure();

    /**
     * 同步到配置文件中
     */
    void updateToConfigure();

protected:
    /**
     * 由于静态库全局变量初始化的问题，建议在裸机环境中以force为true初始化
     * @param force
     */
    void initInstance();

private:
    Format              _format;
    Registers           _registers;
    Settings            _settings;
    // cfg::Visitor*       _visitor;

    cfg::DeviceVersion      _deviceVersion;
    cfg::Environment        _environment;
    cfg::DeviceConfigure    _deviceConfigure;
    cfg::DeviceControl      _deviceControl;
    cfg::DebugOutput        _debugOutput;

    cfg::RangeIntensityFilter       _rangeIntensityFilter;
    cfg::RangeFilter                _rangeFilter;
    cfg::AngleFilter                _angleFilter;
    cfg::HeightFilter               _heightFilter;
    cfg::Resample                   _resample;
    cfg::RangeByIntensityFilter     _rangeByIntensityFilter;
    cfg::PointFreqAngleRangeFilter  _pointFreqAngleRangeFilter;

    cfg::RangeConstPara             _rangeConstPara;
    cfg::LaserRangeConstPara        _laserRangeConstPara;
    cfg::RangeTemperPara            _rangeTemperPara;

    cfg::IntensityTable             _intensityTable;

    cfg::CommonCalibParams  _commonCalibParams;
    cfg::ApCalibParams      _apCalibParams;
    cfg::RaCalibParams      _raCalibParams;
    cfg::AkCalibParams      _akCalibParams;

    cfg::UaCalibParams      _uaCalibParams;
    cfg::StageCalibParams   _stageCalibParams;
    cfg::Interpolation      _interpolation;
    cfg::ApCmpCoor          _apCmpCoor;
    cfg::PosCmpCoor         _posCmpCoor;
    cfg::GeoProjection      _geoProjection;
    cfg::GpsUAPosition      _gpsUAPosition;
    cfg::OrientUA           _orientUA;
	cfg::AnlignAngleRFans _anlignAngleRFans;

	// fixme factory没有拷贝构造函数，
	// 此处最佳方案是使用静态变量的方式，
	// 但由于静态库在ZYNQ上链接存在不初始化的问题，
	// 故删除了Configure的拷贝和移动构造以保证内存安全
	cfg::ReaderBuilder	_readerBuilder;
	cfg::WriterBuilder 	_writerBuilder;
	cfg::VisitorBuilder _visitorBuilder;
};

}

#endif //__STAR_CONFIGURE_H
