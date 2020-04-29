/**
 * @author   lucb
 * @date     2020/2/28
 */

#ifndef __STAR_SDK_MSG_CMD_STAR_PACKET_H
#define __STAR_SDK_MSG_CMD_STAR_PACKET_H

#include <star/Star.h>
#include <star/msg/cmd/Packet.h>
#include <star/utils/factory.h>

 //added by zhubing 2020.4.14
 const uint8_t _except_data_lenth = 0x0F;

namespace ss {
namespace dev {
namespace cmd {
namespace star {

//校验和计算公式
struct __star_export ChecksumCalculator {
public:
    template<typename _Buffer>
    uint16_t calculate(_Buffer& buffer, std::size_t offset, std::size_t length)
    {
        uint32_t sum = 0;
        utils::native_buffer_reader<_Buffer> reader(buffer, false);
        std::size_t pos = reader.tellg();
        reader.seekg(offset);
        while (length > 1u) {
            const uint16_t value = reader.template get_value<uint16_t>();
            sum += value;
            if (sum & 0x80000000)
                sum = (sum & 0xffff) + (sum >> 16);
            length -= 2;
        }
        if (1 == length) {
            sum += reader.template get_value<uint8_t>();
        }
        while (sum >> 16) {
            sum = (sum & 0xffff) + (sum >> 16);
        }
        reader.seekg(pos);
        return sum;
    }
};

//数据包头
class __star_export PacketHeader {
public:
    PacketHeader() = default;

    void setBodyLength(std::size_t length) noexcept;

    std::size_t bodyLength() const noexcept;

    template<typename _Writer>
    ssize_t serialize(_Writer& __writer) noexcept
    {
        if (__writer.size() < this->size()) {
            return -1;
        }
        std::size_t offset = __writer.tellp();
        __writer.put_value(_length);
        __writer.put_value(_version);
        __writer.put_value(_flags);
        __writer.put_value(_enc_padding);
        __writer.put_value(_reservation);
        __writer.put_value(_cmd_set);
        __writer.put_value(_cmd);
        __writer.put_value(_sequence);

        _checksum = _calculator.calculate(__writer.buf(), offset, this->size() - 2);

        __writer.put_value(_checksum);
        return size();
    }

    template<typename _Reader>
    ssize_t deserialize(_Reader& __reader, std::size_t __length) noexcept
    {
        if (__length < size()) {
            return -1;
        }
        const uint16_t checksum = _calculator.calculate(__reader.buf(), __reader.tellg(), this->size() - 2);

        _length = __reader.template get_value<uint16_t>();
        _version = __reader.template get_value<uint8_t>();
        _flags = __reader.template get_value<uint8_t>();
        _enc_padding = __reader.template get_value<uint8_t>();
        _reservation = __reader.template get_value<uint8_t>();
        _cmd_set = __reader.template get_value<uint8_t>();
        _cmd = __reader.template get_value<uint8_t>();
        _sequence = __reader.template get_value<uint16_t>();
        _checksum = __reader.template get_value<uint16_t>();

        if (_checksum != checksum) {
            return -1;
        }
        return size();
    }

    uint16_t length() const noexcept;

    uint8_t version() const noexcept;
    void set_version(uint8_t version);

    uint8_t flags() const noexcept;
    void set_flags(uint8_t flags);

    uint8_t end_padding() const noexcept;
    void set_enc_padding(uint8_t encPadding);

    uint8_t cmd_set() const;
    void set_cmd_set(uint8_t cmdSet);

    uint8_t cmd() const;
    void set_cmd(uint8_t cmd);

    uint16_t sequence() const;
    void set_sequence(uint16_t sequence);

    uint16_t checksum() const;

	/**
     * 区分数据类别
     */
    uint64_t dataId() const noexcept;

    static uint64_t dataId(uint64_t flags, uint64_t cmdset, uint64_t cmd) noexcept;

    /**
     * 序号
     */
    uint64_t identifier() const noexcept;

    static constexpr std::size_t size() noexcept
    {
        return sizeof(_length) + sizeof(_version) + sizeof(_flags) + sizeof(_enc_padding)
            + sizeof(_reservation) + sizeof(_cmd_set) + sizeof(_cmd) + sizeof(_sequence)
            + sizeof(_checksum);
    }

private:
    uint16_t _length;
    uint8_t  _version;
    uint8_t  _flags;
    uint8_t  _enc_padding;
    uint8_t  _reservation;
    uint8_t  _cmd_set;
    uint8_t  _cmd;
    uint16_t _sequence;
    uint16_t _checksum;
    ChecksumCalculator _calculator;
};

template <typename _Tp>
class Data : public msg::cmd::Payload {
public:
    using data_type = _Tp;
    Data() = default;

    explicit Data(const data_type& data) :
        _data(data)
    {
    }

    ssize_t serialize(utils::buffer_writer& __writer) noexcept override
    {
        if (__writer.available() < sizeof(data_type)) {
            return -1;
        }
        __writer.put_value(_data);
        return sizeof(data_type);
    }

    ssize_t deserialize(utils::buffer_reader& __reader, std::size_t __length) noexcept override
    {
        if (__length < sizeof(data_type)) {
            return -1;
        }
        _data = __reader.get_value<data_type>();
        return sizeof(data_type);
    }

    data_type& data()
    {
        return _data;
    }

    const data_type& data() const
    {
        return _data;
    }

    void set(const data_type& data)
    {
        _data = data;
    }

private:
    data_type _data{ };
};

//定义消息体
class __star_export Command : public Data<uint16_t> {
public:
    Command() = default;
    explicit Command(uint16_t command);

    uint16_t command() const;
    void setCommand(uint16_t command);
};

class __star_export Acknowledge : public Data<uint16_t> {
public:
    Acknowledge() = default;
    explicit Acknowledge(uint16_t status);

    uint16_t status() const;
    void setStatus(uint16_t status);
};

//added by zhubing 2020.4.8
class __star_export GetWorkStateAck : public msg::cmd::Payload {
public:
	GetWorkStateAck() = default;
	GetWorkStateAck(uint16_t status, uint16_t workState);

	ssize_t serialize(utils::buffer_writer& __writer) noexcept override;
	ssize_t deserialize(utils::buffer_reader& __reader, std::size_t __length) noexcept override;

	uint16_t status() const;
	void setStatus(uint16_t status);

	uint16_t workState() const;
	void setWorkState(uint16_t workState);

	static std::size_t size();
private:
	uint16_t _status{ 0 };
	uint16_t _workState{ 0 };
};

//added by zhubing 2020.4.8
class __star_export GetScanFreqAck : public msg::cmd::Payload {
public:
	GetScanFreqAck() = default;
	GetScanFreqAck(uint16_t status, uint16_t scanFreq);

	ssize_t serialize(utils::buffer_writer& __writer) noexcept override;
	ssize_t deserialize(utils::buffer_reader& __reader, std::size_t __length) noexcept override;

	uint16_t status() const;
	void setStatus(uint16_t status);

	uint16_t scanFreq() const;
	void setScanFreq(uint16_t scanFreq);

	static std::size_t size();
private:
	uint16_t _status{ 0 };
	uint16_t _scanFreq{ 0 };
};

//added by zhubing 2020.4.8
class __star_export GetScanSpeedAck : public msg::cmd::Payload {
public:
	GetScanSpeedAck() = default;
	GetScanSpeedAck(uint16_t status, uint16_t scanSpeed);

	ssize_t serialize(utils::buffer_writer& __writer) noexcept override;
	ssize_t deserialize(utils::buffer_reader& __reader, std::size_t __length) noexcept override;

	uint16_t status() const;
	void setStatus(uint16_t status);

	uint16_t scanSpeed() const;
	void setScanSpeed(uint16_t scanSpeed);

	static std::size_t size();
private:
	uint16_t _status{ 0 };
	uint16_t _scanSpeed{ 0 };
};

//added by zhubing 2020.4.8
class __star_export GetRangeMaxAck : public msg::cmd::Payload {
public:
	GetRangeMaxAck() = default;
	GetRangeMaxAck(uint16_t status, uint16_t rangeMax);

	ssize_t serialize(utils::buffer_writer& __writer) noexcept override;
	ssize_t deserialize(utils::buffer_reader& __reader, std::size_t __length) noexcept override;

	uint16_t status() const;
	void setStatus(uint16_t status);

	uint16_t rangeMax() const;
	void setRangeMax(uint16_t rangeMax);

	static std::size_t size();
private:
	uint16_t _status{ 0 };
	uint16_t _rangeMax{ 0 };
};

//added by zhubing 2020.4.8
class __star_export GetRangeMinAck : public msg::cmd::Payload {
public:
	GetRangeMinAck() = default;
	GetRangeMinAck(uint16_t status, uint16_t rangeMin);

	ssize_t serialize(utils::buffer_writer& __writer) noexcept override;
	ssize_t deserialize(utils::buffer_reader& __reader, std::size_t __length) noexcept override;

	uint16_t status() const;
	void setStatus(uint16_t status);

	uint16_t rangeMin() const;
	void setRangeMin(uint16_t rangeMin);

	static std::size_t size();
private:
	uint16_t _status{ 0 };
	uint16_t _rangeMin{ 0 };
};


class __star_export GetEchoTypeAck : public msg::cmd::Payload {
public:
    GetEchoTypeAck() = default;
    GetEchoTypeAck(uint16_t status, uint16_t echoType);

    ssize_t serialize(utils::buffer_writer& __writer) noexcept override;
    ssize_t deserialize(utils::buffer_reader& __reader, std::size_t __length) noexcept override;

    uint16_t status() const;
    void setStatus(uint16_t status);

    uint16_t echoType() const;
    void setEchoType(uint16_t echoType);

    static std::size_t size();
private:
    uint16_t _status{ 0 };
    uint16_t _echoType{ 0 };
};

//added by zhubing 2020.4.20
class __star_export GetPushAPXStateAck : public msg::cmd::Payload {
public:
	GetPushAPXStateAck() = default;
	GetPushAPXStateAck(uint16_t status, uint8_t pushState, uint8_t linkState, uint8_t GPSNumber, uint16_t packetSize);

	ssize_t serialize(utils::buffer_writer& __writer) noexcept override;
	ssize_t deserialize(utils::buffer_reader& __reader, std::size_t __length) noexcept override;

	uint16_t status() const;
	void setStatus(uint16_t status);

	uint8_t pushState() const;
	void setPushState(uint8_t pushState);

	uint8_t linkState() const;
	void setLinkState(uint8_t linkState);

	uint8_t GPSNumber() const;
	void setGPSNumber(uint8_t GPSNumber);

	uint16_t packetSize() const;
	void setPacketSize(uint16_t packetSize);

	static std::size_t size();
private:
	uint16_t _status{ 0 };
	uint8_t  _pushState{ 0 };
	uint8_t  _linkState{ 0 };
	uint8_t  _GPSNumber{ 0 };
	uint16_t _packetSize{ 0 };
};

//added by zhubing 2020.4.20
class __star_export GetPushSDStateAck : public msg::cmd::Payload {
public:
	GetPushSDStateAck() = default;
	GetPushSDStateAck(uint16_t status, uint8_t pushState, uint8_t linkState, uint8_t ISFNumber, uint16_t packetSize);

	ssize_t serialize(utils::buffer_writer& __writer) noexcept override;
	ssize_t deserialize(utils::buffer_reader& __reader, std::size_t __length) noexcept override;

	uint16_t status() const;
	void setStatus(uint16_t status);

	uint8_t pushState() const;
	void setPushState(uint8_t pushState);

	uint8_t linkState() const;
	void setLinkState(uint8_t linkState);

	uint8_t ISFNumber() const;
	void setISFNumber(uint8_t ISFNumber);

	uint16_t packetSize() const;
	void setPacketSize(uint16_t packetSize);

	static std::size_t size();
private:
	uint16_t _status{ 0 };
	uint8_t  _pushState{ 0 };
	uint8_t  _linkState{ 0 };
	uint8_t  _ISFNumber{ 0 };
	uint16_t _packetSize{ 0 };
};

class __star_export GetPushStateAck : public msg::cmd::Payload {
public:
    GetPushStateAck() = default;
    GetPushStateAck(uint16_t status, uint8_t pushState, uint8_t linkState, uint16_t packetSize);

    ssize_t serialize(utils::buffer_writer& __writer) noexcept override;
    ssize_t deserialize(utils::buffer_reader& __reader, std::size_t __length) noexcept override;

    uint16_t status() const;
    void setStatus(uint16_t status);

    uint8_t pushState() const;
    void setPushState(uint8_t pushState);

    uint8_t linkState() const;
    void setLinkState(uint8_t linkState);

    uint16_t packetSize() const;
    void setPacketSize(uint16_t packetSize);

    static std::size_t size();
private:
    uint16_t _status{ 0 };
    uint8_t  _pushState{ 0 };
    uint8_t  _linkState{ 0 };
    uint16_t _packetSize{ 0 };
};

//added by zhubing 2020.3.25
class __star_export GetPushDateAck : public msg::cmd::Payload {
public:
	GetPushDateAck() = default;
	GetPushDateAck(uint16_t status,uint16_t gpsWeek, uint32_t gpsTime, uint64_t latitude, uint64_t longitude, uint64_t altitude, uint32_t northVelocity, uint32_t eastVelocity, uint32_t downVelocity, uint64_t pitch, uint64_t heading, uint64_t roll);

	ssize_t serialize(utils::buffer_writer& __writer) noexcept override;
	ssize_t deserialize(utils::buffer_reader& __reader, std::size_t __length) noexcept override;

	//added by zhubing 2020.4.20
	uint16_t status() const;
	void setStatus(uint16_t status);

	uint16_t gpsWeek() const;
	void setgpsWeek(uint16_t gpsWeek);

	uint32_t gpsTime() const;
	void setgpsTime(uint32_t gpsTime);

	uint64_t latitude() const;
	void setLatitude(uint64_t latitude);

	uint64_t longitude() const;
	void setLongitude(uint64_t longitude);

	uint64_t altitude() const;
	void setAltitude(uint64_t altitude);

	uint32_t northVelocity() const;
	void setnorthVelocity(uint32_t northVelocity);

	uint32_t eastVelocity() const;
	void seteastVelocity(uint32_t eastVelocity);

	uint32_t downVelocity() const;
	void setdownVelocity(uint32_t downVelocity);

	uint64_t pitch() const;
	void setPitch(uint64_t pitch);

	uint64_t heading() const;
	void setHeading(uint64_t heading);

	uint64_t roll() const;
	void setRoll(uint64_t roll);

	static std::size_t size();

private:
	uint16_t _status{ 0 };
	uint16_t _gpsWeek{ 0 };
	uint32_t _gpsTime{ 0 };
	uint64_t _latitude{ 0 };
	uint64_t _longitude{ 0 };
	uint64_t _altitude{ 0 };
	uint32_t _northVelocity{ 0 };
	uint32_t _eastVelocity{ 0 };
	uint32_t _downVelocity{ 0 };
	uint64_t _pitch{ 0 };
	uint64_t _heading{ 0 };
	uint64_t _roll{ 0 };
};

//end added by zhubing 2020.3.25

class __star_export GetCameraWorkStateAck : public msg::cmd::Payload {
public:
    GetCameraWorkStateAck() = default;
    GetCameraWorkStateAck(uint16_t status, uint16_t workState);

    ssize_t serialize(utils::buffer_writer& __writer) noexcept override;
    ssize_t deserialize(utils::buffer_reader& __reader, std::size_t __length) noexcept override;

    uint16_t status() const;
    void setStatus(uint16_t status);

    uint16_t workState() const;
    void setWorkState(uint16_t workState);

    static std::size_t size();
private:
    uint16_t _status{ 0 };
    uint16_t _workState{ 0 };
};

class __star_export GetCameraWorkModeAck : public msg::cmd::Payload {
public:
    GetCameraWorkModeAck() = default;
    GetCameraWorkModeAck(uint16_t status, uint16_t workMode);

    ssize_t serialize(utils::buffer_writer& __writer) noexcept override;
    ssize_t deserialize(utils::buffer_reader& __reader, std::size_t __length) noexcept override;

    uint16_t status() const;
    void setStatus(uint16_t status);

    uint16_t workMode() const;
    void setWorkMode(uint16_t workMode);

    static std::size_t size();
private:
    uint16_t _status{ 0 };
    uint16_t _workMode{ 0 };
};

class __star_export GetCameraWorkIntervalAck : public msg::cmd::Payload {
public:
    GetCameraWorkIntervalAck() = default;
    GetCameraWorkIntervalAck(uint16_t status, uint16_t workInterval);

    ssize_t serialize(utils::buffer_writer& __writer) noexcept override;
    ssize_t deserialize(utils::buffer_reader& __reader, std::size_t __length) noexcept override;

    uint16_t status() const;
    void setStatus(uint16_t status);

    uint16_t workInterval() const;
    void setWorkInterval(uint16_t workInterval) {_workInterval = workInterval;}

    static std::size_t size();
private:
    uint16_t _status{ 0 };
    uint16_t _workInterval{ 0 };
};

//added by zhubing 2020.4.20
class __star_export GetCameraExposureNumberAck : public msg::cmd::Payload {
public:
	GetCameraExposureNumberAck() = default;
	GetCameraExposureNumberAck(uint16_t status, uint16_t exposureNumber);

	ssize_t serialize(utils::buffer_writer& __writer) noexcept override;
	ssize_t deserialize(utils::buffer_reader& __reader, std::size_t __length) noexcept override;

	uint16_t status() const;
	void setStatus(uint16_t status);

	uint16_t exposureNumber() const;
	void setExposureNumber(uint16_t exposureNumber) { _exposureNumber = exposureNumber; }

	static std::size_t size();
private:
	uint16_t _status{ 0 };
	uint16_t _exposureNumber{ 0 };
};





//定义消息体工厂
class __star_export PacketBodyFactory : public utils::shared_factory<uint64_t, msg::cmd::Payload> {
public:
    using key_type = uint64_t;
    using body_type = std::shared_ptr<msg::cmd::Payload>;
    PacketBodyFactory() noexcept;

    template <typename _Type>
    bool material(uint64_t flags, uint64_t cmdset, uint64_t cmd)
    {
        return utils::shared_factory<uint64_t, msg::cmd::Payload>::material<_Type>(PacketHeader::dataId(flags, cmdset, cmd));
    }

    bool material(uint64_t flags, uint64_t cmdset, uint64_t cmd)
    {
        return utils::shared_factory<uint64_t, msg::cmd::Payload>::material(PacketHeader::dataId(flags, cmdset, cmd));
    }

    std::shared_ptr<msg::cmd::Payload> make(const PacketHeader& header);

    std::size_t max_length(const PacketHeader& header) const;

};

#if 0
class __star_export PacketBodyBuilder {
public:
    using body_type = typename PacketBodyFactory::body_type;
    using key_type  = typename PacketBodyFactory::key_type;

//    using singleton = ss::utils::singleton<PacketBodyBuilder>;

    PacketBodyBuilder() = default;

    template<typename _Head>
    static body_type make(const _Head& head)
    {
        return _factory.make(head);
    }

    template<typename _Head>
    static std::size_t get_max_length(const _Head& head)
    {
        return _factory.max_length(head);
    }

private:
    static PacketBodyFactory _factory;
};
#endif

using PacketTail = msg::cmd::PacketChecksum<uint16_t, ChecksumCalculator, PacketHeader::size() + 1, 0>;

class __star_export Packet : public msg::cmd::Packet<
        msg::cmd::Prefix<uint8_t, static_cast<uint8_t>(0xaau)>,
        PacketHeader,
        PacketBodyFactory,
        PacketTail,
        void>
{
public:
    static constexpr uint8_t CMD = (0x00u << 5u);
    static constexpr uint8_t ACK = (0x01u << 5u);

    uint64_t dataId() const noexcept override
    {
        return head().dataId();
    }

    uint64_t identifier() const noexcept override
    {
        return head().identifier();
    }
};


}
}
}
}

#endif //__STAR_SDK_MSG_CMD_STAR_PACKET_H
