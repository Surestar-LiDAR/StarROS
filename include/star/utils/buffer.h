/**
 * @author xiaoma 
 * @date 2019/11/23.
 */

#ifndef __STAR_SDK_UTILS_BUFFER_H
#define __STAR_SDK_UTILS_BUFFER_H

#include <iterator>
#include <algorithm>
#include <stdexcept>

#include <star/utils/detail/buffer.h>

namespace ss {
namespace utils {

namespace detail {

template <typename _Byte>
class basic_const_buffer {
public:
    using value_type      = _Byte;
    using reference       = value_type const&;
    using const_reference = value_type const&;
    using pointer         = value_type const*;
    using const_pointer   = value_type const*;

    basic_const_buffer() noexcept :
        _data(nullptr),
        _end(nullptr)
    {
    }

    basic_const_buffer(pointer data, std::size_t size) noexcept :
        _data(data),
        _end(data + size)
    {
    }

    inline pointer data() const noexcept
    {
        return _data;
    }

    inline pointer end() const noexcept
    {
        return _end;
    }

    inline std::size_t left() const noexcept
    {
        return static_cast<std::size_t>(this->_end - this->_data);
    }

    inline basic_const_buffer& operator+=(std::size_t offset) noexcept
    {
        _data = _data + offset;
        return *this;
    }

    inline basic_const_buffer& operator-=(std::size_t offset) noexcept
    {
        _data = _data - offset;
        return *this;
    }

private:
    pointer       _data;
    pointer       _end;
};

template <typename _Byte>
class basic_mutable_buffer {
public:
    using value_type      = _Byte;
    using reference       = value_type&;
    using const_reference = value_type const&;
    using pointer         = value_type*;
    using const_pointer   = value_type const*;

    basic_mutable_buffer() noexcept :
        _data(nullptr),
        _end(nullptr)
    {
    }

    basic_mutable_buffer(pointer data, std::size_t size) noexcept :
        _data(data),
        _end(data + size)
    {
    }

    inline pointer data() noexcept
    {
        return _data;
    }

    inline const_pointer data() const noexcept
    {
        return _data;
    }

    inline const_pointer end() const noexcept
    {
        return _end;
    }

    inline std::size_t left() const noexcept
    {
        return static_cast<std::size_t>(this->_end - this->_data);
    }

    inline basic_mutable_buffer& operator+=(std::size_t offset) noexcept
    {
        _data = _data + offset;
        return *this;
    }

    inline basic_mutable_buffer& operator-=(std::size_t offset) noexcept
    {
        _data = _data - offset;
        return *this;
    }

protected:
    pointer       _data;
    pointer       _end;
};

template <typename _Buffer>
class basic_buffer {
public:
    using buffer_type            = _Buffer;
    using value_type             = typename buffer_type::value_type;
    using reference              = typename buffer_type::reference;
    using const_reference        = typename buffer_type::const_reference;
    using pointer                = typename buffer_type::pointer;
    using const_pointer          = typename buffer_type::const_pointer;
    using iterator               = typename buffer_type::pointer;
    using const_iterator         = typename buffer_type::const_pointer;
//    using reverse_iterator       = std::reverse_iterator<iterator>;
//    using const_reverse_iterator = std::reverse_iterator<const_iterator>;

    basic_buffer() noexcept :
        _head(nullptr)
    {
    }

    basic_buffer(pointer data, std::size_t length) noexcept :
        _head(data),
        _buffer(data, length)
    {
    }

    inline std::size_t tell() const noexcept
    {
        return static_cast<std::size_t>(data() - _head);
    }

    inline void seek(std::size_t offset) noexcept
    {
	    const std::size_t tellpos = tell();
	    const ssize_t distance = static_cast<ssize_t>(offset) - static_cast<ssize_t>(tellpos);
        if(distance < 0) {
            offset = std::min(static_cast<std::size_t>(-distance), tellpos);
            _buffer -= offset;
        }
        else {
            offset = std::min(static_cast<std::size_t>(distance), this->_buffer.left());
            _buffer += offset;
        }
    }

    void sync() noexcept
    {
        this->_head = _buffer.data();
    }

    inline bool eof() const noexcept
    {
        return this->_buffer.data() == this->_buffer.end();
    }

    inline pointer data() noexcept
    {
        return _buffer.data();
    }

    inline const_pointer data() const noexcept
    {
        return _buffer.data();
    }

protected:
    pointer         _head;
    buffer_type     _buffer;
};

#if 0
template <typename _Tp>
class basic_buffer {
public:
    using value_type      = _Tp;
    using traits_type     = basic_buffer_traits<value_type>;
    using reference       = value_type&;
    using const_reference = value_type const&;
    using pointer         = value_type*;
    using const_pointer   = value_type const*;

    basic_buffer(pointer buf, std::size_t capacity) noexcept:
        _data(buf),
        _position(0u),
        _capacity(capacity)
    {
    }

    std::size_t write(const_pointer buf, std::size_t length) noexcept
    {
        if(length > available()) {
            return 0;
        }
        traits_type::copy(_data + _position, buf, length);
        _position += length;
        return length;
    }

    std::size_t readsome(pointer buf, std::size_t length) noexcept
    {
        length = std::min(length, this->size());
        traits_type::copy(buf, _data + _position, length);
        _position += length;
    }

    std::size_t read(pointer buf, std::size_t length) noexcept
    {
        if(length > this->sizeg()) {
            return 0;
        }
        traits_type::copy(buf, _data + _position, length);
        _position += length;
        return length;
    }

    std::size_t peek(pointer buf, std::size_t length) const noexcept
    {
        if(length > this->sizeg()) {
            return 0;
        }
        traits_type::copy(buf, _data + _position, length);
        return length;
    }

    std::size_t tellpos() const noexcept
    {
        return _position;
    }

    void seekpos(std::size_t position) noexcept
    {
        _position = position;
    }

    pointer data() noexcept
    {
        return _data;
    }

    const_pointer data() const noexcept
    {
        return _data;
    }

    std::size_t sizeg() const noexcept
    {
        return _capacity - _position;
    }

    std::size_t sizep() const noexcept
    {
        return _position;
    }

    std::size_t available() const noexcept
    {
        return _capacity - _position;
    }

    std::size_t position() const noexcept
    {
        return _position;
    }

    std::size_t capacity() const noexcept
    {
        return _capacity;
    }

    value_type& value(std::size_t offset) noexcept
    {
        return _data[offset];
    }

    const value_type& value(std::size_t offset) const noexcept
    {
        return _data[offset];
    }

private:
    pointer     _data;
    std::size_t _position;
    std::size_t _capacity;
};
#endif

template <typename _Container, typename _Iterator, typename _ConstIterator>
class basic_buffer_reverse_iterator_operator
{
public:
    using reverse_iterator       = std::reverse_iterator<_Iterator>;
    using const_reverse_iterator = std::reverse_iterator<_ConstIterator>;

    reverse_iterator rbegin() noexcept
    {
        return _Container::end() - 1;
    }

    const_reverse_iterator rbegin() const noexcept
    {
        return _Container::end() - 1;
    }

    const_reverse_iterator crbegin() const noexcept
    {
        return _Container::end() - 1;
    }

    reverse_iterator rend() noexcept
    {
        return _Container::begin() - 1;
    }

    const_reverse_iterator rend() const noexcept
    {
        return _Container::begin() - 1;
    }

    const_reverse_iterator crend() const noexcept
    {
        return _Container::begin() - 1;
    }
};

}

template <typename _Byte>
class rdonly_buffer : public detail::basic_buffer<detail::basic_const_buffer<_Byte>>,
        public detail::basic_buffer_reverse_iterator_operator<rdonly_buffer<_Byte>,
                typename detail::basic_buffer<detail::basic_const_buffer<_Byte>>::iterator,
                typename detail::basic_buffer<detail::basic_const_buffer<_Byte>>::const_iterator> {
public:
    using basic_buffer           = typename detail::basic_buffer<detail::basic_const_buffer<_Byte>>;
    using impl_type              = typename basic_buffer::buffer_type;
    using value_type             = typename basic_buffer::value_type;
    using reference              = typename basic_buffer::reference;
    using const_reference        = typename basic_buffer::const_reference;
    using pointer                = typename basic_buffer::pointer;
    using const_pointer          = typename basic_buffer::const_pointer;
    using iterator               = value_type const*;
    using const_iterator         = value_type const*;
    using reverse_iterator       = std::reverse_iterator<iterator>;
    using const_reverse_iterator = std::reverse_iterator<const_iterator>;

    rdonly_buffer() = default;
    rdonly_buffer(value_type* buf, std::size_t length) :
        basic_buffer(buf, length)
    {
    }

    inline std::size_t readsome(value_type* data, std::size_t length) noexcept
    {
        length = std::min(length, this->size());
        detail::memcpy(data, this->data(), length);
        this->_buffer += length;
        return length;
    }

    inline std::size_t read(value_type* data, std::size_t length)
    {
        if(this->size() < length) {
            throw std::out_of_range("rdonly_buffer::read()");
        }
//        std::copy(__data, __data + length, data);
        detail::memcpy(data, this->data(), length);
        this->_buffer += length;
        return length;
    }

    inline std::size_t tellg() const noexcept
    {
        return this->tell();
    }

    inline void seekg(std::size_t offset) noexcept
    {
        this->seek(offset);
    }

    reference at(std::size_t offset)
    {
        if(offset >= this->size()) {
            throw std::out_of_range("rdonly_buffer:at()");
        }
        return this->data()[offset];
    }

    const_reference at(std::size_t offset) const
    {
        if(offset >= this->size()) {
            throw std::out_of_range("rdonly_buffer:at()");
        }
        return this->data()[offset];
    }

    inline reference operator[](std::size_t offset)
    {
        return this->data()[offset];
    }

    inline const_reference operator[](std::size_t offset) const
    {
        return this->data()[offset];
    }

    inline rdonly_buffer& operator+=(std::size_t offset) noexcept
    {
        offset = std::min(offset, this->size());
        this->_buffer.operator+=(offset);
        return *this;
    }

    inline rdonly_buffer& operator-=(std::size_t offset) noexcept
    {
        offset = std::min(offset, this->data() - this->_head);
        this->_buffer.operator-=(offset);
        return *this;
    }

    /**
     * 获取缓存中可以写入的数据长度，只读缓存不可写入，永远为0
     * @return 0
     */
    inline std::size_t available() const noexcept
    {
        return 0;
    }

    /**
     * 缓存中的数据长度
     * @return 缓存中的数据长度
     */
    inline std::size_t size() const noexcept
    {
        return this->_buffer.left();
    }

    iterator begin() noexcept
    {
        return this->_buffer.data();
    }

    const_iterator begin() const noexcept
    {
        return this->_buffer.data();
    }

    const_iterator cbegin() const noexcept
    {
        return this->begin();
    }

    iterator end() noexcept
    {
        return this->_buffer.data() + this->_buffer.left();
    }

    const_iterator end() const noexcept
    {
        return this->_buffer.data() + this->_buffer.left();
    }

    const_iterator cend() const noexcept
    {
        return this->end();
    }

};

template <typename _Byte>
class wronly_buffer : public detail::basic_buffer<detail::basic_mutable_buffer<_Byte>>,
          public detail::basic_buffer_reverse_iterator_operator<wronly_buffer<_Byte>,
                  typename detail::basic_buffer<detail::basic_mutable_buffer<_Byte>>::iterator,
                  typename detail::basic_buffer<detail::basic_mutable_buffer<_Byte>>::const_iterator> {
public:
    using basic_buffer           = typename detail::basic_buffer<detail::basic_mutable_buffer<_Byte>>;
    using impl_type              = typename basic_buffer::buffer_type;
    using value_type             = typename basic_buffer::value_type;
    using reference              = typename basic_buffer::reference;
    using const_reference        = typename basic_buffer::const_reference;
    using pointer                = typename basic_buffer::pointer;
    using const_pointer          = typename basic_buffer::const_pointer;
    using iterator               = value_type const*;
    using const_iterator         = value_type const*;
    using reverse_iterator       = std::reverse_iterator<iterator>;
    using const_reverse_iterator = std::reverse_iterator<const_iterator>;

    wronly_buffer() = default;

    wronly_buffer(value_type* buf, std::size_t length) :
        basic_buffer(buf, length)
    {
    }

    inline std::size_t write(const value_type* data, std::size_t length)
    {
        if(this->available() < length) {
            throw std::out_of_range("buffer_writer::write()");
        }

        detail::memcpy(this->data(), data, length);
        this->_buffer += length;
        return length;
    }

    inline std::size_t read(value_type* data, std::size_t length)
    {
        // if(this->size() < length) {
        //     throw std::out_of_range("buffer_writer::write()");
        // }

        detail::memcpy(data, this->data(), length);
        this->_buffer += length;
        return length;
    }

    inline std::size_t peek(value_type* data, std::size_t length, std::size_t offset = 0)
    {
        // if(this->size() + offset < length) {
        //     throw std::out_of_range("buffer_writer::write()");
        // }

        detail::memcpy(data, this->data() + offset, length);
        return length;
    }

    inline std::size_t tellp() const noexcept
    {
        return this->tell();
    }

    inline void seekp(std::size_t offset) noexcept
    {
        this->seek(offset);
    }

    inline std::size_t tellg() const noexcept
    {
        return this->tell();
    }

    inline void seekg(std::size_t offset) noexcept
    {
        this->seek(offset);
    }

    inline value_type& operator[](std::size_t offset) noexcept
    {
        return this->_head[offset];
    }

    inline const value_type& operator[](std::size_t offset) const noexcept
    {
        return this->_head[offset];
    }

    value_type& at(std::size_t offset)
    {
        if(this->size() < offset) {
            throw std::out_of_range("buffer::at()");
        }
        return this->operator[](offset);
    }

    const value_type& at(std::size_t offset) const
    {
        if(this->size() < offset) {
            throw std::out_of_range("buffer::at()");
        }
        return this->operator[](offset);
    }

    inline wronly_buffer& operator+=(std::size_t offset) noexcept
    {
        offset = std::min(offset, this->available());
        this->_buffer.operator+=(offset);
        return *this;
    }

    inline wronly_buffer& operator-=(std::size_t offset) noexcept
    {
        offset = std::min(offset, this->size());
        this->_buffer.operator-=(offset);
        return *this;
    }

    /**
     * 获取缓存中的数据长度，指写入的数据长度
     * @return 已经写入的数据长度
     */
    inline std::size_t size() const noexcept
    {
        return this->_buffer.data() - this->_head;
    }

    /**
      * 获取缓存剩余空间的大小
      * @return
      */
    inline std::size_t available() const noexcept
    {
        return this->_buffer.left();
    }

    iterator begin() noexcept
    {
        return this->_head;
    }

    const_iterator begin() const noexcept
    {
        return this->_head;
    }

    const_iterator cbegin() const noexcept
    {
        return this->begin();
    }

    iterator end() noexcept
    {
        return this->_buffer.data();
    }

    const_iterator end() const noexcept
    {
        return this->_buffer.data();
    }

    const_iterator cend() const noexcept
    {
        return this->end();
    }
};

template <typename _Byte>
using const_buffer = rdonly_buffer<_Byte>;
template <typename _Byte>
using mutable_buffer = wronly_buffer<_Byte>;

}
}

#endif //__STAR_SDK_UTILS_BUFFER_H
