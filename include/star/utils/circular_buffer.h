/**
 * @author      : John 
 * @date        : 2018-09-20
 */

#ifndef __STAR_SDK_UTILS_CIRCULAR_BUFFER_H
#define __STAR_SDK_UTILS_CIRCULAR_BUFFER_H

#include <star/utils/detail/basic_circular.h>

namespace ss {
namespace utils {

template <typename _Tp, class _Policy, class _Alloc = std::allocator<_Tp>>
class basic_circular_buffer {
public:
    using basic_circular         = detail::basic_circular<_Tp, _Policy, _Alloc>;
    using value_type             = typename basic_circular::value_type;
    using reference              = typename basic_circular::reference;
    using const_reference        = typename basic_circular::const_reference;
    using pointer                = typename basic_circular::pointer;
    using iterator               = typename basic_circular::iterator;
    using const_iterator         = typename basic_circular::const_iterator;
    using reverse_iterator       = typename basic_circular::reverse_iterator;
    using const_reverse_iterator = typename basic_circular::const_reverse_iterator;

    struct push_sync_event {
        bool        sync;
        std::size_t head;
        std::size_t tail;
    };

    explicit basic_circular_buffer(std::size_t capacity = 4096) noexcept :
        _head(0),
        _tail(0),
        _circular(capacity)
    {
        sync();
    }

    basic_circular_buffer(const basic_circular_buffer& other) noexcept :
        _head(other._head),
        _tail(other._tail),
        _circular(other._circular)
    {
    }

    basic_circular_buffer(basic_circular_buffer&& other) noexcept :
        _head(other._head),
        _tail(other._tail),
        _circular(std::move(other._circular))
    {
        other._head = 0;
        other._tail = 0;
    }

    ~basic_circular_buffer() noexcept = default;

    basic_circular_buffer& operator=(const basic_circular_buffer& other) noexcept
    {
        if(&other == this) {
            return *this;
        }
        _head = other._head;
        _tail = other._tail;
        _circular = other._circular;
        return *this;
    }

    basic_circular_buffer& operator=(basic_circular_buffer&& other) noexcept
    {
        _head = other._head;
        _tail = other._tail;
        _circular = std::move(other._circular);

        other._head = 0;
        other._tail = 0;
        return *this;
    }

    void clear()
    {
        _head = 0;
        _tail = 0;
        _circular.clear();
    }

    inline std::size_t readsome(value_type *data, std::size_t length) noexcept
    {
        length = std::min(length, size());
        if(length == 0u) {
            return 0;
        }
        _circular.unchecked_peek_front(data, length);
        _circular.unchecked_pop_front(length);
        return length;
    }

    inline std::size_t read(value_type *data, std::size_t length)
    {
        if(this->size() < length) {
            throw std::out_of_range("circular_buffer::read()");
        }

        _circular.unchecked_peek_front(data, length);
        _circular.unchecked_pop_front(length);
        return length;
    }

    inline std::size_t write(const value_type *data, std::size_t length)
    {
        auto event = this->pre_push(length);
        if(!_circular.push_prepare(length)) {
            throw std::out_of_range("circular_buffer::write()");
        }
        _circular.unchecked_push_back(data, length);
        this->post_push(event);
        return length;
    }

    inline std::size_t tellp() const noexcept
    {
        return _circular.tellp(_tail);
    }

    inline void seekp(std::size_t offset) noexcept
    {
        _circular.seekp(_tail, offset);
    }

    inline std::size_t tellg() const noexcept
    {
        return _circular.tellg(_head);
    }

    inline void seekg(std::size_t offset) noexcept
    {
        _circular.seekg(_head, offset);
    }

    void sync() noexcept
    {
        _head = _circular.head();
        _tail = _circular.tail();
    }

    void push_front(const value_type *data, std::size_t length)
    {
        auto event = this->pre_push(length);
        _circular.push_front(data, length);
        this->post_push(event);
    }

    void push_front(const value_type &data)
    {
        auto event = this->pre_push(1);
        _circular.push_front(data);
        this->post_push(event);
    }

    void push_front(value_type &&data)
    {
        auto event = this->pre_push(1);
        _circular.push_front(std::move(data));
        this->post_push(event);
    }

    void push_back(const value_type *data, std::size_t length)
    {
        auto event = this->pre_push(length);
        _circular.push_back(data, length);
        this->post_push(event);
    }

    void push_back(const value_type &data)
    {
        auto event = this->pre_push(1);
        _circular.push_back(data);
        this->post_push(event);
    }

    void push_back(value_type &&data)
    {
        auto event = this->pre_push(1);
        _circular.push_back(std::move(data));
        this->post_push(event);
    }

    void push(const value_type *data, std::size_t length)
    {
        push_back(data, length);
    }

    void push(const value_type &data)
    {
        push_back(data);
    }

    void push(value_type &&data)
    {
        push_back(data);
    }

    void pop_front(std::size_t length)
    {
        _circular.pop_front(length);
    }

    void pop_front()
    {
        _circular.pop_front();
    }

    void pop_back(std::size_t length)
    {
        _circular.pop_back(length);
    }

    void pop_back()
    {
        _circular.pop_back();
    }

    void pop()
    {
        pop_front();
    }

    void pop(std::size_t length)
    {
        pop_front(length);
    }

    void free(std::size_t length)
    {
        pop(length);
    }

    void peek_front(value_type *data, std::size_t length) const
    {
        _circular.peek_front(data, length);
    }

    void peek_back(value_type *data, std::size_t length) const
    {
        _circular.peek_back(data, length);
    }

    void peek(value_type *data, std::size_t length) const
    {
        return peek_front(data, length);
    }

    void peek(value_type *data, std::size_t length, std::size_t offset) const
    {
        return peek_front(data, length, offset);
    }

    const_reference front() const
    {
        return _circular.front();
    }

    reference front()
    {
        return _circular.front();
    }

    const_reference back() const
    {
        return _circular.back();
    }

    reference back()
    {
        return _circular.back();
    }

    inline reference operator[](std::size_t offset) noexcept
    {
        return _circular.unchecked_front_at(offset);
    }

    inline const_reference operator[](std::size_t offset) const noexcept
    {
        return _circular.unchecked_front_at(offset);
    }

    reference at(std::size_t offset)
    {
        if(offset > size()) {
            throw std::out_of_range("circular_buffer::at()");
        }
        return this->operator[](offset);
    }

    const_reference at(std::size_t offset) const
    {
        if(offset > size()) {
            throw std::out_of_range("circular_buffer::at() const");
        }
        return this->operator[](offset);
    }

    void reserve(std::size_t capacity)
    {
        _circular.reserve(capacity);
    }

    inline std::size_t size() const noexcept
    {
        return _circular.size();
    }

    inline std::size_t length() const noexcept
    {
        return size();
    }

    inline std::size_t capacity() const noexcept
    {
        return _circular.capacity();
    }

    inline std::size_t available() const noexcept
    {
        return capacity() - size();
    }

    inline bool empty() const noexcept
    {
        return _circular.empty();
    }

    inline bool full() const noexcept
    {
        return _circular.full();
    }

    iterator begin() noexcept
    {
        return _circular.begin();
    }

    iterator end() noexcept
    {
        return _circular.end();
    }

    const_iterator begin() const noexcept
    {
        return _circular.cbegin();
    }

    const_iterator end() const noexcept
    {
        return _circular.cend();
    }

    const_iterator cbegin() const noexcept
    {
        return _circular.cbegin();
    }

    const_iterator cend() const noexcept
    {
        return _circular.cend();
    }

    reverse_iterator rbegin() noexcept
    {
        return _circular.rbegin();
    }

    reverse_iterator rend() noexcept
    {
        return _circular.rend();
    }

    const_reverse_iterator rbegin() const noexcept
    {
        return _circular.crbegin();
    }

    const_reverse_iterator rend() const noexcept
    {
        return _circular.crend();
    }

    const_reverse_iterator crbegin() const noexcept
    {
        return _circular.crbegin();
    }

    const_reverse_iterator crend() const noexcept
    {
        return _circular.crend();
    }

    push_sync_event pre_push(std::size_t length) noexcept
    {
        if(this->available() < length) {
            return {true, _circular.head(), _circular.tail()};
        }
        return {false, 0, 0};
    }

    void post_push(const push_sync_event& event) noexcept
    {
        if(event.sync) {
            _head = _circular.get_position(
                    event.head, static_cast<ssize_t>(_head) -
                                static_cast<ssize_t>(event.head));

            _tail = _circular.get_position(
                    event.tail, static_cast<ssize_t>(_tail) -
                                static_cast<ssize_t>(event.tail));
        }
    }

protected:
    std::size_t     _head;
    std::size_t     _tail;
    basic_circular  _circular;
};

template <typename _Tp, typename _Alloc = std::allocator<_Tp>>
using fixed_circular_buffer = basic_circular_buffer<_Tp, detail::circular_policy<_Tp, true>, _Alloc>;

template <typename _Tp, typename _Alloc = std::allocator<_Tp>>
using aligned_fixed_circular_buffer = basic_circular_buffer<_Tp, detail::aligned_circular_policy<_Tp, true>, _Alloc>;

template <typename _Tp, typename _Alloc = std::allocator<_Tp>>
using circular_buffer = basic_circular_buffer<_Tp, detail::circular_policy<_Tp, false>, _Alloc>;

template <typename _Tp, typename _Alloc = std::allocator<_Tp>>
using aligned_circular_buffer = basic_circular_buffer<_Tp, detail::aligned_circular_policy<_Tp, false>, _Alloc>;

}
}

#endif //__STAR_SDK_UTILS_CIRCULAR_BUFFER_H
