/**
 * @author      : John
 * @date        : 2018-09-20
 */

#ifndef __STAR_SDK_UTILS_BASIC_CIRCULAR_H
#define __STAR_SDK_UTILS_BASIC_CIRCULAR_H

#include <star/star.h>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <stdexcept>

#include <star/utils/detail/buffer.h>

namespace ss {
namespace utils {
namespace detail {

template <typename _Tp, bool _Fixed>
struct circular_policy_base {
public:
    typedef _Tp      value_type;

    inline static std::size_t get_capacity(std::size_t capacity) noexcept
    {
        return capacity + 1;
    }

    inline static std::size_t get_position(std::size_t position, ssize_t offset, std::size_t capacity) noexcept
    {
#if 1
        if(offset < 0 && position < static_cast<std::size_t>(-offset)) {
            return (capacity + offset + position);
        }

        position = position + offset;
        if(position < capacity) {
            return position;
        }
        return position - capacity;
#else
        return (capacity + position + offset) % capacity;
#endif
    }

    inline static std::size_t get_length(std::size_t start, std::size_t stop, std::size_t capacity)
    {
        return get_size(start, stop, capacity);
    }

//    static std::size_t get_capacity(std::size_t capacity) noexcept
//    {
//        return capacity;
//    }

    inline static bool empty(std::size_t head, std::size_t tail, std::size_t capacity) noexcept
    {
		(void)capacity;
        return head == tail;
    }

    inline static bool full(std::size_t head, std::size_t tail, std::size_t capacity) noexcept
    {
#if 0
        return ((tail + 1) % capacity) == head;
#else
        std::size_t t = tail + 1;
        if(t >= capacity) {
            return t - capacity == head;
        }
        return t == head;
#endif
    }

    inline static std::size_t get_size(std::size_t head, std::size_t tail, std::size_t capacity) noexcept
    {
#if 1
        if(head <= tail) {
            return tail - head;
        }
        return capacity - head + tail;
#else
        return (capacity - head + tail) % capacity;
#endif
    }

    inline  static constexpr bool is_fixed()
    {
        return _Fixed;
    };
};

template <typename _Policy, typename _Tp, bool isTrivial>
struct __circular_destroy_helper
{
public:
    typedef _Tp value_type;
    inline static void destroy (
            value_type* data,
            std::size_t position,
            ssize_t length,
            std::size_t capacity) noexcept
    {
        if(length < 0) {
            for (ssize_t idx = length + 1; idx <=0 ; ++idx) {
                value_type* d = data + _Policy::get_position(position, idx, capacity);
                try {
                    d->~value_type();
                }
                catch (const std::exception&) {
                }
            }
        }
        else {
            for (ssize_t idx = 0; idx < length ; ++idx) {
                value_type* d = data + _Policy::get_position(position, idx, capacity);
                try {
                    d->~value_type();
                }
                catch (const std::exception&) {
                }
            }
        }
    }

};

template <typename _Policy, typename _Tp>
struct __circular_destroy_helper<_Policy, _Tp, true>
{
public:
    typedef _Tp value_type;
    inline static void destroy (
            value_type* data,
            std::size_t position,
            ssize_t length,
            std::size_t capacity) noexcept
    {
        (void)data;
        (void)position;
        (void)length;
        (void)capacity;
        //do nothing
    }
};

template <typename _Policy>
struct circular_destroy_helper : public __circular_destroy_helper<
        _Policy,
        typename _Policy::value_type,
        std::is_trivial<typename _Policy::value_type>::value>
{
};

template <typename _Tp, bool _Fixed>
struct circular_policy : public circular_policy_base<_Tp, _Fixed> {
public:
    typedef typename circular_policy_base<_Tp, _Fixed>::value_type      value_type;

    inline static void destroy (
            value_type* data,
            std::size_t offset, 
            ssize_t length, 
            std::size_t capacity) noexcept
    {
        circular_destroy_helper<circular_policy<_Tp, _Fixed>>::destroy(data, offset, length, capacity);
    }
};

template <int value>
struct get_exp_ceil;

template <>
struct get_exp_ceil<4> {
    typedef uint32_t value_type ;
    inline int operator()(value_type value) const
    {
		const volatile auto fx = static_cast<float>(value);
		const volatile auto pix = (reinterpret_cast<const volatile uint32_t *>(&fx));
		const volatile auto ix = *pix;
        volatile int exp = static_cast<int>((ix >> 23u) & 0xffu) - 127;
        if(value & ((1u << exp) - 1)) {
            return exp + 1;
        }
        return exp;
    }
};

template <>
struct get_exp_ceil<8> {
    typedef uint64_t value_type ;
    inline int operator()(uint64_t value) const
    {
		const volatile auto fx = static_cast<double>(value);
		const volatile auto pix = (reinterpret_cast<const volatile uint64_t *>(&fx));
	    const volatile auto ix = *pix;
        volatile int exp = static_cast<int>((ix >> 52u) & 0x7ffu) - 1023;
        if(value & ((1u << exp) - 1)) {
            return exp + 1;
        }
        return exp;
    }
};

template <typename _Tp, bool _Fixed = true>
struct aligned_circular_policy : public circular_policy<_Tp, _Fixed> {
public:
    typedef typename circular_policy<_Tp, _Fixed>::value_type      value_type;

    inline static std::size_t get_capacity(std::size_t capacity)
    {
        return 1u << get_exp_ceil<sizeof(std::size_t)>()(capacity);
    }

    inline static std::size_t get_position(std::size_t position, ssize_t offset, std::size_t capacity) noexcept
    {
        return (position + offset) & (capacity - 1);
    }

    inline static bool empty(std::size_t head, std::size_t tail, std::size_t capacity) noexcept
    {
        (void)capacity;
        return head == tail;
    }

    inline static bool full(std::size_t head, std::size_t tail, std::size_t capacity) noexcept
    {
        return ((tail + 1) & (capacity - 1)) == head;
    }

    inline static std::size_t size(std::size_t head, std::size_t tail, std::size_t capacity) noexcept
    {
        return ((capacity + tail - head) & (capacity - 1));
    }
};

template <typename _Tp>
struct basic_circular_iterator {
public:
    typedef _Tp value_type;
    inline basic_circular_iterator(value_type* data, std::size_t head, std::size_t capacity) noexcept :
        _data(data),
        _end(data + capacity),
        _value(data + head)
    {
    }

    inline bool operator ==(const basic_circular_iterator &other) const noexcept
    {
        return _data == other._data
               && _end == other._end
               && _value == other._value;
    }

    inline bool operator !=(const basic_circular_iterator &other) const noexcept
    {
        return !(this->operator==(other));
    }

protected:
    inline void move_value(ssize_t offset) noexcept
    {
        _value = get_value(offset);
    }

    inline const value_type* get_value(ssize_t offset)
    {
        if(offset < 0) {
#if 0
            if(_value - _data < -offset) {
                return _end + offset + (_value - _data) ;
            }
            return _value + offset;
#endif
            return _value + (offset < _data - _value ? offset + _end - _data : offset);
        }
        return (_value + (offset < (_end - _value) ? offset : offset - (_end - _data)));
    }

    inline const value_type* value() const
    {
        return _value;
    }

protected:
    const value_type  *_data;
    const value_type  *_end;
    const value_type  *_value;
};

template <typename _Tp>
struct circular_iterator : public basic_circular_iterator<_Tp> {
public:
    typedef typename basic_circular_iterator<_Tp>::value_type value_type;
    inline circular_iterator(value_type* data, size_t head, size_t capacity) noexcept :
        basic_circular_iterator<_Tp>(data, head, capacity)
    {
    }

    inline value_type& operator *() noexcept
    {
        return *const_cast<value_type* >(this->value());
    }

    inline value_type* operator ->() noexcept
    {
        return const_cast<value_type* >(this->value());
    }

    inline value_type* pointer() noexcept
    {
        return const_cast<value_type* >(this->value());
    }

    inline circular_iterator &operator++() noexcept
    {
        basic_circular_iterator<_Tp>::move_value(1);
        return *this;
    }

    inline const circular_iterator operator++(int) noexcept
    {
        circular_iterator temp = *this;
        basic_circular_iterator<_Tp>::move_value(1);
        return temp;
    }

    inline circular_iterator &operator--() noexcept
    {
        basic_circular_iterator<_Tp>::move_value(-1);
        return *this;
    }

    inline const circular_iterator operator--(int) noexcept
    {
        circular_iterator temp = *this;
        basic_circular_iterator<_Tp>::move_value(-1);
        return temp;
    }
};

template <typename _Tp>
struct const_circular_iterator : public basic_circular_iterator<_Tp> {
public:
    typedef typename basic_circular_iterator<_Tp>::value_type value_type;

    inline const_circular_iterator(value_type* data, size_t head, size_t capacity) noexcept :
        basic_circular_iterator<_Tp>(data, head, capacity)
    {
    }

    inline const value_type& operator*() const noexcept
    {
        return *this->value();
    }

    inline const value_type* operator->() const noexcept
    {
        return this->value();
    }

    inline const_circular_iterator& operator++() noexcept
    {
        basic_circular_iterator<_Tp>::move_value(1);
        return *this;
    }

    inline const const_circular_iterator operator++(int) noexcept
    {
        const_circular_iterator temp = *this;
        basic_circular_iterator<_Tp>::move_value(1);
        return temp;
    }

    inline const_circular_iterator &operator--() noexcept
    {
        basic_circular_iterator<_Tp>::move_value(-1);
        return *this;
    }

    inline const const_circular_iterator operator--(int) noexcept
    {
        const_circular_iterator temp = *this;
        basic_circular_iterator<_Tp>::move_value(-1);
        return temp;
    }
};

template <typename _CircularIterator>
struct reverse_circular_iterator {
public:
    typedef _CircularIterator                   iterator_type;
    typedef typename iterator_type::value_type  value_type;
    inline reverse_circular_iterator(value_type* data, size_t head, size_t capacity) noexcept :
        _iterator(data, head, capacity)
    {
    }

    inline reverse_circular_iterator& operator++() noexcept
    {
        --_iterator;
        return *this;
    }

    inline const reverse_circular_iterator operator++(int) noexcept
    {
        reverse_circular_iterator temp = *this;
        --_iterator;
        return temp;
    }

    inline reverse_circular_iterator& operator--() noexcept
    {
        ++_iterator;
        return *this;
    }

    inline const reverse_circular_iterator operator--(int) noexcept
    {
        reverse_circular_iterator temp = *this;
        ++_iterator;
        return temp;
    }

    inline const value_type& operator*() const noexcept
    {
        iterator_type _temp = _iterator;
        --_temp;
        return *_temp;
    }

    inline const value_type* operator->() const noexcept
    {
        iterator_type _temp = _iterator;
        --_temp;
        return _temp.operator->();
    }

    inline bool operator ==(const reverse_circular_iterator &other) const noexcept
    {
        return _iterator == other._iterator;
    }

    inline bool operator !=(const reverse_circular_iterator &other) const noexcept
    {
        return _iterator != other._iterator;
    }

protected:
    iterator_type _iterator;
};

/**
 * 环形结构
 * -------------------------------------
 * |   | H | * | * | * | * | * |  T |
 * -------------------------------------
 */
template <typename _Tp, typename _Policy, class _Alloc>
class basic_circular {
public:
    using value_type              = _Tp;
    using policy                  = _Policy;
    using alloc_type              = typename std::allocator_traits<_Alloc>::template rebind_alloc<value_type>;
	using alloc_traits			  = std::allocator_traits<alloc_type>;
    using reference               = value_type&;
    using const_reference         = value_type const&;
    using pointer                 = value_type*;
    using const_pointer           = value_type const*;
    using iterator                = circular_iterator<_Tp>;
    using const_iterator          = const_circular_iterator<_Tp>;
    using reverse_iterator        = reverse_circular_iterator<iterator>;
    using const_reverse_iterator  = reverse_circular_iterator<const_iterator>;

    explicit basic_circular(std::size_t capacity) noexcept :
        _data(nullptr),
        _head(0),
        _tail(0),
        _capacity(capacity)
    {
        this->reserve(this->_capacity);
    }

    basic_circular(const basic_circular& other) noexcept :
        _data(nullptr),
        _head(0),
        _tail(0),
        _capacity(other._capacity)
    {
        this->reserve(this->_capacity);
        for(auto iter = other.cbegin(); iter != other.cend(); ++iter) {
            this->push_back(*iter);
        }
    }

    basic_circular& operator=(const basic_circular& other) noexcept
    {
        if(&other == this) {
            return *this;
        }
        this->clear();
        this->reserve(other._capacity);
        for(auto iter = other.cbegin(); iter != other.cend(); ++iter) {
            this->push_back(*iter);
        }
        return *this;
    }

    basic_circular(basic_circular&& other) noexcept :
        _data(other._data),
        _head(other._head),
        _tail(other._tail),
        _capacity(other._capacity),
        _allocator(std::move(other._allocator))
    {
        other._data = nullptr;
        other._head = 0;
        other._tail = 0;
        other._capacity = 0;
    }

    basic_circular& operator=(basic_circular&& other) noexcept
    {
        if(this->_data != nullptr) {
            this->clear();
        }
        _allocator.deallocate(_data, _capacity);
        _data = nullptr;
        this->swap(other);
        return *this;
    }

    ~basic_circular() noexcept
    {
        clear();

        if (_data != nullptr) {
            _allocator.deallocate(_data, _capacity);
        }
        _data = nullptr;
    }

    inline void clear()
    {
    	if(_data != nullptr)
    	{
			policy::destroy(_data, _head, this->size(), _capacity);
    	}
        _head = 0;
        _tail = 0;
    }

    inline void push_front(const value_type& data)
    {
        if (!this->push_prepare(1)) {
            throw std::out_of_range("circular::push_front()");
        }
        unchecked_push_front(data);
    }

    inline void unchecked_push_front(const value_type& data) noexcept
    {
        _head = policy::get_position(_head, -1, _capacity);
        new (_data + _head) value_type(data);
    }


    inline void push_front(value_type&& data)
    {
        if (!this->push_prepare(1)) {
            throw std::out_of_range("circular::push_front()");
        }
        unchecked_push_front(std::move(data));
    }

    inline void unchecked_push_front(value_type&& data) noexcept
    {
        _head = policy::get_position(_head, -1, _capacity);
        new (_data + _head) value_type(std::move(data));
    }

    inline void push_front(const value_type* data, std::size_t length)
    {
        if (!this->push_prepare(length)) {
            throw std::out_of_range("circular::push_front()");
        }
        unchecked_push_front(data, length);
    }

    inline void unchecked_push_front(const value_type* data, std::size_t length) noexcept
    {
        if (_head < length) {
            size_t len = length - _head;
            detail::memcpy(_data, data + len, _head);
            detail::memcpy(_data + _capacity - len, data, len);
            _head = policy::get_position(_head, -static_cast<ssize_t>(length), _capacity);
        }
        else {
            _head = policy::get_position(_head, -static_cast<ssize_t>(length), _capacity);
            detail::memcpy(_data + _head, data, length);
        }
    }

    inline void push_back(const value_type& data)
    {
        if (!this->push_prepare(1)) {
            throw std::out_of_range("circular::push_back()");
        }
        unchecked_push_back(data);
    }

    inline void unchecked_push_back(const value_type& data) noexcept
    {
        new (_data + _tail) value_type(data);
        _tail = policy::get_position(_tail, 1, _capacity);
    }

    inline void push_back(value_type&& data)
    {
        if (!this->push_prepare(1)) {
            throw std::out_of_range("circular::push_back()");
        }
        unchecked_push_back(std::move(data));
    }

    inline void unchecked_push_back(value_type&& data) noexcept
    {
        new (_data + _tail) value_type(std::move(data));
        _tail = policy::get_position(_tail, 1, _capacity);
    }

    inline void push_back(const value_type* data, std::size_t length)
    {
        if (!this->push_prepare(length)) {
            throw std::out_of_range("circular::push_back()");
        }
        unchecked_push_back(data, length);
    }

    inline void unchecked_push_back(const value_type* data, std::size_t length) noexcept
    {
        if (_tail + length > _capacity) {
            std::size_t len1 = _capacity - _tail;
            detail::memcpy(_data + _tail, data, len1);
            detail::memcpy(_data, data + len1, length - len1);
        }
        else {
            detail::memcpy(_data + _tail, data, length);
        }

        _tail = policy::get_position(_tail, length, _capacity);
    }

    inline void pop_front(std::size_t length)
    {
        if(this->size() < length) {
            throw std::out_of_range("circular::pop_front()");
        }
        unchecked_pop_front(length);
    }

    inline void unchecked_pop_front(std::size_t length) noexcept
    {
        policy::destroy(_data, _head, length, _capacity);
        _head = policy::get_position(_head, length, _capacity);
    }

    inline void pop_front()
    {
        if(this->size() < 1) {
            throw std::out_of_range("circular::pop_front()");
        }
        unchecked_pop_front();
    }

    inline void unchecked_pop_front() noexcept
    {
        policy::destroy(_data, _head, 1, _capacity);
        _head = policy::get_position(_head, 1, _capacity);
    }

    inline void pop_back(std::size_t length)
    {
        if(this->size() < length) {
            throw std::out_of_range("circular::pop_back()");
        }
        unchecked_pop_back(length);
    }

    inline void unchecked_pop_back(std::size_t length) noexcept
    {
        policy::destroy(_data, _tail, -static_cast<ssize_t>(length), _capacity);
        _tail = policy::get_position(_tail, -static_cast<ssize_t>(length), _capacity);
    }

    inline void pop_back()
    {
        if(this->size() < 1) {
            throw std::out_of_range("circular::pop_back()");
        }
        unchecked_pop_back();
    }

    inline void unchecked_pop_back() noexcept
    {
        policy::destroy(_data, _tail, -1, _capacity);
        _tail = policy::get_position(_tail, -1, _capacity);
    }

    inline void peek_front(value_type* data, std::size_t length) const
    {
        if(this->size() < length) {
            throw std::out_of_range("circular::peek_front()");
        }
        unchecked_peek_front(data, length);
    }

    inline void unchecked_peek_front(value_type* data, std::size_t length) const noexcept
    {
        std::size_t head = policy::get_position(_head, 0, _capacity);
        if (head + length > _capacity) {
            std::size_t len1 = _capacity - head;
            detail::memcpy(data, _data + head, len1);
            detail::memcpy(data + len1, _data, length - len1);
        }
        else {
            detail::memcpy(data, _data + head, length);
        }
    }

    inline void peek_back(value_type* data, std::size_t length) const
    {
        if(this->size() < length) {
            throw std::out_of_range("circular::peek_front()");
        }
        unchecked_peek_back(data, length);
    }

    inline void unchecked_peek_back(value_type* data, std::size_t length) const noexcept
    {
        std::size_t tail = policy::get_position(_tail, 0, _capacity);
        if (tail < length) {
            std::size_t len1 = length - tail;
            detail::memcpy(data, _data + _capacity - len1, len1);
            detail::memcpy(data + len1, _data, length - len1);
        }
        else {
            detail::memcpy(data, _data + tail - length, length);
        }
    }

    std::size_t tellg(std::size_t head) const noexcept
    {
        return policy::get_length(head, _head, _capacity);
    }

    std::size_t tellp(std::size_t tail) const noexcept
    {
        return policy::get_length(tail, _tail, _capacity);
    }

    void seekg(std::size_t head, std::size_t offset) noexcept
    {
        _head = policy::get_position(head, offset, _capacity);
    }

    void seekp(std::size_t tail, std::size_t offset) noexcept
    {
        //offset = std::min(offset, this->capacity() - policy::get_size(_head, tail, _capacity));
        _tail = policy::get_position(tail, offset, _capacity);
    }

    inline std::size_t head() const noexcept
    {
        return _head;
    }

    inline std::size_t tail() const noexcept
    {
        return _tail;
    }

    inline std::size_t get_position(std::size_t position, ssize_t offset) const noexcept
    {
        return policy::get_position(position, offset, _capacity);
    }

    inline reference front_at(std::size_t offset)
    {
        if(offset > this->size()) {
            throw std::out_of_range("circular::front_at");
        }
        return unchecked_front_at(offset);
    }

    inline const_reference front_at(std::size_t offset) const
    {
        if(offset > this->size()) {
            throw std::out_of_range("circular::front_at");
        }
        return unchecked_front_at(offset);
    }

    inline reference back_at(std::size_t offset)
    {
        if(offset > this->size()) {
            throw std::out_of_range("circular::back_at");
        }
        return unchecked_back_at(offset);
    }

    inline const_reference back_at(std::size_t offset) const
    {
        if(offset > this->size()) {
            throw std::out_of_range("circular::back_at");
        }
        return unchecked_back_at(offset);
    }

    inline reference unchecked_front_at(std::size_t offset) noexcept
    {
        return this->_data[policy::get_position(_head, offset, _capacity)];
    }

    inline const_reference unchecked_front_at(std::size_t offset) const noexcept
    {
		return this->_data[policy::get_position(_head, offset, _capacity)];
    }

    inline reference unchecked_back_at(std::size_t offset) noexcept
    {
        return this->_data[policy::get_position(_tail, -1 - offset, _capacity)];
    }

    inline const_reference unchecked_back_at(std::size_t offset) const noexcept
    {
        return this->_data[policy::get_position(_tail, -1 - offset, _capacity)];
    }

    inline std::size_t size() const noexcept
    {
        return policy::get_size(_head, _tail, _capacity);
    }

    inline std::size_t capacity() const noexcept
    {
        return _capacity - 1;
    }

    inline bool full() const noexcept
    {
        return policy::full(_head, _tail, _capacity);
    }

    inline bool empty() const noexcept
    {
        return policy::empty(_head, _tail, _capacity);
    }

    bool reserve(std::size_t capacity) noexcept
    {
        value_type* data = nullptr;
        capacity = policy::get_capacity(static_cast<uint32_t>(capacity));
        try {
            data = _allocator.allocate(capacity);
            if(data == nullptr) {
                return false;
            }
            this->peek_front(data, this->size());
        }
        catch (const std::exception&) {
            return false;
        }

        if(!empty()) {
            //for (iterator iter = begin(); iter != end(); ++iter) {
            //    alloc_traits::destroy(_allocator, iter.pointer());
            //}

            policy::destroy(_data, _tail, -static_cast<ssize_t>(this->size()), _capacity);
            _allocator.deallocate(_data, _capacity);
        }
        _tail = this->size();
        _head = 0;
        _capacity = capacity;
        _data = data;
        return true;
    }

    inline iterator begin() noexcept
    {
        return iterator(_data, _head, _capacity);
    }

    inline iterator end() noexcept
    {
        return iterator(_data, _tail, _capacity);
    }

    inline const_iterator cbegin() const noexcept
    {
        return const_iterator(_data, _head, _capacity);
    }

    inline const_iterator cend() const noexcept
    {
        return const_iterator(_data, _tail, _capacity);
    }

    inline reverse_iterator rbegin() noexcept
    {
        return reverse_iterator(_data, _tail, _capacity );
    }

    inline const_reverse_iterator crbegin() const noexcept
    {
        return const_reverse_iterator(_data, _tail, _capacity);
    }

    inline reverse_iterator rend() noexcept
    {
        return reverse_iterator(_data, _head, _capacity);
    }

    inline const_reverse_iterator crend() const noexcept
    {
        return const_reverse_iterator(_data, _head, _capacity);
    }

    inline constexpr bool is_fixed()
    {
        return policy::is_fixed();
    }

    inline bool push_prepare(std::size_t length)
    {
        if(capacity() - size() < length) {
            if(is_fixed()) {
                return false;
            }
            if (length > (_capacity << 1)) {
                return this->reserve(length << 1);
            }

            return this->reserve(_capacity << 1);
        }
        return true;
    }

    void swap(basic_circular& other) noexcept
    {
        std::swap(this->_data, other._data);
        std::swap(this->_head, other._head);
        std::swap(this->_tail, other._tail);
        std::swap(this->_capacity, other._capacity);
        std::swap(this->_allocator, other._allocator);
    }

protected:
    value_type*     _data;
    std::size_t     _head;
    std::size_t     _tail;
    std::size_t     _capacity;
    alloc_type      _allocator;
};

}
}
}

#endif //__STAR_SDK_UTILS_BASIC_CIRCULAR_H
