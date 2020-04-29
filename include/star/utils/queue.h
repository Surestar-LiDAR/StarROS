/**
 * @author      : John 
 * @date        : 2018-09-21
 */

#ifndef __LIGHT_EXT_QUEUE_H
#define __LIGHT_EXT_QUEUE_H

#include <star/utils/detail/basic_circular.h>
#include <mutex>
#include <condition_variable>
#include <vector>

namespace ss {
namespace utils {

template <typename _Tp, class _Policy, class _Alloc = std::allocator<_Tp>>
class basic_queue {
public:
    using basic_circular          = detail::basic_circular<_Tp, _Policy, _Alloc>  ;
    using value_type             = typename basic_circular::value_type            ;
    using reference              = typename basic_circular::reference             ;
    using const_reference        = typename basic_circular::const_reference       ;
    using pointer                = typename basic_circular::pointer               ;
    using iterator               = typename basic_circular::iterator              ;
    using const_iterator         = typename basic_circular::const_iterator        ;
    using reverse_iterator       = typename basic_circular::reverse_iterator      ;
    using const_reverse_iterator = typename basic_circular::const_reverse_iterator;

    explicit basic_queue(std::size_t capacity = 16) noexcept :
        _circular(capacity)
    {
    }

    void push_back(const value_type& data)
    {
        if (!_circular.push_prepare(1)) {
            throw std::out_of_range("queue::push_back()");
        }
        _circular.push_back(data);
    }

    void push_back(value_type&& data)
    {
        if (!_circular.push_prepare(1)) {
            throw std::out_of_range("queue::push_back()");
        }
        _circular.push_back(std::move(data));
    }

    template <typename _Iterator>
    void push_back(_Iterator begin, _Iterator end)
    {
        std::size_t count = std::distance(begin, end);
        if (!_circular.push_prepare(count)) {
            throw std::out_of_range("queue::push_back()");
        }

        for(_Iterator iter = begin; iter != end; ++iter) {
            _circular.push_back(std::forward(*iter));
        }
    }

    void push(const value_type& data)
    {
        push_back(data);
    }

    void push(value_type&& data)
    {
        push_back(data);
    }

    template <typename _Iterator>
    void push(_Iterator begin, _Iterator end)
    {
        push_back(begin, end);
    }

    void pop_front()
    {
        if(empty()) {
            throw std::out_of_range("queue::pop_front()");
        }

        _circular.pop_front();
    }

    void pop()
    {
        pop_front();
    }

    std::vector<value_type> take()
    {
        std::vector<value_type> result(0);
        result.reserve(this->size());
        while (!this->empty()) {
            const value_type& value = this->front();
            result.push_back(value);
            this->pop_front();
        }
        return result;
    }

    const_reference front() const
    {
        if(empty()) {
            throw std::out_of_range("queue::front() const");
        }
        return _circular.unchecked_front_at(0);
    }

    reference front()
    {
        if(empty()) {
            throw std::out_of_range("queue::front()");
        }
        return _circular.unchecked_front_at(0);
    }

    const_reference back() const
    {
        if(empty()) {
            throw std::out_of_range("queue::back() const");
        }
        return _circular.unchecked_back_at(0);
    }

    reference back()
    {
        if(empty()) {
            throw std::out_of_range("queue::back()");
        }
        return _circular.unchecked_back_at(0);
    }

    void clear()
    {
        _circular.clear();
    }

    reference operator[](std::size_t offset) noexcept
    {
        return _circular.at(offset);
    }

    const_reference operator[](std::size_t offset) const noexcept
    {
        return _circular.at(offset);
    }

    reference at(std::size_t offset)
    {
        if(offset > size()) {
            throw std::out_of_range("queue::at()");
        }
        return _circular.at(offset);
    }

    const_reference at(std::size_t offset) const
    {
        if(offset > size()) {
            throw std::out_of_range("queue::at() const");
        }
        return this->operator[](offset);
    }

    void reverse(std::size_t capacity)
    {
        _circular.reserve(capacity);
    }

    std::size_t size() const noexcept
    {
        return _circular.size();
    }

    std::size_t length() const noexcept
    {
        return size();
    }

    std::size_t capacity() const noexcept
    {
        return _circular.capacity();
    }

    std::size_t available() const noexcept
    {
        return capacity() - size();
    }

    bool empty() const noexcept
    {
        return _circular.empty();
    }

    bool full() const noexcept
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

private:
    basic_circular  _circular;
};

template <typename _Tp, class _Alloc = std::allocator<_Tp> >
using queue = basic_queue<_Tp, detail::circular_policy<_Tp, false>, _Alloc>;

template <typename _Tp, class _Alloc = std::allocator<_Tp> >
using fixed_queue = basic_queue<_Tp, detail::circular_policy<_Tp, true>, _Alloc>;

template <typename _Queue, typename _Mutex, typename _ConditionVariable>
class basic_block_queue {
public:
    using queue_type    = _Queue;
    using mutex_type    = _Mutex;
    using waiter_type   = _ConditionVariable;
    using value_type    = typename queue_type::value_type;

    explicit basic_block_queue(std::size_t capacity = 16) noexcept :
        _queue(capacity)
    {
    }

    void push_back(const value_type& value)
    {
        std::unique_lock<mutex_type> lock(_mutex);
        if(this->unsafe_full()) {
            _producer_waiter.wait(lock);
        }
        if(this->unsafe_full()) {
            throw std::out_of_range("queue::push()");
        }
        _queue.push_back(value);
        this->_consumer_waiter.notify_one();
    }

    void push_back(value_type&& value)
    {
        std::unique_lock<mutex_type> lock(_mutex);
        if(this->unsafe_full()) {
            _producer_waiter.wait(lock);
        }
        if(this->unsafe_full()) {
            throw std::out_of_range("queue::push()");
        }
        _queue.push_back(std::move(value));
        this->_consumer_waiter.notify_one();
    }

    value_type pop_front()
    {
        std::unique_lock<mutex_type> lock(_mutex);
        if(this->unsafe_empty()) {
            _consumer_waiter.wait(lock);
        }
        if(this->unsafe_empty()) {
            throw std::out_of_range("queue::pop()");
        }
        value_type value = _queue.front();
        _queue.pop_front();
        this->_producer_waiter.notify_one();
        return value;
    }

    std::vector<value_type> take()
    {
        std::lock_guard<mutex_type> lock(_mutex);
        std::vector<value_type> result(_queue.size());
        while(!empty()) {
            const value_type& value = _queue.front();
            result.push_back(value);
            _queue.pop_front();
        }

        this->_producer_waiter.notify_one();
        return result;
    }

    bool full() const noexcept
    {
        std::lock_guard<mutex_type> lock(_mutex);
        return _queue.full();
    }

    bool empty() const noexcept
    {
        std::lock_guard<mutex_type> lock(_mutex);
        return _queue.empty();
    }

protected:
    bool unsafe_full() const noexcept
    {
        return _queue.full();
    }

     bool unsafe_empty() const noexcept
     {
         return _queue.empty();
     }

private:
    queue_type          _queue;
    mutable mutex_type  _mutex;
    waiter_type         _consumer_waiter;
    waiter_type         _producer_waiter;
};

template <typename _Type, typename _Alloc = std::allocator<_Type>>
using fixed_block_queue = basic_block_queue<fixed_queue<_Type, _Alloc>, std::recursive_mutex, std::condition_variable_any>;

}
}

#endif //__LIGHT_EXT_QUEUE_H
