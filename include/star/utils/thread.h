/**
 * Copyright (c) 2019 QuantumCTek. All rights reserved.
 * @author      : John 
 * @date        : 2019-07-30
 * @version     : 1.0.0
 */

#ifndef __STAR_SDK_UTILS_THREAD_H
#define __STAR_SDK_UTILS_THREAD_H

#include <star/Star.h>

#ifndef __USE_STD_THREAD__

#include <chrono>
#include <type_traits>
#include <utility>
#include <iostream>
#include <functional>
#include <memory>

namespace ss {
namespace utils {

class runnable {
public:
    virtual ~runnable() = default;
    virtual void run() = 0;
};

namespace detail {
    template <typename _Callable>
    class thread_impl : public runnable {
    public:
        explicit thread_impl(_Callable&& __callable) :
            _callable(std::forward<_Callable>(__callable))
        {
        }

        void run() override
        {
            _callable();
        }
    private:
        _Callable _callable;
    };
}

class __star_export thread {
public:
    using native_handle_type = void*;

    class id
    {
    public:
        id() noexcept :
            _thread(nullptr)
        {
        }

        explicit id(native_handle_type __id) noexcept :
            _thread(__id)
        {
        }

    private:
        friend class thread;
        friend struct std::hash<thread::id>;

        friend bool operator==(thread::id __x, thread::id __y) noexcept
        {
            return __x._thread == __y._thread;
        }

        friend bool operator<(thread::id __x, thread::id __y) noexcept
        {
            return __x._thread < __y._thread;
        }

        template<class _CharT, class _Traits>
        friend std::basic_ostream<_CharT, _Traits>& operator<<(
                std::basic_ostream<_CharT, _Traits>& __basic_ostream,
                const thread::id& __id) noexcept
        {
            if (__id == ss::utils::thread::id()) {
                return __basic_ostream << "thread::id of a non-executing thread";
            }
            else {
                return __basic_ostream << __id._thread;
            }
        }

    private:
        native_handle_type    _thread;
    };

    thread() noexcept = default;

    template<typename _Callable,
		typename = typename std::enable_if<
			!std::is_integral<
				typename std::remove_reference<_Callable>::type>::value,
			int>::type,
		typename... _Args>
    explicit thread(
    		_Callable && __f,
			_Args&&... __args)
    {
        start_thread(make_routine(std::bind(
                std::forward<_Callable>(__f),
                std::forward<_Args>(__args)...)), 0);
    }

    template<typename _Callable, typename... _Args>
	thread(std::size_t stack_size, _Callable&& __f, _Args&&... __args)
	{
		start_thread(make_routine(std::bind(
				std::forward<_Callable>(__f),
				std::forward<_Args>(__args)...)), stack_size);
	}


    thread(thread&) = delete;
    thread(const thread&) = delete;

    ~thread() noexcept;

    thread& operator=(const thread&) = delete;

    thread& operator=(thread&& __t) noexcept
    {
        if (joinable())
            std::terminate();
        swap(__t);
        return *this;
    }

    thread(thread&& __t) noexcept:
        _id()
    {
        swap(__t);
    }

    bool joinable() const noexcept
    {
        return !(_id == id());
    }

    void join();

    void detach();

    static unsigned int hardware_concurrency() noexcept;

    thread::id get_id() const noexcept
    {
        return _id;
    }

    /** @pre thread is joinable
     */
    native_handle_type native_handle()
    {
        return _id._thread;
    }

    // Returns a value that hints at the number of hardware thread contexts.

    void swap(thread& __t) noexcept
    {
        std::swap(_id, __t._id);
    }

private:
    void start_thread(std::unique_ptr<runnable>&& __runnable, std::size_t stack_size);

    template<typename _Callable>
    std::unique_ptr<detail::thread_impl<_Callable>> make_routine(_Callable&& callable)
    {
        auto impl = new detail::thread_impl<_Callable>(std::forward<_Callable>(callable));
        return std::unique_ptr<detail::thread_impl<_Callable>>(impl);
    }

private:
    id  _id;
};

namespace this_thread
{
    thread::id get_id();
    void yield() noexcept;

    void __sleep_for(std::chrono::seconds, std::chrono::nanoseconds);

    template<typename _Rep, typename _Period>
    inline void sleep_for(const std::chrono::duration<_Rep, _Period>& __rtime)
    {
        auto __s = std::chrono::duration_cast<std::chrono::seconds>(__rtime);
        auto __ns = std::chrono::duration_cast<std::chrono::nanoseconds>(__rtime - __s);
        __sleep_for(__s, __ns);
    }

    /// sleep_until
    template<typename _Clock, typename _Duration>
    inline void sleep_until(const std::chrono::time_point<_Clock, _Duration>& __atime)
    {
        sleep_for(__atime - _Clock::now());
    }
}

}
}

namespace std {

inline void swap(ss::utils::thread& __x, ss::utils::thread& __y) noexcept
{
    __x.swap(__y);
}

inline bool
operator!=(ss::utils::thread::id __x, ss::utils::thread::id __y) noexcept
{
    return !(__x == __y);
}

inline bool
operator<=(ss::utils::thread::id __x, ss::utils::thread::id __y) noexcept
{
    return !(__y < __x);
}

inline bool
operator>(ss::utils::thread::id __x, ss::utils::thread::id __y) noexcept
{
    return __y < __x;
}

inline bool
operator>=(ss::utils::thread::id __x, ss::utils::thread::id __y) noexcept
{
    return !(__x < __y);
}

template<>
struct hash<ss::utils::thread::id>
        : public std::unary_function<ss::utils::thread::id, size_t>
{
    std::size_t operator()(const ss::utils::thread::id& __id) const noexcept
    {
        return std::_Hash_impl::hash(__id._thread);
    }
};

}

namespace std {
using thread = ss::utils::thread;
namespace this_thread {
    using namespace ss::utils::this_thread;
}
}

#else
#include <thread>
#endif

#endif //__STAR_SDK_UTILS_THREAD_H
