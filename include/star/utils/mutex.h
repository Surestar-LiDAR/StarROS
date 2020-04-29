/**
 * @author xiaoma 
 * @date 2019/7/24.
 */

#ifndef __STAR_SDK_UTILS_MUTEX_H
#define __STAR_SDK_UTILS_MUTEX_H

#include <star/Star.h>

#include <mutex>

#ifndef __USE_STD_THREAD__
#include <cstdint>
#include <chrono>

namespace ss {
namespace utils {
namespace detail {
    class __star_export basic_mutex {
    public:
        using native_handle_type = void*;

        basic_mutex() noexcept;
        ~basic_mutex() noexcept;

        basic_mutex(const basic_mutex&) = delete;
        basic_mutex& operator=(const basic_mutex&) = delete;

        void lock();
        bool try_lock() noexcept;
        bool try_lock_for_microsecond(int64_t microsecond) noexcept;
        void unlock();

        native_handle_type native_handle();
    private:
        native_handle_type _handle;
    };

class __star_export basic_recursive_mutex {
public:
    using native_handle_type = basic_mutex::native_handle_type;
    using native_thread_type = void*;

    basic_recursive_mutex() noexcept;
    ~basic_recursive_mutex() noexcept;

    basic_recursive_mutex(const basic_recursive_mutex&) = delete;
    basic_recursive_mutex& operator=(const basic_recursive_mutex&) = delete;

    void lock();
    bool try_lock();
    bool try_lock_for_microsecond(int64_t microsecond);
    void unlock();

private:
    basic_mutex         _mutex;
    native_thread_type  _owner;
    int                 _lock_count;
};
}

template <typename _Mutex>
class basic_mutex {
public:
    using mutex_impl_type = _Mutex;
    using native_handle_type = typename mutex_impl_type::native_handle_type;

    void lock()
    {
        _mutex.lock();
    }
    bool try_lock()
    {
        return _mutex.try_lock();
    }

    void unlock()
    {
        _mutex.unlock();
    }

    native_handle_type native_handle()
    {
        return _mutex.native_handle();
    }
private:
    _Mutex _mutex;
};

template <typename _Mutex>
class basic_timed_mutex {
public:
    using mutex_impl_type = _Mutex;
    using native_handle_type = typename _Mutex::native_handle_type;

    void lock()
    {
        _mutex.lock();
    }
    bool try_lock()
    {
        return _mutex.try_lock();
    }

    void unlock()
    {
        _mutex.unlock();
    }

    template <class Rep, class Period>
    bool try_lock_for(const std::chrono::duration<Rep, Period>& duration)
    {
        return _mutex.try_lock_for_microsecond(
                std::chrono::duration_cast<std::chrono::milliseconds>(duration).count());
    }

    template <class Clock, class Duration>
    bool try_lock_until(const std::chrono::time_point<Clock, Duration>& time_point)
    {
        typename Clock::time_point now = Clock::now();
        return _mutex.try_lock_for_microsecond(
                std::chrono::duration_cast<std::chrono::milliseconds>(time_point - now).count());
    }

    native_handle_type native_handle()
    {
        return _mutex.native_handle();
    }
private:
    mutex_impl_type _mutex;
};

using mutex = basic_mutex<detail::basic_mutex>;
using recursive_mutex = basic_mutex<detail::basic_recursive_mutex>;

using timed_mutex = basic_timed_mutex<detail::basic_mutex>;
using recursive_timed_mutex = basic_timed_mutex<detail::basic_recursive_mutex>;

}
}

namespace std {
using mutex = ss::utils::mutex;
using recursive_mutex = ss::utils::recursive_mutex;

using timed_mutex = ss::utils::timed_mutex;
using recursive_timed_mutex = ss::utils::recursive_timed_mutex;
}
#endif

#endif //__STAR_SDK_UTILS_MUTEX_H
