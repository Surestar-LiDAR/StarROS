/**
 * @author      : John 
 * @date        : 2019-08-01
 */

#ifndef __STAR_SDK_UTILS_CONDITION_VARIABLE_H
#define __STAR_SDK_UTILS_CONDITION_VARIABLE_H
#include <star/Star.h>

#if !defined(__USE_STD_THREAD__)

#include <chrono>
#include <star/utils/mutex.h>
#include <memory>
#include <mutex>

namespace ss {
namespace utils {

enum class cv_status { no_timeout, timeout };

class condition_variable {
public:
    using native_handle_type = void*;
    using __clock_t = std::chrono::steady_clock;

    condition_variable();
    ~condition_variable() noexcept;

    condition_variable(const condition_variable&) = delete;
    condition_variable& operator=(const condition_variable&) = delete;

    void notify_one() noexcept;

    void notify_all() noexcept;

    void wait(std::unique_lock<mutex>& __lock);

    template<typename _Predicate>
    void wait(std::unique_lock<mutex>& __lock, _Predicate __p)
    {
        while (!__p()) {
            wait(__lock);
        }
    }

    template<typename _Duration>
    utils::cv_status wait_until(std::unique_lock<mutex>& __lock,
               const std::chrono::time_point<__clock_t, _Duration>& __atime)
    {
        return __wait_until_impl(__lock, __atime);
    }

    template<typename _Clock, typename _Duration>
    utils::cv_status wait_until(std::unique_lock<mutex>& __lock,
               const std::chrono::time_point<_Clock, _Duration>& __atime)
    {
        // DR 887 - Sync unknown clock to known clock.
        const typename _Clock::time_point __c_entry = _Clock::now();
        const __clock_t::time_point __s_entry = __clock_t::now();
        const auto __delta = __atime - __c_entry;
        const auto __s_atime = __s_entry + __delta;

        return __wait_until_impl(__lock, __s_atime);
    }

    template<typename _Clock, typename _Duration, typename _Predicate>
    bool wait_until(std::unique_lock<mutex>& __lock,
               const std::chrono::time_point<_Clock, _Duration>& __atime,
               _Predicate __p)
    {
        while (!__p())
            if (wait_until(__lock, __atime) == utils::cv_status::timeout)
                return __p();
        return true;
    }

    template<typename _Rep, typename _Period>
    utils::cv_status wait_for(std::unique_lock<mutex>& __lock,
             const std::chrono::duration<_Rep, _Period>& __rtime)
    {
        return wait_until(__lock, __clock_t::now() + __rtime);
    }

    template<typename _Rep, typename _Period, typename _Predicate>
    bool wait_for(std::unique_lock<mutex>& __lock,
             const std::chrono::duration<_Rep, _Period>& __rtime,
             _Predicate __p)
    {
        return wait_until(__lock, __clock_t::now() + __rtime, std::move(__p));
    }

    native_handle_type native_handle() noexcept
    {
        return _handle;
    }

protected:
    template<typename _Dur>
    utils::cv_status __wait_until_impl(std::unique_lock<mutex>& __lock, const std::chrono::time_point<__clock_t, _Dur>& __atime)
    {
        auto __s = std::chrono::time_point_cast<std::chrono::seconds>(__atime);
        auto __ns = std::chrono::duration_cast<std::chrono::nanoseconds>(__atime - __s);

        return __wait_for(__lock, __s, __ns);
    }

    utils::cv_status __wait_for(std::unique_lock<mutex>& __lock, const std::chrono::seconds& seconds, const std::chrono::nanoseconds& nanoseconds);

private:
    native_handle_type _handle;

};

/// condition_variable_any
// Like above, but mutex is not required to have try_lock.
class condition_variable_any
{
    typedef std::chrono::system_clock	__clock_t;
    condition_variable			_M_cond;
    std::shared_ptr<mutex>			_M_mutex;

    // scoped unlock - unlocks in ctor, re-locks in dtor
    template<typename _Lock>
    struct _Unlock
    {
        explicit _Unlock(_Lock& __lk) : _M_lock(__lk) { __lk.unlock(); }

        ~_Unlock() noexcept(false)
        {
            if (std::uncaught_exception())
            {
                __try
                { _M_lock.lock(); }
                __catch(const __cxxabiv1::__forced_unwind&)
                { __throw_exception_again; }
                __catch(...)
                { }
            }
            else
                _M_lock.lock();
        }

        _Unlock(const _Unlock&) = delete;
        _Unlock& operator=(const _Unlock&) = delete;

        _Lock& _M_lock;
    };

public:
    condition_variable_any() : _M_mutex(std::make_shared<mutex>()) { }
    ~condition_variable_any() = default;

    condition_variable_any(const condition_variable_any&) = delete;
    condition_variable_any& operator=(const condition_variable_any&) = delete;

    void
    notify_one() noexcept
    {
        std::lock_guard<mutex> __lock(*_M_mutex);
        _M_cond.notify_one();
    }

    void
    notify_all() noexcept
    {
        std::lock_guard<mutex> __lock(*_M_mutex);
        _M_cond.notify_all();
    }

    template<typename _Lock>
    void
    wait(_Lock& __lock)
    {
        std::shared_ptr<mutex> __mutex = _M_mutex;
        std::unique_lock<mutex> __my_lock(*__mutex);
        _Unlock<_Lock> __unlock(__lock);
        // *__mutex must be unlocked before re-locking __lock so move
        // ownership of *__mutex lock to an object with shorter lifetime.
        std::unique_lock<mutex> __my_lock2(std::move(__my_lock));
        _M_cond.wait(__my_lock2);
    }


    template<typename _Lock, typename _Predicate>
    void
    wait(_Lock& __lock, _Predicate __p)
    {
        while (!__p())
            wait(__lock);
    }

    template<typename _Lock, typename _Clock, typename _Duration>
    cv_status
    wait_until(_Lock& __lock,
               const std::chrono::time_point<_Clock, _Duration>& __atime)
    {
        std::shared_ptr<mutex> __mutex = _M_mutex;
        std::unique_lock<mutex> __my_lock(*__mutex);
        _Unlock<_Lock> __unlock(__lock);
        // *__mutex must be unlocked before re-locking __lock so move
        // ownership of *__mutex lock to an object with shorter lifetime.
        std::unique_lock<mutex> __my_lock2(std::move(__my_lock));
        return _M_cond.wait_until(__my_lock2, __atime);
    }

    template<typename _Lock, typename _Clock,
            typename _Duration, typename _Predicate>
    bool
    wait_until(_Lock& __lock,
               const std::chrono::time_point<_Clock, _Duration>& __atime,
               _Predicate __p)
    {
        while (!__p())
            if (wait_until(__lock, __atime) == cv_status::timeout)
                return __p();
        return true;
    }

    template<typename _Lock, typename _Rep, typename _Period>
    cv_status
    wait_for(_Lock& __lock, const std::chrono::duration<_Rep, _Period>& __rtime)
    { return wait_until(__lock, __clock_t::now() + __rtime); }

    template<typename _Lock, typename _Rep,
            typename _Period, typename _Predicate>
    bool
    wait_for(_Lock& __lock,
             const std::chrono::duration<_Rep, _Period>& __rtime, _Predicate __p)
    { return wait_until(__lock, __clock_t::now() + __rtime, std::move(__p)); }
};

}
}
namespace std {
    using cv_status = ss::utils::cv_status;
    using condition_variable = ss::utils::condition_variable;
    using condition_variable_any = ss::utils::condition_variable_any;
}

#else
#include <condition_variable>
#endif

#endif //__STAR_SDK_UTILS_CONDITION_VARIABLE_H
