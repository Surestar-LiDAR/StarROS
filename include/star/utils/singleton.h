/**
* @author      : John
* @date        : 2016-10-20
*/

#ifndef ___STAR_UTILS_SINGLETON_H
#define ___STAR_UTILS_SINGLETON_H

#include <cstddef>
#include <star/utils/mutex.h>

#ifdef _MSC_VER
#pragma warning(disable:4661)
#endif

namespace ss {
namespace utils {
/**
 * 单实例模式基类
 * @tparam _Object 派生类
 *
 * 单实例模式基类对象，基于原子操作，线程安全，
 * 定义
 * class SingletonObject : public singleton<SingletonObject>
 * {
 * }
 *
 * 在CPP文件中调用SINGLETON_STATIC_DECLARE宏定义实例变量指针
 * SINGLETON_STATIC_DECLARE(SingletonObject);
 *
 */
template<typename _Object, typename _Mutex = std::mutex>
class singleton {
public:
    typedef _Object* instance_type;
    typedef _Mutex mutex_type;

    /**
     * 获取实例对象指针
     * @return 对象指针
     */
    static _Object* getInstance()
    {
        if (_object != nullptr) {
            return _object;
        }

        _mutex.lock();
        if (_object == nullptr) {
            _object = new _Object;
        }
        _mutex.unlock();
        return _object;
    }

    /**
     * 释放资源，在程序结束时调用
     * @note 单实例对象并不会自动调用该函数，需要用户手动调用
     */
    static void releaseInstance()
    {
        if (_object != nullptr) {
            _mutex.lock();
            if (_object != nullptr) {
                delete _object;
                _object = nullptr;
            }
            _mutex.unlock();
        }
    }

protected:
    virtual ~singleton()
    {
    }

    singleton()
    {
    }

    static mutex_type _mutex;
    static instance_type _object;
};

#if defined(_MSC_VER)
#define SINGLETON_STATIC_DECLARE(_Object) \
    template<> _Object::mutex_type ss::utils::singleton<_Object, _Object::mutex_type>::_mutex; \
    template<> _Object::instance_type ss::utils::singleton<_Object, _Object::mutex_type>::_object = nullptr;

#elif defined(__MINGW32__)
//使用__dllexport导出静态变量，解决在MinGW和cygwin中的链接错误
#define SINGLETON_STATIC_DECLARE(_Object) \
    template<> _Object::mutex_type __attribute__((dllexport)) ss::utils::singleton<_Object, _Object::mutex_type>::_mutex; \
    template<> _Object::instance_type __attribute__((dllexport)) ss::utils::singleton<_Object, _Object::mutex_type>::_object(nullptr);

#else
#define SINGLETON_STATIC_DECLARE(_Object) \
    template<> _Object::mutex_type ss::utils::singleton<_Object, _Object::mutex_type>::_mutex __attribute__((section(".data"))) = {}; \
    template<> _Object::instance_type ss::utils::singleton<_Object, _Object::mutex_type>::_object __attribute__((section(".data"))) = nullptr;
#endif
}
}

#endif //___STAR_UTILS_SINGLETON_H
