/**
 * @author      : John 
 * @date        : 2019-09-06
 */

#ifndef __STAR_FIRE_FACTORY_H
#define __STAR_FIRE_FACTORY_H

#include <cstdint>
#include <memory>
#include <cstdio>
#include <map>

namespace ss {
namespace utils {

template <typename _Type>
struct factory_allocator {
    virtual ~factory_allocator() = default;
    virtual _Type allocate() = 0;
    virtual void deallocate(_Type& object) = 0;

    virtual std::size_t max_length() const noexcept = 0;
};

namespace detail {

    struct __has_static_max_length_impl
    {
        template <typename _Object, typename = decltype(_Object::max_length())>
        static std::true_type Test(int);

        template <typename>
        static std::false_type Test(...);
    };

    template <typename _Object>
    struct __has_static_max_length :
            public decltype(__has_static_max_length_impl::Test<_Object>(0))
    {
    };

    struct __has_static_size_impl
    {
        template <typename _Object, typename = decltype(_Object::size())>
        static std::true_type Test(int);

        template <typename>
        static std::false_type Test(...);
    };

    template <typename _Object>
    struct __has_static_size :
            public decltype(__has_static_size_impl::Test<_Object>(0))
    {
    };

    template <typename _Object>
    struct run_max_length
    {
        static std::size_t get()
        {
            return _Object::max_length();
        }
    };

template <typename _Object>
    struct run_size
    {
        static std::size_t get()
        {
            return _Object::size();
        }
    };

template <typename _Object>
    struct run_sizeof
    {
        static std::size_t get()
        {
            return sizeof(_Object);
        }
    };

    template <typename _Object>
    std::size_t get_max_length()
    {
        using runner = typename std::conditional<
                __has_static_max_length<_Object>::value,
                run_max_length<_Object>,
                typename std::conditional<
                        __has_static_size<_Object>::value,
                        run_size<_Object>,
                        run_sizeof<_Object>>::type
                >::type;
        return runner::get();
    }

    template<typename _Type, typename _Object, typename _Allocator>
    struct factory_object_allocator : factory_allocator<_Type*> {
        typedef typename _Allocator::template rebind<_Object>::other allocator_type;

        inline _Type *allocate() override
        {
            _Object *object = allocator.allocate(1);
            allocator.construct(object);
            return object;
        }

        inline void deallocate(_Type*& object) override
        {
            if(object != nullptr) {
                allocator.destroy((_Object *) object);
                allocator.deallocate((_Object *) object, 1);
            }
            object = nullptr;
        }

        std::size_t max_length() const noexcept override
        {
            return get_max_length<_Object>();
        }

    private:
        allocator_type allocator;
    };

    template<typename _Type, typename _Object, typename _Allocator>
    struct factory_shared_pointer_allocator : factory_allocator<std::shared_ptr<_Type>> {
        typedef typename _Allocator::template rebind<_Object>::other allocator_type;

        inline std::shared_ptr<_Type> allocate() override
        {
            auto object = std::dynamic_pointer_cast<_Type>(std::allocate_shared<_Object>(allocator));
            return object;
        }

        inline void deallocate(std::shared_ptr<_Type>& object) override
        {
            object = nullptr;
        }

        std::size_t max_length() const noexcept override
        {
            return get_max_length<_Object>();
        }
    private:
        allocator_type allocator;
    };

    template <typename _Tp>
    struct pointer_factory_policy {
        using pointer_type = _Tp*;

        using basic_allocator = factory_allocator<pointer_type>;

        template <typename _Object, typename _Allocator>
        using object_allocator = factory_object_allocator<_Tp, _Object, _Allocator>;

        static pointer_type get_null()
        {
            return nullptr;
        }
    };

    template <typename _Tp>
    struct shared_factory_policy {
        using pointer_type = std::shared_ptr<_Tp>;

        using basic_allocator = factory_allocator<pointer_type>;

        template <typename _Object, typename _Allocator>
        using object_allocator = factory_shared_pointer_allocator<_Tp, _Object, _Allocator>;

        static pointer_type get_null()
        {
            return std::shared_ptr<_Tp>();
        }
    };
}

/**
 * 对象工厂，用于对象池实现和消息解析中消息体创建
 * _Allocator 使用通用的allocator
 */
template<typename _Id, typename _Type, class _Allocator, class _Policy>
class basic_factory {
public:
    using policy_type    = _Policy;
    using pointer        = typename policy_type::pointer_type;
    using allocator_type = typename policy_type::basic_allocator;
    using factory_type   = std::map<_Id, allocator_type*> ;

public:
    basic_factory()
    {
        _allocators = {};
    }

    virtual ~basic_factory()
    {
        for (typename factory_type::iterator ite = _allocators.begin(); ite != _allocators.end();) {
            delete ite->second;
            ite = _allocators.erase(ite);
        }
    }

    template<typename _Object, typename _Alloc = typename policy_type::template object_allocator<_Object, _Allocator>>
    bool material(const _Id& id)
    {
        allocator_type *allocator = new _Alloc();
        return material(id, allocator);
    }

    bool material(const _Id& id)
    {
        return material(id, nullptr);
    }

    bool material(const _Id& id, allocator_type* allocator)
    {
        std::pair<typename factory_type::iterator, bool> ret = _allocators.insert(std::make_pair(id, allocator));
        if(!ret.second) {
            delete allocator;
            return false;
        }
        return true;
    }

    pointer make(const _Id& id) const
    {
        auto iter = _allocators.find(id);
        if (iter == _allocators.end()) {
            return policy_type::get_null();
        }
        if(iter->second == nullptr) {
            return policy_type::get_null();
        }
        return iter->second->allocate();
    }

    std::size_t max_length(const _Id& id) const noexcept
    {
        auto ite = _allocators.find(id);
        if (ite == _allocators.end()) {
            return 0u;
        }
        if(ite->second == nullptr) {
            return 0u;
        }
        return ite->second->max_length();
    }

    bool contains(const _Id& id) const
    {
        typename factory_type::const_iterator iter = _allocators.find(id);
        return iter != _allocators.end();
    }

    allocator_type* find(const _Id& id) const
    {
        auto iter = _allocators.find(id);
        if (iter == _allocators.end()) {
            return nullptr;
        }
        return iter->second;
    }

private:
    factory_type _allocators;
};

template<typename _Id, typename _Type, class _Allocator = std::allocator<_Type>>
using object_factory = basic_factory<_Id, _Type, _Allocator, detail::pointer_factory_policy<_Type>>;

template<typename _Id, typename _Type, class _Allocator = std::allocator<_Type>>
using shared_factory = basic_factory<_Id, _Type, _Allocator, detail::shared_factory_policy<_Type>>;

}
}

#endif //__STAR_FIRE_FACTORY_H
