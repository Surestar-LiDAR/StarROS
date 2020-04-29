/**
 * @author      : John 
 * @date        : 2018-10-12
 */

#ifndef __STAR_SDK_UTILS_BUFFER_TRAITS_H
#define __STAR_SDK_UTILS_BUFFER_TRAITS_H

#include <cstddef>
#include <cstdint>

namespace ss {
namespace utils {
namespace detail {

template<typename _Buffer>
struct buffer_traits;

template<typename _Tp>
struct buffer_traits<_Tp *> {
    typedef _Tp         item_type;
    typedef _Tp*        data_type;
    typedef _Tp*        reference;
    typedef const _Tp*  const_reference;
};

template<typename _Tp, std::size_t _Num>
struct buffer_traits<_Tp[_Num]> {
    typedef _Tp         item_type;
    typedef _Tp*        data_type;
    typedef _Tp         (&reference)[_Num];
    typedef const _Tp   (&const_reference)[_Num];
};

template<typename _Tp, typename _Traits, typename _Alloc, template<class X, class Y, class A> class _Buffer>
struct buffer_traits<_Buffer<_Tp, _Traits, _Alloc> > {
    typedef _Tp                             item_type;
    typedef _Buffer<_Tp, _Traits, _Alloc>   data_type;
    typedef data_type&                      reference;
    typedef const data_type&                const_reference;
};

template<typename _Tp, typename _Alloc, template<class X, class A> class _Buffer>
struct buffer_traits<_Buffer<_Tp, _Alloc> > {
    typedef _Tp                     item_type;
    typedef _Buffer<_Tp, _Alloc>    data_type;
    typedef data_type&              reference;
    typedef const data_type&        const_reference;
};

template<typename _Tp, template<class X> class _Buffer>
struct buffer_traits<_Buffer<_Tp> > {
    typedef _Tp                 item_type;
    typedef _Buffer<_Tp>        data_type;
    typedef data_type&          reference;
    typedef const data_type&    const_reference;
};

}
}
}

#endif //__STAR_SDK_UTILS_BUFFER_TRAITS_H
