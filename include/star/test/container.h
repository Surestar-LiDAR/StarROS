/**
 * @author xiaoma 
 * @date 2019/11/26
 */

#ifndef __LIGHT_TEST_EXT_CONTAINER_H
#define __LIGHT_TEST_EXT_CONTAINER_H

namespace container {

template<typename _Container>
size_t get_size(const _Container& c) {
    return c.size();
}

template<class _Tp, size_t _Nm>
size_t get_size(_Tp (& __arr)[_Nm]) {
    (void) __arr;
    return _Nm;
}

template<typename _Container1, typename _Container2>
bool equals(const _Container1& c1, const _Container2& c2, std::size_t length = 0) {
    auto c1_size = get_size(c1);
    auto c2_size = get_size(c2);
    if (length == 0 && c1_size != c2_size) {
        return false;
    }
    auto iter1 = std::begin(c1);
    auto iter2 = std::begin(c2);
    std::size_t idx = 0;
    for (; iter1 != std::end(c1) && iter2 != std::end(c2) && (length != 0 && idx < length);
           ++iter1, ++iter2, ++idx) {
        if (*iter1 != *iter2) {
            return false;
        }
    }
    return true;
}

template<typename _Container>
void pop_front(_Container& container, std::size_t length) {
    for (std::size_t idx = 0; idx < length; ++idx) {
        container.pop_front();
    }
}


template<typename _Container>
void pop_back(_Container& container, std::size_t length) {
    for (std::size_t idx = 0; idx < length; ++idx) {
        container.pop_back();
    }
}

template<typename _Container>
_Container peek_front(_Container& container, std::size_t length) {
    _Container r;
    auto iter = std::begin(container);
    for (std::size_t idx = 0; idx < length; ++idx) {
        r.push_back(*iter++);
    }
    return r;
}


template<typename _Container>
_Container peek_back(_Container& container, std::size_t length) {
    _Container r;
    auto iter = std::rbegin(container);
    for (std::size_t idx = 0; idx < length; ++idx) {
        r.push_back(*iter++);
    }
    return r;
}

}

#endif //__LIGHT_TEST_EXT_CONTAINER_H
