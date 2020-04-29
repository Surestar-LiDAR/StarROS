/**
 * @author      : John 
 * @date        : 2018-09-10
 */

#ifndef __STAR_SDK_UTILS_SLIDING_WINDOW_H
#define __STAR_SDK_UTILS_SLIDING_WINDOW_H

#include <star/utils/queue.h>

namespace ss {
namespace utils {

template <typename _Pos, typename _Tp, typename _Alloc = std::allocator<_Tp>>
class sliding_window {
protected:
    struct slide_node {
        _Pos        position;
        _Tp         value;
    };
public:
    typedef _Tp                                                     value_type;
    typedef _Pos                                                    pos_type;
    typedef typename _Alloc::template rebind<slide_node>::other     allocator_type;
    typedef ss::utils::queue<slide_node, allocator_type>            queue_type;

    sliding_window(pos_type range, std::size_t capacity) :
        _range(range),
        _queue(capacity + 1)
    {
    }

    std::pair<pos_type, value_type> get_value(const pos_type &pos) const
    {
        pos_type first = pos - _range;
        pos_type realFirst = first;
        value_type value = 0;
#if 0
        for (auto iter = _queue.rbegin(); iter != _queue.rend(); ++iter) {
#if 0
            if ((*iter).position < first) {
                ++iter;
                _queue.pop_front();
            }
            else {
                value += (*iter).value;
                ++iter;
            }
#endif
            if ((*iter).position < first) {
                break;
            }
            else {
                real_first = (*iter).position;
                value += (*iter).value;
            }
    }
#endif
        for (auto iter = _queue.rbegin(); iter != _queue.rend(); ++iter) {
            if ((*iter).position < first) {
                break;
            }
            realFirst = (*iter).position;
            value += (*iter).value;
        }
        if (_queue.empty()) {
            return { pos, value };
        }
        return { pos - realFirst == 0 ? 1 : pos - realFirst, value };
    }

    void push(const pos_type &pos, const value_type &value)
    {
        try {
            pos_type first = pos - _range;
            for(auto iter = _queue.begin(); iter != _queue.end(); ) {
                if((*iter).position < first) {
                    ++iter;
                    _queue.pop_front();
                }
                else {
                    break;
                }
            }
            if(!_queue.empty() && _queue.back().position == pos) {
                _queue.back().value += value;
            }
            else {
                _queue.push_back({pos, value});
            }
        }
        catch(const std::exception &ex) {
            TRACE("%s\n", ex.what());
        }
    }

    void clear()
    {
        _queue.clear();
    }

private:
	pos_type        _range;
    queue_type      _queue;
};

}
}

#endif //__STAR_SDK_UTILS_SLIDING_WINDOW_H
