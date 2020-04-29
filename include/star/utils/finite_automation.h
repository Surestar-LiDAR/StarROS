/**
 * @author xiaoma
 * @date 2018/12/23
 */

#ifndef __STAR_SDK_UTILS_FINITE_AUTOMATION_H
#define __STAR_SDK_UTILS_FINITE_AUTOMATION_H

#include <star/utils/detail/basic_state_machine.h>

namespace ss {
namespace utils {
template <typename _Policy, typename _Alloc = std::allocator<typename _Policy::char_type> >
class basic_finite_automation {
public:
    using basic_automation = detail::basic_state_machine<_Policy, _Alloc>;
    using char_type        = typename basic_automation::char_type;
    using data_type        = typename basic_automation::data_type;
    using state_node       = typename basic_automation::state_node;

    basic_finite_automation() = default;

    basic_finite_automation(const char_type *characters,
        std::size_t length,
        const data_type& data) noexcept
    {
        transit(characters, length, data);
    }

    template<std::size_t length>
    basic_finite_automation(
        const char_type(&characters)[length],
        const data_type& data) noexcept
    {
        transit(characters, length, data);
    }

    state_node* transit(
        state_node* state,
        const char_type& character,
        const data_type& data,
        bool terminal = false) noexcept
    {
        _automation.transit(state, character, data, terminal);
    }

    state_node* transit(
        state_node* state,
        const char_type& character,
        state_node* next) noexcept
    {
        _automation.transit(state, character, next);
    }
    
    state_node* transit(
        state_node* state,
        const char_type *characters,
        std::size_t length,
        const data_type& data) noexcept
    {
        for (std::size_t idx = 0; idx < length - 1; ++idx) {
            state = _automation.transit(state, characters[idx], data_type{}, false);
            if (state == nullptr) { //todo 删除已经添加的状态
                return nullptr;
            }
        }
        return _automation.transit(state, characters[length -1], data, true);
    }

    template<std::size_t length>
    state_node* transit(
        state_node* state,
        const char_type(&characters)[length],
        const data_type& data) noexcept
    {
        return transit(state, characters, length, data);
    }

    state_node* transit(
        const char_type *characters,
        std::size_t length,
        const data_type& data)
    {
        return transit(first_state(), characters, length, data);
    }

    template<std::size_t length>
    state_node* transit(
        const char_type(&characters)[length],
        const data_type& data) noexcept
    {
        return transit(first_state(), characters, length, data);
    }

    bool merge(const basic_finite_automation &other)
    {
        return _automation.merge(other._automation);
    }

#if 0
    bool merge(basic_finite_automation &&other)
    {
        return _automation.merge(std::move(other._automation));
    }
#endif

    bool remove(state_node* state, const char_type *characters, std::size_t length)
    {
        state_node* next = walk(state, *characters);
        if (length == 1) {
            if (next == nullptr) {
                return false;
            }
            return _automation.remove(state, next);
        }
        return remove(next, characters + 1, --length);
    }

    bool remove(const char_type *characters, std::size_t length)
    {
        return this->remove(this->first_state(), characters, length);
    }

    const state_node* walk(const state_node* state, const char_type &character) const
    {
        if(state != nullptr) {
            return  _automation.walk(state, character);
        }
        return nullptr;
    }

    state_node* walk(state_node* state, const char_type &character) const
    {
        if(state != nullptr) {
            return  _automation.walk(state, character);
        }
        return nullptr;
    }

    bool is_accepted(const state_node* state) const
    {
        if (state == nullptr) {
            //throw std::runtime_error("basic_finite_automation::is_accepted() state is null.");
            return false;
        }
        return _automation.is_terminal(state);
    }

    bool is_terminal(const state_node* state) const
    {
        if (state == nullptr) {
            //throw std::runtime_error("basic_finite_automation::is_terminal() state is null.");
            return false;
        }
        return _automation.is_terminal(state);
    }

    state_node* first_state() noexcept
    {
        return _automation.first_state();
    }

    const state_node* first_state() const noexcept
    {
        return _automation.first_state();
    }

protected:
    basic_automation  _automation;
};

template <typename _Automation>
class basic_finite_automation_walker {
public:
    using state_node = typename _Automation::state_node;
    using char_type  = typename _Automation::char_type;
    using data_type  = typename _Automation::data_type;

    explicit basic_finite_automation_walker(const _Automation &automation) :
        _automation(automation),
        _state(_automation.first_state())
    {
    }

    bool walk(const char_type &character)
    {
        if(_state != nullptr) {
            _state = _automation.walk(_state, character);
        }
        return _state != nullptr;
    }

    bool is_accepted()
    {
        if(_state != nullptr) {
            return _automation.is_accepted(_state);
        }
        return false;
    }

    const state_node *state() const
    {
        return _state;
    }

private:
    const _Automation &_automation;
    const state_node *_state;
};

template <typename _Automation>
class basic_finite_automation_matcher {
public:
    using walker_type = basic_finite_automation_walker<_Automation>;
    using char_type   = typename walker_type::char_type;
    using data_type   = typename walker_type::data_type;

#if defined(__GNUC__)
    static const std::size_t npos = static_cast<std::size_t>(-1);
#else
    static constexpr auto npos{ static_cast<std::size_t>(-1) };
#endif
    explicit basic_finite_automation_matcher(const _Automation& automation) :
        _automation(automation)
    {
    }

    std::pair<std::size_t, data_type> match(const char_type* characters, std::size_t length)
    {
        return match(characters, length, false);
    }

    std::pair<std::size_t, data_type> match(const char_type* characters, std::size_t length, bool greedy)
    {
        walker_type walker(_automation);
        std::pair<std::size_t, data_type> result = { npos, data_type{} };
        std::size_t offset = 0;

        while (offset < length) {
            std::pair<std::size_t, data_type> temp = match(walker, characters, length, offset);
            if (temp.first == npos) {
                return result;
            }
            else {
                if (!greedy) { //不是贪心匹配
                    return temp;
                }
                result = temp;
                offset = result.first;
            }
        }

        return result;
        
    }

    std::pair<std::size_t, data_type> match_strict(const char_type* characters, std::size_t length)
    {
        std::pair<std::size_t, data_type> result = match(characters, length, true);
        if (result.first == length) {
            return result;
        }
        return { npos, data_type{} };
    }

protected:
    std::pair<std::size_t, data_type> match(walker_type &walker, const char_type* characters, std::size_t length, std::size_t offset) 
    {
        while (offset < length) {
            if (!walker.walk(characters[offset++])) {
                break;
            }
            if (walker.is_accepted()) {
                return { offset, walker.state()->data };
            }
        }

        return { npos, data_type{} };
    }

private:
    const _Automation &_automation;
};

#if defined(__GNUC__)
template <typename _Automation>
const std::size_t basic_finite_automation_matcher<_Automation>::npos;
#endif

template <typename CharType, 
        typename DataType,
        typename _Alloc = std::allocator<CharType> >
using finite_automation = 
            basic_finite_automation<
                detail::fsm::basic_state_machine_policy<CharType, DataType>,
                _Alloc>;

template <typename CharType, 
        typename DataType, 
        typename _Alloc = std::allocator<CharType> >
using finite_automation_matcher = 
            basic_finite_automation_matcher<
                finite_automation<CharType, DataType, _Alloc> >;

}

}

#endif //__STAR_SDK_UTILS_FINITE_AUTOMATION_H
