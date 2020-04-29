/**
 * @author xiaoma
 * @date 2019/5/27
 */

#ifndef __STAR_SDK_UTILS_DETAIL_BASIC_STATE_MACHINE_H
#define __STAR_SDK_UTILS_DETAIL_BASIC_STATE_MACHINE_H

#include <star/utils/detail/hash_table.h>

#include <limits>
#include <cstring>
#include <memory>

#ifdef __GNUC__
#pragma GCC diagnostic ignored "-Wchar-subscripts"
#endif

namespace ss {
namespace utils {
namespace detail {
namespace fsm {

template <typename _Data>
struct basic_state_node {
    bool         terminal;  //终结节点
    std::size_t  state;     //状态编号
    _Data        data;      //节点数据
    std::size_t  references;//引用次数
};

#if 0
template <typename _Char, typename _Data>
struct hash_state_node : basic_state_node<_Data> {

    //struct node_type : utils::detail::signal_hash_table_node<>

    typedef std::map<_Char, hash_state_node*> edge_table;

	explicit hash_state_node(std::size_t state) :
		basic_state_node<_Data>{ false, state, _Data(), 0 }
	{
	}

    hash_state_node(std::size_t state, const _Data &data, bool terminal) :
        basic_state_node<_Data>{ terminal, state, data, 0 }
    {
    }

    hash_state_node(const hash_state_node &) = delete;
    hash_state_node(hash_state_node &&) = delete;
    hash_state_node &operator=(const hash_state_node &) = delete;
    hash_state_node &operator=(hash_state_node &&) = delete;

    template <typename _Alloc>
    hash_state_node* clone(_Alloc &allocator)
    {
        hash_state_node *node = allocator.allocate(node);
		std::allocator_traits<_Alloc>::construct(allocator, node);
        node->state    = this->state;
        node->data     = this->data;
        node->terminal = this->terminal;
        for (auto iter = _edges.begin(); iter != _edges.end(); ++iter) {
            hash_state_node *edge = iter->second.clone();
            node->_edges.insert(std::make_pair(iter.first, edge));
        }
        return *this;
    }

    template <typename _Alloc>
    void clear(_Alloc &allocator)
    {
        for (auto iter = _edges.begin(); iter != _edges.end(); ++iter) {
            allocator.deallocate();
        }
    }

    const hash_state_node* walk(const _Char& character) const noexcept
    {
        auto iter = _edges.find(character);
        if (iter != _edges.end()) {
            return iter->second;
        }

        return nullptr;
    }

    hash_state_node* walk(const _Char& character) noexcept
    {
        auto iter = _edges.find(character);
        if (iter != _edges.end()) {
            return iter->second;
        }

        return nullptr;
    }

    bool transit(const _Char& character, hash_state_node* state) noexcept
    {
        auto iter = _edges.insert(std::make_pair(character, state));
        return iter.second;
    }

    const hash_state_node* find(std::size_t state) const noexcept
    {
        for (auto iter = _edges.begin(); iter != _edges.end(); ++iter) {
            if (iter->second->state == state) {
                return iter.second;
            }
        }
        return nullptr;
    }

    hash_state_node* find(std::size_t state) noexcept
    {
        for (auto iter = _edges.begin(); iter != _edges.end(); ++iter) {
            if (iter->second->state == state) {
                return iter.second;
            }
        }
        return nullptr;
    }

    edge_table _edges;
};
#endif

template <typename _Char, typename _Data>
struct table_state_node : basic_state_node<_Data> { //todo move and copy constructor

    using char_type = _Char;
    using data_type = _Data;

    static constexpr std::size_t table_size =
            static_cast<std::size_t>(std::numeric_limits<_Char>::max()
                - std::numeric_limits<_Char>::min());

    table_state_node() :
        basic_state_node<_Data>{ false, 0, {}, 0 },
        _size(0),
        _edges{nullptr}
    {
    }

    table_state_node(std::size_t __state, const _Data& data, bool terminal) :
        basic_state_node<_Data>{ terminal, __state, data, 0 },
        _size(0),
        _edges{ nullptr }
    {
    }
  
    template <typename _Alloc>
    static table_state_node* create(_Alloc& allocator)
    {
        table_state_node *node = allocator.allocate(1);
		std::allocator_traits<_Alloc>::construct(allocator, node);
        return node;
    }

    template <typename _Alloc>
    static table_state_node* create(std::size_t __state, _Alloc& allocator)
    {
        table_state_node *node = allocator.allocate(1);
		std::allocator_traits<_Alloc>::construct(allocator, node, __state, {}, false);
        return node;
    }

    template <typename _Alloc>
    static table_state_node* create(std::size_t __state, const _Data& data, bool terminal, _Alloc& allocator)
    {
        table_state_node *node = allocator.allocate(1);
		std::allocator_traits<_Alloc>::construct(allocator, node, __state, data, terminal);
        return node;
    }

    template <typename _Alloc>
    static void destroy(table_state_node* node, _Alloc& allocator)
    {
        node->clear(allocator);
        std::allocator_traits<_Alloc>::destroy(allocator, node);
        allocator.deallocate(node, 1);
    }

    template <typename _Alloc>
    table_state_node* clone(_Alloc& allocator) const
    {
        table_state_node* node = create(allocator);

        node->state      = this->state;
        node->data       = this->data;
        node->terminal   = this->terminal;
        node->_size      = this->_size;
        for (std::size_t idx = 0, count = 0; idx < table_size && count < size(); ++idx) {
            if (_edges[idx] != nullptr) {
                node->_edges[idx] = _edges[idx]->clone(allocator);
                ++node->_edges[idx]->references;
                ++count;
            }
        }
        return node;
    }

    table_state_node* move(table_state_node* to)
    {
        for (std::size_t idx = 0; idx < table_size; ++idx) {
            to->_edges[idx] = _edges[idx];
            to->_edges[idx] = nullptr;
        }
    }

    template <typename _Alloc>
    void clear(_Alloc& allocator)
    {
        for (auto& node : _edges) {
            if(empty()) {
                break;
            }
            if (node != nullptr) {
                if(--node->references == 0) {
                    node->clear(allocator);
                    std::allocator_traits<_Alloc>::destroy(allocator, node);
                    allocator.deallocate(node, 1);
                }
                --_size;
                node = nullptr;
            }
        }
#if 0
        for (std::size_t idx = 0; idx < table_size && !empty(); ++idx) {
            if (_edges[idx] != nullptr) {
                if(--_edges[idx]->references == 0) {
                    _edges[idx]->clear(allocator);
					std::allocator_traits<_Alloc>::destroy(allocator, _edges[idx]);
                    allocator.deallocate(_edges[idx], 1);
                }
                _edges[idx] = nullptr;
            }
        }
#endif
    }

    template <typename _Alloc>
    bool remove(table_state_node* state, _Alloc &allocator)
    {
	    if(state == nullptr) {
	        return true;
	    }
        for (auto& node : _edges) {
            if (node == state) {
                if(--node->references == 0) {
                    node->clear(allocator);
					std::allocator_traits<_Alloc>::destroy(allocator, node);
                    allocator.deallocate(node, 1);
                }
                node = nullptr;
                return true;
            }
        }
        return false;
    }

    template <typename _Automation>
    bool merge(const table_state_node* state, _Automation &automation)
    {
        for (std::size_t idx = 0, count = 0; idx < table_size && count < state->size(); ++idx) {
            if (state->_edges[idx] != nullptr) {
                if (_edges[idx] == nullptr) {
                    _edges[idx] = state->_edges[idx]->clone(automation.get_allocator());
                    _edges[idx]->state = automation.next_state();
                    ++_size;
                    ++_edges[idx]->references;
                }
                else {
                    if (state->_edges[idx]->terminal) {
                        if(_edges[idx]->terminal && _edges[idx]->data != state->_edges[idx]->data) {
                            return false;
                        }
                        _edges[idx]->terminal = true;
                        _edges[idx]->data = state->_edges[idx]->data;
                    }
                    _edges[idx]->merge(state->_edges[idx], automation);
                }
            }
        }
        return true;
    }

    bool transit(const _Char& character, table_state_node* __state) noexcept
    {
        if (_edges[character] == nullptr) {
            _edges[character] = __state;
            ++__state->references;
            ++_size;
            return true;
        }
        return false;
    }

    const table_state_node* find(std::size_t __state) const noexcept
    {
        for (auto iter = std::begin(_edges); iter != std::end(_edges); ++iter) {
            if ((*iter)->state == __state) {
                return *iter;
            }
        }
        return nullptr;
    }

    table_state_node* find(std::size_t __state) noexcept
    {
        for (auto iter = std::begin(_edges); iter != std::end(_edges); ++iter) {
            if ((*iter)->state == __state) {
                return *iter;
            }
        }
        return nullptr;
    }

    const table_state_node* walk(const _Char& character) const noexcept
    {
        return _edges[character];
    }

    table_state_node* walk(const _Char& character) noexcept
    {
        return _edges[character];
    }

    std::size_t size() const noexcept
    {
        return _size;
    }

    bool empty() const noexcept
    {
	    return _size == 0;
    }

private:
    std::size_t       _size;//根的个数
    table_state_node* _edges[table_size];
};

template <int _Size, typename _Char, typename _Data>
struct make_state_node_impl {
    //using state_node = hash_state_node<_Char, _Data>;
};

template <typename _Char, typename _Data>
struct make_state_node_impl<1, _Char, _Data> {
    using state_node = table_state_node<_Char, _Data>;
};

template <typename _Char, typename _Data>
struct make_state_node : make_state_node_impl<sizeof(_Char), _Char, _Data> {
};

template <typename _Char, typename _Data>
struct basic_state_machine_policy {
    using char_type  = _Char;
    using data_type  = _Data;
    using state_node = typename make_state_node<_Char, _Data>::state_node;

    template <typename _Automation>
    static void destroy(_Automation& automation, state_node* state)
    {
        state_node::destroy(state, automation.get_allocator());
    }

    template <typename _Automation>
    static state_node* clone(_Automation& automation, const state_node* state)
    {
        return state->clone(automation.get_allocator());
    }

    template <typename _Automation>
    static void clear(_Automation& automation, state_node* state)
    {
        state->clear(automation.get_allocator());
    }

    template <typename _Automation>
    static bool remove(_Automation& automation, state_node* from, state_node *state)
    {
        if(from != nullptr) {
            return from->remove(state, automation.get_allocator());
        }
        return false;
    }

    template <typename _Automation>
    static bool merge(_Automation& automation, state_node* state, const state_node* other)
    {
        return state->merge(other, automation);
    }

    template <typename _Automation>
    static state_node *transit(
        _Automation& automation,
        state_node* state,
        const char_type& character,
        const data_type& data,
        bool terminal)
    {
        state_node* next_node = state->walk(character);

        if (next_node == nullptr) { //是一个全新的节点
            next_node = state_node::create(automation.next_state(), data, terminal,
                automation.get_allocator());
            if (state->transit(character, next_node)) {
                return next_node;
            }
            else {
                state_node::destroy(next_node, automation.get_allocator());
                return nullptr;
            }
        }
        else {
            if (terminal && next_node->terminal) { //已经存在以一个终结状态，再插入一个出现重复
                return nullptr;
            }
            if (terminal) {
                next_node->data = data;
                next_node->terminal = terminal;
            }
            return next_node;
        }
    }

    template <typename _Automation>
    static state_node *transit(
        _Automation& automation,
        state_node* state,
        const char_type& character,
        state_node* next_state)
    {
        (void)automation;
        if (state != nullptr) {
            if (state->transit(character, next_state)) {
                return next_state;
            }
        }
        return nullptr;
    }

    static state_node *walk(state_node* state, const char_type& character)
    {
        if (state != nullptr) {
            return state->walk(character);
        }
        return nullptr;
    }

    static const state_node *walk(const state_node* state, const char_type& character)
    {
        if (state != nullptr) {
            return state->walk(character);
        }
        return nullptr;
    }
};

}

template <typename _Policy, typename _Alloc = std::allocator<typename _Policy::char_type> >
class basic_state_machine {
public:
    using policy       = _Policy;
    using char_type    = typename policy::char_type;
    using data_type    = typename policy::data_type;
    using state_node   = typename policy::state_node;
	using alloc_type   = typename std::allocator_traits<_Alloc>::template rebind_alloc<state_node>;
	using alloc_traits = std::allocator_traits<alloc_type>;

    basic_state_machine() :
        _first_state(),
        _state_count(0)
    {
    }

    ~basic_state_machine()
    {
        policy::clear(*this, this->first_state());
    }

    state_node* transit(
        state_node* state,
        const char_type& character,
        const data_type& data,
        bool terminal)
    {
        return policy::transit(*this, state, character, data, terminal);
    }

    state_node* transit(state_node* state, const char_type& character, state_node* next)
    {
        return policy::transit(*this, state, character, next);
    }

    state_node* walk(state_node* state, const char_type& character) const
    {
        return policy::walk(state, character);
    }

    const state_node* walk(const state_node* state, const char_type& character) const
    {
        return policy::walk(state, character);
    }

    bool remove(state_node* state)
    {
        return remove(first_state(), state);
    }

    bool remove(state_node* from, state_node *state)
    {
        return policy::remove(*this, from, state);
    }

    //fixme merge后的内存处理
	bool merge(const basic_state_machine& other)
	{
		return merge(this->first_state(), other.first_state());
	}

#if 0
    bool merge(basic_state_machine&& other)
    {
        bool retult = merge(this->first_state(), other.first_state());
        other._first_state = {}; //不再持有节点
        return retult;
    }
#endif

	bool merge(state_node* state, const state_node* other)
	{
        if (state == nullptr || other == nullptr) {
            return true;
        }

        return policy::merge(*this, state, other);
	}

    std::size_t next_state()
    {
        return _state_count++;
    }

    state_node* first_state()
    {
        return &_first_state;
    }

    const state_node* first_state() const
    {
        return &_first_state;
    }

    alloc_type& get_allocator() noexcept
    {
        return _allocator;
    }

    bool is_terminal(const state_node* state) const noexcept
    {
        if (state == nullptr) {
            return false;
        }
        return state->terminal;
    }

    //todo 
//    bool has(const state_node* state)
//    {
//
//    }

private:
    state_node  _first_state;
    std::size_t _state_count;
    alloc_type  _allocator;
};

}
}
}

#endif //__STAR_SDK_UTILS_DETAIL_BASIC_STATE_MACHINE_H
