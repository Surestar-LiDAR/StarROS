/**
 * @author      : John 
 * @date        : 2017-03-20
 */
#ifndef __STAR_UTILS_STRING_TOKENIZER_H
#define __STAR_UTILS_STRING_TOKENIZER_H

namespace ss {
namespace utils {

template<typename _String>
struct string_traits;

template<template<typename X> class _String, typename _Char>
struct string_traits<_String<_Char> >
{
    typedef _Char char_type;
};

template<template<typename X, typename A> class _String, typename _Char, typename _Allocator>
struct string_traits<_String<_Char, _Allocator> >
{
    typedef _Char char_type;
};

template<template<typename X, typename T, typename A> class _String, typename _Char, typename _Traits, typename _Allocator>
struct string_traits<_String<_Char, _Traits, _Allocator> >
{
    typedef _Char char_type;
};

template<typename _Char, typename _Traits, typename _Allocator>
struct string_traits<std::basic_string<_Char, _Traits, _Allocator> >
{
    typedef _Char char_type;
};

template<>
struct string_traits<std::string>
{
    typedef char char_type;
};

template<>
struct string_traits<std::wstring>
{
    typedef wchar_t char_type;
};

/**
 * 字符串分词器
 * @tparam _String 字符串类型
 */
template<typename _String>
class string_tokenizer {
public:
    string_tokenizer(const _String& source, const _String& delimiters, bool containsDelimiters) :
        _offset(0),
        _source(source),
        _delimiters(delimiters),
        _containsDelimiters(containsDelimiters)
    {
    }

    /**
     * 是否有下一个
     * @return 有则返回true，否则返回false
     */
    bool hasNext() const
    {
        return _offset < _source.length();
    }

    /**
     * 获取下一个字符串
     * @return
     */
    _String next()
    {
        std::basic_stringbuf<typename string_traits<_String>::char_type> buffer;
        bool escape = false;
        for (; _offset < _source.length(); ++_offset) {
            if (!escape && _source.at(_offset) == '\\') {
                escape = true;
                buffer.sputc(_source.at(_offset));
                continue;
            } else if (escape) { //如果是转移字符，一下个字符直接保存
                escape = false;
                buffer.sputc(_source.at(_offset));
                continue;
            }

            if (_delimiters.find(_source.at(_offset)) != std::string::npos) { //是终结符
                if (buffer.in_avail()== 0 && _containsDelimiters) {
                    buffer.sputc(_source.at(_offset));
                    ++_offset;
                    break;
                } else {
                    if (!_containsDelimiters) {
                        ++_offset; //略去分割符
                    }
                    break;
                }
            }
            else {
                buffer.sputc(_source.at(_offset));
            }
        }

        return buffer.str();
    }

private:
    size_t _offset;
    _String _source;
    _String _delimiters;
    bool _containsDelimiters;
};

}
}

#endif //__STAR_UTILS_STRING_TOKENIZER_H
