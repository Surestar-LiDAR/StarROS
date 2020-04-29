/**
 * @author      : John 
 * @date        : 2017-03-23
 */
#ifndef __STAR_SDK_FILE_SYSTEM_H
#define __STAR_SDK_FILE_SYSTEM_H

#include <star/Star.h>

#include <string>

#if defined(_WIN32)
#include <io.h>
#else
#include <sys/types.h>
#include <dirent.h>
struct dirent;
#endif

namespace ss {

/**
 * 文件系统基本操作，提供一组静态函数对文件和文件夹进行操作
 */
class __star_export FileSystem {
public:
    /**
     * 获取父级目录
     * @param path 当前路径
     * @return 返回path的父级目录，如果path是文件，返回的是当前文件所在的文件夹路径，如果path是文件夹，返回的是上一层文件夹路径
     */
    static std::string parent(const std::string& path);

    /**
     * 判断指定的路径是否存在
     * @param path 指定的路径，可以是文件也可以是文件夹
     * @return 存在则返回true，否则返回false
     */
    static bool exists(const std::string& path);

    /**
     * 创建目录
     * @param path 需要创建目录的路径
     * @param mode 创建目录指定的权限，等同于Linux系统中chmod的权限，该参数在Windows下忽略
     * @return 创建成功则返回true，失败则返回false
     * @note mkdir并不会创建父级目录，如果父级目录不存在则会创建失败
     */
    static bool mkdir(const std::string& path, unsigned int mode = 0777);

    /**
     * 类似于mkdir()，但当父级目录不存在时，会尝试着递归创建整个路径，以保证目录常见成功
     */
    static bool mkpath(const std::string& path, unsigned int mode = 0777);

    /**
     * 删除文件夹
     * @param path 要删除文件夹的路径
     * @param recursion 决定是否递归删除文件夹中的内容，当该参数为false时，只能删除空文件夹如果文件夹不为空则删除失败，
     *        为true时，如果文件夹不为空，则会连同文件夹下的所有内容一起删除
     * @return 删除成功则返回true，否则返回false
     */
    static bool rmdir(const std::string& path, bool recursion = false);

    /**
     * 删除文件
     * @param path 要删除文件的路径
     * @return 删除成功则返回true，否则返回false
     */
    static bool unlink(const std::string& path);

    /**
     * 判断路径是否是绝对路径
     * @param path
     * @return
     */
    static bool is_absolute(const std::string& path);

    /**
     * 获取系统的分隔符
     * @return Windows下返回'\'，Linux下返回'/'
     */
    static char separator();
};

namespace fs {

class __star_export File {
public:
    class Iterator {
    public:
        explicit Iterator(const File& file);
        ~Iterator();
        /**
         * 遍历文件夹时获取下一个文件或者文件夹
         * @return 如果获取成功返回true，获取失败返回false，失败表示目录下已经没有更多的文件或者文件夹了
         */
        bool next();
        /**
         * 遍历文件夹时获取当前遍历到的文件或者文件夹
         * @return 当前的文件或者文件夹对应的File对象
         */
        File get() const;

    protected:
        void open(const std::string& path, const std::string& filter);

    private:
        std::string         _path;
        std::string         _filter;
#if defined(_WIN32)
        intptr_t            _handle;
        _finddatai64_t      _data;
#else
        DIR                 *_handle;
        struct dirent       *_data;
#endif
    };
    explicit File(const std::string &path);
#if defined(_WIN32)
    explicit File(const std::string &dir, const _finddatai64_t &data);
#else
    explicit File(const std::string &dir, const struct dirent *data);
#endif

    /**
     * 获取当前路径
     * @return 返回当前路径
     */
    const std::string &path() const;

    /**
     * 获取文件名
     * @return 当前路径为文件时返回文件名，否则返回空串
     */
    std::string fileName() const;

    /**
     * 获取文件名，不包含扩展名
     * @return 返回不包含扩展名和路径的文件名
     */
    std::string fileNameOnly() const;

    /**
     * 获取上一级目录
     * @return 上一级目录的路径
     */
    std::string parent() const;

    /**
     * 判断当前路径是否为文件夹
     * @return 是文件夹则返回true，否则返回false
     * @note 如果文件不存在也会返回false
     */
    bool isDir() const;

    /**
     * 判断当前路径是否为文件
     * @return 是文件则返回true，否则返回false
     * @note 如果文件不存在也会返回false
     */
    bool isFile() const;

    /**
     * 获取文件的创建的时间
     * @return 创建文件的epoch时间，单位毫秒
     */
    int64_t timeCreate() const;

    /**
     * 获取文件最后一次访问的时间
     * @return 最后一次访问的epoch时间，单位毫秒
     */
    int64_t timeAccess() const;

    /**
     * 获取文件最后一次的修改时间
     * @return 最后一次修改的epoch时间，单位毫秒
     */
    int64_t timeWrite() const;

    /**
     * 获取文件的大小
     * @return 文件的大小，该大小为文件存储在磁盘上的大小，并不是文件实际的大小。
     */
    uint64_t size() const;

    /**
     * 获取文件扩展名
     * @return 文件的扩展名
     */
    std::string suffix() const;

    /**
     * 判断文件是否存在
     * @return 存在则返回true，否则返回false
     */
    bool exists() const;

    /**
     * 删除文件，如果是文件夹则默认删除空文件夹
     * @return 删除成功则返回true，否则返回false
     */
    bool remove();

    /**
     * 删除文件
     * @return 删除成功则返回true，失败返回false
     * @note 如果当前不是文件（为文件夹）时，则无法删除，返回失败
     */
    bool unlink();

    /**
     * 同@Dir::rmdir(recursion)
     */
    bool rmdir(bool recursion = false);

    Iterator iterator() const;

protected:
    /**
     * 加载文件属性
     */
    void loadAttributes();

private:
    std::string     _path;
    int64_t         _timeCreate;
    int64_t         _timeAccess;
    int64_t         _timeWrite;
    int64_t         _size;
    unsigned long   _attributes;
};
}

}

#endif //__STAR_SDK_FILE_SYSTEM_H
