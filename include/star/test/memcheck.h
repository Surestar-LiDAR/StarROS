/**
 * Copyright (c) 2018 QuantumCTek. All rights reserved.
 * @author      : John
 * @date        : 2019-5-21
 * @version     : 1.0.0
 */

#ifndef __NOISE_TEST_EXT_MEMCHECK_H
#define __NOISE_TEST_EXT_MEMCHECK_H

struct memcheck_counter {
    memcheck_counter()
    {
        ++malloc_count;
        ++ref_count;
    }

    memcheck_counter(const memcheck_counter &)
    {
        ++malloc_count;
        ++ref_count;
    }

    ~memcheck_counter()
    {
        ++free_count;
        --ref_count;
    }
    static std::size_t malloc_count;
    static std::size_t free_count;
    static std::size_t ref_count;
};

std::size_t memcheck_counter::malloc_count = 0;
std::size_t memcheck_counter::free_count = 0;
std::size_t memcheck_counter::ref_count = 0;

#endif //__NOISE_TEST_EXT_MEMCHECK_H
