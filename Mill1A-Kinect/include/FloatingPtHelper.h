#ifndef _FLOATPOINT_H
#define _FLOATPOINT_H

#include <cmath>
#include <limits>

#include <type_traits>
#include <algorithm>

template<typename T>

bool fequal(T a, T b, int N) {
    return std::fabs(a - b) < N*std::numeric_limits<T>::epsilon();
}

#endif