#ifndef UTILS
#define UTILS

#include <ostream>
#include <array>
#include <iterator>
#include <iomanip>

template <class T, std::size_t N>
std::ostream &operator<<(std::ostream &o, const std::array<T, N> &arr)
{
    std::cout << std::setprecision(3); // TODO make configurable
    std::copy(arr.cbegin(), arr.cend(), std::ostream_iterator<T>(o, " "));
    return o;
}

#endif