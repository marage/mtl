#ifndef MTL_HPP
#define MTL_HPP
#include <chrono>
#include <boost/cstdint.hpp>

#ifdef MTL_SHARED_LIBRARY
#ifdef MTL_LIBRARY
    #define MTL_EXPORT __declspec(dllexport)
#else
    #define MTL_EXPORT __declspec(dllimport)
#endif
#else
#define MTL_EXPORT
#endif

#ifdef _MSC_VER
#pragma warning(disable: 4251)
#pragma warning(disable: 4661)
#pragma warning(disable: 4998)
#pragma warning(disable: 4996)
#endif

#endif // MTL_HPP
