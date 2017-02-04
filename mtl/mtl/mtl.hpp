#ifndef MTL_HPP
#define MTL_HPP
#include <boost/cstdint.hpp>
#include <ctime>

#ifdef _MSC_VER
#ifdef MTL_EXPORT
    #define _MTL_EXPORT __declspec(dllexport)
#else
    #define _MTL_EXPORT __declspec(dllimport)
#endif
#else
#define _MTL_EXPORT
#endif

#ifdef _MSC_VER
#pragma warning(disable: 4251)
#pragma warning(disable: 4661)
#pragma warning(disable: 4998)
#pragma warning(disable: 4996)
#endif

#endif // MTL_HPP
