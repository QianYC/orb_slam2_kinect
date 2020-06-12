//
// Created by yc_qian on 19-12-20.
//

#ifndef ORB_SLAM2_KINETIC_MACROS_H
#define ORB_SLAM2_KINETIC_MACROS_H

#ifndef DEG2RAD
#define DEG2RAD(x) ((x) * 0.01745329251994329575)
#endif

#ifndef RAD2DEG
#define RAD2DEG(x) ((x) * 57.29577951308232087721)
#endif

// GCC the one and only
#if defined(__GNUC__)
#  define G2O_ATTRIBUTE_CONSTRUCTOR(func) \
     static void func(void)__attribute__ ((constructor)); \
     static void func(void)

#  define G2O_ATTRIBUTE_UNUSED __attribute__((unused))
#  define G2O_ATTRIBUTE_FORMAT12 __attribute__ ((format (printf, 1, 2)))
#  define G2O_ATTRIBUTE_FORMAT23 __attribute__ ((format (printf, 2, 3)))
#  define G2O_ATTRIBUTE_WARNING(func) func __attribute__((warning))
#  define G2O_ATTRIBUTE_DEPRECATED(func) func __attribute__((deprecated))

#ifdef ANDROID
# define g2o_isnan(x)     isnan(x)
# define g2o_isinf(x)     isinf(x)
# define g2o_isfinite(x)  isfinite(x)
#else
# define g2o_isnan(x)     std::isnan(x)
# define g2o_isinf(x)     std::isinf(x)
# define g2o_isfinite(x)  std::isfinite(x)
#endif

// MSVC on Windows
#elif defined _MSC_VER
#  define __PRETTY_FUNCTION__ __FUNCTION__

/**
Modified by Mark Pupilli from:

	"Initializer/finalizer sample for MSVC and GCC.
    2010 Joe Lowe. Released into the public domain."

	"For MSVC, places a ptr to the function in the user initializer section (.CRT$XCU), basically the same thing the compiler does for the constructor calls for static C++ objects. For GCC, uses a constructor attribute."

	(As posted on Stack OVerflow)
*/
#  define G2O_ATTRIBUTE_CONSTRUCTOR(f) \
     __pragma(section(".CRT$XCU",read)) \
     static void __cdecl f(void); \
     __declspec(allocate(".CRT$XCU")) void (__cdecl*f##_)(void) = f; \
     static void __cdecl f(void)

#  define G2O_ATTRIBUTE_UNUSED
#  define G2O_ATTRIBUTE_FORMAT12
#  define G2O_ATTRIBUTE_FORMAT23
#  define G2O_ATTRIBUTE_WARNING(func) func
#  define G2O_ATTRIBUTE_DEPRECATED(func) func

#include <float.h>

# define g2o_isnan(x)    _isnan(x)
# define g2o_isinf(x)    (_finite(x) == 0)
# define g2o_isfinite(x) (_finite(x) != 0)

// unknown compiler
#else
#  ifndef __PRETTY_FUNCTION__
#    define __PRETTY_FUNCTION__ ""
#  endif
#  define G2O_ATTRIBUTE_CONSTRUCTOR(func) func
#  define G2O_ATTRIBUTE_UNUSED
#  define G2O_ATTRIBUTE_FORMAT12
#  define G2O_ATTRIBUTE_FORMAT23
#  define G2O_ATTRIBUTE_WARNING(func) func
#  define G2O_ATTRIBUTE_DEPRECATED(func) func

#include <math.h>
#define g2o_isnan(x)    isnan(x)
#define g2o_isinf(x)    isinf(x)
#define g2o_isfinite(x) isfinite(x)

#endif

// some macros that are only useful for c++
#ifdef __cplusplus

#define G2O_FSKIP_LINE(f) \
   {char c=' ';while(c != '\n' && f.good() && !(f).eof()) (f).get(c);}

#ifndef PVAR
#define PVAR(s) \
    #s << " = " << (s) << std::flush
#endif

#ifndef PVARA
#define PVARA(s) \
  #s << " = " << RAD2DEG(s) << "deg" << std::flush
#endif

#ifndef FIXED
#define FIXED(s) \
  std::fixed << s << std::resetiosflags(std::ios_base::fixed)
#endif

#endif // __cplusplus

#endif //ORB_SLAM2_KINETIC_MACROS_H
