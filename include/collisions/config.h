
#ifndef ODE_CONFIG_H
#define ODE_CONFIG_H

/* #cmakedefine HAVE_ALLOCA 1 */
/* #cmakedefine HAVE_ATAN2F 1 */
/* #cmakedefine HAVE_COPYSIGN 1 */
/* #cmakedefine HAVE_COPYSIGNF 1 */
/* #cmakedefine HAVE_COSF 1 */
/* #cmakedefine HAVE_FABSF 1 */
/* #cmakedefine HAVE_FLOOR 1 */
/* #cmakedefine HAVE_FMODF 1 */
/* #cmakedefine HAVE_GETTIMEOFDAY 1 */
/* #cmakedefine HAVE_ISNAN 1 */
/* #cmakedefine HAVE_ISNANF 1 */
/* #cmakedefine HAVE_MALLOC 1 */
/* #cmakedefine HAVE_MEMMOVE 1 */
/* #cmakedefine HAVE_MEMSET 1 */
/* #cmakedefine HAVE_REALLOC 1 */
/* #cmakedefine HAVE_SELECT 1 */
/* #cmakedefine HAVE_SINF 1 */
/* #cmakedefine HAVE_SPRINTF 1 */
/* #cmakedefine HAVE_SQRT 1 */
/* #cmakedefine HAVE_SQRTF 1 */
/* #cmakedefine HAVE_VPRINTF 1 */
/* #cmakedefine HAVE_VSNPRINTF 1 */
/* #cmakedefine HAVE_SNPRINTF 1 */

/* #cmakedefine HAVE_ALLOCA_H 1 */
/* #cmakedefine HAVE_BOOST_SHARED_ARRAY_HPP 1 */
/* #cmakedefine HAVE_FLOAT_H 1 */
/* #cmakedefine HAVE_DLFCN_H 1 */
/* #cmakedefine HAVE_INTTYPES_H 1 */
/* #cmakedefine HAVE_MALLOC_H 1 */
/* #cmakedefine HAVE_MATH_H 1 */
/* #cmakedefine HAVE_MEMORY_H 1 */
/* #cmakedefine HAVE_STDARG_H 1 */
/* #cmakedefine HAVE_STDBOOL_H 1 */
/* #cmakedefine HAVE_STDINT_H 1 */
/* #cmakedefine HAVE_STDIO_H 1 */
/* #cmakedefine HAVE_STDLIB_H 1 */
/* #cmakedefine HAVE_STRINGS_H 1 */
/* #cmakedefine HAVE_STRING_H 1 */
/* #cmakedefine HAVE_SYS_SELECT_H 1 */
/* #cmakedefine HAVE_SYS_SOCKET_H 1 */
/* #cmakedefine HAVE_SYS_STAT_H 1 */
/* #cmakedefine HAVE_SYS_TIME_H 1 */
/* #cmakedefine HAVE_SYS_TYPES_H 1 */
/* #cmakedefine HAVE_TIME_H 1 */
/* #cmakedefine HAVE_UNISTD_H 1 */
/* #cmakedefine HAVE_LIMITS 1 */

/* #cmakedefine HAVE_LIBM 1 */
/* #cmakedefine HAVE_OBSTACK 1 */


#ifdef HAVE_ALLOCA_H
#include <alloca.h>
#endif
#ifdef HAVE_MALLOC_H
#include <malloc.h>
#endif
#ifdef HAVE_STDINT_H
#include <stdint.h>
#endif
#ifdef HAVE_INTTYPES_H
#include <inttypes.h>
#endif

/* an integer type that we can safely cast a pointer to and
 * from without loss of bits.
 */
typedef uintptr_t intP;

/* Define to the type of arg 1 for `select'. */
#define SELECT_TYPE_ARG1 int

/* Define to the type of args 2, 3 and 4 for `select'. */
#define SELECT_TYPE_ARG234 (fd_set *)

/* Define to the type of arg 5 for `select'. */
#define SELECT_TYPE_ARG5 (struct timeval *)


/* compiling for a X86_64 system on a gcc-based platform? */
#define X86_64_SYSTEM 1

/* libou namespace for ODE */
#define _OU_NAMESPACE gazebo_odeou

/* Atomic API of OU is enabled */
#define dATOMICS_ENABLED 1

/* Generic OU features are enabled */
#define dOU_ENABLED 1

/* Thread Local Storage API of OU is enabled */
#define dTLS_ENABLED 1

#define dDOUBLE 1
#define dTRIMESH_ENABLED 1
#define dTRIMESH_GIMPACT 0
#define dTRIMESH_OPCODE 1
#define __ODE__ 1
#define STD_HEADERS 1

#ifdef dSINGLE
       #define dEpsilon  FLT_EPSILON
#else
       #define dEpsilon  DBL_EPSILON
#endif

#endif
