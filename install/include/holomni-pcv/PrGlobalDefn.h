/***************************************************************
 * PrGlobalDefn.h
 *
 * This files contains global constants and typedef's.
 *
 ****************************************************************/

/*
 * modification history
 *----------------------
 *
 * 10/26/97: K.C. Chang: created.
 */

#ifndef _PrGlobalDefn_h_
#define _PrGlobalDefn_h_

//#define NDEBUG  // no (complie) assert() for <assert.h>
#include <assert.h>

#include <stdio.h>

#ifndef NULL
#define NULL 0
#endif

//typedef enum { FALSE, TRUE } bool;
//typedef int bool;
//#define TRUE 1
//#define FALSE 0
//#define TRUE  true
//#define FALSE  false


#include <math.h>

#ifndef M_PI
#define M_PI        3.14159265358979323846  /* pi */
#endif
#ifndef M_PI_2
#define M_PI_2      1.57079632679489661923  /* pi/2 */
#endif
#ifndef M_PI_4
#define M_PI_4      0.78539816339744830962  /* pi/4 */
#endif
#ifndef HYPOT
#define HYPOT(x,y) (sqrt(((x)*(x))+((y)*(y)))) /* Hypotenuse */
#endif

#define PR_THOUSAND        1000
#define PR_MILLION         (PR_THOUSAND*PR_THOUSAND)
#define PR_BILLION         (PR_MILLION*PR_THOUSAND)

#define PR_LB_TO_KG        (0.45359237) //=(1.0/2.2046226)
#define PR_INCH_TO_METER   (0.0254)
#define PR_DEGREE_TO_RAD   (M_PI/180.0)
#define PR_RAD_TO_DEGREE   (1.0/PR_DEGREE_TO_RAD)

#define PR_GRAVITY_CONSTANT (9.81)

#define PR_BITS_PER_BYTE  8
#define PR_BYTES_PER_WORD (sizeof(unsigned int))

#define PR_DOUBLE_PRECISION // double precision
#ifdef PR_DOUBLE_PRECISION
typedef double Float;
#else // PR_DOUBLE_PRECISION
typedef float Float;
#endif // PR_DOUBLE_PRECISION

typedef void (*VoidFuncPtr)(void *arg);
typedef void (*VoidNoArgFuncPtr)();

#define PR_EMPTY_ID      (-1)
#define PR_XRBASE_ID     7

// use with setprio() and getprio()
#define PR_PRIORITY_MIN   1
#define PR_PRIORITY_LOW  10  // default
#define PR_PRIORITY_MID  15
#define PR_PRIORITY_HIGH 19  // highest for non-super user
#define PR_PRIORITY_MAX  29  // highest for super user

#endif // _PrGlobalDefn_h_
