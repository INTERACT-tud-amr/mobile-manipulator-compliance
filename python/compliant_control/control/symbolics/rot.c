/* This file was automatically generated by CasADi 3.6.7.
 *  It consists of: 
 *   1) content generated by CasADi runtime: not copyrighted
 *   2) template code copied from CasADi source: permissively licensed (MIT-0)
 *   3) user code: owned by the user
 *
 */
#ifdef __cplusplus
extern "C" {
#endif

/* How to prefix internal symbols */
#ifdef CASADI_CODEGEN_PREFIX
  #define CASADI_NAMESPACE_CONCAT(NS, ID) _CASADI_NAMESPACE_CONCAT(NS, ID)
  #define _CASADI_NAMESPACE_CONCAT(NS, ID) NS ## ID
  #define CASADI_PREFIX(ID) CASADI_NAMESPACE_CONCAT(CODEGEN_PREFIX, ID)
#else
  #define CASADI_PREFIX(ID) rot_ ## ID
#endif

#include <math.h>

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int long long int
#endif

/* Add prefix to internal symbols */
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)

/* Symbol visibility in DLLs */
#ifndef CASADI_SYMBOL_EXPORT
  #if defined(_WIN32) || defined(__WIN32__) || defined(__CYGWIN__)
    #if defined(STATIC_LINKED)
      #define CASADI_SYMBOL_EXPORT
    #else
      #define CASADI_SYMBOL_EXPORT __declspec(dllexport)
    #endif
  #elif defined(__GNUC__) && defined(GCC_HASCLASSVISIBILITY)
    #define CASADI_SYMBOL_EXPORT __attribute__ ((visibility ("default")))
  #else
    #define CASADI_SYMBOL_EXPORT
  #endif
#endif

static const casadi_int casadi_s0[10] = {6, 1, 0, 6, 0, 1, 2, 3, 4, 5};
static const casadi_int casadi_s1[15] = {3, 3, 0, 3, 6, 9, 0, 1, 2, 0, 1, 2, 0, 1, 2};

/* rot:(i0[6])->(o0[3x3]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a3, a30, a31, a32, a33, a34, a35, a36, a37, a38, a4, a5, a6, a7, a8, a9;
  a0=arg[0]? arg[0][0] : 0;
  a1=cos(a0);
  a2=arg[0]? arg[0][1] : 0;
  a3=cos(a2);
  a4=(a1*a3);
  a0=sin(a0);
  a5=-3.6732051036381108e-06;
  a2=sin(a2);
  a6=(a5*a2);
  a7=(a0*a6);
  a4=(a4-a7);
  a7=arg[0]? arg[0][2] : 0;
  a8=cos(a7);
  a9=(a4*a8);
  a10=(a1*a2);
  a11=(a5*a3);
  a12=(a0*a11);
  a10=(a10+a12);
  a12=-9.9999999997301536e-01;
  a7=sin(a7);
  a13=(a12*a7);
  a14=(a10*a13);
  a15=-9.9999999999325395e-01;
  a16=(a15*a0);
  a17=7.3464102066435888e-06;
  a18=(a17*a7);
  a19=(a16*a18);
  a14=(a14+a19);
  a9=(a9-a14);
  a14=arg[0]? arg[0][3] : 0;
  a19=cos(a14);
  a20=(a9*a19);
  a4=(a4*a7);
  a21=(a12*a8);
  a22=(a10*a21);
  a17=(a17*a8);
  a23=(a16*a17);
  a22=(a22+a23);
  a4=(a4+a22);
  a14=sin(a14);
  a22=(a5*a14);
  a23=(a4*a22);
  a24=-7.3464102066435888e-06;
  a10=(a24*a10);
  a16=(a12*a16);
  a10=(a10+a16);
  a16=9.9999999999325395e-01;
  a25=(a16*a14);
  a26=(a10*a25);
  a23=(a23+a26);
  a20=(a20-a23);
  a23=arg[0]? arg[0][4] : 0;
  a26=cos(a23);
  a27=(a5*a26);
  a28=(a20*a27);
  a9=(a9*a14);
  a29=(a5*a19);
  a30=(a4*a29);
  a31=(a16*a19);
  a32=(a10*a31);
  a30=(a30+a32);
  a9=(a9+a30);
  a23=sin(a23);
  a30=(a9*a23);
  a4=(a15*a4);
  a10=(a5*a10);
  a4=(a4+a10);
  a10=(a15*a26);
  a32=(a4*a10);
  a30=(a30+a32);
  a28=(a28-a30);
  a30=arg[0]? arg[0][5] : 0;
  a32=cos(a30);
  a33=(a5*a32);
  a34=(a28*a33);
  a35=(a15*a23);
  a36=(a4*a35);
  a9=(a9*a26);
  a36=(a36-a9);
  a9=(a5*a23);
  a37=(a20*a9);
  a36=(a36-a37);
  a30=sin(a30);
  a37=(a36*a30);
  a20=(a16*a20);
  a4=(a5*a4);
  a20=(a20-a4);
  a4=(a16*a32);
  a38=(a20*a4);
  a37=(a37+a38);
  a34=(a34+a37);
  if (res[0]!=0) res[0][0]=a34;
  a34=(a0*a3);
  a6=(a1*a6);
  a34=(a34+a6);
  a6=(a34*a8);
  a11=(a1*a11);
  a0=(a0*a2);
  a11=(a11-a0);
  a0=(a11*a13);
  a1=(a15*a1);
  a37=(a1*a18);
  a0=(a0+a37);
  a6=(a6+a0);
  a0=(a6*a19);
  a37=(a11*a21);
  a38=(a1*a17);
  a37=(a37+a38);
  a34=(a34*a7);
  a37=(a37-a34);
  a34=(a37*a22);
  a11=(a24*a11);
  a12=(a12*a1);
  a11=(a11+a12);
  a12=(a11*a25);
  a34=(a34+a12);
  a0=(a0+a34);
  a34=(a0*a27);
  a12=(a37*a29);
  a1=(a11*a31);
  a12=(a12+a1);
  a6=(a6*a14);
  a12=(a12-a6);
  a6=(a12*a23);
  a37=(a15*a37);
  a11=(a5*a11);
  a37=(a37+a11);
  a11=(a37*a10);
  a6=(a6+a11);
  a34=(a34+a6);
  a6=(a34*a33);
  a12=(a12*a26);
  a11=(a37*a35);
  a12=(a12-a11);
  a11=(a0*a9);
  a12=(a12-a11);
  a11=(a12*a30);
  a0=(a16*a0);
  a37=(a5*a37);
  a0=(a0+a37);
  a37=(a0*a4);
  a11=(a11+a37);
  a6=(a6+a11);
  if (res[0]!=0) res[0][1]=a6;
  a2=(a16*a2);
  a8=(a2*a8);
  a3=(a16*a3);
  a13=(a3*a13);
  a18=(a5*a18);
  a13=(a13+a18);
  a8=(a8+a13);
  a19=(a8*a19);
  a21=(a3*a21);
  a17=(a5*a17);
  a21=(a21+a17);
  a2=(a2*a7);
  a21=(a21-a2);
  a22=(a21*a22);
  a24=(a24*a3);
  a3=3.6732051035389906e-06;
  a24=(a24+a3);
  a25=(a24*a25);
  a22=(a22+a25);
  a19=(a19+a22);
  a27=(a19*a27);
  a29=(a21*a29);
  a31=(a24*a31);
  a29=(a29+a31);
  a8=(a8*a14);
  a29=(a29-a8);
  a23=(a29*a23);
  a21=(a15*a21);
  a24=(a5*a24);
  a21=(a21+a24);
  a10=(a21*a10);
  a23=(a23+a10);
  a27=(a27+a23);
  a33=(a27*a33);
  a29=(a29*a26);
  a35=(a21*a35);
  a29=(a29-a35);
  a9=(a19*a9);
  a29=(a29-a9);
  a9=(a29*a30);
  a19=(a16*a19);
  a21=(a5*a21);
  a19=(a19+a21);
  a4=(a19*a4);
  a9=(a9+a4);
  a33=(a33+a9);
  if (res[0]!=0) res[0][2]=a33;
  a36=(a36*a32);
  a16=(a16*a30);
  a33=(a20*a16);
  a36=(a36-a33);
  a30=(a5*a30);
  a33=(a28*a30);
  a36=(a36-a33);
  if (res[0]!=0) res[0][3]=a36;
  a12=(a12*a32);
  a36=(a0*a16);
  a12=(a12-a36);
  a36=(a34*a30);
  a12=(a12-a36);
  if (res[0]!=0) res[0][4]=a12;
  a29=(a29*a32);
  a16=(a19*a16);
  a29=(a29-a16);
  a30=(a27*a30);
  a29=(a29-a30);
  if (res[0]!=0) res[0][5]=a29;
  a28=(a15*a28);
  a20=(a5*a20);
  a28=(a28+a20);
  if (res[0]!=0) res[0][6]=a28;
  a34=(a15*a34);
  a0=(a5*a0);
  a34=(a34+a0);
  if (res[0]!=0) res[0][7]=a34;
  a15=(a15*a27);
  a5=(a5*a19);
  a15=(a15+a5);
  if (res[0]!=0) res[0][8]=a15;
  return 0;
}

CASADI_SYMBOL_EXPORT int rot(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int rot_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int rot_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void rot_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int rot_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void rot_release(int mem) {
}

CASADI_SYMBOL_EXPORT void rot_incref(void) {
}

CASADI_SYMBOL_EXPORT void rot_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int rot_n_in(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_int rot_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real rot_default_in(casadi_int i) {
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* rot_name_in(casadi_int i) {
  switch (i) {
    case 0: return "i0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* rot_name_out(casadi_int i) {
  switch (i) {
    case 0: return "o0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* rot_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* rot_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s1;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int rot_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 1;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}

CASADI_SYMBOL_EXPORT int rot_work_bytes(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 1*sizeof(const casadi_real*);
  if (sz_res) *sz_res = 1*sizeof(casadi_real*);
  if (sz_iw) *sz_iw = 0*sizeof(casadi_int);
  if (sz_w) *sz_w = 0*sizeof(casadi_real);
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
