
#ifndef __UTILS_H
#define __UTILS_H
#ifdef __cplusplus
extern "C"
{
#endif
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "perf_counter.h"

    int arm_cos_f32(int pahse);
    int arm_sin_f32(int pahse);

#define PARK_USE_INT32

#ifdef PARK_USE_FLOAT32
#define DateType float
#elif defined(PARK_USE_FLOAT64)
#define DateType double
#elif defined(PARK_USE_INT32)
#define DateType int32_t
#else
#define DateType float
#endif

/* 为1的话是虚假的三电流输入，本质上还是两电流，为0的话是不用电流相加为0这个公式来计算clark */
#define CLARK_3_current 1

    /*-----------------------------------------------------------------------------
        CLARKE PARK IPARK 变换 结构体
    -----------------------------------------------------------------------------*/
    typedef struct
    {
        /*                     clark      range      park         ipark             desc                         */
        DateType As;     //   << Input             ---------    ---------        phase-a stator variable
        DateType Bs;     //   << Input             ---------    ---------        phase-b stator variable
        DateType Cs;     //   << Input             ---------    ---------        phase-c stator variable
        DateType Alpha;  //   >> Output   0-1000   << Input     >> Output        stationary d-axis stator variable
        DateType Beta;   //   >> Output   0-1000   << Input     >> Output        stationary q-axis stator variable
        DateType Angle;  //   ---------            << Input     << Input         rotating angle (pu)
        DateType Ds;     //   ---------            >> Output    << Input         rotating d-axis stator variable
        DateType Qs;     //   ---------            >> Output    << Input         rotating q-axis stator variable
        DateType Sine;   //  ---------            << Input     << Input         Sine term
        DateType Cosine; //  ---------            << Input     << Input         Cosine term
    } svpwm_t;

/*-----------------------------------------------------------------------------
    CLARKE 变换 宏
-----------------------------------------------------------------------------*/
#define CLARKE_DEFAULTS \
    {0, 0, 0, 0, 0}

#if (defined(PARK_USE_FLOAT32) || defined(PARK_USE_FLOAT64))
#define CLARK_ONEbySQRT3 0.57735026918963f /* 1/sqrt(3) */
#define CLARK_ONEbyTHREE 0.33333333333333f /* 1/3 */
#define GAIN
#elif defined(PARK_USE_INT32)
#define CLARK_ONEbySQRT3 577 /* 1/sqrt(3) 0.5773 */
#define CLARK_ONEbyTHREE 333 /* 1/3 */
#define GAIN / 1000
#endif

#if (defined(PARK_USE_FLOAT32) || defined(PARK_USE_FLOAT64))
    static inline DateType fast_sin(DateType x)
    {
        return sin(x);
    }
    static inline DateType fast_cos(DateType x)
    {
        return cos(x);
    }
#elif defined(PARK_USE_INT32)
static inline DateType fast_sin(DateType x)
{
    return arm_sin_f32(x);
}
static inline DateType fast_cos(DateType x)
{
    return arm_cos_f32(x);
}
#endif
    /*------------------------------------------------------------------------------
        CLARKE 变换 变量
    ------------------------------------------------------------------------------*/
    static const DateType _onebysqrt3 = (CLARK_ONEbySQRT3);
    static const DateType _onebythree = (CLARK_ONEbyTHREE);

    /*------------------------------------------------------------------------------
        CLARKE 变换 函数实体
    ------------------------------------------------------------------------------*/
    /**
     * @brief       clarke变换
     * @param[in]   *v: svpwm_t结构体
     * @param[in]   _A: ABC坐标系下的电流a
     * @param[in]   _B: ABC坐标系下的电流b
     * @param[out]  v->Alpha: Alpha Beta坐标系下的电流Alpha
     *              v->Beta: Alpha Beta坐标系下的电流Beta
     * @retval      none
     * @attention   输入两电流
     */
    static inline void _clarke_calc_2(svpwm_t *v)
    {
        // v->As = _A;
        // v->Bs = _B;

        v->Alpha = v->As;
        v->Beta = ((v->As + (v->Bs * 2)) * _onebysqrt3) GAIN;
    }
    static inline void clarke_calc_2(svpwm_t *v, DateType _A, DateType _B)
    {
        v->As = _A;
        v->Bs = _B;

        _clarke_calc_2(v);
    }

    /**
     * @brief       clarke变换
     * @param[in]   *v: svpwm_t结构体
     * @param[in]   _A: ABC坐标系下的电流a
     * @param[in]   _B: ABC坐标系下的电流b
     * @param[in]   _C: ABC坐标系下的电流c
     * @param[out]  v->Alpha: Alpha Beta坐标系下的电流Alpha
     *              v->Beta: Alpha Beta坐标系下的电流Beta
     * @retval      none
     * @attention   输入三电流
     */
    static inline void _clarke_calc_3(svpwm_t *v)
    {
        // v->As = _A;
        // v->Bs = _B;
        // v->Cs = _C;

#if (CLARK_3_current == 1)
        v->Alpha = v->As;
        v->Beta = ((v->Bs - v->Cs) * _onebysqrt3) GAIN;
#elif (CLARK_3_current == 0)
    v->Alpha = ((2 * v->As - v->Bs - v->Cs) * _onebythree) GAIN;
    v->Beta = ((v->Bs - v->Cs) * _onebysqrt3) GAIN;
#endif
    }
    static inline void clarke_calc_3(svpwm_t *v, DateType _A, DateType _B, DateType _C)
    {
        v->As = _A;
        v->Bs = _B;
        v->Cs = _C;

        _clarke_calc_3(v);
    }

/*-----------------------------------------------------------------------------
    PARK 变换 宏
-----------------------------------------------------------------------------*/
#define PARK_DEFAULTS \
    {                 \
        0,            \
        0,            \
        0,            \
        0,            \
        0,            \
        0,            \
        0,            \
    }

    /*------------------------------------------------------------------------------
        PARK 变换 函数实体
    ------------------------------------------------------------------------------*/
    /**
     * @brief       park变换
     * @param[in]   *v: svpwm_t结构体
     * @param[in]   _Alpha: Alpha Beta坐标系下的电流Alpha
     * @param[in]   _Beta: Alpha Beta坐标系下的电流Beta
     * @param[out]  v->Ds: dq坐标系下的电流d
     *              v->Qs: dq坐标系下的电流q
     * @retval      none
     */
    static inline void _park_calc(svpwm_t *v)
    {

        // v->Alpha = _Alpha;
        // v->Beta = _Beta;
        // v->Angle = _Angle;

        v->Ds = ((v->Alpha * v->Cosine) + (v->Beta * v->Sine)) / 16384;
        v->Qs = ((v->Beta * v->Cosine) - (v->Alpha * v->Sine)) / 16384;
    }
    static inline void park_calc(svpwm_t *v, DateType _Alpha, DateType _Beta, DateType _Angle)
    {

        v->Alpha = _Alpha;
        v->Beta = _Beta;
        v->Angle = _Angle;
        v->Sine = fast_sin(_Angle);
        v->Cosine = fast_cos(_Angle);

        _park_calc(v);
    }

/*-----------------------------------------------------------------------------
    IPARK 变换 宏
-----------------------------------------------------------------------------*/
#define IPARK_DEFAULTS \
    {                  \
        0,             \
        0,             \
        0,             \
        0,             \
        0,             \
        0,             \
        0,             \
    }

    /*------------------------------------------------------------------------------
        IPARK 变换 函数实体
    ------------------------------------------------------------------------------*/
    /**
     * @brief       ipark变换
     * @param[in]   *v: isvpwm_t结构体
     * @param[in]   _Ds: dq坐标系下的电流d
     * @param[in]   _Qs: dq坐标系下的电流q
     * @param[out]  v->Alpha: Alpha Beta坐标系下的电流Alpha
     *              v->Beta: Alpha Beta坐标系下的电流Beta
     * @retval      none
     */
    static inline void _ipark_calc(svpwm_t *v)
    {

        // v->Ds = _Ds;
        // v->Qs = _Qs;
        // v->Angle = _Angle;

        v->Alpha = ((v->Ds * v->Cosine) - (v->Qs * v->Sine)) / 16384;
        v->Beta = ((v->Qs * v->Cosine) + (v->Ds * v->Sine)) / 16384;
    }
    static inline void ipark_calc(svpwm_t *v, DateType _Ds, DateType _Qs, DateType _Angle)
    {
        v->Ds = _Ds;
        v->Qs = _Qs;
        v->Angle = _Angle;
        v->Sine = fast_sin(v->Angle);
        v->Cosine = fast_cos(v->Angle);
        _ipark_calc(v);
    }

    // Compute rising edge timings (0.0 - 1.0) as a function of alpha-beta
    // as per the magnitude invariant clarke transform
    // The magnitude of the alpha-beta vector may not be larger than sqrt(3)/2
    // Returns 0 on success, and -1 if the input was out of range
    int SVM(int alpha, int beta, int *tA, int *tB, int *tC);

    // 获取全局系统时间
    int64_t get_time_us(void);
    int64_t get_time_ms(void);
#ifdef __cplusplus
}
#endif
#endif //__UTILS_H
