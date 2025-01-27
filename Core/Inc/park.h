// /* Define to prevent recursive inclusion -------------------------------------*/
// #ifndef __PARK_H
// #define __PARK_H
// #ifdef __cplusplus
// extern "C" {
// #endif
// /* Includes ------------------------------------------------------------------*/
// #include <stdint.h>

// int arm_cos_f32(int pahse);
// int arm_sin_f32(int pahse);

// #define PARK_USE_INT32

// #ifdef PARK_USE_FLOAT32
// #define DateType float
// #elif defined(PARK_USE_FLOAT64)
// #define DateType double
// #elif defined(PARK_USE_INT32)
// #define DateType int32_t
// #else
// #define DateType float
// #endif

// /* 为1的话是虚假的三电流输入，本质上还是两电流，为0的话是不用电流相加为0这个公式来计算clark */
// #define CLARK_3_current 1



// /*-----------------------------------------------------------------------------
//     CLARKE PARK IPARK 变换 结构体
// -----------------------------------------------------------------------------*/
// typedef struct {
//     /*                     clark           park         ipark             desc                         */
//     DateType As;     //   << Input       ---------    ---------        phase-a stator variable
//     DateType Bs;     //   << Input       ---------    ---------        phase-b stator variable
//     DateType Cs;     //   << Input       ---------    ---------        phase-c stator variable
//     DateType Alpha;  //   >> Output      << Input     >> Output        stationary d-axis stator variable
//     DateType Beta;   //   >> Output      << Input     >> Output        stationary q-axis stator variable
//     DateType Angle;  //   ---------      << Input     << Input         rotating angle (pu)

//     DateType Ds;     //   ---------      >> Output    << Input         rotating d-axis stator variable
//     DateType Qs;     //   ---------      >> Output    << Input         rotating q-axis stator variable

//     DateType Sine;    //  ---------      << Input     << Input         Sine term
//     DateType Cosine;  //  ---------      << Input     << Input         Cosine term
// }svpwm_t;

// /*-----------------------------------------------------------------------------
//     CLARKE 变换 宏
// -----------------------------------------------------------------------------*/
// #define CLARKE_DEFAULTS \
//     { 0, 0, 0, 0, 0 }

// #if (defined(PARK_USE_FLOAT32)|| defined(PARK_USE_FLOAT64)) 
// #define CLARK_ONEbySQRT3 0.57735026918963f /* 1/sqrt(3) */
// #define CLARK_ONEbyTHREE 0.33333333333333f /* 1/3 */
// #define GAIN 
// #elif defined(PARK_USE_INT32)
// #define CLARK_ONEbySQRT3 577 /* 1/sqrt(3) 0.5773 */
// #define CLARK_ONEbyTHREE 333 /* 1/3 */
// #define GAIN /1000
// #endif

// #if (defined(PARK_USE_FLOAT32)|| defined(PARK_USE_FLOAT64))
// static inline DateType fast_sin(DateType x) {
//     return sin(x);
// }
// static inline DateType fast_cos(DateType x) {
//     return cos(x);
// }
// #elif defined(PARK_USE_INT32)
// static inline DateType fast_sin(DateType x) {
//     return arm_cos_f32(x);
// }
// static inline DateType fast_cos(DateType x) {
//     return arm_cos_f32(x);
// }
// #endif
// /*------------------------------------------------------------------------------
//     CLARKE 变换 变量
// ------------------------------------------------------------------------------*/
// static const DateType _onebysqrt3 = (CLARK_ONEbySQRT3);
// static const DateType _onebythree = (CLARK_ONEbyTHREE);

// /*------------------------------------------------------------------------------
//     CLARKE 变换 函数实体
// ------------------------------------------------------------------------------*/
// /**
//  * @brief       clarke变换
//  * @param[in]   *v: svpwm_t结构体
//  * @param[in]   _A: ABC坐标系下的电流a
//  * @param[in]   _B: ABC坐标系下的电流b
//  * @param[out]  v->Alpha: Alpha Beta坐标系下的电流Alpha
//  *              v->Beta: Alpha Beta坐标系下的电流Beta
//  * @retval      none
//  * @attention   输入两电流
//  */
// static inline void _clarke_calc_2(svpwm_t* v) {
//     // v->As = _A;
//     // v->Bs = _B;

//     v->Alpha = v->As;
//     v->Beta = ((v->As + (v->Bs * 2)) * _onebysqrt3)GAIN;
// }
// static inline void clarke_calc_2(svpwm_t* v, DateType _A, DateType _B) {
//     v->As = _A;
//     v->Bs = _B;

//     _clarke_calc_2(v);
// }

// /**
//  * @brief       clarke变换
//  * @param[in]   *v: svpwm_t结构体
//  * @param[in]   _A: ABC坐标系下的电流a
//  * @param[in]   _B: ABC坐标系下的电流b
//  * @param[in]   _C: ABC坐标系下的电流c
//  * @param[out]  v->Alpha: Alpha Beta坐标系下的电流Alpha
//  *              v->Beta: Alpha Beta坐标系下的电流Beta
//  * @retval      none
//  * @attention   输入三电流
//  */
// static inline void _clarke_calc_3(svpwm_t* v) {
//     // v->As = _A;
//     // v->Bs = _B;
//     // v->Cs = _C;

// #if (CLARK_3_current == 1)
//     v->Alpha = v->As;
//     v->Beta = ((v->Bs - v->Cs) * _onebysqrt3)GAIN;
// #elif (CLARK_3_current == 0)
//     v->Alpha = ((2 * v->As - v->Bs - v->Cs) * _onebythree)GAIN;
//     v->Beta = ((v->Bs - v->Cs) * _onebysqrt3)GAIN;
// #endif
// }
// static inline void clarke_calc_3(svpwm_t* v, DateType _A, DateType _B, DateType _C) {
//     v->As = _A;
//     v->Bs = _B;
//     v->Cs = _C;

//     _clarke_calc_3(v);
// }

// /*-----------------------------------------------------------------------------
//     PARK 变换 宏
// -----------------------------------------------------------------------------*/
// #define PARK_DEFAULTS \
//     { 0, 0, 0, 0, 0, 0, 0, }



// /*------------------------------------------------------------------------------
//     PARK 变换 函数实体
// ------------------------------------------------------------------------------*/
// /**
//  * @brief       park变换
//  * @param[in]   *v: svpwm_t结构体
//  * @param[in]   _Alpha: Alpha Beta坐标系下的电流Alpha
//  * @param[in]   _Beta: Alpha Beta坐标系下的电流Beta
//  * @param[out]  v->Ds: dq坐标系下的电流d
//  *              v->Qs: dq坐标系下的电流q
//  * @retval      none
//  */
// static inline void _park_calc(svpwm_t* v) {
//     DateType cosTh, sinTh;

//     // v->Alpha = _Alpha;
//     // v->Beta = _Beta;
//     // v->Angle = _Angle;

//     sinTh = fast_sin(v->Angle);
//     cosTh = fast_cos(v->Angle);

//     v->Ds = ((v->Alpha * cosTh) + (v->Beta * sinTh))/16384;
//     v->Qs = ((v->Beta * cosTh) - (v->Alpha * sinTh))/16384;
// }
// static inline void park_calc(svpwm_t* v, DateType _Alpha, DateType _Beta, DateType _Angle) {
//     DateType cosTh, sinTh;

//     v->Alpha = _Alpha;
//     v->Beta = _Beta;
//     v->Angle = _Angle;

//     _park_calc(v);
// }

// /*-----------------------------------------------------------------------------
//     IPARK 变换 宏
// -----------------------------------------------------------------------------*/
// #define IPARK_DEFAULTS \
//     { 0, 0, 0, 0, 0, 0, 0, }


// /*------------------------------------------------------------------------------
//     IPARK 变换 函数实体
// ------------------------------------------------------------------------------*/
// /**
//  * @brief       ipark变换
//  * @param[in]   *v: isvpwm_t结构体
//  * @param[in]   _Ds: dq坐标系下的电流d
//  * @param[in]   _Qs: dq坐标系下的电流q
//  * @param[out]  v->Alpha: Alpha Beta坐标系下的电流Alpha
//  *              v->Beta: Alpha Beta坐标系下的电流Beta
//  * @retval      none
//  */
// static inline void _ipark_calc(svpwm_t* v) {
//     DateType Cosine, Sine;

//     // v->Ds = _Ds;
//     // v->Qs = _Qs;
//     // v->Angle = _Angle;

//     Sine = fast_sin(v->Angle);
//     Cosine = fast_cos(v->Angle);

//     v->Alpha = ((v->Ds * Cosine) - (v->Qs * Sine))/16384;
//     v->Beta = ((v->Qs * Cosine) + (v->Ds * Sine))/16384;
// }
// static inline void ipark_calc(svpwm_t* v, DateType _Ds, DateType _Qs, DateType _Angle) {
//     DateType Cosine, Sine;

//     v->Ds = _Ds;
//     v->Qs = _Qs;
//     v->Angle = _Angle;

//     _ipark_calc(v);
// }

// #ifdef __cplusplus
// }
// #endif
// #endif //__PARK_H
