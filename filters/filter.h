#ifndef __FILTER_H__
#define __FILTER_H__

#ifdef __cplusplus                      //为了让c和c++可以共存
extern "C" {
#endif

#include <stdlib.h>                                   //malloc需要
#include <stdio.h>
#include <string.h>                                   //memset需要
#include <math.h>

#define PI 3.141592653589793
#define FILTER_SLIDE "slide"                        //define必须大写  模块名加功能名：规范
#define FILTER_LOWPASS "lowpass"                        //define必须大写  模块名加功能名：规范
#define FILTER_MEAN "mean"
#define FILTER_NOTCH "notch"

typedef float filter_type_t;                    // 根据需要来修改float

typedef struct filter_t              
{
    filter_type_t(*FilterFunction)(struct filter_t *filter,filter_type_t in_data);       //写成void* params 是为了方便面向不同种类的滤波器
    void* params;                                           // 同上 是为了可以对应上不同种滤波器的参数包
    void* name;                                                                                    
}filter_t; 


// 定义滑动窗口滤波参数结构体
typedef struct {
    filter_type_t* window;  // 滑动窗口数组
    int size;     // 窗口大小
    int index;    // 当前索引
    filter_type_t sum;      // 当前窗口内元素的总和
} SlidingWindowFilter;

// 定义低通滤波参数结构体
typedef struct {
    float parame;
    filter_type_t last;      
} LowPassFilter;

// 定义均值滤波参数结构体
typedef struct {
    filter_type_t* data;  // 滑动窗口数组
    int size;     // 窗口大小
    int index;    // 当前索引
    filter_type_t sum;      // 当前窗口内元素的总和     
} MeanFilter;

// 定义陷波滤波参数结构体
typedef struct {
    filter_type_t* previousInputs;
    filter_type_t* previousOutputs;      
    filter_type_t FILTER_FREQUENCY;
    filter_type_t Q_FACTOR;
} NotchFilter;

filter_type_t filter_run(filter_t *filter, filter_type_t in_data);
void filter_free(filter_t *filter);

void filter_init_sliding_window(filter_t *filter,int size);
void filter_init_lowpass(filter_t *filter,filter_type_t parame);
void filter_init_mean(filter_t *filter,int size);
void filter_init_notch(filter_t *filter,filter_type_t FILTER_FREQUENCY,filter_type_t Q_FACTOR);


void filter_set_sliding_window(filter_t *filter,int size);
#ifdef __cplusplus
}
#endif

#endif /* __FILTER_H__ */
