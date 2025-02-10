#include "filter.h"

filter_type_t filter_run(filter_t *filter, filter_type_t in_data) {           //抽象层 可以面向不同种类的滤波器
    return filter->FilterFunction(filter,in_data);
}

void filter_free(filter_t *filter)
{
    void* p = filter->params;           //因为要释放指针 还需要释放内存  需要暂时变量来保存指针
    filter->FilterFunction = NULL;
    filter->params = NULL;
    free(p);
}

// 滑动窗口滤波函数
filter_type_t filter_slide(filter_t *filter,filter_type_t in_data) 
{
    SlidingWindowFilter* _filter = (SlidingWindowFilter*)filter->params;                //_filter和filter->params都是指针类型 指的是同一个东西

    // 更新当前窗口的总和，减去最旧的值，加上新的值
    _filter->sum = _filter->sum - _filter->window[_filter->index] + in_data;

    // 将新的值放入窗口
    _filter->window[_filter->index] = in_data;

    // 更新索引，实现环形存储
    _filter->index = (_filter->index + 1) % _filter->size;

    // 返回均值
    return _filter->sum / _filter->size;
}

// 低通滤波函数
filter_type_t filter_lowpass(filter_t *filter,filter_type_t in_data) 
{
    LowPassFilter* _filter = (LowPassFilter*)filter->params;
    float out = (1 - _filter->parame) * _filter->last + _filter->parame * in_data;
    _filter->last = in_data;
    return out;
}

// 均值滤波函数
filter_type_t filter_mean(filter_t *filter,filter_type_t in_data) 
{
    MeanFilter* _filter = (MeanFilter*)filter->params;               
    //更新索引 
    _filter->index = (_filter->index + 1);
    //将新的值放入窗口
    _filter->data[_filter->index] = in_data;
    // 更新当前窗口的总和
    _filter->sum = _filter->sum + in_data;
    if(_filter->index == _filter->size-1)
    {
        _filter->index = 0;
        _filter->sum = 0;
        return  _filter->sum / _filter->size;
    }
    return 0;
}

//陷波滤波函数
filter_type_t filter_notch(filter_t *filter,filter_type_t in_data) 
{
    NotchFilter* _filter = (NotchFilter*)filter->params;       

    float omega = 2.0 * PI * _filter->FILTER_FREQUENCY;
    float alpha = sin(omega) / (2.0 * _filter->Q_FACTOR);

    float a1 = -2.0 * cos(omega);
    float a2 = 1.0 - alpha;
    float b0 = 1.0;
    float b1 = -2.0 * cos(omega);
    float b2 = 1.0;

    float currentOutput = b0 * in_data + b1 * _filter->previousInputs[0] + b2 * _filter->previousInputs[1]
                          - a1 * _filter->previousOutputs[0] - a2 * _filter->previousOutputs[1];

    // 更新输入和输出历史
    _filter->previousInputs[1] = _filter->previousInputs[0];
    _filter->previousInputs[0] = in_data;
    _filter->previousOutputs[1] = _filter->previousOutputs[0];
    _filter->previousOutputs[0] = currentOutput;

    return currentOutput;
}

void filter_init_sliding_window(filter_t *filter,int size)
{
    filter->FilterFunction = filter_slide; 
    filter->name = FILTER_SLIDE;
    //动态申请malloc得到的是全局变量 不然你在里面定义的就是局部变量 函数调用完毕 变量内存就释放了
    SlidingWindowFilter* Slide_filter = (SlidingWindowFilter*)malloc(sizeof(SlidingWindowFilter)+size * sizeof(filter_type_t));  //malloc需要知道你申请的地址块大小 而地址大小是和变量类型有关的 
                                                                                     //返回的东西总不能直接就是值了吧 值是由用户定义的。返回的应该是你申请的地址快地址 
                         //sizeof(SlidingWindowFilter)申请的是 整个结构体的大小（存放参数），，，size * sizeof(int) 申请的的是数组的大小（）存放数据

    // 分配内存空间，用于存储窗口中的值 
    Slide_filter->window = (filter_type_t*)((SlidingWindowFilter*)Slide_filter + sizeof(SlidingWindowFilter));  //按顺序申请的地址 按顺序放。。。这就是得到了你申请的数组的首地址
    // 设置窗口大小
    Slide_filter->size = size;
    // 设置当前窗口的索引
    Slide_filter->index = 0;
    // 设置窗口中元素的和
    Slide_filter->sum = 0;

    // 初始化窗口数组为0
    memset(Slide_filter->window, 0, size * sizeof(int)); 

    filter->params = Slide_filter;                                      //让params和Slide_filter指向同一块地址
}

void filter_init_lowpass(filter_t *filter,filter_type_t parame)
{
    filter->FilterFunction = filter_lowpass; 
    filter->name = FILTER_LOWPASS;
    LowPassFilter* Lowpass_filter = (LowPassFilter*)malloc(sizeof(LowPassFilter));                                         
    // 设置低通滤波系数
    Lowpass_filter->parame = parame;

    Lowpass_filter->last = 0;

    filter->params = Lowpass_filter;                                   
}

void filter_init_mean(filter_t *filter,int size)
{
    filter->FilterFunction = filter_mean; 
    filter->name = FILTER_MEAN;
    MeanFilter* Mean_filter = (MeanFilter*)malloc(sizeof(MeanFilter) + size * sizeof(filter_type_t)); 
    // 分配内存空间，用于存储窗口中的值 
    Mean_filter->data = (filter_type_t*)((MeanFilter*)Mean_filter+sizeof(MeanFilter));  
    // 设置窗口大小
    Mean_filter->size = size;
    // 设置当前窗口的索引
    Mean_filter->index = 0;
    // 设置窗口中元素的和
    Mean_filter->sum = 0;

    // 初始化窗口数组为0
    memset(Mean_filter->data, 0, size * sizeof(int)); 

    filter->params = Mean_filter;                                      //让params和Slide_filter指向同一块地址
}

void filter_init_notch(filter_t *filter,filter_type_t FILTER_FREQUENCY,filter_type_t Q_FACTOR)
{
    filter->FilterFunction = filter_notch; 
    filter->name = FILTER_NOTCH;
    //动态申请malloc得到的是全局变量 不然你在里面定义的就是局部变量 函数调用完毕 变量内存就释放了
    NotchFilter* Notch_filter = (NotchFilter*)malloc(sizeof(NotchFilter)+ 4 * sizeof(filter_type_t));  //malloc需要知道你申请的地址块大小 而地址大小是和变量类型有关的 
                                                                                     //返回的东西总不能直接就是值了吧 值是由用户定义的。返回的应该是你申请的地址快地址 
                                                                    //sizeof(SlidingWindowFilter)申请的是 整个结构体的大小（存放参数），，，size * sizeof(int) 申请的的是数组的大小（）存放数据

    // 分配内存空间，用于存储历史输入数据
    Notch_filter->previousInputs =(filter_type_t*)((NotchFilter*)Notch_filter + sizeof(NotchFilter));  
    // 分配内存空间，用于存储历史输出数据
    Notch_filter->previousOutputs = (filter_type_t*)((NotchFilter*)Notch_filter + sizeof(NotchFilter) + 2 * sizeof(filter_type_t));

    Notch_filter->FILTER_FREQUENCY = FILTER_FREQUENCY;
    Notch_filter->Q_FACTOR = Q_FACTOR;

    // 初始化窗口数组为0
    memset(Notch_filter->previousInputs, 0, 2 * sizeof(int)); 
    memset(Notch_filter->previousOutputs, 0, 2 * sizeof(int)); 

    filter->params = Notch_filter;                                      
}







void filter_set_sliding_window(filter_t *filter,int size)
{
    SlidingWindowFilter* Slide_filter = (SlidingWindowFilter*)filter->params;
    Slide_filter->size = size;
    if(size > Slide_filter->size) //如果窗口大小变大了 需要重新分配内存空间        变小了不管
    {
        Slide_filter->window = (filter_type_t*)realloc(Slide_filter->window,size * sizeof(int));
        memset(Slide_filter->window+Slide_filter->index, 0, (Slide_filter->size - Slide_filter->index) * sizeof(int));//将新加的初始化为0
    }
}
