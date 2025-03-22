# 一、前言背景
STM32 对 AD 采样信号进行快速傅里叶变换（FFT），以获取其频谱信息。
信号源为信号发生器（正点原子DM40A）产生的 100Hz 三角波：

![100Hz三角波](https://i-blog.csdnimg.cn/direct/2f0f38a4a6ec46d394464614601d0599.jpeg#pic_center)

AD 采样由定时器触发，触发频率为 1280Hz。AD 以该频率采样 100Hz 的三角波，并对采样数据进行 FFT。FFT 计算出的结果是复数形式（实部、虚部交替排列的数组，数组大小为 $2N$，$N$ 为FFT的点数）的双边频谱：

![三角波FFT](https://i-blog.csdnimg.cn/direct/7ea37b2f46834c63be3f95bec3d2d1a1.png#pic_center)




即在 $[- \frac{f_{s}}{2} , \frac{f_{s}}{2}]$ 频率范围内对称分布（$f_{s}$为采样率）。对应的计算结果存储在一个复数数组中，该数组的索引范围为 $0$ 到 $2N-1$。我们把 $0$ 到 $N$ 之间的部分，定义为正频率分量；相应的， $N$ 到 $2N -1$ 之间的部分则为负频率分量。

通常，我们在频率分析时，会计算幅频相应：
![三角波幅频相应](https://i-blog.csdnimg.cn/direct/5f267463bb4b44d98ae32358e5f094ca.png#pic_center)


这个应用很常见，马上就是要寻找峰值，然后就可以确定信号的主要频率成分及其幅度；又或者计算相位谱，以分析各频率分量的相位关系等等。
但很少有见过对频率谱进行操作（如滤波、调制等等）后再通过逆傅里叶变换（IFFT）还原信号的应用。

于是笔者突发奇想，进行了一次小小的测试，目标是要**从100Hz的三角波中滤波得到它的基波100Hz的正弦波**并通过**DA将波形输出至示波器展示**，平台是**STM32H743VIT6**。

# 二、项目构思
## 1. 确定FFT点数、采样率、采样点数

AD/DA 的触发源均由定时器提供，定时器源自 PLL 倍频/分频前的 25MHz 晶振。而 100Hz 的三角波由独立的信号发生器产生，因此 AD 采样时钟与信号发生器的输出之间可能存在一定的相位漂移或频率偏差。如果希望波形满足实时性要求，AD 和 DA 需要由同一触发信号驱动，以保证同步（当然，也可以让 DA 的触发频率高于 AD 的触发频率，因为使用片上AD/DA，本质都共享同一时钟源，理论上不会产生频率偏差或相位漂移）。

其次是FFT点数、采样率、采样点数的安排。对于 100Hz 的三角波信号，为了能够较好地表征其频谱信息，选择12.8倍于100Hz的采样率进行采样，即 $f_{s} = 100 \times 12.8 = 1280$Hz。这一采样率远高于奈奎斯特采样定理规定的最低要求（$2 \times 100 = 200$Hz），可以有效减少失真，特别是对于高次谐波较多的三角波信号。

在时域上，这意味着每个周期大约对应 12-13 个采样点。采用 128 点 FFT 进行变换，则 FFT 频率分辨率为：
$$
f_{res} = \frac{f_{s}}{N} = \frac{1280}{128} = 10Hz
$$
即每个 FFT 频谱格宽度为10Hz。而**100Hz是10Hz的整数倍**，因此可以较好地反映100Hz三角波的各次谐波分量。

当然，在理论上，FFT 频率分辨率越高越好（即采样率越低或 FFT 点数越高），可以更精细地刻画信号的频谱结构。然而，我们的系统具有**实时性要求**，即需要在 AD 采样完成后，立即执行 FFT 变换、滤波、IFFT 处理，并将结果通过 DA 输出。因此，处理时间必须控制在 $T$ 的时间窗口内：
$$
T = AD采样点数 \times \frac{1}{采样率}
$$

基于此，我选择 12.8 倍于原始信号频率（100Hz）的采样率，即 1280Hz，并进行 128 点 FFT。

处理完成后，DA 端以相同采样率重建信号，每个周期包含 12-13 个采样点，这会让波形看上去很数字。故需要通过模拟低通滤波器（LPF）滤除采样频率 1280Hz 及其倍频分量，使得输出信号平滑，得到 100Hz 的正弦波。

## 2. 双缓存设计
在 AD 采集 128 个点后，我们需要执行 FFT 变换、滤波、IFFT 处理，并将结果通过 DA 输出。在此过程中，如果仅使用单一缓存，会导致数据处理期间无法继续采样，造成数据丢失。为了解决这一问题，我们采用 双缓存（Ping-Pong Buffering） 机制，使得数据采集与处理能够并行进行，提高实时性。

具体设计如下：
1. 建立两个缓冲区 BufferA 和 BufferB。
2. 当 DMA 将 ADC 采样数据搬运至 BufferA 时，我们对 BufferB 进行 FFT 变换、滤波、IFFT 处理，并准备其数据供 DAC 输出。
3. 当 BufferA 搬运完成时，DMA 产生中断，设置一个标志位，并切换缓冲区：
    -  ADC 的 DMA 目标地址切换到 BufferB，开始采集新数据。
    -  BufferA 处理完成的数据被传输至 DAC，作为 DAC的DMA 的数据源。
4. 在下一个 DMA 传输周期，两个缓冲区的角色再次交换，如此**交替**进行，确保数据采集与处理不间断。

![乒乓操作](https://i-blog.csdnimg.cn/direct/e23923bdc54e400c91735bcb9060ff37.png#pic_center)
# 三、代码实现

## 1. STM32CubeMX配置和HAL库初始化
**ADC配置**
![ADC](https://i-blog.csdnimg.cn/direct/e326398a588b40e7803f946e478adda3.png#pic_center)

![ADC2](https://i-blog.csdnimg.cn/direct/5c5b882606e14a6aa02c7d0224949619.png#pic_center)
**DAC配置（利用OPAMP跟随输出）**
![DAC1](https://i-blog.csdnimg.cn/direct/04b49acaa4ed43a0b5053484fbdf39ea.png#pic_center)

![DAC2](https://i-blog.csdnimg.cn/direct/e95dc843aa04469f8a3fc1803a309e02.png#pic_center)

![DAC3](https://i-blog.csdnimg.cn/direct/a044e3f5faaa49ce97d6d801331601ab.png#pic_center)
并完成**HAL库的初始化**代码：

```c
_Noreturn void Main(void)
{
    RetargetInit(&huart1);

    HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED);
    bsp_Adc_Start(&hadc1);


    bsp_SetTimerFreq(&htim1, SAMPLERATE);
    HAL_TIM_Base_Start_IT(&htim1);


    HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t *)dac_dma_buf, DAC_DMA_BUF_NUM, DAC_ALIGN_12B_R);
    HAL_OPAMP_Start(&hopamp1);


    while (true)
    {
        process_fft_ifft();
    }
}
```

## 2. 核心代码

**定义采样率、ADC采样点数、DAC的DMA缓存点数、FFT点数：**

```c
#define SAMPLERATE  ((100)*(12.8))

#define ADC_DAC_NUM 128

#ifndef DAC_DMA_BUF_NUM
#define DAC_DMA_BUF_NUM ADC_DAC_NUM
#endif

extern uint16_t dac_dma_buf[DAC_DMA_BUF_NUM];

#define ADC_DMA_BUF_NUM ADC_DAC_NUM

#define ADCx 1
#if ADCx==1
extern uint16_t       adc_dma_buffer[ADC_DMA_BUF_NUM];
#elif ADCx==2
extern uint32_t       adc_dma_buffer[ADC_DMA_BUF_NUM];
#endif

#define FFT_NUM ADC_DAC_NUM
```

最核心的代码在ADC的**DMA的完成中断**中执行（这里这样写是因为H7配置了MPU，需要考虑Cache未击中问题）：
使用**两个指针**完成对双缓存的调度

```c
void signalTask(void)
{
    /* 半传输完成中断 */
    if((ADC_DMAx->LISR & ADC_DMA_Streamx_HC) != RESET)
    {
        float32_t *target_buffer = (active_buffer) ? bufferA : bufferB;
        float32_t *processing_buffer = (active_buffer) ? bufferB : bufferA;
        SCB_InvalidateDCache_by_Addr((uint32_t *)(&adc_dma_buffer[0]), ADC_DMA_BUF_NUM);

        for (uint16_t i = 0; i < FFT_NUM / 2; ++i) {
            target_buffer[i] = adc_dma_buffer[i];
            dac_dma_buf[i] = (uint16_t)processing_buffer[i];
        }
        adc_state = 1;
        ADC_DMAx->LIFCR = ADC_DMA_Streamx_HC;
    }

    /* 传输完成中断 */
    if((ADC_DMAx->LISR & ADC_DMA_Streamx_TC) != RESET)
    {
        float32_t *target_buffer = (active_buffer) ? bufferA : bufferB;
        float32_t *processing_buffer = (active_buffer) ? bufferB : bufferA;
        SCB_InvalidateDCache_by_Addr((uint32_t *)(&adc_dma_buffer[ADC_DMA_BUF_NUM / 2]), ADC_DMA_BUF_NUM);

        for (uint16_t i = 0; i < FFT_NUM / 2; ++i) {
            target_buffer[i + FFT_NUM / 2] = adc_dma_buffer[i + FFT_NUM / 2];
            dac_dma_buf[i + FFT_NUM / 2] = (uint16_t)processing_buffer[i + FFT_NUM / 2];
        }

        adc_state = 2;
        ADC_DMAx->LIFCR = ADC_DMA_Streamx_TC;
    }
}
```

根据`adc_state`的状态判断ADC的DMA是否完成，来执行FFT、滤波、IFFT处理 `process_fft_ifft` 。
完成处理置位标志位`active_buffer`。


```c
float32_t fft_input[FFT_NUM*2];

void process_fft_ifft(void)
{
    if (adc_state == 2)
    {
        float32_t *processing_buffer = (active_buffer) ? bufferA : bufferB;
        for (uint16_t i = 0; i < FFT_NUM; ++i)
        {
            fft_input[i * 2] = processing_buffer[i];
            fft_input[i * 2 + 1] = 0;
        }

        arm_cfft_f32(&arm_cfft_sR_f32_len128, fft_input, 0, 1);
        
        uint16_t cutoff_bin = (uint16_t)((120.0 / SAMPLERATE) * (FFT_NUM * 2));

        for (uint16_t i = cutoff_bin; i < FFT_NUM/2; i++) {
            fft_input[2 * i] = 0.0f;
            fft_input[2 * i + 1] = 0.0f;

            fft_input[2 * (FFT_NUM - i)] = 0.0f;
            fft_input[2 * (FFT_NUM - i) + 1] = 0.0f;
        }


        arm_cfft_f32(&arm_cfft_sR_f32_len128, fft_input, 1, 1);

        for (uint16_t i = 0; i < FFT_NUM; i++) {
            processing_buffer[i] = fft_input[i * 2];
        }
        
        active_buffer = !active_buffer;
        adc_state = 0;
    }
}
```
这里的滤波方法是暴力滤波（优化方法：可以采用平滑过渡，例如用 **Hanning** 窗或 **Hamming** 窗在频域做过渡，而不是直接硬性截断）。
在滤波的时候，请不要忘记处理负频率：

```c
for (uint16_t i = cutoff_bin; i < FFT_NUM/2; i++) {
// 正频率
	fft_input[2 * i] = 0.0f;
	fft_input[2 * i + 1] = 0.0f;
// 负频率	
	fft_input[2 * (FFT_NUM - i)] = 0.0f;
	fft_input[2 * (FFT_NUM - i) + 1] = 0.0f;
}
```

# 四、效果展示和后话
**最终效果展示**
这里我没有加上LPF（懒）

![system](https://i-blog.csdnimg.cn/direct/a8b2fe9961ad43b7833b2f0558102a20.jpeg#pic_center)


![波形](https://i-blog.csdnimg.cn/direct/e9ac41b43a8b45159c152fc6733baae0.jpeg#pic_center)
![滤波后的FFT结果](https://i-blog.csdnimg.cn/direct/b671dee67e6c434c89a14f815aee761c.png#pic_center)


**后话**

FFT滤波时，我直接截断高频分量相当于乘以一个**矩形窗**，在时域上会导致**高斯振铃（Gibbs 现象）**（类似于时域上的方波傅里叶变换导致的振铃效应），后续优化可以采用平滑过渡，例如用 Hanning 窗或 Hamming 窗在频域做过渡，而不是直接硬性截断。

例如，使用 Hanning 窗 对截止频率附近的分量进行平滑处理：
```c
for (uint16_t i = cutoff_bin; i < FFT_NUM/2; i++) {
	float window_coeff = 0.22 * (1 + cosf(M_PI * (i - cutoff_bin) / (FFT_NUM/2 - cutoff_bin))); // Hanning 窗
	fft_input[2 * i] *= window_coeff;
	fft_input[2 * i + 1] *= window_coeff;
	fft_input[2 * (FFT_NUM - i)] *= window_coeff;
	fft_input[2 * (FFT_NUM - i) + 1] *= window_coeff;
}
```
这样可以让高频分量**逐渐衰减**，而不是直接消失，减少 Gibbs 现象。
![Hanning](https://i-blog.csdnimg.cn/direct/869c1b53d2c7424faf4e7623bccd87c8.png)
此外，FFT 频域滤波更适合用于选取特定频率分量的滤波器。在实际应用中，我们通常希望滤波器具有一定的过渡带，而不是突变的截止频率。这是因为理想的 “砖墙” 低通滤波器（即瞬间截止所有高频分量）在物理上不可实现，实际滤波器通常需要一定的过渡区域，以减少时域上的振铃效应并改善信号的平滑性。


**项目地址：** [https://github.com/mico845/STM32FFTFilter](https://github.com/mico845/STM32FFTFilter)



