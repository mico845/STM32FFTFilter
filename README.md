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
![Hanning](https://i-blog.csdnimg.cn/direct/869c1b53d2c7424faf4e7623bccd87c8.png#pic_center)
此外，FFT 频域滤波更适合用于选取特定频率分量的滤波器。在实际应用中，我们通常希望滤波器具有一定的过渡带，而不是突变的截止频率。这是因为理想的 “砖墙” 低通滤波器（即瞬间截止所有高频分量）在物理上不可实现，实际滤波器通常需要一定的过渡区域，以减少时域上的振铃效应并改善信号的平滑性。



# 五、项目联想与扩展
## 1. 倍频

在前文中，我们提到 DA的触发频率高于 AD 的触发频率。由于我们使用的是 片上 AD/DA，它们本质上共享同一时钟源，因此 理论上不会产生频率偏差或相位漂移。

在本项目中，我们采用双缓冲技术，理论上可以在 DA 输出缓存区中存储 100 Hz 的正弦波（通过滤波器得到的），然后以更高的触发频率进行输出，从而合成更高频率的正弦波信号。

说干就干，我决定进行实验测试，尝试让系统在**输入100 Hz三角波的情况下，输出300 Hz正弦波**，以实现更灵活的需求。

**代码实现**
这里修改DAC，让定时器2的Trigger Out event事件触发
![定时器2触发](https://i-blog.csdnimg.cn/direct/5809f5de0a264d40b78b6a682504753e.png#pic_center)
配置定时器的触发频率为`DAC_FREQ`：

```c
bsp_SetTimerFreq(&htim2, DAC_FREQ);
HAL_TIM_Base_Start_IT(&htim2);
```

```c
#define SAMPLERATE  ((100)*(12.8))
#define DAC_FREQ    (SAMPLERATE * 3)
```

**效果展示**

![三倍频三角波](https://i-blog.csdnimg.cn/direct/82b5a0d4770e4fdc8d3829577658f6db.jpeg#pic_center)

## 2. 降频
基于倍频的原理，我们同样可以设计降频系统。其方法是保持采样频率不变，但增加采样点数，从而降低输出信号的频率。例如，若希望输出信号的频率降低至原来的一半，则需要将采样点数增加一倍。或者进行插值，增多DA输出点数。
说干就干，我决定进行实验测试，尝试让系统在**输入100 Hz三角波的情况下，输出50Hz正弦波**，以实现更灵活的需求。

采样率和DAC频率如下：
```c
#define SAMPLERATE  ((100)*(12.8) * 2)
#define DAC_FREQ    ((100)*(12.8))
```
这里玩一下宏，利用宏拼接的方式，方便调整FFT点数：

```c
#define FFT_NUM ADC_DAC_NUM

#define __s_arm_cfft_f32(x) arm_cfft_sR_f32_len##x
#define _s_arm_cfft_f32(x) __s_arm_cfft_f32(x)
#define s_arm_cfft_f32 _s_arm_cfft_f32(FFT_NUM)
```

FFT和IFFT就变成了

```c
arm_cfft_f32(&s_arm_cfft_f32, fft_input, 0, 1);
```

```c
arm_cfft_f32(&s_arm_cfft_f32, fft_input, 1, 1);
```

**效果展示**
![降频正弦波](https://i-blog.csdnimg.cn/direct/42803bc829c84fcb9c7a250ebf4710a0.jpeg#pic_center)
但是，由于DA的触发频率低于AD的触发频率，导致采样数据在DA输出时**无法准确对齐**，进而引发**相位偏移**。为了解决这一问题，我决定采用**插值**的方法。

## 3. 插值
### 3.1 线性插值
利用**插值**算法，在保证 ADC 采样率不变的前提下，使 DAC 在采样点之间生成更多的插值点，从而平滑输出，提高信号的连续性。

我修改了DMA 完成中断的一些细节，以确保数据存储、处理和输出的正确：
```c
void signalTask(void)
{
    /* 半传输完成中断 */
    if((ADC_DMAx->LISR & ADC_DMA_Streamx_HC) != RESET)
    {
        float32_t *target_buffer = (active_buffer) ? bufferA : bufferB;
        float32_t *processing_buffer = (active_buffer) ? bufferB : bufferA;
        SCB_InvalidateDCache_by_Addr((uint32_t *)(&adc_dma_buffer[0]), ADC_DMA_BUF_NUM);

        for (uint16_t i = 0; i < ADC_DMA_BUF_NUM / 2; ++i) {
            target_buffer[i] = adc_dma_buffer[i];

        }
        for (uint16_t i = 0; i < DAC_DMA_BUF_NUM / 2; ++i) {
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

        for (uint16_t i = 0; i < ADC_DMA_BUF_NUM / 2; ++i) {
            target_buffer[i + ADC_DMA_BUF_NUM / 2] = adc_dma_buffer[i + ADC_DMA_BUF_NUM / 2];

        }
        for (uint16_t i = 0; i < DAC_DMA_BUF_NUM / 2; ++i) {
            dac_dma_buf[i + DAC_DMA_BUF_NUM / 2] = (uint16_t)processing_buffer[i + DAC_DMA_BUF_NUM / 2];
        }

        adc_state = 2;
        ADC_DMAx->LIFCR = ADC_DMA_Streamx_TC;
    }
}
```

接下来，我打算使用**线性插值**做第一次尝试：
> 线性插值（Linear Interpolation，简称 Lerp）是一种用于在已知数据点之间估算新数据点的数学方法。它的基本思想是：如果两个已知点之间的数据是平滑变化的，我们可以用一条直线来近似它们之间的过渡，并计算中间值。

先简单了解一下线性插值算法：
假设有两个已知点：

- $A(x_{1}, y_{1})$
- $B(x_{2}, y_{2})$

你想知道在这两个点之间，某个位置$x$处的$y$是多少。
在数学上，线性插值使用下面的公式：
$$
y = y_{1} + \frac{(x - x_{1})}{(x_{2} - x_{1})} \times (y_{2} - y_{1})
$$
解释一下这个公式，也就是计算$x$在$x_{1}$和$x_{2}$之间的相对位置：
$$
frac =  \frac{(x - x_{1})}{(x_{2} - x_{1})}
$$
这个值介于$0$和$1$之间，表示$x$在$x_{1}$和$x_{2}$之间的**比例**。
然后依照这个比例，计算出$y$方向的偏移量：
$$
\Delta y =  (y_{2} - y_{1}) \times frac
$$

最终得到插值的结果：
$$
y = y1 + \Delta y
$$

在我们的这个系统中，也就是改成数组的方式来解释，可以直接写成：

```c
	uint16_t w = output_size / input_size;
	// 省略 ... 
	index = (float)i / w;
	idx = (uint16_t)index;
	frac = index - idx;
```
我来解释一下，假设 `input_size = 4`，`output_size = 8`，那么 `w = output_size / input_size = 8 / 4 = 2`。
`output` 数组的索引 `i` 从 `0` 到 `7`，然后 `index = i / 2` 变为：

```
	i = 0  →  index = 0 / 2 = 0.0
	i = 1  →  index = 1 / 2 = 0.5
	i = 2  →  index = 2 / 2 = 1.0
	i = 3  →  index = 3 / 2 = 1.5
	i = 4  →  index = 4 / 2 = 2.0
	i = 5  →  index = 5 / 2 = 2.5
	i = 6  →  index = 6 / 2 = 3.0
	i = 7  →  index = 7 / 2 = 3.5
```
相应的`idx`和`frac`也就是：

```
	i = 0  →  index = 0.0,  idx = 0,  frac = 0.0
	i = 1  →  index = 0.5,  idx = 0,  frac = 0.5
	i = 2  →  index = 1.0,  idx = 1,  frac = 0.0
	i = 3  →  index = 1.5,  idx = 1,  frac = 0.5
	i = 4  →  index = 2.0,  idx = 2,  frac = 0.0
	i = 5  →  index = 2.5,  idx = 2,  frac = 0.5
	i = 6  →  index = 3.0,  idx = 3,  frac = 0.0
	i = 7  →  index = 3.5,  idx = 3,  frac = 0.5
```
`frac`等于`0`时，该点为采样点；反之，该点为需要插入的点。
我们手动计算`i=1`时的`frac`，根据公式可以得到`frac = (1 - 0)/(2 - 0) = 0.5`，和我们的计算结果也是吻合的。

从公式上理解，也就是
$$
y = y_{1} + \frac{(x - x_{1})}{(x_{2} - x_{1})} \times (y_{2} - y_{1})
= y_{1} + (y_{2} - y_{1})  \times frac
= y_{1}   \times (1 - frac) + frac \times y_{2}
$$
代码上也就是：
```c
	output[i] = (1 - frac) * input[idx] + frac * input[idx + 1];
```

故，我们完成线性拟合函数：

```c
void lerp(uint16_t *input, uint16_t *output, uint16_t input_size, uint16_t output_size)
{
    uint16_t w = output_size / input_size;
    float index;
    uint16_t idx;
    float frac;

    for (uint16_t i = 0; i < output_size; i++)
    {
        index = (float)i / w;
        idx = (uint16_t)index;
        frac = index - idx;

        if (idx < input_size - 1)
        {
            output[i] = (uint16_t)((1 - frac) * input[idx] + frac * input[idx + 1]);
        }
        else
        {
            output[i] = input[input_size - 1];
        }
    }
}
```


在`process_fft_ifft`调用函数`lerp`：
```c
		// 省略 ...
        arm_cfft_f32(&s_arm_cfft_f32, fft_input, 1, 1);

        uint16_t interpolated_output[DAC_DMA_BUF_NUM];
        uint16_t interpolated_input[FFT_NUM];

        for (uint16_t i = 0; i < FFT_NUM; i++) {
            interpolated_input[i] = fft_input[2 * i];
        }

        lerp(interpolated_input, interpolated_output, FFT_NUM, DAC_DMA_BUF_NUM);

        for (uint16_t i = 0; i < DAC_DMA_BUF_NUM; i++) {
            processing_buffer[i] = interpolated_output[i];
        }
        
        active_buffer = !active_buffer;
        adc_state = 0;
        // 省略 ...
```

这是对系统的采样率，DA触发频率，AD/DA的DMA缓存区大小配置：
```c
#define INTERP_FACTOR 2

#define SAMPLERATE  ((100)*(12.8))
#define DAC_FREQ    (SAMPLERATE * INTERP_FACTOR)

#define ADC_DAC_NUM 128

#ifndef DAC_DMA_BUF_NUM
#define DAC_DMA_BUF_NUM (ADC_DAC_NUM * INTERP_FACTOR)

#define FFT_NUM ADC_DAC_NUM
```

**效果展示**


![线性插值算法](https://i-blog.csdnimg.cn/direct/5ba214741c0d4922aaebbe5936435c2c.jpeg#pic_center)

### 3.2 样条插值
ARM_DSP库支持了样条插补，双线性插补和线性插补。
在这里我打算追求更高的精度、平滑性，尝试使用**样条插值**。样条插值常见的有：
- 二次样条插值（Quadratic Spline）
- 三次样条插值（Cubic Spline）

样条插值的核心思想是：

1. 用多个低阶多项式拼接，形成平滑曲线
2. 确保曲线在数据点处连续且光滑（导数连续）

例如，三次样条插值公式就是：
每个区间$[x_{i},x_{i+1}]$定义一个三次多项式：
$$
S_{i} = a_{i} + b_{i}(x - x_{i}) + c_{i}(x - x_{i})^{2} + d_{i}(x - x_{i})^{3}
$$
需要解的参数是$a_{i}$、$b_{i}$、$c_{i}$、$d_{i}$，它们满足：
- 函数值连续（相邻多项式在连接点处相等）
- 一阶导数连续（保证曲线的光滑性）
- 二阶导数连续（减少振荡）

这些条件形成一个线性方程组，求解后就能得到插值函数。

而ARM_DSP的样条插补主要通过这两个函数实现：`arm_spline_init_f32` 和 `arm_spline_f32`。

这里面需要理解的配置参数有：
- `ARM_SPLINE_NATURAL` 自然样条插补
- `ARM_SPLINE_PARABOLIC_RUNOUT` 抛物线样条插补

自然样条插补对应三次样条插值，精度更高，更光滑，但计算量更大；抛物线样条插补对应二次样条插值，比三次样条计算量小，性能略逊自然样条插补。

这里我打算使用**抛物线样条插补**。

先定义结构体和变量：

```c
arm_spline_instance_f32 S;
#define SpineTab DAC_DMA_BUF_NUM / FFT_NUM
float32_t xn[FFT_NUM];
float32_t xnpos[DAC_DMA_BUF_NUM];
float32_t coeffs[3*(FFT_NUM - 1)]; /* 插补系数缓冲 */
float32_t tempBuffer[2 * FFT_NUM - 1]; /* 插补临时缓冲 */
```
对`x`轴的下标进行初始化：
```c
void spline_Init(void)
{
    for(uint16_t i = 0; i < FFT_NUM; i++)
    {
        xn[i] = i*SpineTab;
    }
    for(uint16_t i = 0; i < DAC_DMA_BUF_NUM; i++)
    {
        xnpos[i] = i;
    }
}
```
添加到Main函数中
```c
_Noreturn void Main(void)
{
// 省略 ...
    spline_Init();
// 省略 ...
}
```

编写计算函数`spline`：
```c
void spline(float32_t *input, float32_t *output)
{
    arm_spline_init_f32(&S,
                        ARM_SPLINE_PARABOLIC_RUNOUT ,
                        xn,
                        input,
                        FFT_NUM,
                        coeffs,
                        tempBuffer);
/* 样条计算 */
    arm_spline_f32 (&S,
                    xnpos,
                    output,
                    DAC_DMA_BUF_NUM);
}
```

在`process_fft_ifft`调用函数`spline`：

```c
		// 省略 ...
        arm_cfft_f32(&s_arm_cfft_f32, fft_input, 1, 1);

        float32_t interpolated_output[DAC_DMA_BUF_NUM];
        float32_t interpolated_input[FFT_NUM];

        for (uint16_t i = 0; i < FFT_NUM; i++) {
            interpolated_input[i] = fft_input[2 * i];
        }
        
        spline(interpolated_input, interpolated_output);

        for (uint16_t i = 0; i < DAC_DMA_BUF_NUM; i++) {
            processing_buffer[i] = interpolated_output[i];
        }
        
        active_buffer = !active_buffer;
        adc_state = 0;
        // 省略 ...
```

**效果展示**
![二次样条插值](https://i-blog.csdnimg.cn/direct/1e6c0827b9c04cb08555d5bf49a86181.jpeg#pic_center)
这个结果比线性插值的光滑得多了，不再出现时不时一下的不连续问题。

测试时，将插值点数提高，最终点数来到`12800`点时：

```c
#define INTERP_FACTOR 100

#define SAMPLERATE  ((100)*(12.8))
#define DAC_FREQ    (SAMPLERATE * INTERP_FACTOR)


#define ADC_DAC_NUM 128

#ifndef DAC_DMA_BUF_NUM
#define DAC_DMA_BUF_NUM (ADC_DAC_NUM * INTERP_FACTOR)
#endif
```
![12800点插值](https://i-blog.csdnimg.cn/direct/fb895523dcc04d14a04e7650b23caceb.jpeg#pic_center)
也就相当光滑了。这样同时也提高了DA的采样频率，后续想要制作LPF已经相当容易了。



**项目地址：** [https://github.com/mico845/STM32FFTFilter](https://github.com/mico845/STM32FFTFilter)
