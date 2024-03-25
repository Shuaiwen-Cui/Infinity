# 数字信号处理

![DSP](DSP.png)

## 理论
<div class="grid cards" markdown>
-   :material-book:{ .lg .middle } __理解数字信号处理 🎯__

    ---

    作者: Richard G. Lyons

    阅读进度: [2/13]

    [:octicons-arrow-right-24: <a href="https://learning.oreilly.com/library/view/understanding-digital-signal/9780137028450/" target="_blank"> 传送门 </a>](#)

-  :simple-coursera:{ .lg .middle } __数字信号处理 🎯__

    ---

    由EPFL制作

    [:octicons-arrow-right-24: <a href="https://www.coursera.org/specializations/digital-signal-processing#courses" target="_blank"> 传送门 </a>](#)

-  :fontawesome-solid-blog:{ .lg .middle } __信号处理有关的那些东东 🎯__

    ---

    作者：Mr.看海

    阅读进度：

        时频域分析：[12/18]    
        经验模态分解： [2/17]
        神经网络：[0/2]       
        滤波降噪： [0/7]
        时间序列：[0/2]       
        数据降维： [0/4]
        剩余寿命预测：[0/4]   
        概念辨析： [0/6]
        Matlab 小技巧：[0/2]  

    [:octicons-arrow-right-24: <a href="https://zhuanlan.zhihu.com/p/138141521" target="_blank"> 传送门 </a>](#)


</div>

进度：

**时频域分析:** 

- [x] 时域分析——有量纲特征值含义一网打尽
- [x] 时域分析——无量纲特征值含义一网打尽
- [x] 信号时域分析方法的理解（峰值因子、脉冲因子、裕度因子、峭度因子、波形因子和偏度等）
- [x] 时域特征值提取的MATLAB代码实现（均方根、峰值因子、脉冲因子、裕度因子、峭度因子、波形因子和偏度等）
- [x] 频域特征指标及其MATLAB代码实现（重心频率、均方频率、均方根频率、频率方差、频率标准差）
- [x] 信号频域分析方法的理解（频谱、能量谱、功率谱、倒频谱、小波分析）
- [x] 频域特征值提取的MATLAB代码实现（频谱、功率谱、倒频谱）
- [x] 如何优雅地进行频谱分析—— 一行代码实现绘制MATLAB频谱、功率谱图
- [x] 补充：频域特征值提取的MATLAB代码实现（小波分析）
- [x] 时频域分析的一些常用概念补充（线性时不变系统、卷积、冲激响应、窗函数等）
- [x] 【熵与特征提取】基于“信息熵”的特征指标及其MATLAB代码实现（功率谱熵、奇异谱熵、能量熵）
- [x] 【熵与特征提取】从近似熵，到样本熵，到模糊熵，再到排列熵，究竟实现了什么？（第一篇）——近似熵及其MATLAB实现
- [ ] 【熵与特征提取】从近似熵，到样本熵，到模糊熵，再到排列熵，究竟实现了什么？（第二篇）——样本熵及其MATLAB实现
- [ ] 【熵与特征提取】从近似熵，到样本熵，到模糊熵，再到排列熵，究竟实现了什么？（第三篇）——模糊熵及其MATLAB实现
- [ ] 【熵与特征提取】从近似熵，到样本熵，到模糊熵，再到排列熵，究竟实现了什么？（第四篇）——排列熵及其MATLAB实现
- [ ] 【多尺度熵与特征提取】一文看懂“多尺度熵”——多尺度样本熵、多尺度模糊熵、多尺度排列熵、多尺度包络熵、多尺度功率谱熵、多尺度能量熵、多尺度奇异谱熵及其MATLAB实现
- [ ] 【复合多尺度熵与特征提取】一文看懂“复合多尺度熵”——复合多尺度样本熵、模糊熵、排列熵、包络熵、功率谱熵、能量熵、奇异谱熵及其MATLAB实现
- [ ] 答疑：采样频率fs要怎样设置？
- [x] 从傅里叶变换，到短时傅里叶变换，再到小波分析（CWT），看这一篇就够了（附MATLAB傻瓜式实现代码）


**经验模态分解：**

- [x] 这篇文章能让你明白经验模态分解（EMD）——基础理论篇
- [x] 这篇文章能让你明白经验模态分解（EMD）——IMF的物理含义
- [x] 这篇文章能让你明白经验模态分解（EMD）——EMD在MATLAB中的实现方法
- [x] 希尔伯特-黄变换（HHT）的前世今生——一个从瞬时频率讲起的故事
- [x] 希尔伯特谱、边际谱、包络谱、瞬时频率/幅值/相位——Hilbert分析衍生方法及MATLAB实现
- [x] 类EMD的“信号分解方法”及MATLAB实现（第一篇）——EEMD
- [x] 类EMD的“信号分解方法”及MATLAB实现（第二篇）——CEEMD
- [x] 类EMD的“信号分解方法”及MATLAB实现（第三篇）——CEEMDAN
- [x] 类EMD的“信号分解方法”及MATLAB实现（第四篇）——VMD
- [x] 类EMD的“信号分解方法”及MATLAB实现（第五篇）——ICEEMDAN
- [x] 类EMD的“信号分解方法”及MATLAB实现（第六篇）——LMD
- [x] 类EMD的“信号分解方法”及MATLAB实现（第七篇）——EWT
- [x] 类EMD的“信号分解方法”及MATLAB实现（第八篇）——离散小波变换DWT（小波分解）
- [ ] “类EMD”算法分解后要怎样使用（1）——内涵模态分量IMF的方差贡献率、平均周期、相关系数的计算及MATLAB代码实现
- [ ] “类EMD”算法分解后要怎样使用（2）——高频、低频、趋势项分量判别与重构，及MATLAB代码实现
- [ ] 【滤波专题-第7篇】“类EMD”算法分解后要怎样使用（3）——EMD降噪方法及MATLAB代码实现
- [x] 类EMD的“信号分解方法”及MATLAB实现（第九篇）——小波包变换（WPT）/小波包分解（WPD）


## 学习

滤波器设计：

<div class="grid cards" markdown>

-   :material-book:{ .lg .middle } __数字信号处理与滤波器设计 🎯__

    ---

    [:octicons-arrow-right-24: <a href="https://www.youtube.com/watch?v=xPCgjP21Z7E" target="_blank"> 传送门 </a>](#) 

    [:octicons-arrow-right-24: <a href="https://onedrive.live.com/?authkey=%21AJesg4br%2D57KmBo&id=67647B83ED1AAE5F%21262866&cid=67647B83ED1AAE5F" target="_blank"> 配套代码 </a>](#)

</div>

