# EhduBadApple

有像素的地方，就有BAD APPLE！

Bilibili视频链接：[BV1Ua411x7R2](https://www.bilibili.com/video/BV1Ua411x7R2/)

## Overview

用的是杭电的板子，用到的组件有：

- STM32F207VCT6微控制器
- 外部25MHz晶振
- DP83848C以太网PHY接口
- SWD调试下载接口
- 2.8寸TFT LCD显示屏

### 技术方案

采用的方案是在PC端做好视频帧预处理，通过以太网使用UDP协议逐行发送至开发板。开发板使用定时器周期性发送一个请求包，PC接收后立即发送下一行像素信息。应用层数据固定482字节，其中前两字节为对应行数，剩余480字节每两个字节对应一个像素RGB值。

`Convert.py`转换 $320\times240$ 大小JPG格式图像至二进制文件

`Streamer.py`PC端backend，接受开发板请求并发送对应图像像素数据

## RCC

时钟使用25MHz外部高速晶振，使控制器主频达到最大值120MHz，

## TIM

使用通用定时器TIM2，每隔640 $\mu s$ 产生一次中断。

## ETH

以太网使用RMII接口，全双工通信

## FSMC

TFT LCD通过FSMC控制，数据宽度16位

## TCP/IP协议栈

协议栈通过LwIP实现，使用静态IP分配。
