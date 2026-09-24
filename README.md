# RTTnew-experience

[中文](#中文) | [English](#english)

<a id="中文"></a>

# 中文

## 项目概览

这是一个面向 STM32H742XIHx 的 RT-Thread 固件工程，包含 AD7606、IMU 与 GO 电机相关应用模块。仓库展示了传感采集、固件组织和电机交互实践；当前仍有已记录的集成问题，不能视为完整产品。

## 它能说明什么

- 将外设应用模块组织在 RT-Thread 工程结构中。
- 项目记录提到电机转子位置与温度读取，以及旋转控制。
- 项目记录报告过一次电机通信丢包率为 7%；该结果仅适用于原测试上下文。
- AD7606 SPI 等待问题仍未解决。

## 工程结构

~~~mermaid
flowchart LR
    APP[AD7606 / IMU / GO 电机应用模块] --> RT[RT-Thread 固件]
    RT --> MCU[STM32H742XIHx]
    MCU --> HW[传感器与电机外设]
~~~

该图概括仓库中的应用、RT-Thread 与目标硬件关系，不代表所有外设已完成联调。

## 目录概览

- applications/：AD7606、IMU、电机与入口应用代码。
- drivers/、libraries/、packages/：驱动与依赖组件。
- rt-thread/：RT-Thread 工程内容与配置。
- linkscripts/：STM32H742XIHx 链接脚本。

## 状态与限制

这是一个历史工程基线。仓库未提供可复现的工具链、接线说明或完整构建步骤；7% 丢包记录和未解决的 AD7606 SPI 问题均予以保留，不据此推断整体性能或完成度。

## 技术栈

C · RT-Thread · STM32H742 · AD7606 · IMU · 电机控制

---

<a id="english"></a>

# English

## Project Overview

This repository is an RT-Thread firmware workspace for the STM32H742XIHx target. It contains application modules for AD7606, an IMU, and a GO motor, showing embedded sensing and control work within an RTOS project. It remains an engineering baseline with documented integration issues.

## What It Demonstrates

- Organizing peripheral applications in an RT-Thread firmware project.
- Existing notes describe motor rotor-position and temperature readout, plus rotation control.
- The notes report 7% packet loss in one motor communication test context; this is not a general performance claim.
- The AD7606 SPI wait issue remains unresolved.

## Architecture

~~~mermaid
flowchart LR
    APP[AD7606 / IMU / GO motor modules] --> RT[RT-Thread firmware]
    RT --> MCU[STM32H742XIHx]
    MCU --> HW[Sensor and motor peripherals]
~~~

This diagram summarizes the repository structure and target relationships; it does not imply that every peripheral integration is complete.

## Repository Structure

- applications/: AD7606, IMU, motor, and entry-point application code.
- drivers/, libraries/, packages/: drivers and supporting components.
- rt-thread/: RT-Thread project content and configuration.
- linkscripts/: STM32H742XIHx linker script.

## Status and Limitations

This is a historical engineering baseline. The repository does not include a reproducible toolchain, wiring guide, or complete build procedure. The reported 7% packet loss and unresolved AD7606 SPI issue are retained as project status; they do not support broader performance or completion claims.

## Technology

C · RT-Thread · STM32H742 · AD7606 · IMU · motor control
