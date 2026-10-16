# VL53L8CH 最小测距 Demo

工程基于 正点原子 STM32G474VET6 CubeMX 模板和模块资料中的 VL53LMZ ULD。

## 完成状态

**测距 Demo 已完成，用户于 2026-10-15 确认本阶段验收完成。**

- Compiler 5 构建通过，0 错误、0 警告，已生成并上板运行 HEX。
- 已持续读取并通过串口输出 8×8 距离（mm）、目标状态和各分区目标数，输出间隔约 200 ms（5 Hz）。
- 已收到现场对着约 0.2 m 处电脑屏幕的测距数据。
- 按用户确认的验收口径，后续开发视当前测距结果为准确，不追加精度验证或标定。

## 编译与下载

1. 用 Keil MDK 打开 `MDK-ARM/VL53L8CH_Demo.uvprojx`，选择 `VL53L8CH_Demo` 并按 F7 编译。工程默认使用 ARM Compiler 5.06 update 6。
2. 生成的 HEX 文件位于 `MDK-ARM/Objects/VL53L8CH_Demo.hex`。
3. 用板载下载器下载 HEX。

已补齐 HAL UART 扩展源文件，Compiler 5 构建通过：0 错误、0 警告，并已生成 HEX。构建记录在 `MDK-ARM/build_ac5.log`。

Compiler 6.19 的源文件编译通过，但当前本机在其自带 C 库 `mc_w.l` / `c_wu.l` 中报 `L6907E: Expected an expression`，切换标准 C 库也未消除。若需要使用 Compiler 6，须先处理这套工具链的库脚本错误；记录在 `MDK-ARM/build_ac6.log`。当前使用已通过构建和上板验证的 Compiler 5 版本。

## 接线与串口

- 模块：VCC 接 5 V，GND 共地，SCL 接 PA9，SDA 接 PA8；INT 不接。
- 串口：确认开发板 P3 跳线把 USART1（PB6=TX、PB7=RX）接到板载 CH340，再连接 USB。
- 串口终端：选择 CH340 对应的 COM 口，115200 baud、8 数据位、无校验、1 停止位。

启动成功会显示 `Init OK, module type L8` 和 `Ranging started: 8x8 @ 5 Hz`。随后每帧输出 8×8 距离矩阵（mm）、状态矩阵和每区目标数；状态 5 或 9 表示该目标测距有效。`target_sum` 是 64 个区域目标数的合计。

## 数据使用约定

- `D0`～`D7`：8×8 距离矩阵，单位 mm。
- `S0`～`S7`：与距离矩阵一一对应的目标状态。
- `N0`～`N7`：与距离矩阵一一对应的目标数量。
- 后续仅使用目标数量 `N > 0` 且状态 `S = 5` 或 `S = 9` 的距离值。
- `S = 255` 表示该分区没有检测到目标，对应距离值忽略。

若初始化失败，串口会输出 `ERROR <stage> status=<code>`。请先核对 5 V/共地、PA8/PA9 接线和 P3 串口跳线；ULD 源码及其许可文件在 `VL53LMZ_ULD_API`。
