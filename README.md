# smart-car

基于 RT1064 和逐飞库的摄像头智能车工程。

## 当前源码（2026-11-01）

- 环岛：完整角点或单角点分帧确认，使用当前图像进行受约束补线。
- 十字路口识别与补线；斑马线几何识别已收紧，功能默认关闭。
- 赛道边缘暗障碍绕行：连续确认、侧向避让、出画后保持距离、平滑回正。二值图无法识别红色本身。
- 无线串口 10 Hz 文本输出速度、道路状态和累计里程；图像传输关闭。
- 普通直道/弯道目标为 100/75，绕行上限为 45，单位为编码器计数/10 ms。

红砖保持距离参数位于 [brick_avoid.h](镜头组例程/demo/user/inc/brick_avoid.h)：实测约 11000 计数/米；摄像头到车尾 22 cm，砖长约 27 cm，暂留盲区及余量 33 cm，合计 82 cm（9020 计数）。道路连续恢复后才开始保持计程，同时满足距离与至少 600 ms 后平滑回正。余量及实际车身避让仍需低速实车验证，编码器行程不等于转弯时的精确纵向距离。

## 编译与验证

使用 Keil 打开 [rt1064.uvprojx](镜头组例程/demo/mdk/rt1064.uvprojx)，目标选择 `nor_sdram_zf_dtcm`。当前使用 ARM Compiler 6.21，最近编译为 0 错误、0 警告；尚未完成本次实车验证。

在安装了 Python 的 Visual Studio x64 Native Tools 命令提示符中，可执行：

```text
python tests/test_brick_avoid.py
python tests/test_race_control.py
python tests/test_track_elements.py
python tests/test_zebra_pass.py
python tests/test_wireless_speed.py
```

这些测试检查软件逻辑和模拟硬件下的控制衔接，不能替代实车测试。此次提交更新源码与测试，不更新仓库历史中已跟踪的编译产物；烧录前请重新编译当前源码。

## 依赖与许可

保留逐飞、NXP、CMSIS、FatFs 等原有源码版权及许可证，参见 [逐飞库许可证](镜头组例程/libraries/LICENSE)、[许可说明](镜头组例程/libraries/doc/GPL3_permission_statement.txt)、[CMSIS 许可证](镜头组例程/libraries/sdk/CMSIS/LICENSE.txt) 和 [FatFs 许可证](镜头组例程/libraries/components/fatfs/LICENSE.txt)。`zf_device_config.lib` 是工程引用的上游依赖，需保留。
