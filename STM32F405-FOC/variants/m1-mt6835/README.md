# 共用固件源码

本目录承载唯一一套应用、FOC 和硬件适配代码。顶层及本目录的默认配置均为 **M1 / TLE5012B / L4225-6S420-A / FREE_SHAFT / 安装 4 / WARN**。目录名来自历史参考工程，M0 和 MT6835 驱动仍可选择。

推荐从[工程根目录](../..)构建。也可在本目录独立运行：

```powershell
cmake --preset Debug
cmake --build --preset Debug
python tests/run_tests.py --cc gcc
```

独立构建输出 `build/longqiu/Debug/405_FOC.elf/.hex/.bin`；顶层构建输出名称为 `STM32F405-FOC`。两者共用默认值和工具链，安装编号一致。修改已有构建配置时，注意 CMake 缓存和显式预设会覆盖默认值。

- [当前架构](../../docs/architecture.md)、[应用层](App/README.md)
- [选择接口与校准](../../docs/unified-firmware.md)、[硬件映射](../../docs/hardware-map.md)
- [主机回归](tests/README.md)、[台架工具](tools/bench/README.md)
- [教程](docs/FOC-从零读懂这个工程.md)、[文件导览](docs/文件导览.md)

TLE5012B 使用原 SPI3/PA0；M1 使用 TIM8 和 PC3/PC2 电流采样，M0 使用 TIM1 和 PC0/PC1。更换磁铁、编码器或相线装配后，须按新的安装身份重新校准并验收。

双路轮腿的足端位置接口、主动杆零偏标定和离线可达域工具见[五连杆足端控制](../../docs/fivebar-foot-control.md)。

日常使用[VOFA轮腿按钮面板](../../docs/wheel-leg-quickstart.md)：回零、滑块/足端目标和机内标定保存，不需要手动发送保持循环。
