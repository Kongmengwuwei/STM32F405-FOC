# 在 VS Code 构建并烧录默认固件

这里的默认固件是 **M0 功率接口 + TLE5012B + ZH3620-1 + 安装编号 1**。工作区目录是 `D:\STM32F405-FOC`，固件的 CMake 工程在其下的 `STM32F405-FOC/`。本机已配置 `Debug-local` 预设及 STM32CubeIDE 的 ARM 工具链。

## 操作步骤

1. 用 VS Code “文件 → 打开文件夹”打开 `D:\STM32F405-FOC`。不要只打开历史名称的 `variants/m1-mt6835/` 子目录。把 ST-Link 的 SWDIO、SWCLK、GND 与目标电压参考接到控制板，给控制板提供合适的逻辑电源；首次烧录时先让电机轴空载，不要直接启动功率输出。
2. 按 **`Ctrl+Shift+B`**，运行默认任务 `STM32: Build Debug`。该任务会先配置再编译，并明确把四项选择设回默认组合。终端中应看到 `FOC build: port=M0, encoder=TLE5012B, motor=ZH3620_1, installation=1` 和最后的内存占用。生成文件是 `STM32F405-FOC/build/Debug/STM32F405-FOC.elf`；同目录还会生成 `.bin`、`.hex`。
3. 确认 ST-Link 已连接且未被其他调试软件占用。按 **`Ctrl+Shift+P`**，选 `Tasks: Run Task`（中文界面为“任务: 运行任务”），再选 **`STM32: Build and flash via ST-Link`**。它会重新确认配置和构建，用 SWD 1 MHz 烧录上述 ELF、校验并复位。终端显示下载和校验成功才算完成。
4. 新固件复位后不会自动驱动电机。先检查 USB 枚举、编码器角度、ADC 零偏和输出关断，再按[ZH3620-1 首测清单](zh3620-1-first-test.md)接限流电源并逐级测试。换磁铁或相线安装后改 `FOC_INSTALLATION_ID`，重新构建和 `cal`；旧 v1/v2 零点不会沿用。

## 图形化调试（F5）

在左侧“运行和调试”中选 **`STM32F405: Debug default firmware (ST-Link)`**，再按绿色三角按钮或 `F5`。本机 `.vscode/launch.json` 已填写板上 STM32F405RGT6 的器件名、内核 `Cortex-M4` 和 Debug ELF 路径；启动前会执行 `STM32: Build Debug`，随后 ST-Link 下载固件并停在 `main`。调试时可以在源码行号旁下断点，查看变量、调用栈和寄存器。调试下载会改写板上程序，必须先确认接线、供电和当前默认组合适合实物。

若弹出 `Device not found: run "Setup STM32Cube project(s)" command or set "deviceName" attribute`，表示 STM32 调试扩展尚未识别器件，错误发生在连接 ST-Link 之前。本机配置已显式指定器件，本机也已安装 STM32F4 CMSIS 设备包 `STMicroelectronics.stm32f4xx_dfp.1.2.0`。若弹出 `ENOENT ... .settings/ide.store.json`，表示扩展把本仓库的 CMake 工程当作已生成 STM32Cube IDE 设置的工程，尝试读取实际上不存在的文件。本机调试配置把 `cwd` 指向工作区根目录，让扩展跳过该工程的自动设置读取；构建仍由 `preLaunchTask` 在正确的 CMake 工程目录执行，ELF 使用明确路径。修改配置后重新打开 VS Code 再试。这里不需要运行 `Setup STM32Cube project(s)` 或重新生成 CubeMX 源码。

也可用 CMake Tools 命令面板选择 `CMake: Select Configure Preset` → `Debug-local`，然后执行 `CMake: Configure`、`CMake: Build`；烧录仍使用上述 ST-Link 任务。若找不到 `Debug-local`，核对本机 `STM32F405-FOC/CMakeUserPresets.json` 与工作区根目录。若构建提示找不到 ARM GCC、Ninja 或 objcopy，检查该预设中的安装路径；若 ST-Link 连接失败，检查板卡逻辑供电、SWDIO/SWCLK/GND/目标电压参考，以及是否有别的程序占用调试器。`CMakeUserPresets.json` 和 `.vscode/tasks.json` 含本机绝对工具路径，按 `.gitignore` 仅保留在本机；换电脑需要重新配置对应路径。

默认任务**只烧录默认组合**。若改用 M1、MT6835 或另一台电机，应在独立构建目录选择配置，检查 ELF 和实际接线，再建立指向该 ELF 的专用烧录任务，不能沿用默认任务。[统一固件指南](unified-firmware.md)解释选项和独立构建方法。当前新版固件只通过编译和主机侧测试，USB 枚举、ADC/SPI 时序及电机输出尚未完成实机验收。

当前固件有手工编写的接口差异，`.ioc` 尚不能完整表达全部条件编译路径。直接重新生成 CubeMX 源码可能覆盖这些代码，修改前需同步核对。
