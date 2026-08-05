# RA8P1 三轮全向底盘与升降控制

本工程使用 RA8P1 CPU0 和 FSP 6.5.0 裸机状态机，控制 3 个底盘 STS3215 与 1 个升降 STS3215。LoRa 使用 SCI0（115200），URT-2 舵机总线使用 SCI8（1 Mbps），P502/IRQ26 为常闭下原点输入，GPT0 提供 1 ms 时基。

## 硬件连接

- P501/TXD8 → URT-2 `TX`，P500/RXD8 → URT-2 `RX`（按 URT-2 板载丝印同名连接）。
- LoRa TXD → P6-5/P602/RXD0，LoRa RXD → P6-7/P603/TXD0，LoRa 与 RA8P1 共地。
- URT-2 电平开关置 3.3 V，RA8P1 与 URT-2 共地；V/DTR 不接，MCU 工作时不要连接 URT-2 USB。
- 舵机由独立 12 V、至少 15 A 电源供电，并使用保险和可切断 12 V 的急停。
- P502 通过常闭原点开关接地：正常为低，触发或断线为高。

## 舵机一次性配置

先只连接一只舵机，通过 URT-2 工具分别设置并断电确认：ID1 前轮、ID2 左后轮、ID3 右后轮、ID4 升降；波特率 1 Mbps；四只均为 Mode 1 闭环速度模式。固件不会在每次启动时擦写这些 EEPROM 参数。

## 构建与烧录

在 e² studio 中打开 `configuration.xml` 可检查 FSP 外设，若修改配置请点击 **Generate Project Content**。随后选择 Debug 或 Release，执行 **Build Project**。可通过 Debug 自动下载，也可使用 Renesas Flash Programmer 选择生成的 `.srec` 文件直接烧录；Debug 输出为 `Debug/xxd.srec`，Release 输出为 `Release/xxd.srec`。

## PC 控制端

```powershell
cd pc_controller
py -m pip install -r requirements.txt
py controller.py
```

键盘为 WASD 平移、Q/E 旋转、R/F 升降、Enter 切换使能、Space 立即停用。窗口失焦、串口关闭、手柄掉线或程序退出时会发送多帧零速停用命令。首次连接后先完成空载回零，再低速上升至人工确认的安全最高点，点击“捕获当前为软上限”。

## 安全边界

首版没有上限位开关、急停反馈输入和断电防坠。原点输入采用常闭接地，所以断线与触发都会读为高，机械安全不能只依赖软件。首次落地测试必须架空并限制为 20% 速度；升降机构不得在人员附近带载验收。
