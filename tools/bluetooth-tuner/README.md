# T3 蓝牙调参工具（历史版本）

这个 Windows 上位机曾用于静止换点任务的参数读取、单项修改和运行状态查看。公开内容包括 Tkinter 界面、串口请求队列及二进制协议解析，可用于了解调参交互如何组织。

**当前主控已移除配套蓝牙服务，本工具不能与当前固件握手或调参。** UART0 的 PB0/PB1 和 9600 配置仍保留，但 T3 CSV 日志也已由 `TASK3_USB_TTL_LOG_ENABLED=0` 停用，当前没有蓝牙或 CSV 业务收发。现行系统见[主控说明](../../firmware/H_BallBalance_MSPM0G3507/README.md)。

## 工具保留了什么

| 功能 | 历史交互方式 |
| --- | --- |
| 连接与握手 | 选择 Windows COM 口，按序号匹配从机响应 |
| 参数管理 | 读取 11 项参数，按单项写入，再显示从机回传的实际值 |
| 状态显示 | 请求 100 ms 状态流，显示球位、球速、T3 阶段及执行器状态 |
| 通信处理 | 校验 CRC、识别响应，600 ms 未收到匹配响应时重试，最多 2 次 |

参数包括 Kp、Kd、启动角、PD 接管位置、预测时间、助推进入/退出速度、软角度上下限、步进速度和加速度档。状态字段还包含控制目标角、X42 命令/测量位置、P/N/V 结果及通信错误计数。

## 打开历史界面

需要带 Tkinter 的 Python 3。进入本目录后运行：

```powershell
python -m pip install -r .\requirements.txt
python .\bluetooth_tuner.py
```

上述命令可以打开界面；要得到有效握手和状态数据，还需要实现同一历史协议的设备端。仓库中的当前主控不提供该服务，单独配对蓝牙不会恢复调参能力。

历史接线为蓝牙模块 TX 接 TI PB1、RX 接 PB0，双方共地，串口使用 9600、8N1。该接法仅解释旧系统结构，不是当前主控的启用步骤。

## 历史协议

```text
B5 62 | version | command | sequence | length | payload | CRC16-CCITT
```

最大载荷 32 字节。CRC 覆盖 `version` 至 `payload`，初值 `0xFFFF`、多项式 `0x1021`，低字节先发；响应沿用请求序号。

| 命令 | 方向 | 说明 |
| --- | --- | --- |
| `0x01` | 上位机 → TI | 握手 |
| `0x10` | 上位机 → TI | 读取参数 ID |
| `0x11` | 上位机 → TI | 写参数 ID 和 int32 定点值 |
| `0x20` | 上位机 → TI | 读取一次状态 |
| `0x21` | 上位机 → TI | 开关周期状态流，周期 100～1000 ms |
| `0x81/0x90/0xA0/0xA1` | TI → 上位机 | 信息、参数、状态、确认 |
| `0xE0` | TI → 上位机 | 错误响应 |

界面与请求调度见 [bluetooth_tuner.py](bluetooth_tuner.py)，帧编解码见 [bluetooth_protocol.py](bluetooth_protocol.py)，协议测试见 [test_bluetooth_protocol.py](test_bluetooth_protocol.py)。这些文件说明工具的历史实现，不构成与当前固件联调通过的证据。

[项目阶段记录](../../docs/source-notes)中关于蓝牙、串口日志的“当前”“下一步”和 AI 交接要求属于原调试阶段；阅读本工具时以这里的停用状态为准。
