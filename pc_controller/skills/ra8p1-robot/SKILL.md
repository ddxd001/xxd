---
name: ra8p1-robot
description: 控制本地 RA8P1 全向底盘机器人（LoRa 链路，经本机 Web 上位机后端）——整车状态查询、短定时移动/转向/升降、使能/停用/清故障、屏幕表情与语音事件。当用户要操控"小车 / 底盘 / RA8P1 / 这台机器人"时使用；Box2Robot 云端机械臂是另一套设备（用 box2robot skill），不要混淆。
metadata:
  openclaw:
    requires:
      anyBins: [python3, python]
---

# RA8P1 全向底盘机器人控制（本机）

RA8P1：三轮全向底盘（ID1 前轮、ID2 左后、ID3 右后）+ ID4 升降机构，STS3215 舵机，LoRa 半双工链路。本 skill 通过本机 Web 上位机后端（`web_controller.py`，默认 `http://127.0.0.1:8080`）的 HTTP API 操控，后端独占串口并负责帧序号与安全时序。

## 安全规则（必须遵守）

- **运动只发短定时脉冲**：`move` 时长 ≤3000ms，脉冲结束自动停止；禁止连续滚动发长途移动。
- **ARM（使能）前先口头确认**用户：机器人已架空或周围无障碍。
- `stop` 永远是安全方向，用户说"停/暂停/别动"时立即执行，不必确认。
- 前置条件：后端在运行、串口已连接、遥测新鲜；`state` 返回的错误直接转述给用户。
- 本设备与 Box2Robot 云端机械臂**无关**；用户说"机械臂/左臂/右臂"用 box2robot skill，说"小车/底盘/机器人/升降"用本 skill。
- 升降零点 = 上电位置；ID4 掉线后零点失效，只能断电重启恢复——遇到升降报错要如实告知。

## 命令

```bash
python ra8p1.py state                      # 连接状态 + 整机/舵机/故障遥测快照
python ra8p1.py move --vx 1 --ms 1000      # 前进 1 秒（vx 前+，vy 左+，omega 逆时针+，lift 升+；分量 -1~1）
python ra8p1.py move --omega -1 --ms 500   # 右转 0.5 秒
python ra8p1.py move --lift 1 --ms 2000    # 升降上升 2 秒
python ra8p1.py arm                        # 使能/停用切换（后端按遥测门槛裁决）
python ra8p1.py stop                       # 立即停用（绕过发送槽）
python ra8p1.py clear_fault                # 尝试清除故障
python ra8p1.py face 欢迎回家               # 屏幕表情+语音事件；也接受 0~10 数字代码
```

face 事件名称：待机、启动、欢迎回家、机器人移动中、请缓慢移动、请停止、开始充电、充电完成、检测到儿童、任务完成、安全停靠。

## 常见说法映射

- "前进/后退/左移/右移 N 秒" → `move --vx ±1 --ms N*1000`（vy 同理）
- "左转/右转" → `move --omega ±1`
- "升一点/降一点" → `move --lift ±1 --ms 1000~2000`
- "机器人现在什么状态" → `state`
- "让它说欢迎回家/播充电完成" → `face 欢迎回家` / `face 充电完成`
- "使能/解锁" → `arm`（先确认）；"停" → `stop`

## 环境变量

- `RA8P1_API`：覆盖后端地址（默认 `http://127.0.0.1:8080`）

返回错误时（如"遥测失联"）如实告诉用户并建议检查连接，不要自行重试运动指令。
