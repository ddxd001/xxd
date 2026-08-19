"""一次性联调脚本：模拟浏览器给运行中的 web_controller.py 发一条聊天消息。

用法：先启动 `python web_controller.py`，再运行本脚本。
发送的消息只触发只读的 state 查询，不会产生任何运动。
"""

import asyncio
import json
import sys

import aiohttp


async def main() -> int:
    text = sys.argv[1] if len(sys.argv) > 1 else "机器人现在什么状态？"
    async with aiohttp.ClientSession() as session:
        async with session.ws_connect("ws://127.0.0.1:8080/ws") as ws:
            await ws.send_json({"type": "chat", "text": text})
            print(f">> 已发送: {text}")
            async for msg in ws:
                if msg.type != aiohttp.WSMsgType.TEXT:
                    continue
                event = json.loads(msg.data)
                kind = event.get("type")
                if kind == "chat_delta":
                    print(f"[回复] {event['text']}")
                elif kind == "chat_tool":
                    print(f"[工具] {event['text']}")
                elif kind == "chat_status":
                    print(f"[状态] {event['text']}")
                elif kind == "chat_done":
                    print(">> 完成")
                    return 0
    return 1


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
