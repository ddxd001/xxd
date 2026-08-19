"""一次性联调脚本：触发一键任务并打印全部任务事件（演示模式验证用）。

用法：先启动 `python web_controller.py`，再运行本脚本。不触发实物运动。
"""

import asyncio
import json
import sys

import aiohttp


async def main() -> int:
    t0 = asyncio.get_event_loop().time()
    def stamp() -> str:
        return f"{asyncio.get_event_loop().time() - t0:6.1f}s"
    async with aiohttp.ClientSession() as session:
        async with session.ws_connect("ws://127.0.0.1:8080/ws") as ws:
            await ws.send_json({"type": "start_task"})
            print(">> 已发送 start_task")
            async for msg in ws:
                if msg.type != aiohttp.WSMsgType.TEXT:
                    continue
                event = json.loads(msg.data)
                kind = event.get("type")
                if kind == "task_status":
                    live = "实物" if event.get("live") else "演示"
                    print(f"[{stamp()}][状态] {event.get('state')} step={event.get('step')}/{event.get('total')} "
                          f"{event.get('label', '')} ({live})")
                    if event.get("state") in ("done", "cancelled", "error"):
                        print(">> 任务结束")
                        return 0 if event.get("state") == "done" else 2
                elif kind == "task_voice":
                    print(f"[{stamp()}][语音] {event.get('label')} (code=0x{event.get('code', 0):02X}, "
                          f"{'已下发' if event.get('live') else '演示未下发'})")
                elif kind == "task_motion":
                    print(f"[{stamp()}][运动] vx={event.get('vx')} omega={event.get('omega')} ms={event.get('ms')}")
                elif kind == "task_note":
                    print(f"[{stamp()}][提示] {event.get('text')}")
    return 1


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
