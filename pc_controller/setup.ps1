# RA8P1 项目新电脑一键部署脚本（PC 上位机 + 聊天助手环境）
#
# 用法：在仓库根目录打开 PowerShell：
#   powershell -ExecutionPolicy Bypass -File pc_controller\setup.ps1
#
# 覆盖范围：Python 依赖、Git for Windows、Kimi Code CLI、两个聊天技能、聊天工作目录。
# 不覆盖（脚本会提示）：Kimi Code 登录、Box2Robot 账号登录、CH340 驱动、固件烧录。

$ErrorActionPreference = "Stop"
[Net.ServicePointManager]::SecurityProtocol = [Net.SecurityProtocolType]::Tls12

function Write-Step($t) { Write-Host "`n==> $t" -ForegroundColor Cyan }
function Update-SessionPath {
    $env:Path = [System.Environment]::GetEnvironmentVariable("Path", "Machine") + ";" +
                [System.Environment]::GetEnvironmentVariable("Path", "User")
}

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path   # pc_controller\
$KimiHome  = Join-Path $env:USERPROFILE ".kimi-code"
$SkillsDir = Join-Path $KimiHome "skills"

# ---------- 1. Python ----------
Write-Step "检查 Python"
if (-not (Get-Command python -ErrorAction SilentlyContinue)) {
    Write-Host "未找到 Python，使用 winget 安装 Python 3.12 ..."
    winget install -e --id Python.Python.3.12 --accept-package-agreements --accept-source-agreements
    Update-SessionPath
}
python --version

# ---------- 2. Python 依赖（pyserial / aiohttp） ----------
Write-Step "安装 Python 依赖"
python -m pip install --upgrade pip
python -m pip install -r (Join-Path $ScriptDir "requirements.txt")

# ---------- 3. Git for Windows（Kimi Code CLI 的 shell 环境） ----------
Write-Step "检查 Git for Windows"
if (-not (Get-Command git -ErrorAction SilentlyContinue)) {
    Write-Host "未找到 Git，使用 winget 安装 ..."
    winget install -e --id Git.Git --accept-package-agreements --accept-source-agreements
    Update-SessionPath
}
git --version

# ---------- 4. Kimi Code CLI（聊天助手的运行时） ----------
Write-Step "检查 Kimi Code CLI"
$KimiBin = Join-Path $KimiHome "bin\kimi.exe"
if ((Get-Command kimi -ErrorAction SilentlyContinue) -or (Test-Path $KimiBin)) {
    Write-Host "已安装：$(& $KimiBin --version)"
} else {
    Write-Host "使用官方脚本安装 Kimi Code CLI ..."
    Invoke-RestMethod https://code.kimi.com/kimi-code/install.ps1 | Invoke-Expression
    Update-SessionPath
    if (Test-Path $KimiBin) { $env:Path += ";$(Split-Path $KimiBin)" }
    & $KimiBin --version
}
if (-not (Test-Path (Join-Path $KimiHome "config.toml"))) {
    Write-Host "`n[需要手动] Kimi Code 尚未登录：运行 kimi，输入 /login 完成授权（只需一次）。" -ForegroundColor Yellow
}

# ---------- 5. 聊天技能（用户级，所有项目可用） ----------
Write-Step "安装聊天技能到 $SkillsDir"
New-Item -ItemType Directory -Force $SkillsDir | Out-Null

# 两个技能都从仓库复制：ra8p1-robot（控制 RA8P1 小车）、box2robot-skills（控制 Box2Robot 机械臂）
Copy-Item -Recurse -Force (Join-Path $ScriptDir "skills\ra8p1-robot") $SkillsDir
Copy-Item -Recurse -Force (Join-Path $ScriptDir "skills\box2robot-skills") $SkillsDir
Write-Host "  ra8p1-robot、box2robot-skills 已从仓库安装"
if (-not (Test-Path (Join-Path $env:USERPROFILE ".b2r_token"))) {
    Write-Host "`n[需要手动] Box2Robot 未登录：python `"$SkillsDir\box2robot-skills\b2r.py`" login <用户名> <密码>" -ForegroundColor Yellow
}

# ---------- 6. 聊天会话工作目录 ----------
New-Item -ItemType Directory -Force (Join-Path $KimiHome "b2r-chat") | Out-Null

# ---------- 完成 ----------
Write-Step "部署完成"
Write-Host @"
启动上位机：
  cd pc_controller
  python web_controller.py
  浏览器打开 http://127.0.0.1:8080/

注意事项：
  - LoRa 串口是 CH340/CH343 USB 适配器，Win10/11 一般自动装驱动；识别不到时手动装 CH341SER 驱动
  - 机械臂聊天控制需要 Box2Robot 账号（见上面的登录提示）
  - 聊天助手依赖 Kimi Code CLI 已登录（见上面的登录提示）
  - 固件烧录不在本脚本范围（需要 e2 studio + J-Link，见 README）
"@ -ForegroundColor Green
