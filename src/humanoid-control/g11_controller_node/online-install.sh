#!/usr/bin/env bash
# ============================================================
# G11 遥控器屏幕 app 在线安装/升级脚本 (纯网络通道, 无需本机 make)
#
# 适用场景: 遥控器上还是原厂默认 APP(仅连 WiFi 用), 还没装我们的 G11
#           机器人控制 APP —— 屏幕上点不到"在线升级"。此时:
#           1) 用户先把遥控器连上 WiFi(用原厂 APP 的 WiFi 配置)
#           2) 在本机执行本脚本, 输入遥控器 IP
#           3) 脚本 ping + 特征校验确认是遥控器后, 从 GitCode
#              拉取最新 G11 app + 两个字体, telnet 部署并重启
#
# 下载通道: GitCode API(v5) + base64(与遥控器端"检查屏幕程序更新"同源,
#           规避 raw 直链反爬), release 分支:
#           wangyangxu/G11-Screen -> version.json / app / zh_cn.ttf /
#                                    LiberationSans-Regular.ttf
#
# 出厂设备适配: 除了 app+字体, 本脚本还会补上 /customer/app.sh 并切换守护, 否则原厂 /bin/app 会被一起拉起
#
# 本机依赖(脚本启动时自动检查, 缺哪个会明确报出并给出安装命令):
#   ping / telnet / python3 / curl / ss+ip(iproute2) / util-linux + 常规 coreutils
#   Ubuntu/Debian: sudo apt-get install -y iputils-ping telnet python3 curl iproute2 util-linux
#   不需要任何 Python 第三方库(只用标准库 json / base64 / http.server)
#
# 用法:
#   ./online-install.sh                  # 运行后询问遥控器 IP; 在线拉取最新并更新缓存
#   ./online-install.sh 1.2.3.4          # 直接带 IP 运行 (同上, 跳过询问)
#   ./online-install.sh --offline        # 离线: 不联网, 直接用上次缓存(需先在线跑过)
#   ./online-install.sh --check          # 只校验设备, 不安装
#   ./online-install.sh --force          # 强制更新: 即使设备 app 已与远程/缓存一致(甚至比远端更新)也照样覆盖重装; 覆盖前会再次询问 y/N
#
# 缓存说明: 每次在线拉取成功后, 会把 app/字库/version.json 存入
#   .online-install-cache/ 目录。之后 --offline 断网部署时直接复用该缓存。
# ============================================================
set -u

# ---------- 常量 ----------
GC_API="https://gitcode.com/api/v5/repos/wangyangxu/G11-Screen/contents"
GC_REF="release"
case "${BASH_SOURCE[0]}" in
    */*) WORK_DIR="$(cd "${BASH_SOURCE[0]%/*}" && pwd)";;
    *)   WORK_DIR="$(pwd)";;
esac
# 持久缓存目录: 在线拉取成功后把 app/字库/version.json 存到这里, 供 --offline 断网复用
CACHE_DIR="$WORK_DIR/.online-install-cache"
CACHE_APP="$CACHE_DIR/app"
CACHE_TTF="$CACHE_DIR/zh_cn.ttf"
CACHE_TTF2="$CACHE_DIR/LiberationSans-Regular.ttf"
CACHE_VER="$CACHE_DIR/version.json"
# 每次运行的临时工作目录(仅本次使用, 结束后清理)
DL_DIR="${TMPDIR:-/tmp}/g11-online-install-$$"
APP_LOCAL="$DL_DIR/app"
TTF_LOCAL="$DL_DIR/zh_cn.ttf"
TTF2_LOCAL="$DL_DIR/LiberationSans-Regular.ttf"
VER_JSON="$DL_DIR/version.json"
HTTP_DIR="$DL_DIR/_http"
HTTP_PORT=""
HTTP_PID=""

ONLY_CHECK=0
OFFLINE=0
FORCE=0
DEVICE_IP=""

# ---------- 退出清理 ----------
# 本脚本会起一个临时 HTTP 服务供遥控器 wget 拉取文件。必须在任何退出路径都把它停掉: 正常结束、报错退出(err/exit)、以及运行中被 ^C 或 kill 打断。
# 否则服务会留在后台占着端口, 多跑几次就把可用端口耗尽
cleanup() {
    if [ -n "$HTTP_PID" ]; then
        kill "$HTTP_PID" 2>/dev/null
        HTTP_PID=""
    fi
    if [ -n "$DL_DIR" ] && [ -d "$DL_DIR" ]; then
        rm -rf "$DL_DIR"
    fi
}
# 收到 INT/TERM 时 bash 默认会继续往下跑(此时临时 HTTP 服务已被 kill, 后续部署步骤必然失败), 故中断 handler 里显式 exit。cleanup 幂等, EXIT 再调一次无害。
on_interrupt() {
    cleanup
    exit 130
}
trap cleanup EXIT
trap on_interrupt INT TERM

# 颜色(仅 tty 输出)
if [ -t 1 ]; then
    C_G="\033[32m"; C_R="\033[31m"; C_Y="\033[33m"; C_B="\033[34m"; C_0="\033[0m"
else
    C_G=""; C_R=""; C_Y=""; C_B=""; C_0=""
fi
ok()  { echo -e "${C_G}[OK]${C_0} $*"; }
warn(){ echo -e "${C_Y}[WARN]${C_0} $*"; }
err() { echo -e "${C_R}[ERROR]${C_0} $*"; }
info(){ echo -e "${C_B}[INFO]${C_0} $*"; }

# ---------- 参数解析 ----------
# (放在依赖检查之前: 这样在缺依赖的机器上 --help 仍能正常打印)
while [ $# -gt 0 ]; do
    case "$1" in
        --check|-c)     ONLY_CHECK=1 ;;
        --offline|-o)   OFFLINE=1 ;;
        --force|-f)     FORCE=1 ;;
        -h|--help)
            # 打印文件头注释(跳过 shebang, 到第 2 条 "# ====" 分隔线结束), 随头部改动自动跟随。
            # 纯 bash 实现(不用 sed): 在缺依赖的机器上 --help 也能正常打印
            _first=1; _sep=0
            while IFS= read -r _line; do
                if [ "$_first" = 1 ]; then _first=0; continue; fi   # 跳过 shebang
                case "$_line" in '# ===='*) _sep=$((_sep+1)) ;; esac
                printf '%s\n' "${_line#\# }"
                [ "$_sep" -ge 2 ] && break                          # 第2条分隔线 = 头部结束
            done < "$0"
            exit 0 ;;
        *)
            if [ -z "$DEVICE_IP" ]; then DEVICE_IP="$1"
            else err "未知参数: $1"; exit 1; fi ;;
    esac
    shift
done

# ---------- 依赖检查 ----------
# 本脚本运行在"工控机/开发机"(不是遥控器)上, 需要以下系统工具。
# 分四档: ① 必需  ② 仅在线模式必需(--offline 不需要)  ③ 基础工具  ④ 可选(脚本内有兜底)
DEP_MISS=""
DEP_OPT_MISS=""
add_miss()     { DEP_MISS="$DEP_MISS
    $1"; }
add_opt_miss() { DEP_OPT_MISS="$DEP_OPT_MISS
    $1"; }
# need_cmd <命令> <用途> [安装包]
need_cmd() {
    command -v "$1" >/dev/null 2>&1 && return 0
    if [ -n "${3:-}" ]; then add_miss "$1  —— $2   [包: $3]"
    else                     add_miss "$1  —— $2"; fi
}
# opt_cmd <命令> <用途> [安装包]
opt_cmd() {
    command -v "$1" >/dev/null 2>&1 && return 0
    add_opt_miss "$1  —— $2   [包: ${3:-?}]"
}

info "检查本机依赖 ..."

# ① 必需 —— 设备探测 + 部署通道
need_cmd ping    "探测遥控器是否在线"                             iputils-ping
need_cmd telnet  "登录遥控器执行部署命令(本脚本唯一的部署通道)"   telnet
need_cmd python3 "解析 GitCode 接口 / base64 解码 / 起临时 HTTP 服务" python3
need_cmd ss      "检测 8000-8099 端口是否被残留服务占用"          iproute2
need_cmd ip      "推断与遥控器同网段的本机源地址"                 iproute2
need_cmd setsid  "在设备端后台拉起守护(远端执行)"                 util-linux

# ② 仅在线模式必需 —— --offline 直接读本地缓存, 不联网
if [ "$OFFLINE" -eq 0 ]; then
    need_cmd curl "从 GitCode 拉取 app / 字库 / version.json" curl
fi

# ③ 基础工具 —— 正常 Linux 都有; 精简容器/裁剪镜像可能缺
for t in awk sed grep cut tr head tail md5sum timeout nohup seq stat; do
    need_cmd "$t" "基础文本处理 / 校验 / 超时控制"
done

# ④ 可选 —— 脚本内有兜底
opt_cmd hostname "ip 取不到地址时兜底取本机 IP" hostname

# ── 能力级检查(命令存在 ≠ 参数/模块可用) ──
# grep 需支持 -P(PCRE): 脚本用它从 `ip route` / `ip addr` 输出里取 src / inet
if command -v grep >/dev/null 2>&1 && \
   ! printf 'src 1.2.3.4\n' | grep -qoP 'src \K[0-9.]+' 2>/dev/null; then
    add_miss "grep -P  —— 需要支持 PCRE 的 GNU grep (解析 ip route / ip addr 输出用)"
fi
# python3 版本 + 脚本用到的标准库模块
if command -v python3 >/dev/null 2>&1; then
    if ! python3 -c 'import sys;sys.exit(0 if sys.version_info>=(3,6) else 1)' >/dev/null 2>&1; then
        add_miss "python3  —— 版本过低, 需要 >= 3.6"
    elif ! python3 -c 'import json,base64,http.server' >/dev/null 2>&1; then
        add_miss "python3  —— 缺少标准库 json / base64 / http.server"
    fi
fi

if [ -n "$DEP_MISS" ]; then
    echo ""
    err "缺少必需依赖, 无法继续:$DEP_MISS"
    echo ""
    echo "  Ubuntu / Debian 一键安装:"
    echo "    sudo apt-get update && sudo apt-get install -y \\"
    echo "        iputils-ping telnet python3 curl iproute2 util-linux coreutils grep sed gawk"
    echo ""
    echo "  其他发行版: 用对应的包管理器安装同名包"
    echo "  提示: 不需要任何 Python 第三方库(pip install), 只用标准库"
    echo ""
    exit 1
fi
if [ -n "$DEP_OPT_MISS" ]; then
    warn "可选依赖缺失(有兜底, 一般不影响):$DEP_OPT_MISS"
fi
ok "依赖检查通过"

# ---------- 1. 设备探测(单台直连) ----------
# 探测目标 IP 是否是可部署的遥控器(app.sh 启动架构 + /customer/app), 并取回设备 app md5
# 注意: 本探测依赖 telnet(23) —— 而 telnet 由设备端 app/守护提供;
#       若探测失败多半是 app 没起来, 提示用户重启遥控器(见调用处)
# 输出到全局: HAS_APP_SH / HAS_CUSTOMER_APP / DEV_MD5
DEV_PROBE='
echo "PROBE_BEGIN"
if [ -f /bin/app.sh ]; then echo "HAS_APP_SH=1"; else echo "HAS_APP_SH=0"; fi
if [ -d /customer ] && [ -f /customer/app ]; then echo "HAS_CUSTOMER_APP=1"; else echo "HAS_CUSTOMER_APP=0"; fi
echo "DEV_MD5_LINE=$(md5sum /customer/app 2>/dev/null)"
echo "PROBE_END"
'
probe_device_info() {
    local ip="$1"
    HAS_APP_SH=""; HAS_CUSTOMER_APP=""; DEV_MD5=""
    if ! ping -c1 -W2 "$ip" >/dev/null 2>&1; then return 1; fi
    local out
    out="$({ sleep 1; echo root; sleep 1; echo ""; sleep 1; echo "stty -echo 2>/dev/null"; echo "$DEV_PROBE"; sleep 5; echo exit; sleep 1; } | timeout 25 telnet "$ip" 2>&1)"
    HAS_APP_SH="$(echo "$out" | grep -o 'HAS_APP_SH=[01]' | head -1 | cut -d= -f2)"
    HAS_CUSTOMER_APP="$(echo "$out" | grep -o 'HAS_CUSTOMER_APP=[01]' | head -1 | cut -d= -f2)"
    # 多行发送时提示符与输出同行(如 "/ # DEV_MD5_LINE=xxx  /customer/app"), 直接按 DEV_MD5_LINE= 提取
    DEV_MD5="$(echo "$out" | tr -d '\r' | grep -oE 'DEV_MD5_LINE=[0-9a-f]{32}' | head -1 | cut -d= -f2)"
    if [ "$HAS_APP_SH" = "1" ] && [ "$HAS_CUSTOMER_APP" = "1" ]; then return 0; fi
    return 1
}

# ---------- 1.5 强制覆盖确认 ----------
# --force 时, 真正覆盖设备 app 前让用户二次确认
# (设备已与远程/缓存一致、或设备可能比远端新 等场景都会被覆盖重装)
confirm_force_overwrite() {
    echo ""
    warn "即将强制覆盖遥控器上的 app:"
    echo "   设备当前 md5 : ${DEV_MD5:-未知}"
    echo "   目标安装 md5 : $APP_MD5  (版本 v${VER:-?}, build ${BUILD_DATE:-?})"
    printf "确认强制安装并覆盖设备上的程序? (y/N): "
    local ans=""
    read -r ans
    case "$ans" in
        y|Y|yes|YES)
            echo ""
            info "用户已确认, 继续强制安装 ..."
            return 0 ;;
        *)
            echo "已取消, 未做任何改动"
            return 1 ;;
    esac
}

# ---------- 2. 确定目标遥控器 (交互询问或直连) ----------
if [ -z "$DEVICE_IP" ]; then
    printf "请输入遥控器的 IP 地址 (例如 10.10.30.60): "
    read -r DEVICE_IP
fi
if ! echo "$DEVICE_IP" | grep -qE '^[0-9]{1,3}(\.[0-9]{1,3}){3}$'; then
    err "IP 格式不正确: '$DEVICE_IP'"; exit 1
fi
info "遥控器 IP: $DEVICE_IP"

echo ""
info "ping + 特征校验 ..."
if ! probe_device_info "$DEVICE_IP"; then
    err "目标 $DEVICE_IP 无法连通或不是遥控器 (特征: /bin/app.sh + /customer/app)"
    echo "   请确认: 1) 遥控器已开机; 2) 遥控器已连上与电脑相同的 WiFi"
    echo "            3) 电脑与遥控器在同一网段 (遥控器原厂APP的WiFi页可查本机IP)"
    echo "   若 ping 得通但仍失败: telnet(23) 由设备端 app/守护提供,"
    echo "   多半是 app 未运行 -> 请长按电源键重启遥控器后重试"
    exit 1
fi
ok "设备特征校验通过 (app.sh 启动架构 + /customer/app 存在)"
echo "   当前设备 /customer/app md5: ${DEV_MD5:-未获取}"

if [ "$ONLY_CHECK" -eq 1 ]; then
    echo ""
    ok "只校验模式: 设备在线且具备遥控器特征, 未做任何改动"
    exit 0
fi

# ---------- 4/5. 获取发布物 (在线拉取并存缓存 / 离线读缓存) ----------
mkdir -p "$DL_DIR" "$CACHE_DIR"

fetch_release_info() {
    if ! curl -sf "$GC_API/version.json?ref=$GC_REF" -o "$DL_DIR/_ver_api.json"; then
        return 1
    fi
    python3 - "$DL_DIR/_ver_api.json" "$VER_JSON" <<INNER
import sys, json, base64
with open(sys.argv[1]) as f:
    d = json.load(f)
with open(sys.argv[2], "wb") as f:
    f.write(base64.b64decode(d["content"]))
INNER
    rm -f "$DL_DIR/_ver_api.json"
    return 0
}

fetch_file() {
    local name="$1" out="$2"
    if ! curl -sf "$GC_API/$name?ref=$GC_REF" -o "$DL_DIR/_f.json"; then
        return 1
    fi
    python3 - "$DL_DIR/_f.json" "$out" <<INNER
import sys, json, base64
with open(sys.argv[1]) as f:
    d = json.load(f)
with open(sys.argv[2], "wb") as f:
    f.write(base64.b64decode(d["content"]))
INNER
    rm -f "$DL_DIR/_f.json"
    return 0
}

read_ver_fields() {
    VER="$(python3 -c "import json;print(json.load(open('$1'))['version'])" 2>/dev/null)"
    GIT_COMMIT="$(python3 -c "import json;print(json.load(open('$1'))['git_commit'])" 2>/dev/null)"
    BUILD_DATE="$(python3 -c "import json;print(json.load(open('$1'))['build_date'])" 2>/dev/null)"
    APP_MD5="$(python3 -c "import json;print(json.load(open('$1'))['md5'])" 2>/dev/null)"
    TTF_MD5="$(python3 -c "import json;print(json.load(open('$1')).get('ttf_md5',''))" 2>/dev/null)"
    TTF2_MD5="$(python3 -c "import json;print(json.load(open('$1')).get('ttf2_md5',''))" 2>/dev/null)"
}

if [ "$OFFLINE" -eq 0 ]; then
    echo ""
    info "从 GitCode 拉取发布信息 (wangyangxu/G11-Screen@release) ..."
    if ! fetch_release_info; then
        err "拉取 version.json 失败 (网络不通或 GitCode 不可达)"
        echo "   如已无网络, 可用上次缓存离线安装: ./online-install.sh --offline $DEVICE_IP"
        exit 1
    fi
    read_ver_fields "$VER_JSON"
    if [ -z "$VER" ] || [ -z "$APP_MD5" ]; then
        err "version.json 解析失败"; exit 1
    fi
    echo "   远程版本 : v$VER (commit $GIT_COMMIT, build $BUILD_DATE)"
    echo "   远程 md5 : app=$APP_MD5${TTF_MD5:+ ttf=$TTF_MD5}${TTF2_MD5:+ ttf2=$TTF2_MD5}"

    if [ -n "$DEV_MD5" ] && [ "$DEV_MD5" = "$APP_MD5" ]; then
        echo ""
        if [ "$FORCE" -eq 1 ]; then
            warn "设备 app md5 与远程一致 (--force 强制模式), 将照样重新下载并覆盖安装"
        else
            ok "设备 /customer/app md5 已与远程一致, 已是最新版本, 无需安装"
            echo "   如仍要强制重装, 请加 --force: ./online-install.sh --force $DEVICE_IP"
            rm -rf "$DL_DIR"
            exit 0
        fi
    fi

    echo ""
    info "下载 app ($APP_MD5) ..."
    if ! fetch_file "app" "$APP_LOCAL"; then
        err "下载 app 失败"; exit 1
    fi
    chmod +x "$APP_LOCAL"
    if ! head -c4 "$APP_LOCAL" | grep -q $'\x7fELF'; then
        err "下载的 app 不是 ELF 可执行文件 (可能拉取失败), 已中止"; exit 1
    fi
    APP_MD5_LOCAL="$(md5sum "$APP_LOCAL" | awk '{print $1}')"
    if [ "$APP_MD5_LOCAL" != "$APP_MD5" ]; then
        err "app md5 不符: 远程=$APP_MD5 实际=$APP_MD5_LOCAL, 已中止"; exit 1
    fi
    ok "app 下载完成 md5=$APP_MD5_LOCAL ($(stat -c%s "$APP_LOCAL") bytes)"

    TTF_MD5_LOCAL=""
    TTF2_MD5_LOCAL=""
    if [ -n "$TTF_MD5" ]; then
        echo ""
        info "下载字库 zh_cn.ttf ($TTF_MD5) ..."
        if ! fetch_file "zh_cn.ttf" "$TTF_LOCAL"; then
            warn "下载字库失败, 将仅更新 app (旧字库保留)"
        else
            TTF_MD5_LOCAL="$(md5sum "$TTF_LOCAL" | awk '{print $1}')"
            if [ "$TTF_MD5_LOCAL" = "$TTF_MD5" ]; then
                ok "字库下载完成 md5=$TTF_MD5_LOCAL"
            else
                warn "字库 md5 不符(远程=$TTF_MD5 实际=$TTF_MD5_LOCAL), 忽略字库更新"
                TTF_MD5_LOCAL=""
            fi
        fi
    fi

    # 西文字库(app 启动必需, 缺则段错误)
    if [ -n "$TTF2_MD5" ]; then
        echo ""
        info "下载西文字库 LiberationSans-Regular.ttf ($TTF2_MD5) ..."
        if ! fetch_file "LiberationSans-Regular.ttf" "$TTF2_LOCAL"; then
            warn "下载西文字库失败 (设备缺该字体会启动闪退)"
        else
            TTF2_MD5_LOCAL="$(md5sum "$TTF2_LOCAL" | awk '{print $1}')"
            if [ "$TTF2_MD5_LOCAL" = "$TTF2_MD5" ]; then
                ok "西文字库下载完成 md5=$TTF2_MD5_LOCAL"
            else
                warn "西文字库 md5 不符(远程=$TTF2_MD5 实际=$TTF2_MD5_LOCAL), 忽略"
                TTF2_MD5_LOCAL=""
            fi
        fi
    fi

    echo ""
    info "更新本地缓存 -> $CACHE_DIR"
    cp -f "$VER_JSON" "$CACHE_VER"
    cp -f "$APP_LOCAL" "$CACHE_APP"
    if [ -n "$TTF_MD5_LOCAL" ]; then cp -f "$TTF_LOCAL" "$CACHE_TTF"; else rm -f "$CACHE_TTF"; fi
    if [ -n "$TTF2_MD5_LOCAL" ]; then cp -f "$TTF2_LOCAL" "$CACHE_TTF2"; else rm -f "$CACHE_TTF2"; fi
    sync 2>/dev/null
    APP_LOCAL="$CACHE_APP"; TTF_LOCAL="$CACHE_TTF"; TTF2_LOCAL="$CACHE_TTF2"; VER_JSON="$CACHE_VER"
else
    echo ""
    info "离线模式: 使用本地缓存 ($CACHE_DIR)"
    if [ ! -f "$CACHE_VER" ] || [ ! -f "$CACHE_APP" ]; then
        err "未找到缓存 (需先在线执行一次: ./online-install.sh $DEVICE_IP)"
        exit 1
    fi
    read_ver_fields "$CACHE_VER"
    if [ -z "$VER" ] || [ -z "$APP_MD5" ]; then
        err "缓存 version.json 损坏"; exit 1
    fi
    echo "   缓存版本 : v$VER (commit $GIT_COMMIT, build $BUILD_DATE)"
    echo "   缓存 md5 : app=$APP_MD5${TTF_MD5:+ ttf=$TTF_MD5}${TTF2_MD5:+ ttf2=$TTF2_MD5}"

    if ! head -c4 "$CACHE_APP" | grep -q $'\x7fELF'; then
        err "缓存 app 不是 ELF, 缓存损坏, 请重新在线拉取"; exit 1
    fi
    if [ "$(md5sum "$CACHE_APP" | awk '{print $1}')" != "$APP_MD5" ]; then
        err "缓存 app md5 与缓存 version.json 不一致, 缓存损坏, 请重新在线拉取"; exit 1
    fi
    APP_MD5_LOCAL="$APP_MD5"
    TTF_MD5_LOCAL=""
    TTF2_MD5_LOCAL=""
    if [ -n "$TTF_MD5" ] && [ -f "$CACHE_TTF" ] &&
       [ "$(md5sum "$CACHE_TTF" | awk '{print $1}')" = "$TTF_MD5" ]; then
        TTF_MD5_LOCAL="$TTF_MD5"
    elif [ -n "$TTF_MD5" ]; then
        warn "缓存字库缺失或损坏, 将仅更新 app"
    fi
    if [ -n "$TTF2_MD5" ] && [ -f "$CACHE_TTF2" ] &&
       [ "$(md5sum "$CACHE_TTF2" | awk '{print $1}')" = "$TTF2_MD5" ]; then
        TTF2_MD5_LOCAL="$TTF2_MD5"
    elif [ -n "$TTF2_MD5" ]; then
        warn "缓存西文字库缺失或损坏 (设备缺该字体会启动闪退)"
    fi

    if [ -n "$DEV_MD5" ] && [ "$DEV_MD5" = "$APP_MD5" ]; then
        echo ""
        if [ "$FORCE" -eq 1 ]; then
            warn "设备 app md5 与缓存一致 (--force 强制模式), 将照样用缓存覆盖安装"
        else
            ok "设备 /customer/app md5 已与缓存一致, 无需安装"
            echo "   如仍要强制重装, 请加 --force: ./online-install.sh --force $DEVICE_IP"
            rm -rf "$DL_DIR"
            exit 0
        fi
    fi
    APP_LOCAL="$CACHE_APP"; TTF_LOCAL="$CACHE_TTF"; TTF2_LOCAL="$CACHE_TTF2"
fi

# ---------- 6. 本机临时 HTTP 服务 (遥控器经 telnet 拉文件) ----------
# 遥控器上没有 curl/https 可靠可用, 沿用 deploy.sh 方案: 本机 http.server + wget
# 端口从 8000 起自动找空闲端口(避免与 deploy.sh 的 8000 服务冲突而拉错目录文件)

echo ""
info "准备本机 HTTP 临时服务 (自动选空闲端口) ..."
mkdir -p "$HTTP_DIR"
cp -f "$APP_LOCAL" "$HTTP_DIR/app"
# 需要下发的字体清单(两个字体都不能缺)
FONTS_TO_PUSH=""
if [ -n "$TTF_MD5_LOCAL" ] && [ -f "$TTF_LOCAL" ]; then
    cp -f "$TTF_LOCAL" "$HTTP_DIR/zh_cn.ttf"
    FONTS_TO_PUSH="$FONTS_TO_PUSH zh_cn.ttf"
fi
if [ -n "$TTF2_MD5_LOCAL" ] && [ -f "$TTF2_LOCAL" ]; then
    cp -f "$TTF2_LOCAL" "$HTTP_DIR/LiberationSans-Regular.ttf"
    FONTS_TO_PUSH="$FONTS_TO_PUSH LiberationSans-Regular.ttf"
fi

HTTP_PORT=""
for p in $(seq 8000 8099); do
    if ! ss -tln 2>/dev/null | grep -q ":$p "; then
        HTTP_PORT="$p"
        break
    fi
done
if [ -z "$HTTP_PORT" ]; then
    err "未找到可用 HTTP 端口 (8000-8099 都被占用)"
    echo "     占用者:" >&2
    ss -tlnp 2>/dev/null | grep -oP ':80[0-9][0-9]\s.*' | head -10 >&2
    echo "     多为此脚本/ deploy.sh 以前残留的 http.server, 可用下面命令清理:" >&2
    echo "       pkill -f 'python3 -m http.server'" >&2
    exit 1
fi

# 启动临时 HTTP 服务; 通过子 shell 的 stdout 拿真实子进程 PID
# (不用 /tmp 暂存文件: PID 可能被复用, 且少一个需要清理的临时文件)
HTTP_PID="$(
    cd "$HTTP_DIR" || exit 1
    nohup python3 -m http.server "$HTTP_PORT" >/dev/null 2>&1 &
    echo $!
)"
sleep 1
if [ -z "$HTTP_PID" ] || ! ss -tln 2>/dev/null | grep -q ":$HTTP_PORT "; then
    err "HTTP 服务启动失败 (端口 $HTTP_PORT)"
    exit 1
fi

# 本机 IP 必须选"遥控器能访问到"的地址, 优先级:
#   1) **路由法(首选)**: `ip route get <设备IP>` 取内核选用的源地址
#      (自动适配 /23 等任意掩码与多网卡)
#   2) 同 /24 匹配 -> hostname -I 兜底
DEV_NET="$(echo "$DEVICE_IP" | cut -d. -f1-3)"
ROUTE_LINE="$(ip route get "$DEVICE_IP" 2>/dev/null | head -1)"
ROUTE_SRC="$(echo "$ROUTE_LINE" | grep -oP 'src \K[0-9.]+' | head -1)"
if [ -n "$ROUTE_SRC" ]; then
    LOCAL_IP="$ROUTE_SRC"
    if echo "$ROUTE_LINE" | grep -q ' via '; then
        warn "到设备需经网关, 设备可能不在同一网段; 若拉取失败请检查网络"
    fi
else
    LOCAL_IP="$(ip -4 addr show 2>/dev/null | grep -oP 'inet \K[0-9.]+' | grep -v '^127\.' | grep -F "$DEV_NET." | head -1)"
    if [ -z "$LOCAL_IP" ]; then
        LOCAL_IP="$(hostname -I 2>/dev/null | awk '{print $1}')"
    fi
    if [ -z "$LOCAL_IP" ]; then
        err "无法获取本机 IP"; exit 1
    fi
    warn "内核路由未给出源地址, 设备可能不在同一网络 (回退选用 $LOCAL_IP)"
fi
ok "本机 HTTP 服务 http://$LOCAL_IP:$HTTP_PORT (目录 $HTTP_DIR, 设备网段 $DEV_NET.x)"

# ---------- 7. telnet 部署 ----------
# --force: 覆盖前二次确认 (同版本重装 / 设备可能比远端新而被回退 等场景)
if [ "$FORCE" -eq 1 ] && ! confirm_force_overwrite; then
    info "已取消安装; 临时 HTTP 服务与目录将由退出钩子自动清理"
    exit 0
fi

echo ""
info "通过 telnet 部署到遥控器 $DEVICE_IP ..."
# 与 deploy.sh 一致的原子替换: 先下载 .new -> md5 校验 -> mv 覆盖 -> 重启
# 只 killall app(不杀守护, 由守护拉起新程序)
TELNET_CMD='
# (1) 补守护脚本: 出厂设备没有 /customer/app.sh, 而开机链会退回 /bin/app.sh
#     (它监控原厂 /bin/app) -> 原厂 app 被一起拉起。
if [ ! -f /customer/app.sh ]; then
    sed "s|^APP_PATH=\"/bin/app\"|APP_PATH=\"/customer/app\"|" /bin/app.sh > /customer/app.sh 2>/dev/null
    chmod +x /customer/app.sh
    echo "=== /customer/app.sh provisioned ==="
fi
# (2) 当前守护若还是 /bin/app.sh(监控原厂 app), 换成 /customer/app.sh
if ps | grep -v grep | grep -q "/bin/app.sh"; then
    killall app.sh 2>/dev/null
    sleep 1
    setsid /customer/app.sh >/dev/null 2>&1 < /dev/null &
    echo "=== guard switched to /customer/app.sh ==="
fi

echo "downloading app ..."
rm -f /customer/app.new
wget -q -T 10 http://'"$LOCAL_IP"':'"$HTTP_PORT"'/app -O /customer/app.new || { echo "DOWNLOAD FAIL"; exit 1; }
chmod +x /customer/app.new
if [ "$(md5sum /customer/app.new | awk "{print \$1}")" = "'"$APP_MD5"'" ]; then echo "MD5 OK, replacing..."; mv -f /customer/app.new /customer/app; sync; else echo "MD5 MISMATCH!"; rm -f /customer/app.new; exit 1; fi
'
if [ -n "$FONTS_TO_PUSH" ]; then
    TELNET_CMD="$TELNET_CMD
mkdir -p /customer/assest
for f in $FONTS_TO_PUSH; do
    echo \"downloading \$f ...\"
    rm -f /customer/assest/\$f.new
    wget -q -T 10 http://$LOCAL_IP:$HTTP_PORT/\$f -O /customer/assest/\$f.new || { echo \"\$f DOWNLOAD FAIL\"; continue; }
    mv -f /customer/assest/\$f.new /customer/assest/\$f
    echo \"\$f ok\"
done
sync
"
fi
TELNET_CMD="$TELNET_CMD
echo \"=== device md5 ===\"
md5sum /customer/app /customer/assest/zh_cn.ttf /customer/assest/LiberationSans-Regular.ttf
killall app 2>/dev/null
sleep 4
if ps -ef | grep -v grep | grep -q \"/customer/app\"; then echo \"=== guard restarted app ===\"; else echo \"=== guard did not restart, start manually ===\"; if [ -x /customer/app.sh ]; then setsid /customer/app.sh >/dev/null 2>&1 < /dev/null & else setsid /bin/app.sh >/dev/null 2>&1 < /dev/null & fi; fi
echo \"=== restart done ===\"
"

OUT="$({ sleep 1; echo "root"; sleep 1; echo ""; sleep 1; echo "stty -echo 2>/dev/null"; echo "$TELNET_CMD"; sleep 20; echo "exit"; sleep 1; } | timeout 45 telnet "$DEVICE_IP" 2>&1)"
echo "$OUT" | grep -vE '^Trying |^Connected |Escape character|^Password:|login:|^$|stty -echo' | tail -15

# ---------- 8. 验证 ----------
sleep 6
# telnet(23) 由设备端 app/守护提供; 不可达说明 app 没起来(需重启遥控器)
PROC_OK=0
if timeout 5 bash -c "exec 3<>/dev/tcp/$DEVICE_IP/23" 2>/dev/null; then
    PROC_OK="$( { sleep 1; echo "root"; sleep 1; echo ""; sleep 1; echo "stty -echo 2>/dev/null"; echo "ps -ef | grep -v grep | grep /customer/app"; sleep 2; echo "exit"; sleep 1; } | timeout 20 telnet "$DEVICE_IP" 2>&1 | grep -c '/customer/app')"
fi

echo ""
echo "======================================================"
NEW_MD5="$(echo "$OUT" | tr -d '\r' | grep -E '  /customer/app$' | grep -oE '[0-9a-f]{32}' | head -1)"
if [ -n "$NEW_MD5" ] && [ "$NEW_MD5" = "$APP_MD5" ]; then
    ok "设备端 app md5 一致: $NEW_MD5"
else
    warn "设备端 app md5: '${NEW_MD5:-未取到}' (远程: $APP_MD5)"
fi
if [ "$PROC_OK" -ge 1 ]; then
    ok "/customer/app 进程已运行, 部署成功!"
elif timeout 5 bash -c "exec 3<>/dev/tcp/$DEVICE_IP/23" 2>/dev/null; then
    warn "telnet 可达, 但未检测到 /customer/app 进程, 请查看上方设备输出"
else
    echo "[ERROR] 部署后遥控器 telnet(23) 不可达 —— 设备端 app 未成功启动!"
    echo "        >>> 请长按电源键重启遥控器, 待屏幕正常显示后重新执行本脚本 <<<"
fi
echo "======================================================"
echo ""
info "G11 app v$VER 安装完成. 请到遥控器屏幕确认效果。"
info "若遥控器屏幕仍显示原厂 WiFi APP, 请长按电源键重启遥控器一次。"

# ---------- 9. 清理 ----------
# 临时 HTTP 服务与临时目录由文件头的 trap cleanup 统一收尾 —— 它覆盖正常结束、报错退出、以及 ^C/kill 中断三种情况, 不再需要在此手工清理。
