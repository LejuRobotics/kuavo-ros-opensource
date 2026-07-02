#!/bin/bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
HOTSPOT_PASSWORD="kuavo123456"
HOTSPOT_SSID="kuavo-$(hostname)的热点"

# 根据当前所在分支决定 kuavo-tools 通道：
#   master → release, beta → beta, dev → dev, 其它（含 opensource.*、判不出）→ beta（兜底）
# git-clone 安装：看 .git；deb 安装：看 package_ws_deb 在打包时写入的 .version/GIT_BRANCH
_detect_kuavo_tools_channel() {
  local ws_root branch="" current r mb ts best_ts=0 best=""

  # 脚本位置往上两级即工作区根（git-clone 装时是仓库根；deb 装时是 /opt/ros/leju）
  ws_root="$(cd "$SCRIPT_DIR/../.." && pwd)"

  if [ -d "$ws_root/.git" ]; then
    # 1) 优先：HEAD 直接在 dev/beta/master 任一分支上，直接用
    #    （forward-merge 之后 dev tip 的 commit 同时也在 beta/master 历史里，
    #     用 branch-name 而不是 ancestry 才能区分清楚）
    current=$(git -C "$ws_root" branch --show-current 2>/dev/null)
    case "$current" in
      master|beta|dev) branch="$current" ;;
    esac

    # 2) detached HEAD 或 feature 分支：找与 HEAD 的 merge-base 时间最晚的那条
    #    （= HEAD 最近一次从哪条分支岔出来；解决「feature off dev」误判为 beta 的问题）
    if [ -z "$branch" ]; then
      for r in dev beta master origin/dev origin/beta origin/master; do
        git -C "$ws_root" rev-parse --verify --quiet "$r" >/dev/null || continue
        mb=$(git -C "$ws_root" merge-base HEAD "$r" 2>/dev/null) || continue
        [ -z "$mb" ] && continue
        ts=$(git -C "$ws_root" log -1 --format=%ct "$mb" 2>/dev/null) || continue
        if [ "$ts" -gt "$best_ts" ]; then
          best_ts=$ts
          best=${r#origin/}
        fi
      done
      branch="$best"
    fi
  elif [ -f "$ws_root/.version/GIT_BRANCH" ]; then
    # deb 场景：package_ws_deb 已经写好
    branch=$(cat "$ws_root/.version/GIT_BRANCH" 2>/dev/null)
    case "$branch" in
      master|beta|dev) ;;
      *) branch="" ;;
    esac
  fi

  case "$branch" in
    master) echo "release" ;;
    beta)   echo "beta" ;;
    dev)    echo "dev" ;;
    *)      echo "beta" ;;
  esac
}

install_kuavo_tools() {
  echo -e "\033[32m\n✅ 开始安装 Kuavo 配网工具...\n\033[0m"

  local kuavo_tools_install_url
  if [ -n "${KUAVO_TOOLS_INSTALL_URL:-}" ]; then
    # 显式覆盖：调用方指定的 URL 优先级最高（应急 / 测试链接用）
    kuavo_tools_install_url="$KUAVO_TOOLS_INSTALL_URL"
    echo -e "\033[32m🔧 使用 KUAVO_TOOLS_INSTALL_URL=${kuavo_tools_install_url}\033[0m"
  else
    local channel
    channel=$(_detect_kuavo_tools_channel)
    kuavo_tools_install_url="https://kuavo.lejurobot.com/kuavo-tools/install-${channel}.sh"
    echo -e "\033[32m🔎 自检通道: ${channel} → ${kuavo_tools_install_url}\033[0m"
  fi

  local pipefail_state
  pipefail_state=$(set -o | awk '$1 == "pipefail" {print $2}')
  set -o pipefail

  if curl -fsSL "$kuavo_tools_install_url" | bash; then
    [ "$pipefail_state" = "off" ] && set +o pipefail
    echo -e "\033[32m✅ Kuavo 配网工具安装完成\033[0m"
  else
    [ "$pipefail_state" = "off" ] && set +o pipefail
    echo -e "\033[31m❌ Kuavo 配网工具安装失败，请检查网络或安装日志\033[0m"
    exit 1
  fi
}

echo -e "\033[32m\n🚀🚀🚀 开始安装...\n\033[0m"

# @@@ INSTALL
sudo apt update
wget https://kuavo.lejurobot.com/statics/linux-wifi-hotspot_4.7.2_amd64.deb
sudo dpkg -i linux-wifi-hotspot_4.7.2_amd64.deb || true
sudo apt install -f || true
rm linux-wifi-hotspot_4.7.2_amd64.deb
dpkg -l | grep linux-wifi-hotspot || { echo -e "\e[31m❌ Failed to install linux-wifi-hotspot\e[0m"; exit 1; }


# @@@ CREATE HOTSPOT
wlan0=$(iw dev | awk '$1=="Interface" && $2 !~ /^(ap|lo|docker|veth)/{print $2; exit}')
supported_ap_mode=$(iw list | grep -A 20 'Supported interface modes' | grep '* AP')
if [ -n "$supported_ap_mode" ] && [ -n "$wlan0" ]; then
  echo -e "\033[32m\n✅✅✅ 无线网卡名称: $wlan0, 支持热点模式! \n \033[0m"
else
  echo -e "\033[33m🚫🚫🚫 安装失败, 你的机器人的无线网卡似乎不支持AP模式, 请检查你的网卡型号!\033[0m"
  exit 1
fi

# Read ROBOT_SERIAL_NUMBER from environment file and set it as HOTSPOT_SSID
if [ -f "/etc/environment.d/RRNIS.env" ]; then
    HOTSPOT_SSID=$(grep "ROBOT_SERIAL_NUMBER" /etc/environment.d/RRNIS.env | cut -d'=' -f2)
else
    read -p $'\e[32m🤖🤖🤖 请输入你的机器人序列号(例如:MT-5)来生成你的热点名称:\e[0m' ROBOT_SERIAL_NUMBER; echo "ROBOT_SERIAL_NUMBER=$ROBOT_SERIAL_NUMBER" | sudo tee /etc/environment.d/RRNIS.env > /dev/null; HOTSPOT_SSID="${ROBOT_SERIAL_NUMBER}"
fi

# Loop to check if HOTSPOT_SSID exceeds 24 characters, if so, prompt user to re-enter
max_length=22
while [ ${#HOTSPOT_SSID} -gt $max_length ]; do
    echo -e "\033[31m❌ 热点名称不能超过 $max_length 个字符，请重新输入。\033[0m"
    read -p $'\e[32m🤖🤖🤖 请输入你的机器人序列号(例如:MT-5)来生成你的热点名称:\e[0m' -e ROBOT_SERIAL_NUMBER
    HOTSPOT_SSID="${ROBOT_SERIAL_NUMBER}"
done

HOTSPOT_SSID="${HOTSPOT_SSID}的热点"
echo -e ""

sudo create_ap $wlan0 $wlan0 "$HOTSPOT_SSID" "$HOTSPOT_PASSWORD" --mkconfig /etc/create_ap.conf --freq-band 2.4 > /dev/null 2>&1
if [ $? -eq 0 ]; then
  echo -e "\033[32m🎉🎉🎉 热点创建成功, 热点名称: $HOTSPOT_SSID, 密码:$HOTSPOT_PASSWORD \n \033[0m"
else
  echo -e "\033[31m❌❌❌ 热点创建失败!\033[0m"
  exit 1
fi

# @@@ AUTO START
sudo systemctl enable create_ap
sudo systemctl start create_ap
if [ "$(systemctl is-active create_ap)" = "active" ]; then
  echo -e "\033[32m🚀🚀🚀 热点服务已启动! \n \033[0m"
  install_kuavo_tools
else
  echo -e "\033[31m❌ 热点服务启动失败，跳过 Kuavo 配网工具安装\033[0m"
  exit 1
fi
