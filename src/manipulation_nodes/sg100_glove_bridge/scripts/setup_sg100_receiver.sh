#!/usr/bin/env bash
# SG100 receiver deployment for any new machine or USB port (one command).
# 1) Passively scans CH340 candidates and finds the one with wanted board IDs.
# 2) Writes a udev rule binding /dev/sg100_glove_receiver to that physical port.
# 3) Tightens generic power_board/usb_remote aliases so no service can claim the
#    glove receiver. Teleop afterwards needs no port or ID arguments.
set -euo pipefail

WS="${SG100_WORKSPACE:-$HOME/lihengyu/kuavo-ros-control-wheel}"
BAUD=921600
WANTED="${SG100_BOARD_IDS:-12,2}"
RUNTIME="$WS/src/manipulation_nodes/sg100_glove_bridge/sg100_runtime"
PY="$RUNTIME/.venv_sg100_native/bin/python"
RULE_DIR=/etc/udev/rules.d

[[ -x "$PY" ]] || { echo "Missing SDK interpreter: $PY" >&2; exit 1; }

echo "== 1/3 passive scan of CH340 candidates (no commands sent) =="
cat > /tmp/sg100_scan_tmp.py <<PYEOF
import sys, json
sys.path.insert(0, "$RUNTIME")
from sg100_native.connection import scan_boards
from serial.tools import list_ports
out = []
for p in list_ports.comports():
    if (p.vid, p.pid) != (0x1A86, 0x7523):
        continue
    dev = p.device
    try:
        boards = scan_boards(dev, $BAUD, 3)["boards"]
    except Exception as exc:
        out.append({"device": dev, "error": str(exc)})
        continue
    ids = sorted(row["board_id"] for row in boards)
    out.append({"device": dev, "ids": ids})
print(json.dumps(out))
PYEOF
scan_json="$(sg dialout -c "env -u PYTHONPATH -u PYTHONHOME -u LD_LIBRARY_PATH '$PY' /tmp/sg100_scan_tmp.py")"
rm -f /tmp/sg100_scan_tmp.py
echo "$scan_json"

chosen="$(python3 -c '
import json, sys
data = json.loads(sys.argv[1])
wanted = set(int(x) for x in sys.argv[2].split(",") if x)
for row in data:
    if wanted and wanted.issubset(set(row.get("ids") or [])):
        print(row["device"]); break
' "$scan_json" "$WANTED")"

[[ -n "$chosen" ]] || { echo "No candidate reported boards $WANTED; check gloves are powered and paired." >&2; exit 1; }
echo "Glove receiver: $chosen"

echo "== 2/3 resolve physical USB location =="
syspath="$(udevadm info --query=property --name="$chosen" --property=DEVPATH)"
location="$(python3 -c '
import sys, re
parts = [p for p in sys.argv[1].split("/") if p]
for index, part in enumerate(parts):
    if re.fullmatch(r"\d+-\d+(\.\d+)*", part) and index > 0 and parts[index - 1].startswith("usb"):
        print(part); break
' "$syspath")"
[[ -n "$location" ]] || { echo "Cannot resolve USB location" >&2; exit 1; }
vendor="$(cat "/sys/bus/usb/devices/$location/idVendor")"
product="$(cat "/sys/bus/usb/devices/$location/product")"
echo "USB port $location  vendor=$vendor product=$product"

echo "== 3/3 write udev rules =="
cat > /tmp/98-sg100-glove-receiver.rules.tmp <<EOF
SUBSYSTEM=="tty", KERNEL=="ttyUSB*", KERNELS=="$location", ATTRS{idVendor}=="$vendor", ATTRS{idProduct}=="7523", ATTRS{product}=="$product", ENV{SG100_GLOVE_RECEIVER}="1", MODE:="0660", GROUP:="dialout", SYMLINK+="sg100_glove_receiver"
EOF
cat > /tmp/99-power-board-usb.rules.tmp <<'EOF2'
KERNEL=="ttyUSB*", SUBSYSTEMS=="usb", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", ATTRS{product}=="USB2.0-Serial", ENV{SG100_GLOVE_RECEIVER}!="1", MODE:="0660", GROUP:="dialout", SYMLINK+="power_board"
EOF2
cat > /tmp/usb_remote.rules.tmp <<'EOF3'
KERNEL=="ttyUSB*", ACTION=="add", ATTRS{product}=="LJREMOTE", ENV{SG100_GLOVE_RECEIVER}!="1", MODE:="0660", GROUP:="dialout", ATTR{latency_timer}="1", SYMLINK+="usb_remote"
KERNEL=="ttyUSB*", ACTION=="add", ATTRS{product}=="USB Serial", ENV{SG100_GLOVE_RECEIVER}!="1", MODE:="0660", GROUP:="dialout", ATTR{latency_timer}="1", SYMLINK+="usb_remote"
EOF3
sudo install -m 0644 /tmp/98-sg100-glove-receiver.rules.tmp /tmp/99-power-board-usb.rules.tmp /tmp/usb_remote.rules.tmp "$RULE_DIR/"
rm -f /tmp/98-sg100-glove-receiver.rules.tmp /tmp/99-power-board-usb.rules.tmp /tmp/usb_remote.rules.tmp
sudo udevadm control --reload-rules
for dev in /dev/ttyUSB*; do
    sudo udevadm trigger --action=add --sysname-match="$(basename "$dev")"
done
sleep 1

alias_target="$(readlink -f /dev/sg100_glove_receiver)"
echo
echo "Verify: /dev/sg100_glove_receiver -> $alias_target (expected $chosen)"
[[ "$alias_target" == "$(readlink -f "$chosen")" ]] || { echo "Alias does not point at the verified port" >&2; exit 1; }

echo
echo "Done. Start teleop with:"
echo "  roslaunch noitom_hi5_hand_udp_python launch_quest3_ik.launch hand_control_source:=qianshi launch_sg100_ros_node:=false"
echo "For different board IDs add: qianshi_left_id:=<left> qianshi_right_id:=<right>"
