#!/bin/bash
set -euo pipefail

DEFAULT_TARGET_USER="${SUDO_USER:-$(id -un)}"
TARGET_USER="${TARGET_USER:-${DEFAULT_TARGET_USER}}"
TARGET_UID="${TARGET_UID:-}"
SCRIPT_PATH="/usr/local/bin/eb03-pulse-fix.sh"
SERVICE_PATH="/etc/systemd/system/eb03-pulseaudio-fix.service"
ASOUND_CONF_PATH="/etc/asound.conf"
ASOUND_CONF_BACKUP_SUFFIX=".bak.eb03-pulseaudio-fix"

get_eb03_card_index() {
  aplay -l 2>/dev/null | awk '/EB03_LJ/ {print $2}' | head -n1 | tr -d ':'
}

set_alsa_default_to_eb03() {
  local card_index
  local tmp_asound_conf
  card_index="$(get_eb03_card_index)"

  if [[ -z "${card_index}" ]]; then
    echo "[WARN] EB03_LJ not found in aplay -l, skip writing /etc/asound.conf"
    return 1
  fi

  tmp_asound_conf="$(mktemp)"

  cat > "${tmp_asound_conf}" <<EOF
defaults.pcm.card ${card_index}
defaults.ctl.card ${card_index}

pcm.!default {
    type plug
    slave.pcm "hw:${card_index},0"
}

ctl.!default {
    type hw
    card ${card_index}
}
EOF

  if [[ -f "${ASOUND_CONF_PATH}" ]] && cmp -s "${tmp_asound_conf}" "${ASOUND_CONF_PATH}"; then
    rm -f "${tmp_asound_conf}"
    echo "[OK] ${ASOUND_CONF_PATH} already points to EB03_LJ (card ${card_index})"
    return 0
  fi

  if [[ -f "${ASOUND_CONF_PATH}" ]]; then
    cp -a "${ASOUND_CONF_PATH}" "${ASOUND_CONF_PATH}${ASOUND_CONF_BACKUP_SUFFIX}"
    echo "[INFO] Backed up existing ${ASOUND_CONF_PATH} to ${ASOUND_CONF_PATH}${ASOUND_CONF_BACKUP_SUFFIX}"
  fi

  install -m 644 "${tmp_asound_conf}" "${ASOUND_CONF_PATH}"
  rm -f "${tmp_asound_conf}"

  echo "[OK] ALSA default output now points to EB03_LJ (card ${card_index})"
  return 0
}

if [[ "${EUID}" -ne 0 ]]; then
  echo "Please run as root"
  exit 1
fi

if ! id "${TARGET_USER}" >/dev/null 2>&1; then
  echo "[ERROR] User ${TARGET_USER} does not exist"
  exit 1
fi

if [[ -z "${TARGET_UID}" ]]; then
  TARGET_UID="$(id -u "${TARGET_USER}")"
fi

if [[ "${TARGET_UID}" != "$(id -u "${TARGET_USER}")" ]]; then
  echo "[ERROR] TARGET_UID=${TARGET_UID} does not match user ${TARGET_USER}"
  exit 1
fi

echo "[1/7] Enabling linger for ${TARGET_USER}"
loginctl enable-linger "${TARGET_USER}"

echo "[2/7] Enabling user pulseaudio service and socket"
sudo -u "${TARGET_USER}" \
  XDG_RUNTIME_DIR="/run/user/${TARGET_UID}" \
  DBUS_SESSION_BUS_ADDRESS="unix:path=/run/user/${TARGET_UID}/bus" \
  systemctl --user enable pulseaudio.service pulseaudio.socket

echo "[3/7] Setting ALSA default output to EB03_LJ"
set_alsa_default_to_eb03 || true

echo "[4/7] Writing ${SCRIPT_PATH}"
cat > "${SCRIPT_PATH}" <<'EOF'
#!/bin/bash
set -euo pipefail

USER_NAME="__TARGET_USER__"
USER_ID="__TARGET_UID__"
CARD_NAME="EB03_LJ"
TIMEOUT=45

log() {
  logger -t eb03-pulse-fix "$1"
  echo "$1"
}

if ! id "${USER_NAME}" >/dev/null 2>&1; then
  log "User ${USER_NAME} does not exist"
  exit 1
fi

runtime_uid="$(id -u "${USER_NAME}")"
if [[ "${runtime_uid}" != "${USER_ID}" ]]; then
  log "Configured UID ${USER_ID} does not match current UID ${runtime_uid} for ${USER_NAME}, using current UID"
  USER_ID="${runtime_uid}"
fi

USER_RUNTIME_DIR="/run/user/${USER_ID}"
USER_BUS="unix:path=${USER_RUNTIME_DIR}/bus"
i=1

while [[ "${i}" -le "${TIMEOUT}" ]]; do
  if grep -q "${CARD_NAME}" /proc/asound/cards 2>/dev/null && [ -S "${USER_RUNTIME_DIR}/bus" ]; then
    log "Detected ${CARD_NAME} and user bus after ${i}s, restarting pulseaudio"
    if sudo -u "${USER_NAME}" \
      XDG_RUNTIME_DIR="${USER_RUNTIME_DIR}" \
      DBUS_SESSION_BUS_ADDRESS="${USER_BUS}" \
      systemctl --user restart pulseaudio.service; then
      sleep 2
      if sudo -u "${USER_NAME}" \
        XDG_RUNTIME_DIR="${USER_RUNTIME_DIR}" \
        DBUS_SESSION_BUS_ADDRESS="${USER_BUS}" \
        pactl list short cards 2>/dev/null | grep -q "usb-ShengZhi_EB03_LJ"; then
        log "PulseAudio reloaded and EB03 card is present"
      else
        log "PulseAudio restarted but EB03 card is not visible in pactl output yet"
      fi
      exit 0
    fi
    log "Failed to restart user pulseaudio"
    exit 1
  fi
  sleep 1
  i=$((i + 1))
done

log "Timed out waiting for ${CARD_NAME} or user bus, skipping restart"
exit 0
EOF
sed -i \
  -e "s|__TARGET_USER__|${TARGET_USER}|g" \
  -e "s|__TARGET_UID__|${TARGET_UID}|g" \
  "${SCRIPT_PATH}"
chmod 755 "${SCRIPT_PATH}"

echo "[5/7] Writing ${SERVICE_PATH}"
cat > "${SERVICE_PATH}" <<'EOF'
[Unit]
Description=Restart PulseAudio after EB03 USB audio becomes ready
After=mock_zed_replugging.service systemd-user-sessions.service sound.target multi-user.target
Wants=systemd-user-sessions.service sound.target

[Service]
Type=oneshot
ExecStart=/bin/bash /usr/local/bin/eb03-pulse-fix.sh

[Install]
WantedBy=multi-user.target
EOF

echo "[6/7] Enabling system service"
systemctl daemon-reload
systemctl enable eb03-pulseaudio-fix.service
systemctl reset-failed eb03-pulseaudio-fix.service || true

echo "[7/7] Current status"
systemctl status eb03-pulseaudio-fix.service --no-pager || true

echo "Deployment complete"
