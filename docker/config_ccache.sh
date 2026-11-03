#!/bin/bash

# install ccache
sudo apt-get update
sudo apt-get install ccache -y

# configure ccache
add_ccache_to_rc_file() {
  local rc_file=$1
  if [ -f "$rc_file" ]; then
    if ! grep -Fxq 'export CC="ccache gcc"' "$rc_file"; then
      echo 'export CC="ccache gcc"' >> "$rc_file"
    fi
    if ! grep -Fxq 'export CXX="ccache g++"' "$rc_file"; then
      echo 'export CXX="ccache g++"' >> "$rc_file"
    fi
  else
    echo "$rc_file not found"
  fi
}

# Ubuntu bashrc returns early for non-interactive shells; append ccache after kuavo setup line.
add_ccache_to_bashrc_after_setup() {
  local rc_file="${HOME}/.bashrc"
  [ -f "$rc_file" ] || return 0
  if grep -q 'export CC="ccache gcc"' "$rc_file"; then
    return 0
  fi
  if grep -q 'setup_jammy_container_env.sh' "$rc_file"; then
    sed -i '/setup_jammy_container_env\.sh/a export CC="ccache gcc"\nexport CXX="ccache g++"' "$rc_file"
  else
    add_ccache_to_rc_file "$rc_file"
  fi
}

add_ccache_to_bashrc_after_setup
add_ccache_to_rc_file ~/.zshrc

echo "ccache enabled"

ccache --max-size=10G
