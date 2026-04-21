#!/bin/bash

if [[ -n "${BASH_SOURCE[0]}" ]]; then
  script_path="${BASH_SOURCE[0]}"
else
  script_path="$0"
fi

script_dir="$(cd "$(dirname "$script_path")" && pwd)"

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$script_dir/lib
export PATH=$PATH:$script_dir/bin
export CMAKE_PREFIX_PATH=$script_dir:$CMAKE_PREFIX_PATH
export KUAVO_ASSETS_PATH=$script_dir/share/kuavo_assets
export KUAVO_COMMON_PATH=$script_dir/share/kuavo_common
export HARDWARE_PLANT_PATH=$script_dir/share/hardware_plant