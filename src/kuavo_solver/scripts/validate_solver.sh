#!/usr/bin/env bash
# kuavo_solver 验证统一入口（薄包装）
#
# 用法:
#   ./validate_solver.sh                    # 图形界面（默认）
#   ./validate_solver.sh hub              # 同上
#   ./validate_solver.sh quick             # L1 smoke + 友好彩色摘要
#   ./validate_solver.sh check <version>   # 全机构验证指定版本，逐机构报告
#   ./validate_solver.sh view <version> [mechanism]  # 快速启动调试 viewer
#   ./validate_solver.sh baseline          # 全验证 + 保存基线
#   ./validate_solver.sh smoke             # L1：canonical 去重矩阵，5 samples，仅位置
#   ./validate_solver.sh convert [<ver> <mod> [side dir values...]]  # motor↔joint 转换
#   ./validate_solver.sh full [opts]       # L2：position + jacobian（默认 50 samples）
#   ./validate_solver.sh list-matrix       # 打印 canonical 矩阵
#   ./validate_solver.sh preflight         # L0：import + MJCF 抽查
#   ./validate_solver.sh <ver> <mod> [cmd] # 单点 CLI
#
# 前置: catkin build kuavo_solver && source devel/setup.bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RUNNER="${SCRIPT_DIR}/solver_test_runner.py"
CLI="${SCRIPT_DIR}/kuavo_solver_validator.py"

# ANSI 颜色
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m'  # No Color

usage() {
  cat <<'EOF'
kuavo_solver 验证入口

  ./validate_solver.sh                  交互菜单（推荐）
  ./validate_solver.sh quick              L1 smoke + 友好彩色摘要
  ./validate_solver.sh check <version>    全机构验证指定版本，逐机构报告
  ./validate_solver.sh view <version> [mechanism]  快速启动调试 viewer
  ./validate_solver.sh baseline           全验证 + 保存基线到 /tmp/solver_baseline_*
  ./validate_solver.sh smoke              L1：canonical 去重矩阵，5 samples，仅位置
  ./validate_solver.sh convert [opts]     motor<->joint 全版本位置转换验证（纯 solver）
  ./validate_solver.sh convert <ver> <mod> [side] [j2m|m2j] <values>  单点转换
  ./validate_solver.sh full [opts]        L2：position + jacobian（默认 50 samples）
  ./validate_solver.sh list-matrix        打印 canonical 矩阵
  ./validate_solver.sh preflight          L0：import + MJCF 抽查
  ./validate_solver.sh <ver> <mod> [cmd]  单点 CLI

full 可选: --samples N  --output-dir DIR  --format all|html|json|csv|md|plots  --token T  --module M  --by-version
单点 CLI: [jacobian|position-verify] [--num-steps N] [--json] [--seed N]
EOF
}

color_pass()    { echo -e "${GREEN}PASS${NC}"; }
color_fail()    { echo -e "${RED}FAIL${NC}"; }
color_warn()    { echo -e "${YELLOW}WARN${NC}"; }

print_hints() {
    local context="$1"
    echo ""
    echo -e "${CYAN}提示:${NC}"
    case "${context}" in
        quick|smoke)
            echo "  单版本调试:  ./validate_solver.sh view s70 ankle"
            echo "  单版本验证:  ./validate_solver.sh check s70"
            echo "  详细报告:    cat /tmp/solver_test_results/solver_test_summary.md"
            ;;
        check)
            echo "  查看 viewer:          ./validate_solver.sh view s70 ankle"
            echo "  可选机构:             ankle / knee / waist / arm_elbow / arm_wrist"
            echo "  查看版本/机构映射:    ./validate_solver.sh list-matrix"
            ;;
        view)
            echo "  STL 缺失时加 --kinematics-only 仅显示关节/轴线"
            echo "  查看资产审计:         python3 ${CLI} -t 7gen audit-assets --module ankle"
            ;;
        baseline)
            echo "  基线目录包含 HTML 报告，浏览器打开即可"
            echo "  与将来结果对比:       diff <(jq . ${BASELINE_DIR}/solver_test_report.json) <(jq . /tmp/solver_next/solver_test_report.json)"
            ;;
        convert)
            echo "  单侧转换:             python3 ${CLI} -t 7gen convert-position --module ankle --side L --direction j2m --values 0.5"
            ;;
        *)  echo "  查看帮助: ./validate_solver.sh help" ;;
    esac
}

if [[ $# -lt 1 ]]; then
  exec python3 "${CLI}"
fi

cmd="$1"
shift

case "${cmd}" in
  hub|gui|ui)
    exec python3 "${CLI}"
    ;;

  quick)
    echo -e "${BOLD}kuavo_solver quick check${NC}"
    echo ""
    tmpfile=$(mktemp)
    set +e
    python3 "${RUNNER}" --smoke "$@" 2>&1 | tee "${tmpfile}"
    exit_code=$?
    set -e
    echo ""
    echo -e "${BOLD}====== 摘要 ======${NC}"
    if [ $exit_code -eq 0 ]; then
        echo -e "  总体: $(color_pass) -- 所有 canonical case 通过"
    else
        echo -e "  总体: $(color_fail) -- 存在失败项"
    fi
    pass_count=$(grep -c "\[PASS\]" "${tmpfile}" 2>/dev/null || echo 0)
    fail_count=$(grep -c "\[FAIL\]" "${tmpfile}" 2>/dev/null || echo 0)
    error_count=$(grep -c "\[ERROR\]" "${tmpfile}" 2>/dev/null || echo 0)
    echo "  PASS:  ${pass_count}"
    echo "  FAIL:  ${fail_count}"
    echo "  ERROR: ${error_count}"
    rm -f "${tmpfile}"
    if [ "${fail_count}" -gt 0 ] || [ "${error_count}" -gt 0 ]; then
        print_hints "quick"
    fi
    exit "${exit_code}"
    ;;

  check)
    if [[ $# -lt 1 ]]; then
      echo -e "${RED}用法:${NC} ./validate_solver.sh check <version> [--samples N]"
      echo "  例如: ./validate_solver.sh check s70"
      echo "       ./validate_solver.sh check 70 --samples 10"
      print_hints "check"
      exit 1
    fi
    version="$1"
    shift
    echo -e "${BOLD}全机构验证: version=${version}${NC}"
    echo ""
    set +e
    python3 "${RUNNER}" --smoke --version "${version}" "$@"
    exit_code=$?
    set -e
    echo ""
    if [ "${exit_code}" -eq 0 ]; then
        echo -e "  所有机构通过 $(color_pass)"
    else
        echo -e "  存在失败 $(color_fail)"
        print_hints "check"
    fi
    exit "${exit_code}"
    ;;

  view)
    if [[ $# -lt 1 ]]; then
      echo -e "${RED}用法:${NC} ./validate_solver.sh view <version> [mechanism] [--kinematics-only]"
      echo "  例如: ./validate_solver.sh view s70 ankle"
      echo "       ./validate_solver.sh view 70 waist --kinematics-only"
      print_hints "view"
      exit 1
    fi
    version="$1"
    shift
    mechanism=""
    extra_args=()
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --*) extra_args+=("$1") ;;
            *)
                if [ -z "$mechanism" ]; then
                    mechanism="$1"
                else
                    extra_args+=("$1")
                fi
                ;;
        esac
        shift
    done
    if [ -z "$mechanism" ]; then
        mechanism="ankle"
    fi
    echo -e "${BOLD}启动 viewer: version=${version}  mechanism=${mechanism}${NC}"
    echo "  关闭 MuJoCo 窗口即退出"
    exec python3 "${CLI}" -t "${version}" viewer --module "${mechanism}" "${extra_args[@]}"
    ;;

  baseline)
    BASELINE_DIR="/tmp/solver_baseline_$(date +%Y%m%d_%H%M%S)"
    echo -e "${BOLD}运行全验证并保存基线到: ${BASELINE_DIR}${NC}"
    echo ""
    set +e
    python3 "${RUNNER}" --all --output-dir "${BASELINE_DIR}" "$@"
    exit_code=$?
    set -e
    echo ""
    if [ "${exit_code}" -eq 0 ]; then
        echo -e "  基线 $(color_pass) -- 全部通过"
    else
        echo -e "  基线 $(color_fail) -- 存在失败项"
    fi
    echo "  基线目录: ${BASELINE_DIR}"
    echo "  基线报告: ${BASELINE_DIR}/solver_test_report.html"
    print_hints "baseline"
    exit "${exit_code}"
    ;;

  smoke)
    exec python3 "${RUNNER}" --smoke "$@"
    ;;

  convert)
    # 如果带 version/module 参数，走单点转换；否则走全版本矩阵
    if [[ $# -ge 2 ]]; then
        # 单点转换: convert <version> <module> [side] [j2m|m2j] [values...]
        c_version="$1"
        c_module="$2"
        shift 2
        echo -e "${BOLD}单点转换: version=${c_version} module=${c_module}${NC}"
        exec python3 "${CLI}" -t "${c_version}" convert-position --module "${c_module}" "$@"
    fi
    exec python3 "${RUNNER}" --convert "$@"
    ;;

  full)
    exec python3 "${RUNNER}" --all "$@"
    ;;

  list-matrix)
    exec python3 "${RUNNER}" --list-matrix "$@"
    ;;

  preflight)
    exec python3 "${RUNNER}" --preflight "$@"
    ;;

  -h|--help|help)
    usage
    exit 0
    ;;

  *)
    if [[ $# -lt 1 ]]; then
      echo "单点模式需要 <version> <module> [jacobian|position-verify]"
      usage
      exit 1
    fi
    version="${cmd}"
    module="$1"
    shift
    subcmd="position-verify"
    if [[ $# -ge 1 && ("$1" == "jacobian" || "$1" == "position-verify") ]]; then
      subcmd="$1"
      shift
    fi
    exec python3 "${CLI}" -t "${version}" "${subcmd}" --module "${module}" "$@"
    ;;
esac
