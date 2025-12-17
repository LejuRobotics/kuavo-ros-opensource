#!/usr/bin/env bash

set -euo pipefail

# ------------------------
# 配置路径
# ------------------------
REPO_ROOT="$(git rev-parse --show-toplevel)"
TOOLS_DIR="$REPO_ROOT/tools/lint_tools/cppcheck_tools"
CPPCHECK_SCRIPT="$REPO_ROOT/ci_scripts/tools/lint_tools/cppcheck_lint/cppcheck_lint_tool_xml.sh"
DIFF_PY="$TOOLS_DIR/report_diff.py"
OUT_DIR="$TOOLS_DIR/cppcheck_results"

mkdir -p "$OUT_DIR"

# ------------------------
# 交互/非交互 拉取控制
# ------------------------
# 第三个可选参数为交互控制：--non-interactive 表示禁用交互，其它情况默认允许交互
AUTH_MODE_FLAG="${3:-}"
GIT_NONINTERACTIVE="${GIT_NONINTERACTIVE:-}"
if [[ "$AUTH_MODE_FLAG" == "--non-interactive" ]]; then
    GIT_NONINTERACTIVE=1
fi
if [[ -n "$GIT_NONINTERACTIVE" && "$GIT_NONINTERACTIVE" != "0" ]]; then
    GIT_ENV_PREFIX="GIT_TERMINAL_PROMPT=0 GIT_ASKPASS="
else
    GIT_ENV_PREFIX=""
fi

# ------------------------
# 检查依赖
# ------------------------
for cmd in git cppcheck python3; do
    command -v "$cmd" >/dev/null 2>&1 || {
        echo "Error: '$cmd' not found in PATH." >&2
        exit 1
    }
done

[[ -f "$CPPCHECK_SCRIPT" ]] || {
    echo "Error: cppcheck script not found at $CPPCHECK_SCRIPT" >&2
    exit 1
}
[[ -f "$DIFF_PY" ]] || {
    echo "Error: diff script not found at $DIFF_PY" >&2
    exit 1
}

chmod +x "$CPPCHECK_SCRIPT"

# ------------------------
# 参数输入
# ------------------------
BASELINE_BRANCH="${1:-}"
DEV_BRANCH="${2:-}"

if [[ -z "$BASELINE_BRANCH" ]]; then
    read -rp "请输入 baseline 分支名称: " BASELINE_BRANCH
fi
if [[ -z "$DEV_BRANCH" ]]; then
    read -rp "请输入 dev 分支名称: " DEV_BRANCH
fi

CURRENT_BRANCH=$(git rev-parse --abbrev-ref HEAD)

echo "当前分支: $CURRENT_BRANCH"
echo "Baseline: $BASELINE_BRANCH"
echo "Develop:  $DEV_BRANCH"
echo "输出目录: $OUT_DIR"
echo

# 防护：两个分支同名则退出
if [[ "$BASELINE_BRANCH" == "$DEV_BRANCH" ]]; then
    echo "Error: baseline 与 develop 分支名相同，终止。" >&2
    exit 1
fi

# ------------------------
# 创建临时工作树函数
# ------------------------
create_worktree_for_branch() {
    local branch="$1"
    local tmpdir
    tmpdir=$(mktemp -d -t cppcheck-${branch//\//_}-XXXX)
    echo ">>> 创建源码目录: $branch → $tmpdir" >&2

    # 若与当前分支一致：直接基于当前分支HEAD创建干净worktree（不依赖远端、不拷贝未提交/未跟踪）
    if [[ "$branch" == "$CURRENT_BRANCH" ]]; then
        if git worktree add -f "$tmpdir" "$branch" >/dev/null 2>&1; then
            echo "$tmpdir"
            return 0
        else
            echo "Error: 基于当前分支创建 worktree 失败: $branch" >&2
            rm -rf "$tmpdir" || true
            return 1
        fi
    fi

    # 快速验证远端分支存在（允许交互凭据）
    local origin_url
    origin_url=$(git remote get-url origin)
    if ! eval "$GIT_ENV_PREFIX git ls-remote --exit-code --heads \"$origin_url\" \"$branch\" >/dev/null 2>&1"; then
        echo "Error: 远端分支不存在或认证失败: $branch" >&2
        rm -rf "$tmpdir" || true
        return 1
    fi

    # 在切换/创建工作树之前，先 fetch 单分支到本地，确保引用最新
    if ! eval "$GIT_ENV_PREFIX git fetch origin \"$branch\" --prune"; then
        echo "Error: git fetch 失败（认证/网络问题）: $branch" >&2
        rm -rf "$tmpdir" || true
        return 1
    fi

    # 使用本地最新的 origin/<branch> 创建临时工作树目录（优先 detached 引用以避免“已被检出”冲突）
    if ! git worktree add -f "$tmpdir" "origin/$branch" >/dev/null 2>&1; then
        echo "Warn: 直接基于 origin/$branch 创建 worktree 失败，尝试创建临时本地分支..." >&2
        local tmp_branch="cppcheck_tmp_${branch//\//_}_$(date +%s)"
        if git branch -f "$tmp_branch" "origin/$branch" >/dev/null 2>&1 && \
           git worktree add -f "$tmpdir" "$tmp_branch" >/dev/null 2>&1; then
            TMP_BRANCHES+=("$tmp_branch")
        else
            echo "Error: 创建临时本地分支或 worktree 失败: $tmp_branch" >&2
            rm -rf "$tmpdir" || true
            # 尝试删除失败的临时分支（若已创建）
            git branch -D "$tmp_branch" >/dev/null 2>&1 || true
            return 1
        fi
    fi

    echo "$tmpdir"
}

# ------------------------
# 清理函数（退出时调用）
# ------------------------
cleanup() {
    echo
    echo ">>> 清理工作树..."
    for wt in "${WORKTREES[@]:-}"; do
        if [[ -d "$wt" ]]; then
            echo "移除 $wt"
            # 保护：不要误删主仓库目录
            if [[ "$wt" == "$REPO_ROOT" ]]; then
                continue
            fi
            rm -rf "$wt" || true
        fi
    done
    # 清理临时分支
    for tb in "${TMP_BRANCHES[@]:-}"; do
        if git show-ref --verify --quiet "refs/heads/$tb"; then
            echo "删除临时分支: $tb"
            git branch -D "$tb" >/dev/null 2>&1 || true
        fi
    done
}
trap cleanup EXIT

# ------------------------
# 为两个分支创建工作树
# ------------------------
WORKTREES=()
TMP_BRANCHES=()
BASELINE_PATH=$(create_worktree_for_branch "$BASELINE_BRANCH")
WORKTREES+=("$BASELINE_PATH")
DEV_PATH=$(create_worktree_for_branch "$DEV_BRANCH")
WORKTREES+=("$DEV_PATH")

# ------------------------
# 运行 cppcheck
# ------------------------
run_cppcheck() {
    local src_path="$1"
    local branch="$2"
    local report_path="$OUT_DIR/cppcheck_${branch//\//_}.xml"

    echo
    echo ">>> 在 $branch 分支运行 cppcheck"
    echo "源代码路径: $src_path"

    # 在 worktree 自身的脚本目录运行，以使用该分支的白名单/配置
    local wt_cppcheck_dir="$src_path/ci_scripts/tools/lint_tools/cppcheck_lint"
    local wt_cppcheck_script="$wt_cppcheck_dir/cppcheck_lint_tool_xml.sh"
    if [[ ! -x "$wt_cppcheck_script" ]]; then
        echo "Error: 脚本不存在或不可执行: $wt_cppcheck_script" >&2
        return 1
    fi

    pushd "$wt_cppcheck_dir" >/dev/null

    set +e
    # 传入参数：第一个为源码路径，第二个为并发数（2）
    bash "$wt_cppcheck_script" "$src_path" 2
    local cpp_rc=$?
    set -e

    # 报告位于当前 worktree 脚本目录下的 cppcheck_results
    local result_xml="$wt_cppcheck_dir/cppcheck_results/cppcheck_report.xml"
    if [[ -f "$result_xml" ]]; then
        cp "$result_xml" "$report_path"
        echo "已生成报告: $report_path"
    else
        echo "Error: cppcheck 报告未找到 ($result_xml)" >&2
        # 即使子脚本返回非 0，也尽量继续，让另一侧分支仍可产出报告
    fi

    popd >/dev/null
}

run_cppcheck "$BASELINE_PATH" "$BASELINE_BRANCH"
run_cppcheck "$DEV_PATH" "$DEV_BRANCH"

# ------------------------
# 生成差异报告
# ------------------------
echo
echo ">>> 生成 cppcheck 差异报告"
DIFF_OUTPUT="$OUT_DIR/cppcheck_diff_${DEV_BRANCH//\//_}_vs_${BASELINE_BRANCH//\//_}.txt"

# 先校验两个 XML 报告是否合法，避免无效输入导致对比失败
validate_xml() {
    local xml_file="$1"
    if [[ ! -s "$xml_file" ]]; then
        echo "Error: 报告不存在或为空: $xml_file" >&2
        return 1
    fi
    if command -v xmllint >/dev/null 2>&1; then
        if ! xmllint --noout "$xml_file" >/dev/null 2>&1; then
            echo "Error: XML 解析失败: $xml_file" >&2
            return 1
        fi
    else
        # 简易校验：包含 <results 并以 </results> 收尾
        if ! grep -q "<results" "$xml_file"; then
            echo "Error: 非法 XML（缺少 <results）: $xml_file" >&2
            return 1
        fi
        if ! tail -n 2 "$xml_file" | grep -q "</results>"; then
            echo "Error: 非法 XML（缺少 </results> 收尾）: $xml_file" >&2
            return 1
        fi
    fi
    return 0
}

which cppcheck
CUR_XML="$OUT_DIR/cppcheck_${DEV_BRANCH//\//_}.xml"
BASE_XML="$OUT_DIR/cppcheck_${BASELINE_BRANCH//\//_}.xml"

if ! validate_xml "$CUR_XML" || ! validate_xml "$BASE_XML"; then
    echo "请修复报告生成问题后重新运行。" >&2
    exit 1
fi

python3 "$DIFF_PY" \
    "$CUR_XML" \
    "$BASE_XML"

echo
echo "差异报告生成完成: $DIFF_OUTPUT"
echo "✅ 所有任务完成"
