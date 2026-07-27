#!/usr/bin/env bash
# Repeatedly alternate between depth locomotion and AMP through the navigation topic.
#
# Example:
#   ./test_nav_switch_depth_loco_topic.sh --count 20 --interval 0.5

set -euo pipefail

TOPIC="/humanoid_controller/nav_switch_rl_controller_by_name"
DEPTH_CONTROLLER="depth_loco_controller"
AMP_CONTROLLER="amp_controller"
COUNT=200
INTERVAL=0.5
VERIFY_TIMEOUT=1
LIST_SERVICE="/humanoid_controller/get_controller_list"
OUTPUT=""
SUMMARY_OUTPUT=""

usage() {
    cat <<'EOF'
Usage: test_nav_switch_depth_loco_topic.sh [options]

Alternate between `depth_loco_controller` and `amp_controller` N times, beginning
with `depth_loco_controller`.

Options:
  --count N                 Number of switch requests (default: 1)
  --interval SECONDS        Delay between requests (default: 0.5)
  --verify-timeout SECONDS  Maximum wait for the requested controller (default: 3)
  --output PATH             CSV result path (default: nav_switch_depth_loco_<timestamp>.csv)
  --summary-output PATH     Text summary path (default: CSV path with _summary.txt suffix)
  --list-service NAME       Controller-list service (default: /humanoid_controller/get_controller_list)
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --count)
            COUNT="$2"
            shift 2
            ;;
        --interval)
            INTERVAL="$2"
            shift 2
            ;;
        --verify-timeout)
            VERIFY_TIMEOUT="$2"
            shift 2
            ;;
        --output)
            OUTPUT="$2"
            shift 2
            ;;
        --summary-output)
            SUMMARY_OUTPUT="$2"
            shift 2
            ;;
        --list-service)
            LIST_SERVICE="$2"
            shift 2
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "Unknown argument: $1" >&2
            usage >&2
            exit 2
            ;;
    esac
done

if ! [[ "$COUNT" =~ ^[1-9][0-9]*$ ]]; then
    echo "--count must be a positive integer" >&2
    exit 2
fi
if ! [[ "$INTERVAL" =~ ^([0-9]+([.][0-9]*)?|[.][0-9]+)$ ]]; then
    echo "--interval must be a non-negative number" >&2
    exit 2
fi
if ! [[ "$VERIFY_TIMEOUT" =~ ^([0-9]+([.][0-9]*)?|[.][0-9]+)$ ]]; then
    echo "--verify-timeout must be a non-negative number" >&2
    exit 2
fi

VERIFY_TIMEOUT_MS=$(awk "BEGIN { printf \"%d\", ${VERIFY_TIMEOUT} * 1000 }")
OUTPUT=${OUTPUT:-"nav_switch_depth_loco_$(date +%Y%m%d_%H%M%S).csv"}
mkdir -p "$(dirname "$(readlink -f "$OUTPUT")")"
if [[ -z "$SUMMARY_OUTPUT" ]]; then
    SUMMARY_OUTPUT="${OUTPUT%.csv}_summary.txt"
fi
mkdir -p "$(dirname "$(readlink -f "$SUMMARY_OUTPUT")")"

csv_escape() {
    local value=${1//$'\n'/ }
    value=${value//\"/\"\"}
    printf '"%s"' "$value"
}

get_current_controller() {
    local response
    if ! response=$(rosservice call "$LIST_SERVICE" 2>&1); then
        GET_CONTROLLER_ERROR="$response"
        return 1
    fi
    GET_CONTROLLER_VALUE=$(sed -n 's/^current_controller: "\([^"]*\)".*/\1/p' <<< "$response" | head -n 1)
    if [[ -z "$GET_CONTROLLER_VALUE" ]]; then
        GET_CONTROLLER_ERROR="Unable to parse current_controller from ${LIST_SERVICE}: ${response}"
        return 1
    fi
}

verify_target_controller() {
    local target=$1
    local started_ms now_ms
    started_ms=$(date +%s%3N)
    VERIFY_ACTUAL=""
    VERIFY_ERROR=""

    while true; do
        if get_current_controller; then
            VERIFY_ACTUAL=$GET_CONTROLLER_VALUE
            if [[ "$VERIFY_ACTUAL" == "$target" ]]; then
                return 0
            fi
            VERIFY_ERROR="Current controller is '${VERIFY_ACTUAL}', expected '${target}'"
        else
            VERIFY_ERROR=$GET_CONTROLLER_ERROR
        fi

        now_ms=$(date +%s%3N)
        if (( now_ms - started_ms >= VERIFY_TIMEOUT_MS )); then
            return 1
        fi
        sleep 0.1
    done
}

printf '%s\n' 'request_time,target_controller,publish_success,actual_controller,verified,elapsed_ms,error' > "$OUTPUT"
failures=0
run_started=$(date +'%Y-%m-%dT%H:%M:%S.%3N%:z')

for ((index = 1; index <= COUNT; index++)); do
    if (( index % 2 == 1 )); then
        controller="$DEPTH_CONTROLLER"
    else
        controller="$AMP_CONTROLLER"
    fi

    echo "[$index/$COUNT] publishing ${controller} to ${TOPIC}"
    request_time=$(date +'%Y-%m-%dT%H:%M:%S.%3N%:z')
    started_ms=$(date +%s%3N)
    publish_success=false
    verified=false
    actual_controller=""
    error=""

    if publish_output=$(rostopic pub -1 "$TOPIC" std_msgs/String "data: '${controller}'" 2>&1); then
        publish_success=true
        if verify_target_controller "$controller"; then
            verified=true
        else
            actual_controller=$VERIFY_ACTUAL
            error=$VERIFY_ERROR
        fi
    else
        error=$publish_output
    fi

    finished_ms=$(date +%s%3N)
    elapsed_ms=$((finished_ms - started_ms))
    if [[ -z "$actual_controller" && -n ${VERIFY_ACTUAL:-} ]]; then
        actual_controller=$VERIFY_ACTUAL
    fi
    printf '%s,%s,%s,%s,%s,%s,%s\n' \
        "$(csv_escape "$request_time")" \
        "$(csv_escape "$controller")" \
        "$publish_success" \
        "$(csv_escape "$actual_controller")" \
        "$verified" \
        "$elapsed_ms" \
        "$(csv_escape "$error")" >> "$OUTPUT"

    if [[ "$verified" != true ]]; then
        failures=$((failures + 1))
        echo "[$index/$COUNT] switch verification failed: $error" >&2
    fi

    if (( index < COUNT )) && [[ "$INTERVAL" != "0" && "$INTERVAL" != "0.0" ]]; then
        sleep "$INTERVAL"
    fi
done

run_finished=$(date +'%Y-%m-%dT%H:%M:%S.%3N%:z')
successes=$((COUNT - failures))
if (( failures == 0 )); then
    overall_result="PASS"
else
    overall_result="FAIL"
fi
cat > "$SUMMARY_OUTPUT" <<EOF
Navigation RL controller switch test summary
Start time: $run_started
End time: $run_finished
Requested switch requests: $COUNT
Verified success: $successes
Failure: $failures
Overall result: $overall_result
CSV result: $(readlink -f "$OUTPUT")
EOF

echo "CSV result: $(readlink -f "$OUTPUT")"
echo "Summary: $(readlink -f "$SUMMARY_OUTPUT")"
if (( failures > 0 )); then
    echo "${failures}/${COUNT} switch requests failed verification" >&2
    exit 1
fi
