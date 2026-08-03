#!/usr/bin/env bash
# Live progress monitor for the Section 7.5 sensor-configuration comparison.
#
#   ./watch_config_compare.sh                     #默认 run 目录
#   ./watch_config_compare.sh <run-dir>           # 指定 run 目录
#
# Ctrl-C 只退出监控，不会影响正在跑的计算。

set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RUN_DIR="${1:-$HERE/runs/config_compare_2026-07-26}"
CKPT="$RUN_DIR/checkpoints"
LOG="$RUN_DIR/run.log"
TOTAL=806          # 31 configurations x 2 models x 13 outer folds
STARTED_AT=$(date +%s)

if [ ! -d "$RUN_DIR" ]; then
  echo "run 目录不存在: $RUN_DIR" >&2
  exit 1
fi

bar() {                                  # bar <done> <total> <width>
  local done=$1 total=$2 width=$3
  local filled=$(( done * width / total ))
  local i
  printf '['
  for ((i = 0; i < width; i++)); do
    if [ "$i" -lt "$filled" ]; then printf '#'; else printf '.'; fi
  done
  printf ']'
}

while true; do
  clear 2>/dev/null || printf '\033[2J\033[H'
  NOW=$(date +%s)
  ELAPSED=$(( NOW - STARTED_AT ))

  DONE=$(ls "$CKPT" 2>/dev/null | wc -l | tr -d ' ')
  DONE=${DONE:-0}
  # macOS 的 pgrep 没有 -c，用管道数行
  PROCS=$(pgrep -f "config_compare_eval" 2>/dev/null | wc -l | tr -d ' ')
  PROCS=${PROCS:-0}

  echo "SpineSense 7.5 传感器配置比较"
  echo "run: $RUN_DIR"
  printf '监控已运行 %02d:%02d:%02d   |   工作进程 %s\n' \
    $((ELAPSED/3600)) $(((ELAPSED%3600)/60)) $((ELAPSED%60)) "$PROCS"
  echo "--------------------------------------------------------------------"

  printf '折进度  %s %s/%s (%s%%)\n' \
    "$(bar "$DONE" "$TOTAL" 40)" "$DONE" "$TOTAL" $(( DONE * 100 / TOTAL ))

  # 每个模型各自的完成度
  for MODEL in logistic rbf_svm; do
    M_DONE=$(ls "$CKPT" 2>/dev/null | grep -c "^${MODEL}__" || true)
    M_CFG=$(ls "$CKPT" 2>/dev/null | grep "^${MODEL}__" | sed -E "s/^${MODEL}__(.*)__T[0-9]+\.json$/\1/" | sort -u | wc -l | tr -d ' ')
    printf '  %-9s %s %3s/403 折   配置 %2s/31\n' \
      "$MODEL" "$(bar "${M_DONE:-0}" 403 24)" "${M_DONE:-0}" "${M_CFG:-0}"
  done

  echo "--------------------------------------------------------------------"

  # 正在进行中的配置（有折但未满 13）
  echo "进行中："
  ls "$CKPT" 2>/dev/null \
    | sed -E 's/__T[0-9]+\.json$//' \
    | sort | uniq -c \
    | awk '$1 < 13 {printf "  %-46s %2d/13\n", $2, $1}' \
    | head -8
  [ -z "$(ls "$CKPT" 2>/dev/null | sed -E 's/__T[0-9]+\.json$//' | sort | uniq -c | awk '$1<13')" ] \
    && echo "  (无——上一批刚收尾或正在启动下一批)"

  echo "--------------------------------------------------------------------"
  echo "日志尾部："
  tail -n 8 "$LOG" 2>/dev/null | sed 's/^/  /'

  # 复现门一旦判定，单独醒目提示
  if [ -f "$RUN_DIR/reproduction_check.json" ]; then
    echo "--------------------------------------------------------------------"
    if grep -q '"match": true' "$RUN_DIR/reproduction_check.json"; then
      echo "复现门：通过 ✓（与 Track A 归档逐行一致）"
    else
      echo "复现门：未通过 ✗ —— 见 reproduction_check.json，运行已中止"
    fi
  fi

  if [ -f "$RUN_DIR/run_manifest.json" ]; then
    echo "===================================================================="
    echo "全部完成。主表：$RUN_DIR/config_summary.csv"
    break
  fi

  sleep 10
done
