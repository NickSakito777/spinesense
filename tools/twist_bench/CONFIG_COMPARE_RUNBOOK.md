# 7.5 传感器配置比较 · Windows 运行手册

论文依据：`SpineSense FYP/论文写作/SS-Dissertation 中文稿子.md` 第 7.5 节「传感器配置比较」。
本次运行**取代** `SpineSense Dissertation 大框架 v3-fable-2026-07-22.md` 里的旧消融方案（固定根、逻辑回归+随机森林、池化指标、十项 Wilcoxon + Holm）。旧段落只作历史记录。

---

## 0. 启动 prompt

把下面整段贴给 Windows 上的 agent，不要改写：

```text
在 SpineSense FYP/tools/twist_bench 目录下执行第 7.5 节的传感器配置比较。任务书是同目录的
CONFIG_COMPARE_RUNBOOK.md，逐节照做，不要自行改动协议。

硬性约束：
1. 不修改 config_compare_plan.py、config_compare_eval.py、win_compat.py、locked_track_a/
   任何文件、runs/ablation_2026-07-20/、runs/allpairs_2026-07-21/、runs/locked_track_a_2026-07-21/
   的任何内容。这些是冻结输入与协议本体。
2. 计划一旦冻结（run_plan.json 变只读），不得再生成或覆盖。
3. 五颗全用配置的复现是第一项实际运行。它不通过就停下来报告，不要继续跑其余 30 个配置，
   也不要调参数让它通过。
4. 不做任何显著性检验：不跑 sign-flip、不做 Holm、不报 p 值。脚本里没有这些函数，不要加。
5. 不新增模型、不改网格、不改折划分、不改种子、不改权重方案。
6. 跑完后不解读结果、不写论文结论。只报告：三道门是否通过、31 个配置的分数表、
   四组比较的差值与区间、模型族排序相关，以及任何 warning / failure。

若任何一步的实际观测与本手册不一致（哈希、行数、分数、目录结构），停下来报告差异，
不要绕过。报告里要给出具体数字和文件路径。
```

---

## 1. 环境

Track A 的环境锁（`runs/locked_track_a_2026-07-21/p0/environment_lock.json`）：

| 包 | 版本 |
|---|---|
| Python | 3.12.10 |
| numpy | 2.5.1 |
| pandas | 3.0.3 |
| scikit-learn | 1.9.0 |
| scipy | 1.18.0 |

Windows 上建虚拟环境并装同版本：

```bat
py -3.12 -m venv .win_venv
.win_venv\Scripts\python -m pip install --upgrade pip
.win_venv\Scripts\python -m pip install numpy==2.5.1 pandas==3.0.3 scikit-learn==1.9.0 scipy==1.18.0
```

版本不一致不会被静默接受——第 3 节的复现门会因为数值不同而失败。这是设计意图。

**Windows 兼容说明**：`locked_track_a/core.py` 在模块层 `import resource`（POSIX 专有），Windows 上会直接 ImportError。`win_compat.py` 在 core 之前注册一个替身模块，只提供 `getrusage().ru_maxrss`（core 里唯一的用途，用于日志里的峰值内存）。**不影响任何分数、折划分或参数选择**，并且会写进 `environment_lock.json` 的 `windows_compatibility` 字段留痕。POSIX 上它是空操作。

---

## 2. 冻结运行计划（不算分）

```bat
.win_venv\Scripts\python config_compare_plan.py --out-dir runs\config_compare_2026-07-26
```

预期输出：

```text
configurations  : 31
models          : ['logistic', 'rbf_svm']
contrasts       : ['C1', 'C2', 'C3', 'C4']
detailed configs: ['sacrum', 'sacrum+lower+mid+upper', 'sacrum+lower+mid+upper+sternum', 'sacrum+upper']
reproduction    : sacrum+lower+mid+upper+sternum -> {'logistic': 0.9544739220062376, 'rbf_svm': 0.9631404545650836}
```

`run_plan.json` 写完即设为只读，同目录留 `run_plan.sha256`。计划里锁定了：31 个配置及其**列名与列顺序**、折划分、两个模型的完整参数网格、四组比较、区间算法、随机种子、全部输入与代码哈希。

那两个复现目标值是**从 Track A 归档读出来的**，不是手写常量。

---

## 3. 运行（复现优先）

```bat
.win_venv\Scripts\python config_compare_eval.py --run-dir runs\config_compare_2026-07-26 --jobs 6
```

`--jobs` 取物理核数减一二。进程池用 spawn，脚本已有 `if __name__ == "__main__"` 保护。

**执行顺序**

1. **计划门** — `run_plan.json` 的哈希必须等于 sidecar；`ablation_build.py`、`locked_track_a/core.py`、`placement_maps_v1.json`、本脚本自身的哈希必须与计划一致。
2. **数据门** — 三项：行身份（subject / trial_id / block / bout_index / y / label 逐行一致）、列身份（130 个 `pair_*` 列名**与顺序**一致）、数值一致（共享列最大绝对差 ≤ 1e-12；Mac 实测为 `0.0`）。另检 solo 65 列无 NaN / inf、无全局常量列。
3. **复现运行** — 五颗全用配置（130 维）两个模型跑完整嵌套流程，逐项比对 Track A 归档：

   | 比对项 | 判据 |
   |---|---|
   | 每行外层预测 | 1387 行逐行相同，0 处不符 |
   | 13 折选定参数 | `selected_config_id` 逐折相同 |
   | 13 人分数向量 | 每人 `fixed6_macro_f1` 差值 ≤ 1e-12 |
   | 参与者等权标量 | logistic `0.9544739220062376`、rbf_svm `0.9631404545650836`，差值 ≤ 1e-12 |

   任一不符 → 脚本写出 `reproduction_check.json` 后中止。**这时不要重跑、不要调参，把 JSON 发回来。**

4. **其余 30 个配置** × 2 个模型。按 `网格大小 × 特征维数` 降序调度，重的先开。
5. **汇总** — 31 行主表、四组比较的差值与 95% 百分位区间、模型族排序相关、四个配置的混淆矩阵。

**断点续跑**：检查点粒度是 `模型 × 配置 × 外层参与者`（`checkpoints/<model>__<config>__<T??>.json`，共 31×2×13 = 806 个）。中断后原样重跑同一条命令即可，已完成的折直接跳过。

**只想验复现、先不跑全量**：

```bat
.win_venv\Scripts\python config_compare_eval.py --run-dir runs\config_compare_2026-07-26 --jobs 2 --stop-after-reproduction
```

---

## 4. 预计耗时

从 Track A 自己的 `primary/inner_scores.csv` 实测外推（130 维、单个配置）：

| 模型 | 内层拟合 | 单次中位数 | 单配置耗时 |
|---|---|---|---|
| logistic | 2028 = 13 网格 × 13 外 × 12 内 | 0.466 s | 79 min |
| rbf_svm | 1872 = 12 网格 × 156 | 0.043 s | 1.6 min |

31 个配置的特征维数总和 1105，是单个 130 维配置的 8.5 倍：

- logistic ≈ 11 h 单线程；6 并发下墙上时间约 2–3 h，下限由最重的那个 130 维任务（79 min）决定
- rbf_svm ≈ 15–50 min

Mac 实测参考：rbf_svm 单个外层折（12×12 内层 + 1 次外层）7.0 s。

---

## 5. 产出清单

`runs/config_compare_2026-07-26/`：

| 文件 | 内容 |
|---|---|
| `run_plan.json` / `run_plan.sha256` | 只读运行计划 + 哈希 |
| `data_gate.json` | 数据门实测值 |
| `environment_lock.json` | 版本 + Windows 兼容标记 |
| `reproduction_check.json` | 复现逐项比对 |
| `checkpoints/*.json` | 806 个检查点，含每折内层各参数分数、选定参数、逐行预测 |
| `inner_scores.csv` | 内层各参数分数（全量长表） |
| `selected_configs.csv` | 每折选定参数 + 并列个数 |
| `outer_predictions.csv` | 外层逐行预测 |
| `participant_metrics.csv` | 逐参与者指标 |
| `config_summary.csv` | **31 行主表，进 8.4 的那张** |
| `config_results.json` | 31 个配置全量结果 |
| `focused_contrasts.json` | 四组比较：逐人差值、均值、95% 区间 |
| `model_family_robustness.json` | Spearman 排序相关 + 四组符号一致性 |
| `confusion/*.json` | 4 个配置 × 2 模型的按参与者归一化混淆矩阵 |
| `ledgers/warnings_failures.jsonl` | 拟合告警与失败 |
| `ledgers/runtime.jsonl` | 任务完成记录 |
| `run_manifest.json` | 全部输入 / 代码 / 输出哈希 + 环境 + 证据边界 |

传回 Mac 时**整个目录一起**，`checkpoints/` 不要删——它是审计与续跑的依据。

---

## 6. 协议速查（用于核对，不要改）

| 项 | 值 | 继承自 |
|---|---|---|
| 配置 | 31 个：k=1 用单点通道；k≥2 全配对，**无固定根/星形** | 7.5 正文 |
| 特征维数 | 13 / 13 / 39 / 78 / 130（k=1…5） | 7.5 正文 |
| 片段 | 1387 行全纳入，不按质量筛 | 7.4 主分析 |
| 外层 | 13 折留一参与者 | Track A |
| 内层 | 12 折留一参与者，指标为留出者的 fixed-six macro F1 | Track A |
| 选参规则 | 内层均分最高；最高分 0.01 以内取 `simplicity_rank` 最小者 | `core.select_config` |
| 预处理 | 中位数填补 → `VarianceThreshold(0)` → 标准化，只在训练折 fit | `core.FittedProcedure` |
| 权重 | `subject_only` | Track A `primary_subject_only` |
| 种子 | 20260721 | Track A |
| 主指标 | 参与者等权 fixed-six macro F1 | Track A |
| 主模型 | 正则化逻辑回归，13 个网格点 | 7.5 正文 |
| 稳健性模型 | 径向基核 SVM，12 个网格点 | 7.5 正文 |
| 区间 | 参与者整体重抽 10000 次、种子 20260721、95% 百分位 | 7.5 + Track A |
| 检验 | **无**。不跑 sign-flip、不做 Holm、不报 p 值 | 7.5 正文 |
| 敏感性场景 | 7.4 的三组（换权重 / 剔 T15 / 换种子）**不在 7.5 重跑** | 7.5 未承诺 |

## 7. 四组比较

| 编号 | 左 | 右 | 维数 | 设计理由 |
|---|---|---|---|---|
| C1 | 五颗全用 (130) | 骶部单颗 (13) | 不匹配 | 骶部是 5.3 的骨盆根参考，其余位置的运动都相对它表达 |
| C2 | 五颗全用 (130) | 骶部+上背部 (13) | 不匹配 | 后者正是 7.3 测量验证的弯曲跨度（manifest 原文：`IMU1 sacrum to IMU4 upper`） |
| C3 | 骶部+上背部 (13) | 骶部单颗 (13) | **匹配** | 四组中唯一维数相同；拟合容量固定，但传感器数量、位置与测量对象仍同时改变 |
| C4 | 五颗全用 (130) | 后侧四颗 (78) | 不匹配 | 保留与移除前侧胸骨支线的对照 |

差值方向一律为**左减右**。

## 8. 证据边界（写进 manifest，报告时照抄）

分数描述的是**整套配置**在预分段六分类任务上的预测表现。传感器数量、位置、单点与差分测量方式、特征维数四者同时变化，因此本次运行的任何结果都不能用来证明某颗传感器必要、选定最优传感器数量，或填写任何需求状态。
