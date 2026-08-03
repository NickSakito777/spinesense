# 交接包：7.5 传感器配置比较（Windows 执行）

> 发给 Windows 机的单一入口文件。第 1 节是直接粘贴给 agent 的 prompt，第 2 节是要拷过去的文件清单，第 3 节起是执行步骤。
> 详细任务书在 `CONFIG_COMPARE_RUNBOOK.md`，随包一起拷过去。

---

## 1. 粘贴给 Windows 上 agent 的 prompt

复制下面整段，不要改写：

```text
在 SpineSense FYP/tools/twist_bench 目录下执行 SpineSense 毕业论文第 7.5 节
「传感器配置比较」的计算。完整任务书是同目录的 CONFIG_COMPARE_RUNBOOK.md，
先完整读一遍再动手，逐节照做。

你的角色是执行者，不是设计者。协议已经冻结，你的工作是让它跑完并把证据带回来。

硬性约束：
1. 不修改以下任何文件：config_compare_plan.py、config_compare_eval.py、
   win_compat.py、locked_track_a/ 下任何文件、ablation_build.py、
   config/placement_maps_v1.json、runs/ 下任何已存在的内容。
   这些是冻结输入与协议本体。你只能新建 runs/config_compare_2026-07-26/。
2. 运行计划一旦冻结（run_plan.json 会变成只读），不得再生成或覆盖，
   不得使用 --force。
3. 五颗全用配置的复现是第一项实际运行，不是预检。它不通过就立刻停下来报告，
   不要继续跑其余 30 个配置，不要重跑试运气，不要改任何参数让它通过。
4. 不做任何显著性检验：不跑 sign-flip、不做 Holm 校正、不报 p 值。
   脚本里故意没有这些函数，不要添加。
5. 不新增模型、不改参数网格、不改折划分、不改随机种子、不改权重方案、
   不改特征列或列顺序。
6. 依赖版本必须完全一致：Python 3.12、numpy 2.5.1、pandas 3.0.3、
   scikit-learn 1.9.0、scipy 1.18.0。版本不同会导致数值不同，复现门会拦住你。
   自己新建 .win_venv；目录里已有的 .venv 与 .track_a_venv 是 macOS 虚拟环境，
   在 Windows 上不可用，不要尝试调用它们里面的任何可执行文件。
7. 跑完不要解读结果、不要写论文结论、不要判断"五颗传感器是否必要"。
   这些是论文作者的工作。

需要你报告的内容：
- 三道门（计划门、数据门、复现门）各自是否通过，附实测数字
- 31 个配置的分数表（config_summary.csv 的内容）
- 四组比较的差值与 95% 区间（focused_contrasts.json）
- 模型族排序相关（model_family_robustness.json）
- 全部 warning 与 failure（ledgers/ 下两个 jsonl）
- 总耗时

若任何一步的实际观测与任务书不一致（文件哈希、行数、分数、目录结构、
版本号），停下来报告差异，给出具体数字和文件路径，不要绕过、不要自行修补。

跑完把整个 runs/config_compare_2026-07-26/ 目录打包传回，checkpoints/ 子目录
不要删除，它是审计与断点续跑的依据。
```

---

## 2. 要拷过去的文件（17 个，9.6 MB）

保持**相对路径结构不变**，根目录是 `SpineSense FYP/tools/twist_bench/`。

| 文件 | 类别 | 大小 | sha256 前 16 位 |
|---|---|---|---|
| `config_compare_plan.py` | 代码 | 19 KB | `93cdb069f37234a6` |
| `config_compare_eval.py` | 代码 | 43 KB | `0a9c40aac7100067` |
| `win_compat.py` | 代码 | 3 KB | `f9f35eb159472483` |
| `CONFIG_COMPARE_RUNBOOK.md` | 任务书 | 10 KB | `bb16f175ed31a639` |
| `HANDOFF-7.5-config-compare.md` | 本文件 | — | — |
| `locked_track_a/__init__.py` | 协议本体 | 0 KB | `bb2804b9fc5aa12d` |
| `locked_track_a/core.py` | 协议本体 | 27 KB | `5ac472961c4b4aad` |
| `ablation_build.py` | 特征构建程序 | 13 KB | `a82e7517d14349bf` |
| `config/placement_maps_v1.json` | 映射表 | 13 KB | `0daaa092ac9b0547` |
| `runs/ablation_2026-07-20/channel_features.csv` | **冻结数据** | 4.3 MB | `a22c2b3d874537f9` |
| `runs/ablation_2026-07-20/ablation_manifest.json` | **冻结数据** | 2 KB | `eb99464ac621de26` |
| `runs/allpairs_2026-07-21/features_model.csv` | **冻结数据** | 2.7 MB | `f4bef2ed69fcd92e` |
| `runs/allpairs_2026-07-21/dataset_manifest.json` | **冻结数据** | 7 KB | `eb855ee3b2defbdf` |
| `runs/locked_track_a_2026-07-21/primary/outer_predictions.csv` | 复现对照 | 2.5 MB | `13177453f746ed36` |
| `runs/locked_track_a_2026-07-21/primary/participant_metrics.csv` | 复现对照 | 29 KB | `7c3999eff2a27bfa` |
| `runs/locked_track_a_2026-07-21/primary/model_summary.csv` | 复现对照 | 4 KB | `c0fa3ac3927aa6e4` |
| `runs/locked_track_a_2026-07-21/primary/scenario_manifest.json` | 复现对照 | 27 KB | `74e18369fee3e6af` |
| `runs/locked_track_a_2026-07-21/p0/environment_lock.json` | 版本参照 | 4 KB | `554a8cf23620e81f` |

`locked_track_a/runner.py`（3060 行）**不用拷**。`core.py` 不依赖它，`__init__.py` 也只导入 core。少拷一个，少一处出错的地方。

### 传输方式：必须走二进制

**用 zip 打包传，或用二进制模式的同步工具。不要让任何工具做文本模式的换行转换。**

理由：脚本按 sha256 校验 `channel_features.csv` 与 `features_model.csv`。Windows 上如果 CRLF 转换动了这两个 CSV 一个字节，哈希就变，数据门直接拦住——那时候排查方向会跑偏到"数据坏了"，而其实只是传输方式不对。

Mac 上打包：

```bash
cd "SpineSense FYP/tools/twist_bench" && zip -r ~/Desktop/spinesense-7.5-handoff.zip config_compare_plan.py config_compare_eval.py win_compat.py CONFIG_COMPARE_RUNBOOK.md HANDOFF-7.5-config-compare.md ablation_build.py locked_track_a/__init__.py locked_track_a/core.py config/placement_maps_v1.json runs/ablation_2026-07-20/channel_features.csv runs/ablation_2026-07-20/ablation_manifest.json runs/allpairs_2026-07-21/features_model.csv runs/allpairs_2026-07-21/dataset_manifest.json runs/locked_track_a_2026-07-21/primary/outer_predictions.csv runs/locked_track_a_2026-07-21/primary/participant_metrics.csv runs/locked_track_a_2026-07-21/primary/model_summary.csv runs/locked_track_a_2026-07-21/primary/scenario_manifest.json runs/locked_track_a_2026-07-21/p0/environment_lock.json
```

Windows 上到货后先验哈希（两个大 CSV 是关键）：

```bat
certutil -hashfile runs\ablation_2026-07-20\channel_features.csv SHA256
certutil -hashfile runs\allpairs_2026-07-21\features_model.csv SHA256
```

前 16 位应为 `a22c2b3d874537f9` 和 `f4bef2ed69fcd92e`。不对就重传，别往下走。

### 方式二：共享整个 `SpineSense FYP` 文件夹

本次运行的全部代码与数据都在 `SpineSense FYP/tools/twist_bench/` 树内，所以共享该文件夹也能跑。但要注意三件事：

**一、体积不对等。** 整个 `SpineSense FYP` 是 6.7 GB，本次运行只需要 9.6 MB。其中 `tools/twist_bench/data`（2.1 GB）与 `data_clean`（1.6 GB）运行完全不碰。

**二、不要碰那两个 macOS 虚拟环境。** `tools/twist_bench/.venv` 与 `.track_a_venv` 是 macOS 下建的，里面是 macOS 二进制，Windows 上 `bin/python` 不存在（Windows 用 `Scripts\python.exe`）。**必须新建 `.win_venv`，不要尝试调用这两个目录里的任何东西。**

**三、输出不要写进共享目录。** 运行过程会写 806 个检查点 JSON，同步服务处理密集小文件写入容易产生冲突副本或半写文件；检查点一旦损坏，断点续跑会读到坏 JSON。

输入路径是相对脚本位置解析的，运行目录由 `--run-dir` 独立指定，两者可以分开。所以共享文件夹模式下这样跑——**共享目录当只读输入，结果写本地盘**：

```bat
D:\spinesense_run\.win_venv\Scripts\python config_compare_plan.py --out-dir D:\spinesense_run\config_compare_2026-07-26
```

```bat
D:\spinesense_run\.win_venv\Scripts\python config_compare_eval.py --run-dir D:\spinesense_run\config_compare_2026-07-26 --jobs 6
```

虚拟环境也建在共享目录之外。跑完把 `D:\spinesense_run\config_compare_2026-07-26\` 整个拷回共享文件夹的 `tools/twist_bench/runs/` 下。

共享模式下同样先用 `certutil` 验一下那两个大 CSV 的哈希——同步服务一般不动二进制内容，但验一次只要几秒。

---

## 3. 环境

```bat
py -3.12 -m venv .win_venv
.win_venv\Scripts\python -m pip install --upgrade pip
.win_venv\Scripts\python -m pip install numpy==2.5.1 pandas==3.0.3 scikit-learn==1.9.0 scipy==1.18.0
.win_venv\Scripts\python -c "import numpy,pandas,sklearn,scipy;print(numpy.__version__,pandas.__version__,sklearn.__version__,scipy.__version__)"
```

最后一行应打印 `2.5.1 3.0.3 1.9.0 1.18.0`。

**Windows 兼容说明**（不用手动做，脚本已处理）：`locked_track_a/core.py` 在模块层 `import resource`，这是 POSIX 专有模块，Windows 上会直接 ImportError。`win_compat.py` 在 core 之前注册一个替身，只提供 `getrusage().ru_maxrss`——core 里唯一的用途是日志里的峰值内存。不影响任何分数、折划分或参数选择，并会写进 `environment_lock.json` 的 `windows_compatibility` 字段留痕。

---

## 4. 三条命令

```bat
.win_venv\Scripts\python config_compare_plan.py --out-dir runs\config_compare_2026-07-26
```

```bat
.win_venv\Scripts\python config_compare_eval.py --run-dir runs\config_compare_2026-07-26 --jobs 2 --stop-after-reproduction
```

```bat
.win_venv\Scripts\python config_compare_eval.py --run-dir runs\config_compare_2026-07-26 --jobs 6
```

第二条先只验复现（rbf_svm 约 2 分钟出结论，logistic 那个 130 维的要 79 分钟）。过了再跑第三条全量。第三条会自动跳过已完成的检查点，中断后原样重跑即可续。

`--jobs` 取物理核数减一二。

---

## 5. 验收标准

### 计划冻结应打印

```text
configurations  : 31
models          : ['logistic', 'rbf_svm']
contrasts       : ['C1', 'C2', 'C3', 'C4']
detailed configs: ['sacrum', 'sacrum+lower+mid+upper', 'sacrum+lower+mid+upper+sternum', 'sacrum+upper']
reproduction    : sacrum+lower+mid+upper+sternum -> {'logistic': 0.9544739220062376, 'rbf_svm': 0.9631404545650836}
```

### 数据门应打印

```text
data gate     : 1387 rows, 13 participants, shared-column max abs diff 0.0
```

`1387` 与 `13` 必须一致。共享列最大绝对差 Mac 实测为 `0.0`，门限是 `≤1e-12`。

### 复现门必须两个模型都 OK

```text
reproduction  : logistic  rows=1387 pred_mismatch=0 fold_mismatch=0 scalar_delta=0.000e+00 OK
reproduction  : rbf_svm   rows=1387 pred_mismatch=0 fold_mismatch=0 scalar_delta=0.000e+00 OK
```

四项判据：1387 行外层预测逐行相同、13 折选定参数逐折相同、13 人分数向量每人差值 ≤1e-12、参与者等权标量差值 ≤1e-12。

Mac 上已实测过 rbf_svm × T02 折：选定参数 `rbf_svm:00` 与归档相同、120 行预测 0 处不符、参与者 F1 `0.9834943639291466` 与归档**逐位相同**。所以这道门在正确环境下应当通过；不通过就是环境或传输的问题，把 `reproduction_check.json` 发回来。

### 预计耗时

| 模型 | 单配置（130 维实测） | 31 个配置合计 |
|---|---|---|
| logistic | 79 min（2028 次内层拟合，中位 0.466 s） | 单线程约 11 h；6 并发约 2–3 h |
| rbf_svm | 1.6 min（1872 次，中位 0.043 s） | 15–50 min |

31 个配置的特征维数总和 1105，是单个 130 维配置的 8.5 倍。6 并发下的墙上时间下限由最重的那个 130 维 logistic 任务（79 min）决定。

---

## 6. 传回来的东西

整个 `runs/config_compare_2026-07-26/` 目录打包，**`checkpoints/` 不要删**（806 个文件，是审计与续跑依据）。

| 文件 | 内容 |
|---|---|
| `run_plan.json` / `run_plan.sha256` | 只读运行计划 |
| `data_gate.json` | 数据门实测值 |
| `environment_lock.json` | 版本 + Windows 兼容标记 |
| `reproduction_check.json` | 复现四项逐一比对 |
| `config_summary.csv` | **31 行主表，直接进 8.4** |
| `config_results.json` | 31 个配置全量结果 |
| `focused_contrasts.json` | 四组比较：逐人差值、均值、95% 区间 |
| `model_family_robustness.json` | Spearman 排序相关 + 符号一致性 |
| `inner_scores.csv` | 内层各参数分数 |
| `selected_configs.csv` | 每折选定参数 |
| `outer_predictions.csv` | 外层逐行预测 |
| `participant_metrics.csv` | 逐参与者指标 |
| `confusion/*.json` | 4 配置 × 2 模型的按参与者归一化混淆矩阵 |
| `checkpoints/*.json` | 806 个检查点 |
| `ledgers/*.jsonl` | 告警 / 失败 / 运行记录 |
| `run_manifest.json` | 全部哈希 + 环境 + 证据边界 |

---

## 7. 协议速查（用于核对，任何一项对不上就停）

| 项 | 值 |
|---|---|
| 配置 | 31 个：k=1 用单点通道；k≥2 全配对，**无固定根/星形** |
| 特征维数 | 13 / 13 / 39 / 78 / 130（k=1…5） |
| 片段 | 1387 行全纳入，不按质量筛 |
| 外层 | 13 折留一参与者 |
| 内层 | 12 折留一参与者；指标为留出者的 fixed-six macro F1 |
| 选参规则 | 内层均分最高；最高分 0.01 以内取 simplicity_rank 最小者 |
| 预处理 | 中位数填补 → VarianceThreshold(0) → 标准化，只在训练折 fit |
| 权重 | `subject_only` |
| 种子 | 20260721 |
| 主指标 | 参与者等权 fixed-six macro F1 |
| 主模型 | 正则化逻辑回归，13 个网格点 |
| 稳健性模型 | 径向基核 SVM，12 个网格点 |
| 区间 | 参与者整体重抽 10000 次、种子 20260721、95% 百分位 |
| 检验 | **无** |
| 敏感性场景 | 7.4 的三组不在 7.5 重跑 |

### 四组比较（差值一律左减右）

| 编号 | 左 | 右 | 维数 | 设计理由 |
|---|---|---|---|---|
| C1 | 五颗全用 (130) | 骶部单颗 (13) | 不匹配 | 骶部是 5.3 的骨盆根参考 |
| C2 | 五颗全用 (130) | 骶部+上背部 (13) | 不匹配 | 后者是 7.3 测量验证的弯曲跨度 |
| C3 | 骶部+上背部 (13) | 骶部单颗 (13) | **匹配** | 四组中唯一维数相同 |
| C4 | 五颗全用 (130) | 后侧四颗 (78) | 不匹配 | 有无前侧胸骨支线的对照 |

---

## 8. 本次运行取代旧方案

`SpineSense Dissertation 大框架 v3-fable-2026-07-22.md` 里的消融段落仍写着固定根通道、逻辑回归+随机森林、池化指标、十项 Wilcoxon + Holm 家族。**那些都不适用**。当前依据是活动稿第 7.5 节：k≥2 全配对、逻辑回归为主 + 径向基核 SVM 作模型族检查、参与者等权宏平均 F1、四组比较只报差值与区间。

这条取代声明会写进 `run_plan.json` 的 `supersedes` 字段和 `run_manifest.json`，不靠人记。

## 9. 证据边界

分数描述的是**整套配置**在预分段六分类任务上的预测表现。传感器数量、位置、单点与差分测量方式、特征维数四者同时变化，因此本次运行的任何结果都不能用来证明某颗传感器必要、选定最优传感器数量，或填写任何需求状态。
