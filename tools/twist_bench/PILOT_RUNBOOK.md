# SpineSense Validation1 — MoCap Pilot 运行手册

> 技术测试(不是最终精度验证)。目标:5 IMU 稳定贴放 + 完整记录 + MoCap 给 thorax-vs-pelvis twist 真值 + 留可 ablation 的数据。

## 0. 现状(已就绪,不用再动)

- 固件:`imu_i3c_xyz_sflp` 已烧、已硬件验证(raw + SFLP 同出,125 Hz,5 颗 |q|≈1)。出问题可回烧 `imu_i3c_xyz`(raw-only)。
- 接线映射(已被实测弯腰数据佐证):**U1=Sacrum=IMU0,U2=T12=IMU1,U3=T6=IMU2,U4=T3=IMU3,U5=Sternum=IMU4**,已写入 `data\validation1_layout.json`。
- 3 个程序:`capture_trial.py`(录,Ctrl-C 停)→ `validation1_preflight.py`(当场验)→ `validation1_analysis.py`(回去后处理,自动选最新 + 自动用 layout)。
- VSCode 端口 COM3、波特率 921600 已是默认,直接 Run 即可。

## 一次性设置(每开一次 VSCode)

右下角选 **`.venv` 解释器**(`...\twist_bench\.venv`)。不选 `preflight`/`analysis` 会 `import numpy` 报错。

---

## 1. 每个受试者:贴 + 准备

1. 贴 5 IMU:U1 Sacrum / U2 T12 / U3 T6 / U4 T3 / U5 Sternum。
2. MoCap marker(优先级 **IMU 本体 > 骨盆 > 胸廓**):
   - 每个 IMU 板贴 **3 个非共线 marker** → 刚体簇(直接给每颗 IMU 的 optical orientation)。
   - 骨盆:左右 ASIS、左右 PSIS(至少 PSIS + 骶骨簇)。
   - 胸廓:胸骨上切迹、剑突、C7、T8/T10(不够就胸骨 + C7/T8)。
3. 拍照:正 / 背 / 侧。记 IMU↔位置。
4. MoCap:建 thorax / pelvis / 每个 IMU 刚体 → 标定捕捉体积 → 确认所有 marker 可见。
5. 改 `data\validation1_layout.json` 里的 `subject_id`(如 P01)。
6. 让受试者做几个小动作:看 marker 不掉点、IMU 不滑。

## 2. sync 事件(代替"敲一下")

固定全程用同一种,二选一:
- **脚跟顿地**(首选):踮脚 → 脚跟用力顿地一下。全身一个垂直冲击,所有 IMU 同时尖峰、MoCap 全身 marker 同时下沉。不弯腰、不易滑片。
- **快速深屈弹回**:用力快速前屈到底再立刻弹直,一次。

做法:**静 2s → 做一次 → 静 2s**(前后静止把边界框清楚)。

## 3. 录一整场(一条 capture,一键)

> 6 个动作一次录完。动作之间静止 8-10s(目测即可,不用准);**开头和结尾必须干净静止**。

1. MoCap:起一个**长 take**(整场一个)。
2. VSCode:Run `capture_trial.py`(开始录,终端会提示"Ctrl-C 停")。
3. 受试者按时间轴做:

```
站稳 10s（死站不动）              ← bias/tare 基线，必须干净
sync 一次（脚跟顿地 / 快屈）
动作1  前屈-回正 ×3        → 静 8-10s
动作2  左侧弯/右侧弯 各×3   → 静 8-10s
动作3  骨盆固定的左右扭转 各×3 → 静 8-10s   ★最关键
动作4  自然左右扭转 各×3    → 静 8-10s
动作5  弯+扭 组合 ×3       → 静 8-10s
动作6  连续序列 ×1-2        → 站稳 10s
  连续序列示例：前屈→回正→左侧弯→回正→右扭→回正
sync 再来一次（结尾，用于查时钟漂移）
```

4. 做完 → 在采集终端按 **Ctrl-C** 停 → 停 MoCap take。
   - 每个动作都**慢做**,每次回中立;静止段让全 5 颗 gyro 掉到接近 0。

## 4. 当场抽验 + 命名

1. VSCode:Run `validation1_preflight.py`(自动验最新那条)。
   - 要看到 `raw accel/gyro PASS` + `5 IMU PASS` + `SFLP PRESENT, norm OK (|q|~1)`。
   - `still gyro quality FAIL` 正常(整场有动作),忽略。
2. 把 `data\twist_trial_<时间戳>.log` 重命名 `P01.log`;MoCap take 导出 `P01.c3d` 同名。
3. 填记录表(下)。

## 5. 现场记录表(每个受试者一行)

`subject ID · 各动作完成情况 · IMU 是否滑动 · marker 是否遮挡/丢失 · 线缆是否拉扯 · neutral 是否稳 · 两个 sync 是否明显 · 异常备注`
→ 决定哪条进 ablation、哪条只当 smoke test。

## 6. 换下一个受试者

重贴 → 改 `validation1_layout.json` 的 `subject_id` → 回第 1 节重复。

---

## 7. 后处理(回实验室)

1. 每条 Run `validation1_analysis.py`(裸跑自动选最新 + 用 layout;批量就 `--input data\P01.log`)→ 出 `features.csv / qc.json / summary.md`,自动切 full_B / posterior_only / no_T6 / no_sternum / minimal / sternum_pelvis,再对 MoCap 的 thorax-pelvis axial rotation。
2. **[待建工具]** 分段器(按静止段把一条大 log 切成每个动作)+ SFLP-vs-VQF 一致性表(原型已验证:弯腰数据两路相关性 0.94–0.99)。
3. 写 `KB/output/SpineSense-Validation1-results-2026-06-26.md`:回填实际 placement_map、滑动/marker/sync 状况、raw+SFLP 是否都存上、哪些 ablation 可分析。**再**决定是否更新算法页。

## 8. 成功标准(达到即成功)

5 IMU 贴住完整记录 · MoCap 看到主要 marker · neutral/twist/侧弯/前屈都采到 · 每条有动作标签 + 明显 sync · 后续能做 full_B/posterior_only/no_T6/no_sternum ablation。

## 9. 不能说 / 可以说

- 别说:sternum 已解决 twist drift;IMU 靠位移算 twist;已证明最终精度。
- 可说:验证 sternum 能否作 anterior thorax reference、T6 能否作 mid-thoracic anchor;MoCap 作 thorax-pelvis axial rotation 真值;后续 ablation 比 sternum/T6 贡献。

---

### 一句话循环

起 MoCap take → Run `capture_trial.py` → 死站10s·sync·6动作(中间静8-10s)·死站10s·sync → Ctrl-C → 停 MoCap → Run `preflight` 抽验 → 重命名 → 换人。
