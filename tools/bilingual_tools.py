#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""SS-Dissertation 双语工作稿的两个例行操作。

用法：
    python3 bilingual_tools.py count     # 刷新标题上的 ⟨N w　目标 M⟩（就地改写双语稿）
    python3 bilingual_tools.py export    # 从双语稿抽出纯英文，覆盖 English Version Draft
    python3 bilingual_tools.py report    # 只打印词数表，不改任何文件

词数口径（与 Fox 2026-07-31 的裁剪表一致）：正文 + 标题 + 表格与表注计入；
图片、Figure 图注、`\\cite{}` 键、行间公式不计。
"""
import io, re, sys, os

BASE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "论文写作")
BILINGUAL = os.path.join(BASE, "SS-Dissertation 中文稿子.md")
ENGLISH = os.path.join(BASE, "SS-Dissertation English Version Draft.md")

# Fox 2026-07-31 裁剪目标
TARGET = {
    "Part I": 70, "Part II": 70, "3": 230, "4.1": 120, "4.2": 300, "4.3": 339, "4.4": 200,
    "4.5": 350, "4.6": 300, "5": 70, "5.1": 350, "5.2": 230,
    "5.3": 527,
    "6": 60, "6.1": 400, "6.2": 210, "6.3": 110,
    "7": 40, "7.1": 30, "7.2": 200, "7.3": 80,
}
# 大框架 v4 §0.5 章级预算
BUDGET = {"1": 1500, "2": 200, "11": 1000, "12": 250}

CJK = re.compile(r"[\u4e00-\u9fff]")
ANNOT = re.compile(r"　⟨[^⟩]*⟩\s*$")
PARTS = (("# 第一部分", "Part I"), ("# Part I", "Part I"),
         ("# 第二部分", "Part II"), ("# Part II", "Part II"))


def part_name(line):
    """一级标题若是部分标题，返回其统计键，否则 None。"""
    for prefix, key in PARTS:
        if line.startswith(prefix):
            return key
    return None


def is_en(line):
    """双语稿里判定一行是否为英文正文（无汉字、非结构行）。"""
    s = line.strip()
    if not s or CJK.search(s):
        return False
    return not s.startswith(("#", "---", "|", "$$", "![[", ">"))


def _span(lines, lo_pat, hi_pat):
    try:
        lo = next(i for i, l in enumerate(lines) if l.startswith(lo_pat))
        hi = next(i for i, l in enumerate(lines) if l.startswith(hi_pat))
        return lines[lo:hi]
    except StopIteration:
        return []


def section_words(lines):
    """按节统计英文词数。已双语化的区段：Ch1-7 与 Ch11-12。Ch8-10 仍是单语中文，不计。"""
    lines = (_span(lines, "## 1 引言", "## 8 原型")
             + _span(lines, "## 11 讨论", "## TIPS"))
    words, cur = {}, None
    for l in lines:
        if l.startswith("#"):
            m = re.match(r"^#+\s+(\S+)", l)
            cur = part_name(l) or (m.group(1) if m else None)
            if cur:
                words.setdefault(cur, 0)
            continue
        if cur is None:
            continue
        s = l.strip()
        if not s or s.startswith(("![[", "$$", "---")):
            continue
        if s.startswith("> ") and re.match(r"^> (Figure|图)\s", s):
            continue          # 图注不计
        if s.startswith("|") and not re.sub(r"[|\-\s]", "", s):
            continue          # 表格分隔行不计
        if CJK.search(s):
            continue          # 中文段与中文表格行不计
        s = s.lstrip("\u27f5 ")          # 迁入标记不计词
        words[cur] += len(re.sub(r"\\cite\{[^}]*\}", "", s).split())
    return words


def rollup(words, num):
    if num not in words:
        return None
    return sum(w for k, w in words.items() if k == num or k.startswith(num + "."))


def rollup_target(num):
    ts = [v for k, v in TARGET.items() if k == num or k.startswith(num + ".")]
    return sum(ts) if ts else None


def annotate(words, num):
    w = rollup(words, num)
    if w is None:
        return ""
    t, b = rollup_target(num), BUDGET.get(num)
    if t is not None:
        return "　⟨%d w　目标 %d⟩" % (w, t)
    if b is not None:
        return "　⟨%d w　预算 %d⟩" % (w, b)
    return "　⟨%d w⟩" % w


def load():
    return io.open(BILINGUAL, encoding="utf-8").read().split("\n")


def cmd_count(write=True):
    lines = load()
    words = section_words(lines)
    out = []
    for l in lines:
        if l.startswith("#"):
            m = re.match(r"^#+\s+(\S+)", l)
            num = part_name(l) or (m.group(1) if m else "")
            base = ANNOT.sub("", l)
            a = annotate(words, num)
            out.append(base + a if a else base)
        else:
            out.append(l)
    if write:
        io.open(BILINGUAL, "w", encoding="utf-8").write("\n".join(out))
        print("✓ 已刷新词数注记: %s" % os.path.basename(BILINGUAL))
    return words


def cmd_report():
    words = section_words(load())
    print("%-10s %8s %8s %8s" % ("节", "实际", "目标", "差"))
    print("-" * 38)
    tot_a = tot_t = 0
    for k in sorted(words, key=lambda x: (not x.startswith("Part"), x)):
        a = rollup(words, k)
        t = rollup_target(k) or BUDGET.get(k)
        d = ("%+d" % (a - t)) if t else "—"
        print("%-10s %8d %8s %8s" % (k, a, t if t else "—", d))
        if "." not in k:                       # 只累加顶层，避免重复计
            tot_a += a
            tot_t += (t or 0)
    print("-" * 38)
    print("%-10s %8d %8d %+8d" % ("合计", tot_a, tot_t, tot_a - tot_t))


def cmd_export():
    lines = load()
    try:
        s = next(i for i, l in enumerate(lines) if l.startswith("## 1 引言"))
        e = next(i for i, l in enumerate(lines) if l.startswith("## 6 分类方法"))
    except StopIteration:
        sys.exit("找不到 Ch1 起点或 Ch6 终点，双语稿结构可能变了")

    out, prev_blank = [], True
    for l in lines[s:e]:
        st = l.strip()
        if st.startswith("#"):                                  # 标题：只留英文侧
            hashes = re.match(r"^(#+)", l).group(1)
            body = ANNOT.sub("", l)[len(hashes) + 1:]
            parts = body.split("　")
            num = parts[0].split(" ")[0]
            en = parts[1] if len(parts) > 1 else parts[0]
            out += ["", "%s %s %s" % (hashes, num, en), ""]
            prev_blank = True
            continue
        if st.startswith(("![[", "$$")) or st == "---":         # 中英同源，照抄
            out.append(l); prev_blank = False; continue
        if st.startswith("|") and not CJK.search(st):           # 英文表
            out.append(l); prev_blank = False; continue
        if st.startswith("> ") and not CJK.search(st):          # 英文图注/表注
            out.append(l); prev_blank = False; continue
        if is_en(l):
            out.append(l); prev_blank = False; continue
        if not st and not prev_blank:
            out.append(""); prev_blank = True
    # 折叠多余空行
    txt = re.sub(r"\n{3,}", "\n\n", "\n".join(out)).strip() + "\n"
    head = io.open(ENGLISH, encoding="utf-8").read().split("## 1 Introduction")[0]
    io.open(ENGLISH, "w", encoding="utf-8").write(head + txt)
    print("✓ 已导出英文: %s（%d 词）" % (os.path.basename(ENGLISH), len(txt.split())))


if __name__ == "__main__":
    cmd = sys.argv[1] if len(sys.argv) > 1 else "report"
    {"count": cmd_count, "export": cmd_export, "report": cmd_report}.get(cmd, cmd_report)()
