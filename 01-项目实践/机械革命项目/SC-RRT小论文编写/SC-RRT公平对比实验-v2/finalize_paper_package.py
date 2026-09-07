#!/usr/bin/env python3
"""生成数据驱动中文解读、复制复现输入、校验清单与压缩交付包。"""
from pathlib import Path
import argparse
import hashlib
import json
import shutil
import zipfile
import xml.etree.ElementTree as ET
import numpy as np
import pandas as pd

ROOT=Path(__file__).resolve().parent
OUT=ROOT/"outputs/paper_completion_20260907"


def interpretation():
    main=pd.read_csv(OUT/"statistics/main_summary_long.csv")
    def stat(dim,alg,metric,col="mean"):
        return float(main[(main.dimension==dim)&(main.algorithm==alg)&(main.metric==metric)].iloc[0][col])
    old=pd.read_csv(OUT/"statistics/legacy_ablation_cluster_comparisons.csv")
    matched=pd.read_csv(OUT/"statistics/matched_ssfor_cluster_comparisons.csv")
    activation=pd.read_csv(OUT/"statistics/activation_attempts.csv")
    lines=["# 结果解读：主对比、消融与可写结论", "",
        "以下为实际完成数据的汇总。主对比 6000 次、历史消融 3000 次、新增等启动 SSFOR 对照 2000 次，共 11000 次运行记录。不同批次重复的完整 SC-RRT 不合并为独立样本。", "",
        "## 1. 二维、三维六实现主对比", "",
        "完整数值、样本SD和成功数见 tables/Tab_main_*。第六个 Dynamic-RRT 是本地改写版，不能据此概括原作者方法。", ""]
    for dim in (2,3):
        sc=stat(dim,"SC-RRT","fixed_budget_cost_mm")
        connect=stat(dim,"RRT-Connect","fixed_budget_cost_mm")
        st=stat(dim,"SC-RRT","first_solution_time_s")
        ct=stat(dim,"RRT-Connect","first_solution_time_s")
        rr=stat(dim,"RRT*","fixed_budget_cost_mm")
        inf=stat(dim,"Informed-RRT*","fixed_budget_cost_mm")
        lines += [f"### {dim}D", "",
            f"SC-RRT 的 5000 次扩展终值路径长度为 **{sc:.2f} mm**，RRT-Connect 为 **{connect:.2f} mm**；SC-RRT 平均短 **{100*(connect-sc)/connect:.2f}%**。",
            f"但 SC-RRT 的平均首解时间为 **{st:.4f} s**，RRT-Connect 为 **{ct:.4f} s**，首解扩展数分别为 **{stat(dim,'SC-RRT','first_solution_attempt'):.1f}** 和 **{stat(dim,'RRT-Connect','first_solution_attempt'):.1f}**。因此不能写成 SC-RRT 比 RRT-Connect 更快得到首解。",
            f"RRT* 与 Informed-RRT* 的终值平均长度分别为 **{rr:.2f} mm** 和 **{inf:.2f} mm**，都低于 SC-RRT。SC-RRT 在本设置中体现的是部分速度/质量权衡，而不是终值路径最短。", ""]
    lines += ["## 2. 整组移除消融：收益小，但地图方向一致", "",
        "参照为关闭知情采样及反馈的配置（原始字段 no-ADCS）。此比较不是非对称域对称化的单因素消融。", ""]
    for dim in (2,3):
        r=old[(old.dimension==dim)&(old.reference=="SC-RRT-no-ADCS")&(old.metric=="fixed_budget_cost_mm")].iloc[0]
        lines += [f"- {dim}D：整组移除 **{r.reference_mean:.2f} mm** → 完整方法 **{r.sc_mean:.2f} mm**，缩短 **{r.relative_difference_percent:.3f}%**；分层 bootstrap 95% CI 为 **[{r.hierarchy_pct_ci_low:.3f}%, {r.hierarchy_pct_ci_high:.3f}%]**；**{int(r.sc_better_maps)}/10** 张地图改善，地图级 Holm 校正 p = **{r.map_wilcoxon_p_holm:.6f}**。"]
    lines += ["", "可写：在所测试地图上，完整知情采样/反馈组合带来幅度有限但跨地图一致的终值路径缩短。不可写：非对称性单独贡献了全部收益。", "",
              "## 3. 等启动条件 SSFOR：新补跑的主要结论", "",
              "两方法在相同状态下开始知情采样。固定版本启用后取 gamma=1.5、p=0.8；完整版本使用反馈律。各维度各500对，启动记录逐对一致，完整 SC-RRT 的路径长度、终值节点数与边检查计数均复现主实验。", ""]
    for dim in (2,3):
        r=matched[(matched.dimension==dim)&(matched.metric=="fixed_budget_cost_mm")].iloc[0]
        state="缩短" if r.relative_difference_percent>0 else "增长"
        meaning=("区间整体为正，支持本固定参照下的增量缩短。" if r.hierarchy_pct_ci_low>0 else
                 "区间整体为负，完整反馈在该维度不如这个固定参照。" if r.hierarchy_pct_ci_high<0 else
                 "区间跨过0，尚无稳健的平均路径缩短证据。")
        lines += [f"### {dim}D 匹配对照", "",
             f"固定参数 **{r.reference_mean:.4f} mm**，完整反馈 **{r.sc_mean:.4f} mm**。完整反馈平均{state} **{abs(r.relative_difference_percent):.4f}%**，有符号相对改善的 95% CI 为 **[{r.hierarchy_pct_ci_low:.4f}%, {r.hierarchy_pct_ci_high:.4f}%]**。",
             f"**{int(r.sc_better_maps)}/10** 张地图的均值更短，地图级 Holm 校正 p = **{r.map_wilcoxon_p_holm:.6f}**。{meaning}", ""]
    lines += ["这里比较的是现有反馈律与单一固定参数配置，不是反馈对所有固定参数的胜出证明。收益按完整5000次预算衡量，不把相对剩余误差的较大百分比替代总路径长度改进。", "",
        "二维的 signed-rank 校正 p 略低于0.05，而分层均值差区间跨0，两者使用的统计量和不确定性构造不同，不是同一检验的严格反演。不要只摘 p 值称收益稳健；本文对二维平均缩短采取保守解释。", "",
        "### 启动窗口", ""]
    for dim in (2,3):
        row=activation[(activation.dimension==dim)&(activation.algorithm=="SC-RRT")].iloc[0]
        lines += [f"- {dim}D 首次启动尝试数：最小 {int(row['min'])}、中位数 {row['median']:.1f}、最大 {int(row['max'])}。"]
    lines += ["", "当前控制作用主要位于预算后半段。首解通常早于启动，所以本消融不支持将首解优势归因于 SSFOR。", "",
        "## 4. 旧固定参数对照为什么不进主要因果结论", "",
        "旧固定对照在首解后立即启用知情采样，完整反馈等待反馈窗口。旧结果中完整方法二维略差、三维略好，混合了启动时机和反馈律两个变化。旧数据完整保留，新结果不是删除不利试验，而是针对已识别混杂补充的对照。", "",
        "## 5. 可直接用于结果章节的写作顺序", "",
        "第一段报告样本规模、成功数和首解/终值两个终点。第二段按二维、三维分别讨论 SC-RRT 与 Connect 的质量—首解速度权衡，再主动指出 RRT*/Informed-RRT* 的终值质量优势。第三段报告整组移除的真实幅度。第四段使用上面的匹配 SSFOR 数值及区间，解释效应是否稳定，而不是只报 p 值。最后写适用范围和局限。", "",
        "**建议现在开始写初稿，不再为增加百分比随意改算法。** 但若论文坚持六种原版算法严格复现或非对称性单独创新，相关缺口仍需单独完成；现有材料不能把这两点视为已经解决。", "",
        "消融数值小不自动等于不能发表，显著也不自动等于贡献足够。本文应围绕清晰的任务需求、可解释的有限收益、准确的实现说明和完整的实证证据组织；不承诺具体分区录用。", ""]
    (OUT/"结果解读.md").write_text("\n".join(lines),encoding="utf8")
    improve={dim:100*(stat(dim,"RRT-Connect","fixed_budget_cost_mm")-stat(dim,"SC-RRT","fixed_budget_cost_mm"))/stat(dim,"RRT-Connect","fixed_budget_cost_mm") for dim in (2,3)}
    matched_rows=matched[matched.metric=="fixed_budget_cost_mm"].set_index("dimension")
    ssfor_sentence="；".join(f"{dim}D 的有符号路径缩短为 {matched_rows.loc[dim,'relative_difference_percent']:.3f}%" for dim in (2,3))
    abstract=f"""# 中文摘要草稿

建议题目：**SC-RRT：融合双向知情采样与反馈调节的有限预算路径规划**

针对静态障碍环境中可行解获得速度与有限扩展预算下路径质量之间的权衡，本文研究一种结合双向扩展、局部重布线、两侧知情采样域与反馈参数调节的 SC-RRT 规划实现。采用首解与固定扩展预算两个评价终点，将连接过程的每一步计为一次原子扩展，并在首解后统一运行至5000次扩展。实验在二维和三维空间中各使用10张冻结地图、每图50个规划随机种子。相较 RRT-Connect，SC-RRT 的终值平均路径长度在二维和三维中分别缩短 {improve[2]:.2f}% 和 {improve[3]:.2f}%，但首解速度未超过 RRT-Connect，且终值路径仍长于 RRT* 和 Informed-RRT*。整组机制移除与相同启动条件下的固定参数对照用于区分组合机制的净收益和反馈调节的增量作用。匹配反馈对照中，{ssfor_sentence}，其中二维分层95%区间跨0，三维区间整体为正。结果说明该实现呈现了具有场景与预算限制的速度—质量权衡，反馈收益应作有限解释。

关键词：采样式路径规划；双向RRT；知情采样；反馈调节；消融实验

使用前请按目标期刊字数要求压缩。本草稿没有将本地 Dynamic-RRT 改写版称为原始算法，也没有声称非对称性得到单因素验证。摘要中的具体贡献措辞仍应与相关工作、新颖性论证和正文统计区间对应。若期刊摘要不宜报告过多比较细节，可保留主要质量—首解成本权衡，把匹配反馈的完整数字移入结果章节；不要只删除限制而保留有利数字。
"""
    (OUT/"中文摘要草稿.md").write_text(abstract,encoding="utf8")


def copy_sources():
    dest=OUT/"sources/reproduction"
    for folder in ["src","config","data/environments","tests"]:
        for src in (ROOT/folder).rglob("*"):
            if not src.is_file() or "__pycache__" in src.parts:continue
            target=dest/src.relative_to(ROOT);target.parent.mkdir(parents=True,exist_ok=True)
            shutil.copy2(src,target)
    for name in ["run_benchmark.py","run_matched_ablation.py","prepare_paper_package.py","make_paper_figures.py",
                 "finalize_paper_package.py","build_paper_workbook.mjs"]:
        shutil.copy2(ROOT/name,dest/name)
    for batch in ["formal_main","formal_ablation","formal_matched_ssfor"]:
        for filename in ["runs.csv","traces.csv","metadata.json","manifest.json","completion.json","records.jsonl"]:
            src=ROOT/"results"/batch/filename
            if not src.exists():continue
            target=dest/"results"/batch/filename;target.parent.mkdir(parents=True,exist_ok=True)
            shutil.copy2(src,target)
    text="""# 实验与统计复现

本目录保存三批原始记录、冻结地图、配置及本次使用的实现与分析脚本。

## 已有数据复算

在本目录运行 `python3 prepare_paper_package.py` 可从现有原始记录重新生成统计表。
需要 Python、NumPy、pandas。原实验版本详见各 results 子目录 metadata/manifest。
主机原实验 Python 3.10 / NumPy 1.26.4；补充统计使用已配置的 bundled Python / NumPy 2.3.5。
统计重采样由脚本固定种子；跨 NumPy 大版本不保证抽样位序完全相同。
`python3 make_paper_figures.py --replay` 重放预定代表路径并作图，需 matplotlib，路径重放应使用原实验 NumPy 版本。

## 重新运行实验

`run_benchmark.py` 默认续跑并跳过已完成键。真正新跑请用一个新的输出目录，例如：
`python3 run_benchmark.py --suite formal --output results/new_main`
`python3 run_benchmark.py --suite ablation --output results/new_ablation`
匹配补跑默认使用 `results/formal_matched_ssfor`，并以 `results/formal_main` 核对完整方法。
需重新执行匹配整批时，先备份现有对应结果目录，并在独立复制的工作目录操作；不要直接删除唯一原始记录。

## 范围说明

原始CSV中的Dynamic-RRT是本地改写版；no-ADCS是整组移除；no-SSFOR是历史提前启动固定版本。
正式论文表图已采用更准确的显示标签，原始ID保留不变以维持追溯。
匹配SSFOR的records.jsonl把一次运行的指标、代价轨迹和控制轨迹存为一行。
完整SC-RRT跨批次重复不能增加独立样本量。

Excel为研究结果展示，复杂统计并不在Excel中自动重算。其构建还需已配置的@oai/artifact-tool依赖。
"""
    (dest/"复现说明.md").write_text(text,encoding="utf8")
    report=ROOT/"results/paper_package_qa/tests.xml"
    tree=ET.parse(report)
    suites=list(tree.getroot().iter("testsuite"))
    tests=sum(int(x.attrib.get("tests",0)) for x in suites)
    failures=sum(int(x.attrib.get("failures",0))+int(x.attrib.get("errors",0)) for x in suites)
    assert tests==10 and failures==0
    shutil.copy2(report,OUT/"sources/test_report.xml")
    (OUT/"sources/delivery_verification.json").write_text(json.dumps({
        "unit_tests":tests,"test_failures":failures,"representative_paths_replayed":12,
        "scientific_figures_visually_reviewed":5,"workbook_sheets_visually_reviewed":9,
        "workbook_formula_error_scan_matches":0,
        "original_main_and_legacy_run_records_preserved":True,
    },ensure_ascii=False,indent=2))


def combined_tables():
    text=["# 论文表格合集", "", "所有 ± 表示样本SD。主表保留六个运行实现，其中 Dynamic-RRT 为本地改写。路径与首解指标条件于成功，总时间与节点包含全部运行。完整定义和消融解释范围见《论文写作指南.md》。", ""]
    entries=[("表2a 二维首解", "Tab_main_2D_first"),("表2b 三维首解", "Tab_main_3D_first"),
             ("表3a 二维5000次扩展终值", "Tab_main_2D_5000"),("表3b 三维5000次扩展终值", "Tab_main_3D_5000"),
             ("表4a 二维整组移除", "Tab_bundle_ablation_2D"),("表4b 三维整组移除", "Tab_bundle_ablation_3D"),
             ("表5a 二维等启动SSFOR", "Tab_matched_SSFOR_2D"),("表5b 三维等启动SSFOR", "Tab_matched_SSFOR_3D")]
    for title,name in entries:
        text.extend(["## "+title,"",(OUT/"tables"/(name+".md")).read_text(),""])
    text.extend(["## 表下注释", "",
         "每维度10张地图、每图50个种子。表4为整组知情采样/反馈移除，不单独隔离非对称性。表5中的完整SC-RRT来自同批匹配补跑；其路径与主实验重复一致，但时间不可跨批混合。旧提前启动固定对照保留在Tab_legacy_ablation文件中，不作为表5的替代。", "",
         "表中的已计数边检查不包含全部路径复核底层检查。5000次扩展不是等时间或等碰撞检查预算。若篇幅紧张，可将首解节点、终值节点和边检查移至补充材料，但保留成功率、长度、时间与预算定义。", ""])
    (OUT/"论文表格合集.md").write_text("\n".join(text),encoding="utf8")


def manifest_and_zip():
    files={}
    for p in sorted(OUT.rglob("*")):
        if p.is_file() and p.name!="SHA256SUMS.json":
            files[str(p.relative_to(OUT))]=hashlib.sha256(p.read_bytes()).hexdigest()
    (OUT/"SHA256SUMS.json").write_text(json.dumps(files,ensure_ascii=False,indent=2))
    archive=OUT.with_suffix(".zip")
    with zipfile.ZipFile(archive,"w",compression=zipfile.ZIP_DEFLATED,compresslevel=6) as z:
        for p in sorted(OUT.rglob("*")):
            if p.is_file():z.write(p,Path(OUT.name)/p.relative_to(OUT))
    print("Package",archive,"bytes",archive.stat().st_size)


if __name__=="__main__":
    parser=argparse.ArgumentParser();parser.add_argument("--text-only",action="store_true")
    args=parser.parse_args();interpretation();combined_tables()
    if not args.text_only:
        assert (OUT/"SC-RRT_二维三维对比与消融.xlsx").exists()
        assert len(list((OUT/"figures").glob("*.png")))==5
        copy_sources();manifest_and_zip()
