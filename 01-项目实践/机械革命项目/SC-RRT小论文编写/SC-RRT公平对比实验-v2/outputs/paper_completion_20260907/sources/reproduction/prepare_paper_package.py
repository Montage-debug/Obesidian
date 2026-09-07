#!/usr/bin/env python3
"""按地图聚类的论文统计包。用 bundled Python 运行，无绘图库依赖。"""
from __future__ import annotations

import argparse
import hashlib
import itertools
import json
import math
import platform
import sys
from pathlib import Path
import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parent
OUT = ROOT / "outputs/paper_completion_20260907"
MAIN = ["RRT", "RRT-Connect", "RRT*", "Informed-RRT*", "Dynamic-RRT", "SC-RRT"]
METRICS = ["first_solution_attempt", "first_solution_time_s", "first_solution_nodes",
           "first_solution_cost_mm", "first_solution_efficiency", "fixed_budget_cost_mm",
           "fixed_budget_time_s", "total_nodes", "collision_checks", "fixed_budget_efficiency",
           "efficiency_all_runs", "first_attempt_capped", "success"]
REPS = 10000


def clean(data):
    d = data.copy()
    d = d.replace([np.inf, -np.inf], np.nan)
    d["efficiency_all_runs"] = d.fixed_budget_efficiency.fillna(0)
    d["first_attempt_capped"] = d.first_solution_attempt.fillna(5000)
    return d


def validate(d, expected, label):
    assert len(d) == expected, (label, len(d), expected)
    assert not d.duplicated(["dimension", "environment_id", "seed", "algorithm"]).any()
    assert (d.expansion_attempts == 5000).all()
    assert (d.loc[d.success == 1, "path_valid"] == 1).all()
    assert d.groupby(["dimension", "algorithm", "environment_id"]).size().eq(50).all()
    assert d.groupby(["dimension", "algorithm"]).size().eq(500).all()
    solved=d[d.success==1]
    assert np.isfinite(solved.fixed_budget_cost_mm).all()
    assert np.allclose(solved.fixed_budget_efficiency,1350*np.sqrt(solved.dimension)/solved.fixed_budget_cost_mm)
    assert np.allclose(solved.first_solution_efficiency,1350*np.sqrt(solved.dimension)/solved.first_solution_cost_mm)
    assert (solved.first_solution_attempt<=5000).all()
    return {"dataset": label, "rows":len(d), "invalid_successful_paths":0,
            "duplicate_keys":0, "budget":5000, "maps_per_dimension":10, "seeds_per_map":50}


def wilson(k, n):
    z = 1.959963984540054
    p = k / n
    center = (p + z*z/(2*n)) / (1+z*z/n)
    half = z * math.sqrt(p*(1-p)/n+z*z/(4*n*n))/(1+z*z/n)
    return center-half, center+half


def summary(d, dataset):
    rows = []
    for (dim, algorithm), g in d.groupby(["dimension", "algorithm"], sort=False):
        lo, hi = wilson(int(g.success.sum()), len(g))
        base = dict(dataset=dataset, dimension=int(dim), algorithm=algorithm, n=len(g),
                    successful_n=int(g.success.sum()), success_rate=g.success.mean(),
                    success_wilson_low=lo, success_wilson_high=hi)
        for metric in METRICS:
            x = g[metric].dropna().to_numpy()
            rows.append({**base, "metric":metric, "valid_n":len(x), "mean":np.mean(x),
                         "std":np.std(x, ddof=1), "median":np.median(x),
                         "q1":np.quantile(x,.25), "q3":np.quantile(x,.75)})
    return pd.DataFrame(rows)


def hierarchical_ci(groups, seed):
    """先抽地图，再在每次抽到的地图内抽配对种子；同一组两算法始终成对。"""
    rng = np.random.default_rng(seed)
    m = len(groups)
    boot = np.zeros((REPS, 2))
    selected = rng.integers(0, m, size=(REPS, m))
    for slot in range(m):
        for map_id, pairs in enumerate(groups):
            positions = np.flatnonzero(selected[:, slot] == map_id)
            if not len(positions):
                continue
            idx = rng.integers(0, len(pairs), size=(len(positions), len(pairs)))
            boot[positions] += pairs[idx].mean(axis=1) / m
    delta = boot[:, 0] - boot[:, 1]
    relative = np.divide(delta * 100, boot[:, 0], out=np.full(REPS,np.nan), where=boot[:,0]!=0)
    return np.quantile(delta,[.025,.975]), np.nanquantile(relative,[.025,.975])


def map_wilcoxon(deltas):
    """小样本条件精确 signed-rank：非零差值秩的所有符号，含并列平均秩。"""
    x = np.asarray(deltas)
    x = x[np.abs(x) > 1e-10]
    if not len(x):
        return 1.0
    ranks = pd.Series(np.abs(x)).rank(method="average").to_numpy()
    observed = abs(np.dot(np.sign(x), ranks))
    signs = np.asarray(list(itertools.product([-1, 1], repeat=len(x))))
    return float(np.mean(np.abs(signs @ ranks) >= observed - 1e-10))


def holm(p):
    p = np.asarray(p)
    order = np.argsort(p)
    out = np.empty(len(p))
    out[order] = np.minimum(1,np.maximum.accumulate(p[order] * np.arange(len(p),0,-1)))
    return out


def comparisons(data, refs, dataset):
    rows, map_rows = [], []
    for dim in (2,3):
        block = data[data.dimension==dim]
        sc = block[block.algorithm=="SC-RRT"].set_index(["environment_id","seed"])
        for ref in refs:
            other = block[block.algorithm==ref].set_index(["environment_id","seed"])
            for metric in ["fixed_budget_cost_mm", "fixed_budget_time_s", "first_solution_time_s",
                           "first_attempt_capped", "total_nodes", "collision_checks", "efficiency_all_runs"]:
                joined = other[[metric]].join(sc[[metric]], lsuffix="_ref",rsuffix="_sc").dropna()
                groups = []
                for env, g in joined.groupby(level=0):
                    pairs = g.to_numpy()
                    groups.append(pairs)
                    a,b = pairs.mean(axis=0)
                    map_rows.append(dict(dataset=dataset,dimension=dim,reference=ref,metric=metric,
                                         environment_id=env,paired_n=len(g),reference_mean=a,sc_mean=b,
                                         reference_minus_sc=a-b, improvement_percent=100*(a-b)/a))
                means = np.asarray([g.mean(axis=0) for g in groups])
                deltas = means[:,0]-means[:,1]
                seed = int(hashlib.sha256(f"{dataset}{dim}{ref}{metric}".encode()).hexdigest()[:8],16)
                ci, ci_pct = hierarchical_ci(groups,seed)
                a,b = means.mean(axis=0)
                direction = -1 if metric=="efficiency_all_runs" else 1
                rows.append(dict(dataset=dataset,dimension=dim,reference=ref,metric=metric,
                    paired_n=len(joined),map_n=len(groups),reference_mean=a,sc_mean=b,
                    reference_minus_sc=a-b,relative_difference_percent=100*(a-b)/a,
                    hierarchy_ci_low=ci[0],hierarchy_ci_high=ci[1],
                    hierarchy_pct_ci_low=ci_pct[0],hierarchy_pct_ci_high=ci_pct[1],
                    sc_better_maps=int(np.sum(direction*deltas>1e-10)),
                    tied_maps=int(np.sum(abs(deltas)<=1e-10)),
                    map_wilcoxon_p=map_wilcoxon(deltas)))
    result = pd.DataFrame(rows)
    # 每项指标覆盖此批次全部参照算法及两维度；不跨不同统计终点混用 p。
    for metric, idx in result.groupby("metric").groups.items():
        result.loc[idx,"map_wilcoxon_p_holm"] = holm(result.loc[idx,"map_wilcoxon_p"].to_numpy())
    return result,pd.DataFrame(map_rows)


def pm(g, metric, digits):
    x=g[metric].dropna()
    return f"{x.mean():.{digits}f} ± {x.std(ddof=1):.{digits}f}"


def paper_table(d, dim, algorithms, mode):
    rows=[]
    for alg in algorithms:
        g=d[(d.dimension==dim)&(d.algorithm==alg)]
        if not len(g):continue
        display={"Dynamic-RRT":"Dynamic-RRT (local adaptation)",
                 "SC-RRT-no-ADCS":"SC-RRT w/o informed-sampling/feedback bundle",
                 "SC-RRT-no-SSFOR":"SC-RRT fixed (early activation; legacy)",
                 "SC-RRT-fixed-matched":"SC-RRT fixed (matched activation)"}.get(alg,alg)
        row={"Algorithm":display,"Success (n/N)":f"{int(g.success.sum())}/{len(g)}"}
        if mode=="first":
            row.update({"First path length (mm)":pm(g,"first_solution_cost_mm",2),
                        "First-solution time (s)":pm(g,"first_solution_time_s",4),
                        "Extension attempts":pm(g,"first_solution_attempt",1),
                        "Nodes at first solution":pm(g,"first_solution_nodes",1),
                        "First path efficiency":pm(g,"first_solution_efficiency",4)})
        else:
            row.update({"Path length at 5000 (mm)":pm(g,"fixed_budget_cost_mm",2),
                        "Total time to 5000 (s)":pm(g,"fixed_budget_time_s",4),
                        "Total tree nodes":pm(g,"total_nodes",1),
                        "Path efficiency":pm(g,"fixed_budget_efficiency",4),
                        "Counted edge checks":pm(g,"collision_checks",1)})
        rows.append(row)
    return pd.DataFrame(rows)


def write_table(df, name):
    df.to_csv(OUT/"tables"/(name+".csv"),index=False)
    headers = df.columns.tolist()
    lines=["| "+" | ".join(headers)+" |", "| "+" | ".join(["---"]*len(headers))+" |"]
    lines += ["| "+" | ".join(str(x) for x in row)+" |" for row in df.itertuples(index=False,name=None)]
    (OUT/"tables"/(name+".md")).write_text("\n".join(lines)+"\n",encoding="utf8")


def check_traces(data, paths):
    checks=[]
    for label,path in paths:
        trace=pd.read_csv(path)
        raw=pd.read_csv(path.parent/"runs.csv")
        assert len(trace)==len(raw)*8
        assert not trace.duplicated(["environment_id","seed","algorithm","checkpoint"]).any()
        keys=["environment_id","seed","algorithm"]
        for _,g in trace.groupby(keys):
            x=g.sort_values("checkpoint").best_cost_mm.fillna(np.inf).to_numpy()
            assert len(x)==8 and np.all(x[1:]<=x[:-1]+1e-8)
        last=trace[trace.checkpoint==5000].set_index(keys)["best_cost_mm"]
        target=raw.set_index(keys)["fixed_budget_cost_mm"].reindex(last.index)
        assert len(last)==len(raw) and np.allclose(last,target.replace(np.inf,np.nan),equal_nan=True)
        checks.append({"dataset":label,"trace_rows":len(trace),"checkpoint_count_per_run":8})
    return checks


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--partial",action="store_true",help="仅准备已完成历史数据，不生成最终结论")
    args=parser.parse_args()
    for sub in ["tables","statistics","figures","sources"]:(OUT/sub).mkdir(parents=True,exist_ok=True)
    main_raw=pd.read_csv(ROOT/"results/formal_main/runs.csv")
    old_raw=pd.read_csv(ROOT/"results/formal_ablation/runs.csv")
    checks=[validate(main_raw,6000,"main"),validate(old_raw,3000,"legacy_ablation")]
    checks+=check_traces(None,[("main",ROOT/"results/formal_main/traces.csv"),("legacy_ablation",ROOT/"results/formal_ablation/traces.csv")])
    main_d,old_d=clean(main_raw),clean(old_raw)
    main_s=summary(main_d,"main")
    main_s.to_csv(OUT/"statistics/main_summary_long.csv",index=False)
    old_s=summary(old_d,"legacy_ablation")
    old_s.to_csv(OUT/"statistics/legacy_ablation_summary_long.csv",index=False)
    cp,mp=comparisons(main_d,MAIN[:-1],"main")
    cp.to_csv(OUT/"statistics/main_cluster_comparisons.csv",index=False)
    mp.to_csv(OUT/"statistics/main_per_map_differences.csv",index=False)
    op,om=comparisons(old_d,["SC-RRT-no-ADCS","SC-RRT-no-SSFOR"],"legacy_ablation")
    op.to_csv(OUT/"statistics/legacy_ablation_cluster_comparisons.csv",index=False)
    om.to_csv(OUT/"statistics/legacy_ablation_per_map_differences.csv",index=False)
    for dim in (2,3):
        write_table(paper_table(main_d,dim,MAIN,"first"),f"Tab_main_{dim}D_first")
        write_table(paper_table(main_d,dim,MAIN,"fixed"),f"Tab_main_{dim}D_5000")
        write_table(paper_table(old_d,dim,["SC-RRT","SC-RRT-no-ADCS","SC-RRT-no-SSFOR"],"fixed"),f"Tab_legacy_ablation_{dim}D")
        write_table(paper_table(old_d,dim,["SC-RRT","SC-RRT-no-ADCS"],"fixed"),f"Tab_bundle_ablation_{dim}D")
    datasets={"main":main_s,"legacy_ablation":old_s}
    if not args.partial:
        matched_raw=pd.read_csv(ROOT/"results/formal_matched_ssfor/runs.csv")
        checks.append(validate(matched_raw,2000,"matched_ssfor"))
        matched=clean(matched_raw)
        ms=summary(matched,"matched_ssfor")
        ms.to_csv(OUT/"statistics/matched_ssfor_summary_long.csv",index=False)
        c,m=comparisons(matched,["SC-RRT-fixed-matched"],"matched_ssfor")
        c.to_csv(OUT/"statistics/matched_ssfor_cluster_comparisons.csv",index=False)
        m.to_csv(OUT/"statistics/matched_ssfor_per_map_differences.csv",index=False)
        for dim in (2,3):
            write_table(paper_table(matched,dim,["SC-RRT","SC-RRT-fixed-matched"],"fixed"),f"Tab_matched_SSFOR_{dim}D")
        datasets["matched_ssfor"]=ms
        records=[json.loads(x) for x in (ROOT/"results/formal_matched_ssfor/records.jsonl").read_text().splitlines()]
        groups={}
        for r in records:
            key=(r["run"]["environment_id"],r["run"]["seed"])
            groups.setdefault(key,[]).append(r)
            costs=[v for k,v in sorted(r["cost_trace"].items(),key=lambda kv:int(kv[0]))]
            assert len(costs)==8 and np.all(np.asarray(costs[1:])<=np.asarray(costs[:-1])+1e-8)
            assert abs(costs[-1]-r["run"]["fixed_budget_cost_mm"])<1e-7
        for pair in groups.values():
            assert len(pair)==2 and pair[0]["activation_snapshot"]==pair[1]["activation_snapshot"]
            assert pair[0]["activation_snapshot"] is not None
        activation=matched.groupby(["dimension","algorithm"]).activation_attempt.agg(["min","median","max"])
        activation.to_csv(OUT/"statistics/activation_attempts.csv")
        checks.append({"matched_pairs_with_identical_preactivation_state":len(groups)})
    (OUT/"sources/verification.json").write_text(json.dumps(checks,ensure_ascii=False,indent=2))
    analysis_meta={"python":sys.version,"platform":platform.platform(),"numpy":np.__version__,
                   "pandas":pd.__version__,"bootstrap_replicates":REPS,
                   "bootstrap_seed_rule":"first 8 hexadecimal digits of SHA256(dataset+dimension+reference+metric)",
                   "analysis_script_sha256":hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                   "partial":bool(args.partial),"analysis_is_retrospective_supplement":True}
    (OUT/"sources/analysis_metadata.json").write_text(json.dumps(analysis_meta,ensure_ascii=False,indent=2))
    # 数值型工作簿输入；分析在可复现脚本中完成，工作簿不伪装成可编辑模型。
    payload={name:json.loads(frame.to_json(orient="records")) for name,frame in datasets.items()}
    payload["comparisons"]={"main":json.loads(cp.to_json(orient="records")),
                            "legacy_ablation":json.loads(op.to_json(orient="records"))}
    payload["per_map"]={"main":json.loads(mp.to_json(orient="records")),
                        "legacy_ablation":json.loads(om.to_json(orient="records"))}
    if not args.partial:
        payload["comparisons"]["matched_ssfor"]=json.loads(c.to_json(orient="records"))
        payload["per_map"]["matched_ssfor"]=json.loads(m.to_json(orient="records"))
    (OUT/"sources/workbook_data.json").write_text(json.dumps(payload,ensure_ascii=False))
    print("Prepared",OUT,"partial="+str(args.partial),flush=True)
    print(op[op.metric=="fixed_budget_cost_mm"][["dimension","reference","relative_difference_percent","sc_better_maps","hierarchy_ci_low","hierarchy_ci_high","map_wilcoxon_p_holm"]].to_string(index=False))


if __name__=="__main__":main()
