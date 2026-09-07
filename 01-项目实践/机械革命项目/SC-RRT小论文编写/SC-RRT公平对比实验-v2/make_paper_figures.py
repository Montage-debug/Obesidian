#!/usr/bin/env python3
"""从冻结结果制作科学图；代表路径固定为 map=0, seed_index=0，不选择最好结果。"""
from __future__ import annotations
import argparse
import json
import os
import sys
from pathlib import Path
import numpy as np
import pandas as pd

# 系统环境同时存在两套 mpl_toolkits。仅在本进程选取与 matplotlib 一致的用户安装。
import mpl_toolkits
_toolkit = Path.home() / ".local/lib/python3.10/site-packages/mpl_toolkits"
if _toolkit.exists():
    mpl_toolkits.__path__.insert(0, str(_toolkit))
os.environ.setdefault("MPLCONFIGDIR", "/tmp/sc_rrt_paper_mpl")
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

ROOT=Path(__file__).resolve().parent
OUT=ROOT/"outputs/paper_completion_20260907"
sys.path.insert(0,str(ROOT/"src"))
from environments import environment_from_dict
from planners import build_planner

ALGS=["RRT","RRT-Connect","RRT*","Informed-RRT*","Dynamic-RRT","SC-RRT"]
LABEL={a:a for a in ALGS}
LABEL["Dynamic-RRT"]="Dynamic-RRT (adapted)"
COLORS=dict(zip(ALGS,["#808080","#0072B2","#009E73","#CC79A7","#E69F00","#D55E00"]))
plt.rcParams.update({"font.family":"DejaVu Sans","font.size":10,"axes.spines.top":False,
    "axes.spines.right":False,"svg.fonttype":"none","savefig.facecolor":"white"})


def save(fig,name):
    padding=.40 if name=="Fig_paths_3D" else .10
    fig.savefig(OUT/"figures"/(name+".png"),dpi=300,bbox_inches="tight",pad_inches=padding)
    fig.savefig(OUT/"figures"/(name+".svg"),bbox_inches="tight",pad_inches=padding)
    plt.close(fig)


def replay():
    config=json.loads((ROOT/"config/protocol.json").read_text())
    raw=pd.read_csv(ROOT/"results/formal_main/runs.csv")
    rows=[]
    for dim in (2,3):
        env=environment_from_dict(json.loads((ROOT/f"data/environments/evaluation_{dim}d.json").read_text())["environments"][0])
        seed=dim*10_000_000
        for algorithm in ALGS:
            result=build_planner(algorithm,env,seed,config).plan()
            prior=raw[(raw.dimension==dim)&(raw.map_index==0)&(raw.seed_index==0)&(raw.algorithm==algorithm)].iloc[0]
            assert result.success and abs(result.best_cost-prior.fixed_budget_cost_mm)<1e-7
            assert result.total_nodes==prior.total_nodes
            rows.append({"dimension":dim,"environment_id":env.env_id,"seed":seed,"algorithm":algorithm,
                         "cost_mm":result.best_cost,"path":result.path.tolist(),"matches_original_cost_and_nodes":True})
            print("Replayed",dim,algorithm,flush=True)
    (OUT/"sources/representative_paths.json").write_text(json.dumps(rows,indent=2))


def path_figures():
    paths=json.loads((OUT/"sources/representative_paths.json").read_text())
    for dim in (2,3):
        env=environment_from_dict(json.loads((ROOT/f"data/environments/evaluation_{dim}d.json").read_text())["environments"][0])
        fig=plt.figure(figsize=(12,8))
        for i,alg in enumerate(ALGS):
            ax=fig.add_subplot(2,3,i+1,projection="3d" if dim==3 else None)
            record=next(x for x in paths if x["dimension"]==dim and x["algorithm"]==alg)
            p=np.array(record["path"])
            if dim==2:
                for x,y,r in env.obstacles:
                    ax.add_patch(Circle((x,y),r,fc="#C6CFD6",ec="#8C9AA6",lw=.35))
                ax.plot(*p.T,color=COLORS[alg],lw=1.8,zorder=3)
                ax.scatter(*env.start,color="#009E73",marker="o",s=32,zorder=4)
                ax.scatter(*env.goal,color="#171717",marker="*",s=70,zorder=4)
                ax.set_aspect("equal")
            else:
                # 所有400个球均绘出；透明表面便于观察路径。投影不表示二维碰撞关系。
                u=np.linspace(0,2*np.pi,9);v=np.linspace(0,np.pi,6)
                sphere=np.array([np.outer(np.cos(u),np.sin(v)),np.outer(np.sin(u),np.sin(v)),np.outer(np.ones_like(u),np.cos(v))])
                for x,y,z,r in env.obstacles:
                    ax.plot_surface(x+r*sphere[0],y+r*sphere[1],z+r*sphere[2],color="#AAB8C6",alpha=.19,linewidth=0,shade=False)
                ax.plot(*p.T,color=COLORS[alg],lw=2.0,zorder=10)
                ax.scatter(*env.start,color="#009E73",marker="o",s=32)
                ax.scatter(*env.goal,color="#171717",marker="*",s=70)
                ax.view_init(elev=22,azim=-58)
                ax.set_zlim(0,1500);ax.set_zticks([0,750,1500]);ax.set_zlabel("z (mm)",labelpad=1)
                ax.set_box_aspect((1,1,1));ax.tick_params(labelsize=8,pad=0)
            ax.set_xlim(0,1500);ax.set_ylim(0,1500)
            ax.set_xticks([0,750,1500]);ax.set_yticks([0,750,1500])
            if dim==3:ax.set_yticks([750,1500])  # 避免前角 x=1500 与 y=0 标签重叠。
            ax.set_xlabel("x (mm)",labelpad=1);ax.set_ylabel("y (mm)",labelpad=1)
            ax.set_title(f"({chr(97+i)}) {LABEL[alg]}\nL = {record['cost_mm']:.1f} mm",fontsize=10,pad=5)
        fig.subplots_adjust(wspace=.22,hspace=.30,top=.91,bottom=.06)
        fig.suptitle(f"{dim}D paths at 5000 extension attempts | map 00, seed index 00",fontsize=13)
        save(fig,f"Fig_paths_{dim}D")


def bootstrap_curves(values,seed):
    """地图级bootstrap，保持整条曲线一起重采样；阴影为逐点区间。"""
    rng=np.random.default_rng(seed)
    idx=rng.integers(0,len(values),size=(10000,len(values)))
    curves=values[idx].mean(axis=1)
    return values.mean(axis=0),np.quantile(curves,[.025,.975],axis=0)


def convergence():
    trace=pd.read_csv(ROOT/"results/formal_main/traces.csv")
    fig,axs=plt.subplots(2,2,figsize=(11.4,8),sharex=True)
    for col,dim in enumerate((2,3)):
        d=trace[trace.dimension==dim].copy()
        d["efficiency"]=1350*np.sqrt(dim)/d.best_cost_mm
        d["efficiency"]=d.efficiency.fillna(0)
        d["solved"]=d.best_cost_mm.notna().astype(float)
        for alg in ALGS:
            a=d[d.algorithm==alg]
            for row,metric in enumerate(("efficiency","solved")):
                maps=a.groupby(["environment_id","checkpoint"])[metric].mean().unstack()
                mean,ci=bootstrap_curves(maps.to_numpy(),dim*100+ALGS.index(alg))
                ax=axs[row,col];x=maps.columns.to_numpy()
                ax.plot(x,mean,color=COLORS[alg],label=LABEL[alg],lw=2 if alg=="SC-RRT" else 1.4,marker="o",ms=3)
                ax.fill_between(x,*ci,color=COLORS[alg],alpha=.09,lw=0)
                ax.set_ylim(0,1.025);ax.grid(alpha=.2)
        axs[0,col].set_title(f"{dim}D | 10 maps × 50 paired seeds")
        axs[1,col].set_xlabel("Atomic extension attempts")
    axs[0,0].set_ylabel("Mean efficiency (unsolved = 0)")
    axs[1,0].set_ylabel("Fraction with a valid solution")
    handles,labels=axs[0,0].get_legend_handles_labels()
    fig.legend(handles,labels,loc="lower center",ncol=3,frameon=False,bbox_to_anchor=(.5,-.025))
    fig.tight_layout(rect=(0,.06,1,1))
    save(fig,"Fig_main_convergence")


def ablation():
    old=pd.read_csv(OUT/"statistics/legacy_ablation_per_map_differences.csv")
    new=pd.read_csv(OUT/"statistics/matched_ssfor_per_map_differences.csv")
    oc=pd.read_csv(OUT/"statistics/legacy_ablation_cluster_comparisons.csv")
    nc=pd.read_csv(OUT/"statistics/matched_ssfor_cluster_comparisons.csv")
    fig,axs=plt.subplots(1,2,figsize=(11.4,4.5),sharey=True)
    refs=[("SC-RRT-no-ADCS","Without informed-sampling\nand feedback bundle",old,oc),
          ("SC-RRT-fixed-matched","Fixed parameters\n(matched activation)",new,nc)]
    for ax,dim in zip(axs,(2,3)):
        ax.axvline(0,c="#777777",lw=1,ls="--")
        for y,(ref,label,df,cp) in enumerate(refs):
            m=df[(df.dimension==dim)&(df.reference==ref)&(df.metric=="fixed_budget_cost_mm")]
            r=cp[(cp.dimension==dim)&(cp.reference==ref)&(cp.metric=="fixed_budget_cost_mm")].iloc[0]
            ax.scatter(m.improvement_percent,y+np.linspace(-.14,.14,len(m)),s=20,c="#9BABB8",alpha=.85,zorder=2)
            center=r.relative_difference_percent
            ax.errorbar(center,y,xerr=[[center-r.hierarchy_pct_ci_low],[r.hierarchy_pct_ci_high-center]],fmt="D",c="#D55E00",capsize=4,ms=6,zorder=3)
            ax.annotate(f"{center:+.2f}%",(center,y),xytext=(0,15),textcoords="offset points",ha="center",fontsize=10)
        ax.set_title(f"{dim}D terminal path-length improvement")
        ax.set_xlabel("100 × (reference − SC-RRT) / reference (%)")
        ax.set_yticks([0,1],[r[1] for r in refs]);ax.set_ylim(-.45,1.6);ax.grid(axis="x",alpha=.2)
    axs[0].invert_yaxis()
    fig.tight_layout()
    save(fig,"Fig_ablation_map_effects")


def controller():
    records=[]
    with (ROOT/"results/formal_matched_ssfor/records.jsonl").open() as f:
        for line in f:
            r=json.loads(line)
            if r["run"]["map_index"]==0 and r["run"]["seed_index"]==0:records.append(r)
    fig,axs=plt.subplots(3,2,figsize=(11.4,8),sharex=True)
    for col,dim in enumerate((2,3)):
        for r in records:
            if r["run"]["dimension"]!=dim:continue
            full=r["run"]["algorithm"]=="SC-RRT"
            label="SC-RRT feedback" if full else "Fixed, matched activation"
            color="#D55E00" if full else "#0072B2"
            tr=pd.DataFrame(r["control_trace"])
            for row,key in enumerate(("gamma","p","best_cost_mm")):
                axs[row,col].plot(tr.attempt,tr[key],label=label,c=color,ls="-" if full else "--",lw=1.8)
            for ax in axs[:,col]:
                ax.axvline(r["run"]["activation_attempt"],c="#888888",ls=":",lw=.8);ax.grid(alpha=.2)
        axs[0,col].set_title(f"{dim}D | map 00, seed index 00")
        axs[-1,col].set_xlabel("Atomic extension attempts")
    for ax,label in zip(axs[:,0],("Inflation factor γ","Informed-sampling probability p","Best path length (mm)")):ax.set_ylabel(label)
    axs[0,0].legend(frameon=False,fontsize=9)
    fig.tight_layout()
    save(fig,"Fig_matched_SSFOR_controller")


def main():
    parser=argparse.ArgumentParser();parser.add_argument("--replay",action="store_true")
    args=parser.parse_args()
    (OUT/"figures").mkdir(parents=True,exist_ok=True)
    if args.replay or not (OUT/"sources/representative_paths.json").exists():replay()
    path_figures();convergence();ablation();controller()
    print("Figures complete",OUT/"figures")

if __name__=="__main__":main()
