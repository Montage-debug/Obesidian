import json
import sys
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/"src"))
from core import Environment
from matched_ablation import build_matched


def pair():
    config=json.loads((ROOT/"config/protocol.json").read_text())
    env=Environment("test",2,np.array([[0,1500],[0,1500]]),np.array([75,75]),
                    np.array([1425,1425]),np.empty((0,3)),"test",1)
    return [build_matched(env,7,config,fixed=x) for x in [False,True]]


def test_no_activation_at_first_solution():
    for p in pair():
        p.best_cost=2000
        p.on_solution_improved({"side_costs":[1000,1000]})
        assert p.p_informed==[0,0]
        assert p.activation_attempt is None


def test_same_activation_after_51_observations():
    full,fixed=pair()
    for i in range(1,52):
        for p in [full,fixed]:
            p.budget.attempts=i*50
            p.best_cost=2000
            p._feedback()
            assert np.isfinite(p.gamma+p.p_informed).all()
            if i<=50:assert p.activation_attempt is None and p.p_informed==[0,0]
    assert full.activation_attempt==fixed.activation_attempt==2550
    assert full.activation_snapshot==fixed.activation_snapshot
    assert fixed.gamma==[1.5,1.5] and fixed.p_informed==[.8,.8]
    assert full.gamma!=fixed.gamma


def test_no_activation_without_solution_and_finite_transition():
    full,fixed=pair()
    for i in range(1,61):
        for p in [full,fixed]:
            p.budget.attempts=i*50
            p._feedback()
            assert p.activation_attempt is None
    for p in [full,fixed]:
        p.budget.attempts=3050
        p.best_cost=2000
        p._feedback()
        assert p.activation_attempt==3050
        assert np.isfinite(p.gamma+p.p_informed).all()
    assert full.activation_snapshot==fixed.activation_snapshot
