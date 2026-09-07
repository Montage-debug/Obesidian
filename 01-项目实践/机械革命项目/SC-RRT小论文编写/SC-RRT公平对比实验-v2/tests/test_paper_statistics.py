import importlib.util
from pathlib import Path
import numpy as np

spec=importlib.util.spec_from_file_location("stats",Path(__file__).resolve().parents[1]/"prepare_paper_package.py")
stats=importlib.util.module_from_spec(spec)
spec.loader.exec_module(stats)


def test_map_exact_wilcoxon_and_holm():
    assert stats.map_wilcoxon(np.zeros(10))==1
    assert stats.map_wilcoxon(np.arange(1,11))==2/1024
    assert stats.map_wilcoxon(-np.arange(1,11))==2/1024
    assert np.allclose(stats.holm([.01,.04,.03]),[.03,.06,.06])


def test_hierarchy_preserves_pairs():
    groups=[np.column_stack([np.arange(50)+100,np.arange(50)+98]) for _ in range(10)]
    ci,_=stats.hierarchical_ci(groups,55)
    assert np.allclose(ci,[2,2])


def test_map_variation_not_treated_as_500_independent_maps():
    groups=[np.tile([100.0,100.0-x],(50,1)) for x in range(10)]
    ci,_=stats.hierarchical_ci(groups,123)
    assert ci[1]-ci[0]>2.5
