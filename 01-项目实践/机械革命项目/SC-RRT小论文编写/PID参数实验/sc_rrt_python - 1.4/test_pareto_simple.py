# -*- coding: utf-8 -*-
"""
Test Pareto Node Selection Strategy

Compare configurations:
1. No Pareto selection (use_pareto=False)
2. Pareto selection (default interval=100)
3. Pareto selection (frequent interval=50)
"""

import numpy as np
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

from src.environment import EnvironmentConfig
from src.sc_rrt_basic_pid import SCRRTBasicPID


def test_2d_simple():
    """Simple 2D test"""
    print("="*60)
    print("Testing 2D - Pareto Selection Strategy")
    print("="*60)
    
    env = EnvironmentConfig.generate_2d_environment(
        bounds=[0, 800, 0, 800],
        num_obstacles=15,
        start_point=np.array([50, 50]),
        goal_point=np.array([750, 750]),
        radius_range=(10.0, 25.0),
        seed=42
    )
    
    configs = [
        {'name': 'No Pareto', 'use_pareto': False, 'pareto_interval': 100},
        {'name': 'Pareto (interval=100)', 'use_pareto': True, 'pareto_interval': 100},
        {'name': 'Pareto (interval=50)', 'use_pareto': True, 'pareto_interval': 50},
    ]
    
    Kp, Ki, Kd = 0.25, 0.04, 0.10
    
    for config in configs:
        print(f"\n{'='*60}")
        print(f"Config: {config['name']}")
        print(f"{'='*60}")
        
        planner = SCRRTBasicPID(
            env=env,
            max_iterations=5000,
            mode='custom_pid',
            Kp=Kp, Ki=Ki, Kd=Kd,
            use_pareto=config['use_pareto'],
            pareto_interval=config['pareto_interval'],
            pareto_prob=0.8,
            verbose=True
        )
        
        path, tree, success, metrics = planner.plan()
        
        print(f"\n[Result]")
        print(f"  Success: {success}")
        if success:
            print(f"  Path Length: {metrics['path_length']:.2f}")
            print(f"  Pareto Selections: {len(planner.pareto_selections)}")
        print(f"  Time: {metrics['planning_time']:.3f}s")
        print(f"  Iterations: {metrics['iterations']}")
        print("="*60)


if __name__ == "__main__":
    test_2d_simple()
    print("\nTest completed!")
