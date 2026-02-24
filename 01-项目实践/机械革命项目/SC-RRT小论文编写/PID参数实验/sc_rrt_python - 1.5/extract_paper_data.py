# -*- coding: utf-8 -*-
"""
Extract and validate experimental data from all stages
Generate clean datasets for paper figures
"""

import pandas as pd
import numpy as np
from pathlib import Path
from datetime import datetime


def load_stage1_data():
    """Load Stage 1 coarse search results"""
    
    stage1_file = Path("results/stage1_coarse_20260209_230704.csv")
    
    if not stage1_file.exists():
        print(f"⚠️  Stage 1 file not found: {stage1_file}")
        return None
    
    df = pd.read_csv(stage1_file)
    print(f"✓ Loaded Stage 1 data: {len(df)} configurations")
    print(f"  Best: Kp={df.iloc[0]['Kp']:.2f}, Ki={df.iloc[0]['Ki']:.3f}, Kd={df.iloc[0]['Kd']:.2f}")
    print(f"  Path: {df.iloc[0]['avg_length']:.2f} ± {df.iloc[0]['std_length']:.2f}")
    
    return df


def load_stage2_data():
    """Load Stage 2 fine-tuning results"""
    
    stage2_file = Path("results/stage2_fine_20260209_230704.csv")
    
    if not stage2_file.exists():
        print(f"⚠️  Stage 2 file not found: {stage2_file}")
        return None
    
    df = pd.read_csv(stage2_file)
    print(f"✓ Loaded Stage 2 data: {len(df)} configurations")
    print(f"  Best: Kp={df.iloc[0]['Kp']:.2f}, Ki={df.iloc[0]['Ki']:.3f}, Kd={df.iloc[0]['Kd']:.2f}")
    print(f"  Path: {df.iloc[0]['avg_length']:.2f} ± {df.iloc[0]['std_length']:.2f}")
    
    return df


def load_stage3_data():
    """Load Stage 3 validation results"""
    
    results_dir = Path("results")
    stage3_files = sorted(results_dir.glob("stage3_validation_*.csv"))
    
    if not stage3_files:
        print(f"⚠️  No Stage 3 validation files found")
        return None
    
    # Use the most recent file
    stage3_file = stage3_files[-1]
    df = pd.read_csv(stage3_file)
    
    print(f"✓ Loaded Stage 3 data: {stage3_file.name}")
    print(f"  Runs: {len(df)}")
    print(f"  Success rate: {df['success'].sum()}/{len(df)} ({df['success'].mean()*100:.1f}%)")
    
    if df['success'].sum() > 0:
        successful = df[df['success'] == True]
        print(f"  Path: {successful['path_length'].mean():.2f} ± {successful['path_length'].std():.2f}")
    
    return df


def extract_top10_for_paper(stage1_df, stage2_df):
    """Extract Top 10 results for paper tables"""
    
    print("\n" + "="*80)
    print("EXTRACTING TOP 10 RESULTS FOR PAPER")
    print("="*80)
    
    # Stage 1 Top 10
    stage1_top10 = stage1_df.head(10).copy()
    stage1_top10['rank'] = range(1, 11)
    stage1_top10['stage'] = 'Stage1'
    
    print("\nStage 1 Top 10:")
    print(stage1_top10[['rank', 'Kp', 'Ki', 'Kd', 'success_rate', 'avg_length', 'std_length', 'avg_time']].to_string(index=False))
    
    # Stage 2 Top 10
    stage2_top10 = stage2_df.head(10).copy()
    stage2_top10['rank'] = range(1, 11)
    stage2_top10['stage'] = 'Stage2'
    
    print("\nStage 2 Top 10:")
    print(stage2_top10[['rank', 'Kp', 'Ki', 'Kd', 'success_rate', 'avg_length', 'std_length', 'avg_time']].to_string(index=False))
    
    # Save combined Top 10
    combined = pd.concat([stage1_top10, stage2_top10], ignore_index=True)
    
    output_file = Path("results/top10_combined_for_paper.csv")
    combined.to_csv(output_file, index=False)
    print(f"\n✓ Combined Top 10 saved: {output_file}")
    
    return stage1_top10, stage2_top10


def generate_paper_data_summary(stage1_df, stage2_df, stage3_df):
    """Generate complete data summary for paper"""
    
    print("\n" + "="*80)
    print("PAPER DATA SUMMARY")
    print("="*80)
    
    summary = {
        'stage': ['Stage1', 'Stage2', 'Stage3'],
        'description': [
            'Coarse Grid Search (6×6×5)',
            'Fine-tuning',
            'Final Validation'
        ],
        'num_configs': [
            len(stage1_df),
            len(stage2_df),
            1
        ],
        'runs_per_config': [3, 5, 20],
        'total_runs': [
            len(stage1_df) * 3,
            len(stage2_df) * 5,
            20
        ]
    }
    
    # Add best results
    best_kp = [stage1_df.iloc[0]['Kp'], stage2_df.iloc[0]['Kp'], stage2_df.iloc[0]['Kp']]
    best_ki = [stage1_df.iloc[0]['Ki'], stage2_df.iloc[0]['Ki'], stage2_df.iloc[0]['Ki']]
    best_kd = [stage1_df.iloc[0]['Kd'], stage2_df.iloc[0]['Kd'], stage2_df.iloc[0]['Kd']]
    
    summary['best_Kp'] = best_kp
    summary['best_Ki'] = best_ki
    summary['best_Kd'] = best_kd
    
    # Add performance metrics
    if stage3_df is not None and len(stage3_df[stage3_df['success']]) > 0:
        stage3_successful = stage3_df[stage3_df['success'] == True]
        
        summary['best_path_mean'] = [
            stage1_df.iloc[0]['avg_length'],
            stage2_df.iloc[0]['avg_length'],
            stage3_successful['path_length'].mean()
        ]
        summary['best_path_std'] = [
            stage1_df.iloc[0]['std_length'],
            stage2_df.iloc[0]['std_length'],
            stage3_successful['path_length'].std()
        ]
        summary['success_rate'] = [
            stage1_df.iloc[0]['success_rate'],
            stage2_df.iloc[0]['success_rate'],
            stage3_df['success'].mean()
        ]
    else:
        summary['best_path_mean'] = [
            stage1_df.iloc[0]['avg_length'],
            stage2_df.iloc[0]['avg_length'],
            np.nan
        ]
        summary['best_path_std'] = [
            stage1_df.iloc[0]['std_length'],
            stage2_df.iloc[0]['std_length'],
            np.nan
        ]
        summary['success_rate'] = [
            stage1_df.iloc[0]['success_rate'],
            stage2_df.iloc[0]['success_rate'],
            0.0
        ]
    
    summary_df = pd.DataFrame(summary)
    
    print("\n" + summary_df.to_string(index=False))
    
    # Save summary
    output_file = Path("results/experiment_summary_for_paper.csv")
    summary_df.to_csv(output_file, index=False)
    print(f"\n✓ Experiment summary saved: {output_file}")
    
    return summary_df


def validate_data_consistency():
    """Validate data consistency across files"""
    
    print("\n" + "="*80)
    print("DATA CONSISTENCY VALIDATION")
    print("="*80)
    
    issues = []
    
    # Load data
    stage1_df = load_stage1_data()
    stage2_df = load_stage2_data()
    stage3_df = load_stage3_data()
    
    if stage1_df is None or stage2_df is None:
        print("\n❌ Missing Stage 1 or Stage 2 data files")
        return False
    
    # Check 1: All success rates should be <= 1.0
    if (stage1_df['success_rate'] > 1.0).any():
        issues.append("Stage 1 has success rates > 100%")
    if (stage2_df['success_rate'] > 1.0).any():
        issues.append("Stage 2 has success rates > 100%")
    
    # Check 2: Path lengths should be reasonable (not negative, not too large)
    if (stage1_df['avg_length'] < 0).any() or (stage1_df['avg_length'] > 10000).any():
        issues.append("Stage 1 has unreasonable path lengths")
    if (stage2_df['avg_length'] < 0).any() or (stage2_df['avg_length'] > 10000).any():
        issues.append("Stage 2 has unreasonable path lengths")
    
    # Check 3: Standard deviation should be non-negative
    if (stage1_df['std_length'] < 0).any():
        issues.append("Stage 1 has negative standard deviations")
    if (stage2_df['std_length'] < 0).any():
        issues.append("Stage 2 has negative standard deviations")
    
    # Check 4: Stage 2 best should be from refinement around Stage 1 region
    stage1_best_kp = stage1_df.iloc[0]['Kp']
    stage2_best_kp = stage2_df.iloc[0]['Kp']
    
    if abs(stage2_best_kp - stage1_best_kp) > 0.2:
        issues.append(f"Stage 2 best Kp ({stage2_best_kp:.2f}) is far from Stage 1 best ({stage1_best_kp:.2f})")
    
    # Check 5: Stage 3 should use Stage 2 best configuration
    if stage3_df is not None and len(stage3_df) > 0:
        stage3_kp = stage3_df.iloc[0]['kp']
        stage3_ki = stage3_df.iloc[0]['ki']
        stage3_kd = stage3_df.iloc[0]['kd']
        
        if not (np.isclose(stage3_kp, stage2_best_kp, atol=0.001) and
                np.isclose(stage3_ki, stage2_df.iloc[0]['Ki'], atol=0.001) and
                np.isclose(stage3_kd, stage2_df.iloc[0]['Kd'], atol=0.001)):
            issues.append(f"Stage 3 parameters don't match Stage 2 best")
    
    # Report results
    if issues:
        print("\n❌ Data consistency issues found:")
        for issue in issues:
            print(f"  - {issue}")
        return False
    else:
        print("\n✓ All data consistency checks passed!")
        return True


def main():
    """Main execution"""
    
    print("\n" + "="*80)
    print("EXPERIMENTAL DATA EXTRACTION AND VALIDATION")
    print("="*80)
    print(f"\nTimestamp: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    
    # Load all data
    print("\n" + "-"*80)
    print("Loading experimental data...")
    print("-"*80)
    
    stage1_df = load_stage1_data()
    stage2_df = load_stage2_data()
    stage3_df = load_stage3_data()
    
    if stage1_df is None or stage2_df is None:
        print("\n❌ Critical data files missing. Cannot proceed.")
        return
    
    # Extract Top 10 for paper
    stage1_top10, stage2_top10 = extract_top10_for_paper(stage1_df, stage2_df)
    
    # Generate summary
    summary_df = generate_paper_data_summary(stage1_df, stage2_df, stage3_df)
    
    # Validate consistency
    is_valid = validate_data_consistency()
    
    # Final report
    print("\n" + "="*80)
    print("DATA EXTRACTION COMPLETE")
    print("="*80)
    
    if stage3_df is None or len(stage3_df) == 0:
        print("\n⚠️  WARNING: Stage 3 validation data not found!")
        print("   Run: python run_stage3_validation.py")
    else:
        print("\n✓ All stages complete and validated")
    
    print("\nFiles generated:")
    print("  - results/top10_combined_for_paper.csv")
    print("  - results/experiment_summary_for_paper.csv")
    
    if is_valid:
        print("\n✓ Data validation: PASSED")
    else:
        print("\n⚠️  Data validation: ISSUES FOUND (see above)")
    
    print("="*80 + "\n")


if __name__ == "__main__":
    main()
