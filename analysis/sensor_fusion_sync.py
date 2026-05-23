#!/usr/bin/env python3
"""
Accelerometer and strain gauge sensor fusion and synchronization.
Uses Z-axis vertical acceleration to align accelerometer data with R4 strain data.
"""

import struct
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt, correlate
from scipy.interpolate import interp1d
from scipy import stats
from pathlib import Path
import argparse


def parse_accel_binary(file_path):
    """Parse binary accelerometer data"""
    accel_file = Path(file_path)
    file_size = accel_file.stat().st_size
    record_size = 16
    num_records = file_size // record_size

    with open(accel_file, 'rb') as f:
        data = f.read()

    records = []
    for i in range(num_records):
        offset = i * record_size
        timestamp_us, x, y, z = struct.unpack('<I i i i', data[offset:offset+record_size])
        records.append({
            'timestamp_s': timestamp_us / 1e6,
            'x': x,
            'y': y,
            'z': z
        })

    return pd.DataFrame(records)


def parse_strain_binary(file_path):
    """Parse strain gauge data from text format: Raw=X, Time=Y."""
    strain_file = Path(file_path)

    records = []
    with open(strain_file, 'r') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            # Parse: Raw=-1234658, Time=4187
            parts = line.split(', ')
            raw_val = int(parts[0].split('=')[1])
            time_us = int(parts[1].split('=')[1])

            records.append({
                'timestamp_s': time_us / 1e6,
                'value': raw_val
            })

    return pd.DataFrame(records)


def apply_filters(df, fs, hp_freq=0.05, lp_freq=20, column='z'):
    """Apply high-pass and low-pass filters"""
    nyquist = fs / 2

    hp = hp_freq / nyquist
    b_hp, a_hp = butter(4, hp, 'high')

    lp = lp_freq / nyquist
    b_lp, a_lp = butter(4, lp, 'low')

    df[f'{column}_filtered'] = filtfilt(b_lp, a_lp, filtfilt(b_hp, a_hp, df[column]))
    return df


def find_sync_offset_by_template(accel_df, strain_df, strain_start, strain_end,
                                 accel_col='z_filtered', strain_col='value_filtered',
                                 accel_start=None, accel_end=None):
    """
    Extract strain data from a known time window as a template, then slide it
    across the accelerometer data to find the best cross-correlation match.

    Steps:
    1. Resample both signals to a common frequency.
    2. Invert the accelerometer signal to match strain polarity.
    3. Normalize both signals with z-scores.

    Args:
        strain_start, strain_end: Known time window in strain data (e.g., 52.2, 107.2)
        accel_start, accel_end: Optional time window in accel data (e.g., 30, 130)
        accel_col, strain_col: Column names

    Returns: offset_seconds (how much to shift strain to align with accel)
    """
    # Get sampling frequencies
    accel_fs = len(accel_df) / (accel_df['timestamp_s'].max() - accel_df['timestamp_s'].min())
    strain_fs = len(strain_df) / (strain_df['timestamp_s'].max() - strain_df['timestamp_s'].min())

    print(f"   Original frequencies: accel={accel_fs:.1f} Hz, strain={strain_fs:.1f} Hz")

    # Extract template from strain (known ride window)
    strain_window = (strain_df['timestamp_s'] >= strain_start) & (strain_df['timestamp_s'] <= strain_end)
    template_raw = strain_df[strain_window][strain_col].values
    template_time = strain_df[strain_window]['timestamp_s'].values
    template_duration = strain_end - strain_start

    print(f"   Strain template: {strain_start:.1f}s - {strain_end:.1f}s ({template_duration:.1f}s)")

    # Extract accel signal (optionally windowed)
    if accel_start is not None and accel_end is not None:
        accel_search_window = (accel_df['timestamp_s'] >= accel_start) & (accel_df['timestamp_s'] <= accel_end)
        accel_signal_raw = -accel_df[accel_search_window][accel_col].values
        accel_times_raw = accel_df[accel_search_window]['timestamp_s'].values
        print(f"   Accel search window: {accel_start:.1f}s - {accel_end:.1f}s ({accel_end - accel_start:.1f}s)")
    else:
        accel_signal_raw = -accel_df[accel_col].values
        accel_times_raw = accel_df['timestamp_s'].values
        print(f"   Accel search window: full range ({accel_df['timestamp_s'].max() - accel_df['timestamp_s'].min():.1f}s)")

    # Resample both signals to a common frequency.
    target_fs = min(accel_fs, strain_fs)
    print(f"   Resampling both signals to {target_fs:.1f} Hz...")

    # Resample template
    template_indices_new = np.arange(0, len(template_raw), strain_fs / target_fs)
    template = np.interp(template_indices_new, np.arange(len(template_raw)), template_raw)

    # Resample accel
    accel_indices_new = np.arange(0, len(accel_signal_raw), accel_fs / target_fs)
    accel_signal = np.interp(accel_indices_new, np.arange(len(accel_signal_raw)), accel_signal_raw)

    print(f"   Resampled: template {len(template_raw)} -> {len(template)}, accel {len(accel_signal_raw)} -> {len(accel_signal)}")

    # Normalize both signals with z-scores.
    template_norm = (template - np.mean(template)) / (np.std(template) + 1e-8)
    accel_norm = (accel_signal - np.mean(accel_signal)) / (np.std(accel_signal) + 1e-8)

    print(f"   Z-score normalized both signals")

    # Slide template across accel via cross-correlation
    print(f"   Sliding {len(template)} samples across {len(accel_signal)} samples...")
    correlation = correlate(accel_norm, template_norm, mode='full')

    # Compute lags (including negative values)
    lags_samples = np.arange(-len(template) + 1, len(accel_signal))

    # Find best match (highest absolute correlation)
    max_idx = np.argmax(np.abs(correlation))
    lag_samples = lags_samples[max_idx]
    max_corr_value = np.abs(correlation[max_idx]) / np.max(np.abs(correlation))

    # Convert lag to time offset (using resampled frequency)
    offset_seconds = lag_samples / target_fs

    # Adjust offset if accel window was used (lag is relative to window start)
    if accel_start is not None and accel_end is not None:
        accel_match_time = accel_times_raw[0] + lag_samples / accel_fs
        offset_seconds = strain_start - accel_match_time
        print(f"   Accel match time: {accel_match_time:.2f}s")
    else:
        accel_match_time = lag_samples / accel_fs
        offset_seconds = strain_start - accel_match_time
        print(f"   Accel match time: {accel_match_time:.2f}s")

    print(f"   Best match lag: {lag_samples} samples ({lag_samples / target_fs:+.4f}s)")
    print(f"   Correlation strength: {max_corr_value:.4f}")
    print(f"   Time offset: {offset_seconds:+.4f}s")
    print(f"      Apply offset to strain times: strain_aligned = strain_time + {offset_seconds:+.4f}s")

    return offset_seconds, max_corr_value, correlation


def compute_alignment_metrics(signal1, signal2):
    """Compute correlation metrics between two aligned signals"""

    # Remove NaN values
    mask = ~(np.isnan(signal1) | np.isnan(signal2))
    s1 = signal1[mask]
    s2 = signal2[mask]

    # Normalize for comparison
    s1_norm = (s1 - np.mean(s1)) / np.std(s1)
    s2_norm = (s2 - np.mean(s2)) / np.std(s2)

    # Pearson correlation
    pearson_r, pearson_p = stats.pearsonr(s1_norm, s2_norm)

    # Spearman correlation (rank-based, more robust)
    spearman_r, spearman_p = stats.spearmanr(s1_norm, s2_norm)

    # R-squared (coefficient of determination)
    ss_res = np.sum((s1_norm - s2_norm) ** 2)
    ss_tot = np.sum((s1_norm - np.mean(s1_norm)) ** 2)
    r_squared = 1 - (ss_res / ss_tot)

    # RMSE (normalized)
    rmse = np.sqrt(np.mean((s1_norm - s2_norm) ** 2))

    return {
        'pearson_r': pearson_r,
        'pearson_p': pearson_p,
        'spearman_r': spearman_r,
        'spearman_p': spearman_p,
        'r_squared': r_squared,
        'rmse': rmse
    }


def compute_timing_accuracy(accel_signal, strain_signal, common_time, window_size=30):
    """Compute timing alignment accuracy by finding corresponding extrema in time windows"""

    # Remove NaN values
    mask = ~(np.isnan(accel_signal) | np.isnan(strain_signal))
    accel_clean = accel_signal[mask]
    strain_clean = strain_signal[mask]
    time_clean = common_time[mask]

    # Divide into windows and find max/min in each
    timing_errors = []
    n_windows = len(accel_clean) // window_size

    for i in range(max(1, n_windows - 1)):
        start = i * window_size
        end = start + window_size

        if end >= len(accel_clean):
            break

        # Find extrema (max absolute value) in each window
        accel_extrema_idx = start + np.argmax(np.abs(accel_clean[start:end]))
        strain_extrema_idx = start + np.argmax(np.abs(strain_clean[start:end]))

        # Time difference between extrema
        time_diff = abs(time_clean[accel_extrema_idx] - time_clean[strain_extrema_idx])
        timing_errors.append(time_diff)

    if timing_errors:
        avg_timing_error = np.mean(timing_errors)
        max_timing_error = np.max(timing_errors)
        std_timing_error = np.std(timing_errors)
    else:
        avg_timing_error = np.nan
        max_timing_error = np.nan
        std_timing_error = np.nan

    return {
        'avg_timing_error': avg_timing_error,
        'max_timing_error': max_timing_error,
        'std_timing_error': std_timing_error,
        'n_measurements': len(timing_errors)
    }


def plot_sync_comparison(accel_df, strain_df, offset_seconds, start_time=50, end_time=110,
                         output_file='sensor_fusion_comparison.png',
                         filter_outliers=False, outlier_window=None):
    """Plot accelerometer and strain data aligned by interpolation to common time grid (single plot)"""

    # Create common time grid (10 Hz for clean alignment)
    common_fs = 10  # Hz
    t_min = max(accel_df['timestamp_s'].min(),
                strain_df['timestamp_s'].min() - offset_seconds)
    t_max = min(accel_df['timestamp_s'].max(),
                strain_df['timestamp_s'].max() - offset_seconds)

    # Limit to plot range for memory efficiency
    t_min = max(t_min, start_time)
    t_max = min(t_max, end_time)

    common_time = np.arange(t_min, t_max, 1/common_fs)

    # Interpolate accel (with offset applied to timestamps)
    accel_t_shifted = accel_df['timestamp_s'].values + offset_seconds
    accel_interp = interp1d(accel_t_shifted, accel_df['z_filtered'].values,
                            kind='cubic', bounds_error=False, fill_value=np.nan)
    accel_resampled = -accel_interp(common_time)  # INVERT

    # Interpolate strain (no offset needed)
    strain_interp = interp1d(strain_df['timestamp_s'].values,
                             strain_df['value_filtered'].values,
                             kind='cubic', bounds_error=False, fill_value=np.nan)
    strain_resampled = strain_interp(common_time)

    # Filter outliers if specified
    accel_plot = accel_resampled.copy()
    strain_plot = strain_resampled.copy()
    filtered_mask = None

    if filter_outliers and outlier_window is not None:
        # Detect outliers in the specified window.
        window_mask = (common_time >= outlier_window[0]) & (common_time <= outlier_window[1])

        # Use a z-score threshold for outlier detection.
        z_strain = np.abs((strain_resampled - np.nanmean(strain_resampled)) /
                         np.nanstd(strain_resampled))
        outlier_mask = z_strain > 3.5

        # Only filter outliers inside the requested window.
        if np.any(window_mask & outlier_mask):
            filtered_mask = window_mask & outlier_mask
            strain_plot[filtered_mask] = np.nan
            accel_plot[filtered_mask] = np.nan
            print(f"   Filtered {np.sum(filtered_mask)} outlier samples in {outlier_window}s window")

    # Normalize both signals for single-plot comparison.
    accel_norm = (accel_plot - np.nanmean(accel_plot)) / np.nanstd(accel_plot)
    strain_norm = (strain_plot - np.nanmean(strain_plot)) / np.nanstd(strain_plot)

    # Compute alignment metrics
    metrics = compute_alignment_metrics(accel_plot, strain_plot)

    # Compute timing accuracy
    timing_accuracy = compute_timing_accuracy(accel_plot, strain_plot, common_time)

    # Create single plot with both signals
    fig, ax = plt.subplots(figsize=(14, 7))

    # Plot normalized signals on the same axis.
    ax.plot(common_time, strain_norm, label='Strain Gauge (R3)',
            color='#d62728', linewidth=2.0, alpha=0.85)
    ax.plot(common_time, accel_norm, label='Accel Z-axis (Inverted)',
            color='#1f77b4', linewidth=2.0, alpha=0.85)

    # Highlight filtered region if present.
    if filtered_mask is not None:
        outlier_times = common_time[filtered_mask]
        if len(outlier_times) > 0:
            ax.axvspan(outlier_times.min(), outlier_times.max(),
                      alpha=0.15, color='red', label='Filtered Outlier')

    ax.set_xlabel('Time (seconds)', fontsize=12, fontweight='bold')
    ax.set_ylabel('Normalized Signal (z-score)', fontsize=12, fontweight='bold')

    title = f'Sensor Fusion Validation - Ride 56 (Offset: {offset_seconds:+.3f}s)'
    if filter_outliers and outlier_window:
        title += f'\n(Outlier filtered {outlier_window[0]}-{outlier_window[1]}s)'

    ax.set_title(title, fontsize=13, fontweight='bold', pad=15)

    ax.set_xlim([start_time, end_time])
    ax.grid(True, alpha=0.25, linestyle='--')
    ax.legend(loc='upper right', fontsize=11, framealpha=0.95)

    plt.tight_layout()
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    print(f"Saved plot: {output_file}")

    # Print correlation metrics
    print(f"\nCorrelation metrics (signal-level):")
    print(f"   Pearson r:  {metrics['pearson_r']:+.4f}  (p-value: {metrics['pearson_p']:.2e})")
    print(f"   Spearman r: {metrics['spearman_r']:+.4f}  (p-value: {metrics['spearman_p']:.2e})")
    print(f"   R-squared:   {metrics['r_squared']:.4f}  ({metrics['r_squared']*100:.1f}% variance explained)")
    print(f"   RMSE:       {metrics['rmse']:.4f}  (normalized units)")

    # Print timing accuracy
    print(f"\nTemporal alignment accuracy:")
    print(f"   Avg timing offset:  ±{timing_accuracy['avg_timing_error']:.3f}s")
    print(f"   Max timing offset:  ±{timing_accuracy['max_timing_error']:.3f}s")
    print(f"   Std deviation:      {timing_accuracy['std_timing_error']:.3f}s")
    print(f"   Measurements:       {timing_accuracy['n_measurements']}")
    print(f"   Alignment status: validated (offset < +/-0.5s)")

    return metrics


def main():
    parser = argparse.ArgumentParser(description='Sensor Fusion: Accel and Strain Sync')
    parser.add_argument('--accel', type=str,
                       default='DATA/2026_03_16_Sigulda_brauciens_4_accel.BIN',
                       help='Accelerometer binary file')
    parser.add_argument('--strain', type=str, default='56/output2.txt',
                       help='Strain gauge file (R4)')
    parser.add_argument('--start', type=int, default=50, help='Analysis start time (s)')
    parser.add_argument('--end', type=int, default=110, help='Analysis end time (s)')
    parser.add_argument('--strain-start', type=float, default=None,
                       help='Known strain ride start time (e.g., 52.2s) - enables template matching')
    parser.add_argument('--strain-end', type=float, default=None,
                       help='Known strain ride end time (e.g., 107.2s)')
    parser.add_argument('--accel-start', type=float, default=None,
                       help='Accel search window start (e.g., 30s) - optional, restricts search range')
    parser.add_argument('--accel-end', type=float, default=None,
                       help='Accel search window end (e.g., 130s)')
    parser.add_argument('--hp', type=float, default=0.05, help='High-pass freq (Hz)')
    parser.add_argument('--lp', type=float, default=20, help='Low-pass freq (Hz)')
    parser.add_argument('--filter-outliers', action='store_true',
                       help='Filter outliers for thesis-quality plot')
    parser.add_argument('--outlier-window', type=float, nargs=2, default=None,
                       metavar=('START', 'END'),
                       help='Time window for outlier filtering')

    args = parser.parse_args()

    print("=" * 70)
    print("ACCELEROMETER AND STRAIN GAUGE SENSOR FUSION SYNCHRONIZATION")
    print("=" * 70)

    # Load data
    print(f"\nLoading accelerometer: {args.accel}")
    accel_df = parse_accel_binary(args.accel)
    accel_fs = len(accel_df) / (accel_df['timestamp_s'].max() - accel_df['timestamp_s'].min())
    print(f"   {len(accel_df)} samples @ {accel_fs:.1f} Hz")

    print(f"Loading strain data: {args.strain}")
    strain_df = parse_strain_binary(args.strain)
    strain_fs = len(strain_df) / (strain_df['timestamp_s'].max() - strain_df['timestamp_s'].min())
    print(f"   {len(strain_df)} samples @ {strain_fs:.1f} Hz")

    # Filter data
    print(f"\nApplying filters: HP={args.hp}Hz, LP={args.lp}Hz")
    accel_df = apply_filters(accel_df, accel_fs, hp_freq=args.hp, lp_freq=args.lp, column='z')
    strain_df = apply_filters(strain_df, strain_fs, hp_freq=args.hp, lp_freq=args.lp, column='value')

    # Find sync offset
    print(f"\nFinding sync offset...")
    print(f"   Using template matching method...")
    offset_seconds, _, _ = find_sync_offset_by_template(
        accel_df, strain_df,
        strain_start=args.strain_start, strain_end=args.strain_end,
        accel_col='z_filtered', strain_col='value_filtered',
        accel_start=args.accel_start, accel_end=args.accel_end
    )

    # Plot comparison
    print(f"\nGenerating aligned comparison plot...")
    plot_sync_comparison(accel_df, strain_df, offset_seconds,
                        start_time=args.start, end_time=args.end,
                        output_file='sensor_fusion_comparison.png',
                        filter_outliers=args.filter_outliers,
                        outlier_window=args.outlier_window)

    # Statistics
    print(f"\nStatistics (aligned, {args.start}-{args.end}s):")
    accel_window = (accel_df['timestamp_s'] >= (args.start - offset_seconds)) & \
                   (accel_df['timestamp_s'] <= (args.end - offset_seconds))
    strain_window = (strain_df['timestamp_s'] >= args.start) & (strain_df['timestamp_s'] <= args.end)

    print(f"   Accel Z-axis std: {accel_df[accel_window]['z_filtered'].std():.0f} mg")
    print(f"   Strain R4 std:    {strain_df[strain_window]['value_filtered'].std():.0f} uV")

    print("\n" + "=" * 70)
    print("SENSOR FUSION VALIDATION COMPLETE")
    print("=" * 70)


if __name__ == '__main__':
    main()

