import re
import argparse
import pandas as pd
import numpy as np
import os
from scipy.signal import butter, filtfilt, find_peaks

# Handle scipy version differences for trapz
try:
    from scipy.integrate import trapz
except ImportError:
    from scipy.integrate import trapezoid as trapz

# Optional imports for plotting
try:
    import matplotlib.pyplot as plt
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False

# Pattern for parsing data: Raw=N, Time=T, time in us
pattern = re.compile(r"Raw\s*=\s*(-?\d+)\s*,\s*Time\s*=\s*(\d+)")

def load_data(path):
    """Load data from ADS1220 log file."""
    raw_vals = []
    time_us = []
    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            m = pattern.search(line)
            if m:
                raw_vals.append(int(m.group(1)))
                time_us.append(int(m.group(2)))
    if not raw_vals:
        return pd.DataFrame()

    df = pd.DataFrame({"time_us": time_us, "raw": raw_vals})
    df["time_s"] = df["time_us"] / 1_000_000.0
    return df

def apply_bandpass_filter(df, highpass_cutoff, lowpass_cutoff, order=4):
    """Apply band-pass filter (high-pass then low-pass). Returns filtered series or None if failed."""
    if len(df) <= 1:
        return None

    sampling_rate = 1 / df["time_s"].diff().mean()
    nyquist = 0.5 * sampling_rate

    if highpass_cutoff >= nyquist or lowpass_cutoff >= nyquist:
        return None
    if highpass_cutoff >= lowpass_cutoff:
        return None
    if len(df) <= 3 * (order + 1):
        return None

    try:
        # Apply high-pass first (remove drift)
        normal_cutoff_hp = highpass_cutoff / nyquist
        b_hp, a_hp = butter(order, normal_cutoff_hp, btype='high', analog=False)
        result = filtfilt(b_hp, a_hp, df['raw'].values)

        # Then apply low-pass (remove high-freq noise)
        normal_cutoff_lp = lowpass_cutoff / nyquist
        b_lp, a_lp = butter(order, normal_cutoff_lp, btype='low', analog=False)
        result = filtfilt(b_lp, a_lp, result)

        # Return as pandas Series for consistency with plotting
        return pd.Series(result, index=df.index)
    except Exception:
        return None

def find_max_peak_index(df, column='raw'):
    """Find index of maximum peak (most negative for steering)."""
    return df[column].idxmin()

def find_first_positive_peak(df, threshold=50000, column='filtered'):
    """
    Find first data point ABOVE threshold (positive peak).
    Matches compare_rides.py logic for proper alignment.
    """
    indices = np.where(df[column].values > threshold)[0]

    if len(indices) == 0:
        return None

    # Find continuous segments above threshold (gap > 100 samples)
    gaps = np.where(np.diff(indices) > 100)[0]

    if len(gaps) == 0:
        return indices[0]
    else:
        return indices[0]

def create_window_around_peak(df, peak_index, before_s, after_s):
    """Extract time window around a peak."""
    peak_time = df.loc[peak_index, 'time_s']
    start_time = peak_time + before_s
    end_time = peak_time + after_s

    windowed = df[(df['time_s'] >= start_time) & (df['time_s'] <= end_time)].copy()
    return windowed

def detect_curves(df, filtered_col='filtered', min_distance_s=2.0, height_threshold=None, prominence_threshold=None):
    """
    Detect curve peaks (steering events) in filtered data.

    Args:
        df: DataFrame with filtered data
        filtered_col: Column name containing filtered signal
        min_distance_s: Minimum distance between peaks in seconds
        height_threshold: Minimum peak height (auto-calculated if None)
        prominence_threshold: Minimum prominence (auto-calculated if None)

    Returns:
        peaks_idx: Array of peak indices
        peaks_props: Properties dict from find_peaks
    """
    signal = df[filtered_col].values
    time_s = df['time_s'].values

    # Calculate sampling rate for distance conversion
    dt = (time_s[-1] - time_s[0]) / len(time_s)
    min_distance_samples = int(min_distance_s / dt)

    # Auto-calculate height threshold if not provided (20% of signal range)
    if height_threshold is None:
        signal_range = np.max(signal) - np.min(signal)
        height_threshold = -signal_range * 0.15  # Looking for negative peaks

    # Auto-calculate prominence if not provided
    if prominence_threshold is None:
        prominence_threshold = abs(height_threshold * 0.3)

    # Find peaks (looking for downward deflections in steering)
    peaks_idx, peaks_props = find_peaks(
        -signal,  # Negate to find minima as peaks
        distance=min_distance_samples,
        height=-height_threshold,
        prominence=prominence_threshold
    )

    return peaks_idx, peaks_props, time_s[peaks_idx]

def calculate_curve_metrics(df, peak_idx, window_s=2.0, time_col='time_s', signal_col='filtered'):
    """
    Calculate metrics for a single curve/peak.

    Returns dict with:
    - peak_value: Peak amplitude
    - peak_time: Time of peak
    - crest_factor: Peak / RMS in window
    - rise_time: Time from 10% to 90% of peak
    - fall_time: Time from peak to 10% recovery
    - energy: Integrated area under curve
    - duration_50pct: Time above 50% of peak
    """
    peak_time = df.loc[peak_idx, time_col]
    peak_val = df.loc[peak_idx, signal_col]

    # Extract window around peak
    window_mask = (df[time_col] >= peak_time - window_s) & (df[time_col] <= peak_time + window_s)
    window_df = df[window_mask]

    if len(window_df) < 5:
        return None

    signal_window = window_df[signal_col].values
    time_window = window_df[time_col].values

    # Basic metrics
    metrics = {
        'peak_value': float(peak_val),
        'peak_time': float(peak_time),
        'peak_amplitude': float(abs(peak_val)),
        'rms_window': float(np.sqrt(np.mean(signal_window ** 2))),
    }

    # Crest factor (peak / RMS)
    rms = metrics['rms_window']
    if rms > 0:
        metrics['crest_factor'] = metrics['peak_amplitude'] / rms
    else:
        metrics['crest_factor'] = 0.0

    # Energy (area under curve)
    metrics['energy'] = float(abs(trapz(signal_window, time_window)))

    # Rise and fall times
    peak_abs = abs(peak_val)
    threshold_10 = peak_abs * 0.1
    threshold_90 = peak_abs * 0.9
    threshold_50 = peak_abs * 0.5

    # Find rise time (10% to 90%)
    rise_times = []
    for i in range(len(signal_window) - 1):
        if abs(signal_window[i]) >= threshold_10 and abs(signal_window[i + 1]) >= threshold_90:
            rise_times.append(time_window[i + 1] - time_window[i])
        elif abs(signal_window[i]) <= threshold_10 and abs(signal_window[i + 1]) >= threshold_90:
            rise_times.append(time_window[i + 1] - time_window[i])

    metrics['rise_time'] = float(min(rise_times)) if rise_times else 0.0

    # Fall time (peak to 10%)
    peak_idx_in_window = np.argmin(signal_window)  # Most negative
    fall_times = []
    for i in range(peak_idx_in_window, len(signal_window) - 1):
        if abs(signal_window[i]) >= threshold_10 and abs(signal_window[i + 1]) <= threshold_10:
            fall_times.append(time_window[i + 1] - time_window[i])

    metrics['fall_time'] = float(min(fall_times)) if fall_times else 0.0

    # Duration above 50% of peak
    above_50 = np.sum(np.abs(signal_window) >= threshold_50)
    metrics['duration_50pct'] = float(above_50 / len(signal_window) * (time_window[-1] - time_window[0]))

    # Smoothness (variance of second derivative)
    if len(signal_window) > 2:
        second_deriv = np.diff(signal_window, 2)
        metrics['smoothness'] = float(np.std(second_deriv) if len(second_deriv) > 0 else 0.0)
    else:
        metrics['smoothness'] = 0.0

    return metrics

def analyze_single_ride(filepath, hp_cutoff=0.05, lp_cutoff=5.0, order=4, start_s=None, end_s=None,
                       plot=False, save_path=None, peak_height=None, quiet=False):
    """Analyze curves in a single ride."""

    # Load and filter data
    df = load_data(filepath)
    if df.empty:
        print(f"No data loaded from {filepath}")
        return None

    # Apply time window if specified
    if start_s is not None or end_s is not None:
        start_time = start_s if start_s is not None else df['time_s'].min()
        end_time = end_s if end_s is not None else df['time_s'].max()
        df = df[(df['time_s'] >= start_time) & (df['time_s'] <= end_time)].reset_index(drop=True)
        if df.empty:
            print(f"No data in window [{start_time}s, {end_time}s]")
            return None

    # Apply band-pass filter
    filtered = apply_bandpass_filter(df, hp_cutoff, lp_cutoff, order)
    if filtered is None:
        print(f"Could not apply band-pass filter")
        return None

    df['filtered'] = filtered

    # Detect curves
    peaks_idx, peaks_props, peak_times = detect_curves(df, 'filtered', height_threshold=peak_height)

    if len(peaks_idx) == 0:
        if not quiet:
            print("No curves detected")
        return None

    if not quiet:
        print(f"\n{'='*70}")
        print(f"Ride Analysis: {os.path.basename(filepath)}")
        print(f"Time window: [{df['time_s'].min():.1f}s, {df['time_s'].max():.1f}s]")
        print(f"Detected {len(peaks_idx)} curves (expected ~16)")
        print(f"{'='*70}")

    # Extract metrics for each curve
    metrics_list = []
    for curve_num, peak_idx in enumerate(peaks_idx, 1):
        metrics = calculate_curve_metrics(df, peak_idx)
        if metrics:
            metrics['curve_num'] = curve_num
            metrics_list.append(metrics)

    # Calculate statistics (needed for return value, even if quiet)
    amplitudes = [m['peak_amplitude'] for m in metrics_list]
    crest_factors = [m['crest_factor'] for m in metrics_list]

    # Print summary
    if not quiet:
        print(f"\nCurve Metrics Summary:")
        print(f"{'Curve':<8} {'Time':<10} {'Amplitude':<12} {'Crest':<10} {'Energy':<12} {'Smoothness':<12}")
        print("-" * 70)
        for m in metrics_list:
            print(f"{m['curve_num']:<8} {m['peak_time']:<10.2f} {m['peak_amplitude']:<12.0f} "
                  f"{m['crest_factor']:<10.2f} {m['energy']:<12.0f} {m['smoothness']:<12.2f}")

        print(f"\nStatistics:")
        print(f"  Amplitude: mean={np.mean(amplitudes):.0f}, std={np.std(amplitudes):.0f}")
        print(f"  Crest Factor: mean={np.mean(crest_factors):.2f}, std={np.std(crest_factors):.2f}")
        print(f"{'='*70}\n")

    # Plot if requested
    if (plot or save_path) and HAS_MATPLOTLIB:
        fig, ax = plt.subplots(figsize=(14, 5))

        # Plot filtered signal
        ax.plot(df['time_s'], df['filtered'], color='purple', linewidth=1.5, label='Filtered Signal (0.05-5Hz)', alpha=0.8)

        # Mark detected peaks
        ax.plot(df.loc[peaks_idx, 'time_s'], df.loc[peaks_idx, 'filtered'],
               'r*', markersize=15, label=f'Detected Curves ({len(peaks_idx)})', zorder=5)

        # Add curve numbers
        for i, peak_idx in enumerate(peaks_idx, 1):
            peak_time = df.loc[peak_idx, 'time_s']
            peak_val = df.loc[peak_idx, 'filtered']
            ax.annotate(str(i), xy=(peak_time, peak_val), xytext=(0, -20),
                       textcoords='offset points', ha='center', fontsize=8, color='red')

        ax.axhline(y=0, color='black', linestyle='--', linewidth=0.5, alpha=0.5)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Filtered Signal')
        ax.set_title(f'Curve Detection: {os.path.basename(filepath)}\nBand-Pass {hp_cutoff}-{lp_cutoff}Hz')
        ax.grid(True, alpha=0.3)
        ax.legend(loc='best')

        plt.tight_layout()

        if save_path:
            plt.savefig(save_path, dpi=150)
            print(f"Saved plot to {save_path}")
        if plot:
            plt.show()

    return {
        'filepath': filepath,
        'dataframe': df,
        'peaks_idx': peaks_idx,
        'metrics': metrics_list,
        'summary': {
            'num_curves': len(peaks_idx),
            'avg_amplitude': np.mean(amplitudes),
            'avg_crest_factor': np.mean(crest_factors),
        }
    }

def compare_rides(filepaths, labels=None, hp_cutoff=0.05, lp_cutoff=5.0, order=4,
                 peak_mode='max', peak_threshold=None, peak_height=None, before_s=30, after_s=25,
                 plot=False, save_path=None):
    """Compare curves across multiple rides with alignment."""

    # Load all rides (quietly, without printing individual analysis)
    rides_data = []
    for fp in filepaths:
        result = analyze_single_ride(fp, hp_cutoff, lp_cutoff, order, peak_height=peak_height, quiet=True)
        if result:
            rides_data.append(result)

    if not rides_data:
        print("No rides loaded successfully")
        return None

    if not labels:
        labels = [f"Ride {i+1}" for i in range(len(rides_data))]

    # Use first ride as reference for alignment
    ref_ride = rides_data[0]
    ref_df = ref_ride['dataframe']

    # Find alignment reference point
    if peak_mode == 'max':
        ref_peak_idx = ref_df['filtered'].idxmin()
        ref_peak_time = ref_df.loc[ref_peak_idx, 'time_s']
        print(f"\nAlignment: Using MAX NEGATIVE PEAK at {ref_peak_time:.2f}s from {labels[0]}")
    elif peak_mode == 'max_pos':
        ref_peak_idx = ref_df['filtered'].idxmax()
        ref_peak_time = ref_df.loc[ref_peak_idx, 'time_s']
        print(f"\nAlignment: Using MAX POSITIVE PEAK at {ref_peak_time:.2f}s from {labels[0]}")
    elif peak_mode == 'first':
        if peak_threshold is None:
            print("Error: --threshold required for --peak-mode first")
            return None
        indices = ref_df[ref_df['filtered'] < peak_threshold].index
        if len(indices) == 0:
            print(f"Error: No peaks above threshold {peak_threshold}")
            return None
        ref_peak_idx = indices[0]
        ref_peak_time = ref_df.loc[ref_peak_idx, 'time_s']
        print(f"\nAlignment: Using FIRST PEAK above {peak_threshold} at {ref_peak_time:.2f}s from {labels[0]}")
    else:
        print(f"Unknown peak mode: {peak_mode}")
        return None

    # Extract windows
    print(f"Window: {before_s}s before to {after_s}s after peak")

    aligned_rides = []

    for i, ride_data in enumerate(rides_data):
        df = ride_data['dataframe']

        # Find corresponding peak using SAME method for all rides
        if peak_mode == 'max':
            peak_idx = df['filtered'].idxmin()  # Most negative = steering peak
        elif peak_mode == 'max_pos':
            peak_idx = df['filtered'].idxmax()  # Most positive = pressure peak
        elif peak_mode == 'first':
            # Find FIRST POSITIVE peak above threshold (like compare_rides.py)
            peak_idx = find_first_positive_peak(df.reset_index(drop=True), peak_threshold, 'filtered')
            if peak_idx is None:
                print(f"Warning: No positive peaks above threshold in {labels[i]}, using max peak")
                peak_idx = df['filtered'].idxmin()
            # Convert from reset index back to original
            peak_idx = df.index[peak_idx] if isinstance(peak_idx, (int, np.integer)) else peak_idx
        else:
            peak_idx = df['filtered'].idxmin()

        # Get this ride's peak time
        peak_time = df.loc[peak_idx, 'time_s']

        # Create window RELATIVE to this ride's peak
        start_time = peak_time + before_s
        end_time = peak_time + after_s

        # Extract window FIRST
        windowed = df[(df['time_s'] >= start_time) & (df['time_s'] <= end_time)].reset_index(drop=True).copy()

        if len(windowed) == 0:
            print(f"Warning: No data in window [{start_time}s, {end_time}s] for {labels[i]}")
            continue

        # Detect curves ONLY in the windowed data
        peaks_idx, _, _ = detect_curves(windowed, 'filtered', height_threshold=peak_height)

        # Extract metrics for each curve
        windowed_metrics = []
        for peak_idx_in_window in peaks_idx:
            metrics = calculate_curve_metrics(windowed, peak_idx_in_window)
            if metrics:
                windowed_metrics.append(metrics)

        aligned_rides.append({
            'label': labels[i],
            'filepath': filepaths[i],
            'dataframe': windowed,
            'peaks': peaks_idx,
            'metrics': windowed_metrics,
            'peak_time': peak_time,
            'window_start': start_time,
            'window_end': end_time,
        })

    # Generate comparison table
    print(f"\n{'='*90}")
    print(f"MULTI-RIDE CURVE ANALYSIS (Independent Detection)")
    print(f"Window: {before_s}s before to {after_s}s after peak")
    print(f"Metric Details: Crest = peak/RMS (per curve), Energy = integral (windowed), Smoothness = variance of 2nd deriv")
    print(f"{'='*90}\n")

    comparison_data = []

    for ride_idx, ride in enumerate(aligned_rides):
        print(f"\n{ride['label']}:")
        print(f"{'Curve':<8} {'Time':<10} {'Amplitude':<12} {'Crest':<10} {'Energy':<12} {'Smoothness':<12}")
        print("-" * 70)

        for curve_num, m in enumerate(ride['metrics'], 1):
            print(f"{curve_num:<8} {m['peak_time']:<10.2f} {m['peak_amplitude']:<12.0f} "
                  f"{m['crest_factor']:<10.2f} {m['energy']:<12.0f} {m['smoothness']:<12.2f}")

        # Summary stats
        amplitudes = [m['peak_amplitude'] for m in ride['metrics']]
        crest_factors = [m['crest_factor'] for m in ride['metrics']]
        print(f"\nStats: Amp(mean={np.mean(amplitudes):.0f}, std={np.std(amplitudes):.0f}), "
              f"Crest(mean={np.mean(crest_factors):.2f}, std={np.std(crest_factors):.2f})")

    print(f"\n{'='*90}\n")

    # Plot comparison
    if (plot or save_path) and HAS_MATPLOTLIB:
        n_rides = len(aligned_rides)
        fig, axes = plt.subplots(n_rides, 1, figsize=(14, 5*n_rides), sharex=True)

        if n_rides == 1:
            axes = [axes]

        for ax_idx, ride in enumerate(aligned_rides):
            ax = axes[ax_idx]
            df = ride['dataframe']
            peaks_idx = ride['peaks']
            peak_time = ride['peak_time']

            # Align time: subtract peak_time so peak is at t=0 (same as compare_rides.py)
            time_aligned = df['time_s'] - peak_time

            ax.plot(time_aligned, df['filtered'], color='purple', linewidth=1.5, alpha=0.8, label='Filtered')
            if len(peaks_idx) > 0:
                ax.plot(time_aligned.iloc[peaks_idx], df['filtered'].iloc[peaks_idx],
                       'r*', markersize=12, label='Curves', zorder=5)

                # Add curve number annotations
                for curve_num, peak_idx in enumerate(peaks_idx, 1):
                    x = time_aligned.iloc[peak_idx]
                    y = df['filtered'].iloc[peak_idx]
                    ax.text(x, y - 30000, str(curve_num), fontsize=9, ha='center',
                           bbox=dict(boxstyle='round,pad=0.3', facecolor='yellow', alpha=0.7))

            ax.axhline(y=0, color='black', linestyle='--', linewidth=0.5, alpha=0.5)
            ax.set_ylabel('Signal')
            ax.set_title(f'{ride["label"]} ({ride["filepath"]})')
            ax.grid(True, alpha=0.3)
            ax.legend()

        axes[-1].set_xlabel('Time (s from peak)')
        fig.suptitle('Ride Comparison - Aligned to Peak', fontsize=14, y=1.00)
        plt.tight_layout()

        if save_path:
            plt.savefig(save_path, dpi=150)
            print(f"Saved comparison plot to {save_path}")
        if plot:
            plt.show()

    return {
        'aligned_rides': aligned_rides,
        'comparison': comparison_data,
    }

def main():
    ap = argparse.ArgumentParser(description="Detect and analyze track curves from steering sensor data.")

    # Input files
    ap.add_argument("files", nargs="+", help="Path(s) to data file(s)")

    # Filtering parameters
    ap.add_argument("--hp", "--highpass-cutoff", type=float, default=0.05,
                   help="High-pass cutoff frequency (Hz, default: 0.05)")
    ap.add_argument("--lp", "--lowpass-cutoff", type=float, default=5.0,
                   help="Low-pass cutoff frequency (Hz, default: 5.0)")
    ap.add_argument("--order", type=int, default=4,
                   help="Butterworth filter order (default: 4)")

    # Time window parameters
    ap.add_argument("--start", type=float, metavar="SECONDS",
                   help="Start time in seconds for analysis")
    ap.add_argument("--end", type=float, metavar="SECONDS",
                   help="End time in seconds for analysis")

    # Peak detection parameters
    # best threshold for all 3 rides is about -25000
    ap.add_argument("--peak-height", type=float, metavar="HEIGHT",
                   help="Minimum peak height threshold (auto-calculated if not set)")
    ap.add_argument("--min-distance", type=float, default=2.0, metavar="SECONDS",
                   help="Minimum distance between curves in seconds (default: 2.0)")

    # Multi-ride comparison
    ap.add_argument("--compare", action="store_true",
                   help="Compare multiple rides")
    ap.add_argument("--peak-mode", choices=['max', 'max_pos', 'first'], default='max',
                   help="Peak detection mode for alignment (default: max)")
    ap.add_argument("--threshold", type=float,
                   help="Threshold for --peak-mode first (required for first mode)")
    ap.add_argument("--before", type=float, default=30,
                   help="Seconds before peak for analysis window (default: 30)")
    ap.add_argument("--after", type=float, default=25,
                   help="Seconds after peak for analysis window (default: 25)")

    # Output parameters
    ap.add_argument("--plot", action="store_true",
                   help="Show plot")
    ap.add_argument("--save", metavar="PATH",
                   help="Save plot to file")

    args = ap.parse_args()

    # Route to appropriate analysis
    if args.compare:
        if len(args.files) < 2:
            print("Error: --compare requires at least 2 files")
            return

        if args.peak_mode == 'first' and not args.threshold:
            print("Error: --peak-mode first requires --threshold")
            return

        compare_rides(
            args.files,
            hp_cutoff=args.hp,
            lp_cutoff=args.lp,
            order=args.order,
            peak_mode=args.peak_mode,
            peak_threshold=args.threshold,
            peak_height=args.peak_height,
            before_s=args.before,
            after_s=args.after,
            plot=args.plot,
            save_path=args.save,
        )
    else:
        # Single ride analysis
        if len(args.files) > 1:
            print("Note: Multiple files provided but --compare not set. Analyzing first file only.")

        analyze_single_ride(
            args.files[0],
            hp_cutoff=args.hp,
            lp_cutoff=args.lp,
            order=args.order,
            start_s=args.start,
            end_s=args.end,
            plot=args.plot,
            save_path=args.save,
        )

if __name__ == "__main__":
    main()
