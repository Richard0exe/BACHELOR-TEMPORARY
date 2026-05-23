import re
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt
import sys
import argparse
import os

# Pattern to parse data lines
pattern = re.compile(r"Raw\s*=\s*(-?\d+)\s*,\s*Time\s*=\s*(\d+)")

def load_data(path):
    """Load data from file and return DataFrame with time in seconds."""
    raw_vals = []
    time_us = []
    if not os.path.exists(path):
        return pd.DataFrame()
        
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
    df["time_s"] = df["time_s"] - df["time_s"].iloc[0]
    return df

def apply_bandpass_filter(data, highpass_cutoff=0.05, lowpass_cutoff=50.0, order=4):
    """Apply band-pass filter (0.05-50 Hz to remove drift and noise)."""
    if data.empty or len(data) <= 1:
        return None

    duration = data["time_s"].iloc[-1] - data["time_s"].iloc[0]
    if duration <= 0:
        return None

    sampling_rate = len(data) / duration
    nyquist = 0.5 * sampling_rate

    if highpass_cutoff >= nyquist or lowpass_cutoff >= nyquist:
        return None
    if highpass_cutoff >= lowpass_cutoff:
        return None
    if len(data) <= 3 * (order + 1):
        return None

    try:
        normal_cutoff_hp = highpass_cutoff / nyquist
        b_hp, a_hp = butter(order, normal_cutoff_hp, btype='high', analog=False)
        result = filtfilt(b_hp, a_hp, data['raw'].values)

        normal_cutoff_lp = lowpass_cutoff / nyquist
        b_lp, a_lp = butter(order, normal_cutoff_lp, btype='low', analog=False)
        result = filtfilt(b_lp, a_lp, result)

        return pd.Series(result, index=data.index)
    except Exception as e:
        return None

def compute_steering_metrics(corr_matrix, channel_names, loaded_channels):
    """Compute steering quality metrics from correlation matrix."""
    metrics = {}

    # Separate X and Y channels
    x_channels = [ch for ch in loaded_channels if '(X)' in channel_names[ch]]
    y_channels = [ch for ch in loaded_channels if '(Y)' in channel_names[ch]]

    # 1. Stability Score: Average within-axis correlation (diagonal excluded)
    x_indices = [loaded_channels.index(ch) for ch in x_channels if ch in loaded_channels]
    y_indices = [loaded_channels.index(ch) for ch in y_channels if ch in loaded_channels]

    x_corr_values = []
    if len(x_indices) > 1:
        for i in range(len(x_indices)):
            for j in range(i+1, len(x_indices)):
                x_corr_values.append(corr_matrix.iloc[x_indices[i], x_indices[j]])

    y_corr_values = []
    if len(y_indices) > 1:
        for i in range(len(y_indices)):
            for j in range(i+1, len(y_indices)):
                y_corr_values.append(corr_matrix.iloc[y_indices[i], y_indices[j]])

    metrics['x_stability'] = np.mean(x_corr_values) if x_corr_values else 0
    metrics['y_stability'] = np.mean(y_corr_values) if y_corr_values else 0

    # 2. Coupling Ratio: Average X-Y cross-correlation
    xy_corr_values = []
    if x_indices and y_indices:
        for i in x_indices:
            for j in y_indices:
                xy_corr_values.append(abs(corr_matrix.iloc[i, j]))

    metrics['coupling_ratio'] = np.mean(xy_corr_values) if xy_corr_values else 0

    # 3. Overall Stability (mean of within-axis correlations)
    all_within_axis = x_corr_values + y_corr_values
    metrics['stability_score'] = np.mean(all_within_axis) if all_within_axis else 0

    return metrics

def main():
    parser = argparse.ArgumentParser(description='Analyze correlations between all 8 channels of a ride.')
    parser.add_argument('--dir', type=str, required=True, help='Directory containing the ride data (e.g., 54, 55, 56)')
    parser.add_argument('--start', type=float, help='Start time in seconds')
    parser.add_argument('--end', type=float, help='End time in seconds')
    parser.add_argument('--hp', type=float, default=0.05, help='High-pass cutoff (Hz)')
    parser.add_argument('--lp', type=float, default=50.0, help='Low-pass cutoff (Hz)')
    parser.add_argument('--plot', action='store_true', help='Generate and save plots')
    
    args = parser.parse_args()
    
    ride_dir = args.dir
    if not os.path.isdir(ride_dir):
        print(f"Error: Directory {ride_dir} not found.")
        return

    ride_basename = os.path.basename(ride_dir.rstrip('/'))
    
    if ride_basename == '54':
        channel_names = {
            1: "R1 (Y)", 2: "R4 (X)", 3: "R5 (Y)", 4: "R7 (Y)",
            5: "A1 (Body)", 6: "A3 (Body)", 7: "A4 (Body)", 8: "A5 (Body)"
        }
        arm_channels = [1, 2, 3, 4]
    elif ride_basename in ['55', '56']:
        channel_names = {
            1: "R1 (Y)", 2: "R4 (X)", 3: "R5 (Y)", 4: "R7 (Y)",
            5: "R3 (Y)", 6: "R2 (X)", 7: "R6 (X)", 8: "R8 (X)"
        }
        arm_channels = [1, 2, 3, 4, 5, 6, 7, 8]
    else:
        channel_names = {i: f"Ch {i}" for i in range(1, 9)}
        arm_channels = [1, 2, 3, 4, 5, 6, 7, 8]

    all_channels_data = {}
    
    for ch in arm_channels:
        file_path = os.path.join(ride_dir, f"output{ch}.txt")
        df = load_data(file_path)
        if not df.empty:
            filtered = apply_bandpass_filter(df, highpass_cutoff=args.hp, lowpass_cutoff=args.lp)
            if filtered is not None:
                df['filtered'] = filtered
                all_channels_data[ch] = df

    if not all_channels_data:
        print("Error: No data loaded.")
        return

    start_t = args.start if args.start is not None else 0
    end_t = args.end if args.end is not None else min([df['time_s'].iloc[-1] for df in all_channels_data.values()])
    
    avg_fs = np.mean([len(df) / (df['time_s'].iloc[-1] - df['time_s'].iloc[0]) for df in all_channels_data.values()])
    target_time = np.linspace(start_t, end_t, int((end_t - start_t) * avg_fs))
    aligned_data = pd.DataFrame({'time_s': target_time})
    
    for ch, df in all_channels_data.items():
        aligned_data[f'Ch{ch}'] = np.interp(target_time, df['time_s'], df['filtered'])

    # Filter out "Dead" channels (where standard deviation is near zero)
    loaded_channels = [ch for ch in arm_channels if f'Ch{ch}' in aligned_data.columns]
    active_channels = []
    for ch in loaded_channels:
        if np.std(aligned_data[f'Ch{ch}']) > 1.0: # Threshold for "active"
            active_channels.append(ch)
        else:
            print(f"  Note: Channel {ch} ({channel_names[ch]}) appears to be DEAD (no signal). Skipping.")

    if not active_channels:
        print("Error: No active channels found.")
        return

    # Use only active channels for the matrix
    corr_matrix = aligned_data[[f'Ch{ch}' for ch in active_channels]].corr()

    # Compute steering metrics
    metrics = compute_steering_metrics(corr_matrix, channel_names, active_channels)

    # Compute activity concentration
    roughness = [np.std(np.diff(aligned_data[f'Ch{ch}'])) for ch in active_channels]
    total_activity = sum(roughness)
    activity_pct = [100 * r / total_activity if total_activity > 0 else 0 for r in roughness]

    # Activity concentration: how much is in top 2 channels?
    top_2_activity = sum(sorted(activity_pct, reverse=True)[:2])
    metrics['activity_concentration'] = top_2_activity

    # Print metrics summary to terminal
    print(f"\n{'='*60}")
    print(f"STEERING QUALITY METRICS - Ride {ride_basename}")
    print(f"{'='*60}")
    print(f"Stability Score:       {metrics['stability_score']:.3f}")
    print(f"  X-axis (lateral):    {metrics['x_stability']:.3f}")
    print(f"  Y-axis (vertical):   {metrics['y_stability']:.3f}")
    print(f"\nCoupling Ratio:        {metrics['coupling_ratio']:.3f}")
    print(f"  (X-Y cross-axis correlation)")
    print(f"\nActivity Concentration: {metrics['activity_concentration']:.1f}%")
    print(f"  (Activity in top 2 channels)")
    print(f"\nTop 3 Active Channels:")
    top_3_idx = sorted(range(len(roughness)), key=lambda i: roughness[i], reverse=True)[:3]
    for rank, idx in enumerate(top_3_idx, 1):
        ch = active_channels[idx]
        print(f"  {rank}. {channel_names[ch]}: {roughness[idx]:.0f} ({activity_pct[idx]:.1f}%)")
    print(f"{'='*60}\n")

    # Only generate plots if --plot flag is set
    if not args.plot:
        return

    fig = plt.figure(figsize=(16, 11))
    ax1 = plt.subplot2grid((2, 2), (0, 0), colspan=2)
    for ch in active_channels:
        col = f'Ch{ch}'
        if col in aligned_data.columns:
            ax1.plot(aligned_data['time_s'], aligned_data[col], label=channel_names[ch], alpha=0.7)
    ax1.set_title(f"Filtered Signals (Arm Only) - Ride {ride_basename}")
    ax1.legend(loc='upper right', ncol=4, fontsize=8)
    ax1.grid(True, alpha=0.3)

    ax2 = plt.subplot2grid((2, 2), (1, 0))
    im = ax2.imshow(corr_matrix, cmap='coolwarm', vmin=-1, vmax=1)

    # Correctly map ticks to the active arm channels
    num_channels = len(active_channels)
    ax2.set_xticks(np.arange(num_channels))
    ax2.set_yticks(np.arange(num_channels))
    ax2.set_xticklabels([channel_names[ch] for ch in active_channels], rotation=45)
    ax2.set_yticklabels([channel_names[ch] for ch in active_channels])

    for i in range(num_channels):
        for j in range(num_channels):
            ax2.text(j, i, f"{corr_matrix.iloc[i, j]:.2f}", ha="center", va="center", fontsize=8)
    ax2.set_title("Correlation Matrix")
    fig.colorbar(im, ax=ax2)

    ax3 = plt.subplot2grid((2, 2), (1, 1))
    bars = ax3.bar([channel_names[ch] for ch in active_channels], roughness)
    ax3.set_title("Activity (Micro-corrections)")
    ax3.set_xticklabels([channel_names[ch] for ch in active_channels], rotation=45)

    plt.tight_layout(rect=[0, 0, 1, 0.95])
    output_png = f"correlation_{os.path.basename(ride_dir)}.png"
    plt.savefig(output_png, dpi=150)
    print(f"Saved to {output_png}")

    plt.show()

if __name__ == "__main__":
    main()
