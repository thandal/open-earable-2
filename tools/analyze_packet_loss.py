#!/usr/bin/env python3
"""Analyze packet loss in OpenEarable .oe sensor log files.

Supports file format versions 1-3. Reports per-sensor packet counts,
timing statistics, gap analysis, and comparison with known baselines.

Usage:
    python tools/analyze_packet_loss.py data/sensor_log_*.oe
"""

import argparse
import struct
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np


SENSOR_SID = {
    "imu": 0,
    "barometer": 1,
    "microphone": 2,
    "ppg": 4,
    "optical_temp": 6,
    "bone_acc": 7,
}
SID_NAME = {v: k for k, v in SENSOR_SID.items()}

# Expected timing for each sensor
SENSOR_EXPECTED = {
    "microphone": {
        "frames_per_packet": 48,
        "sample_rate": 48000,
        "nominal_interval_ms": 1.0,  # 48 frames / 48kHz
    },
    "bone_acc": {
        "samples_per_packet": 6,
        "fallback_rate": 6397.0,
        "nominal_interval_ms": 0.94,
    },
}


def parse_file_header(f):
    """Parse file header, returns (version, timestamp_us, device_id, channel).
    device_id and channel are None for versions < 3."""
    data = f.read(10)
    version, timestamp = struct.unpack("<HQ", data)

    device_id = None
    channel = None
    if version >= 3:
        extra = f.read(9)  # uint64 device_id + uint8 channel
        device_id, channel = struct.unpack("<Qb", extra)

    return version, timestamp, device_id, channel


def parse_packets(f, version):
    """Parse all packets, returning dict of sensor_name -> list of (timestamp_s, n_samples)."""
    packets = defaultdict(list)
    sample_size_bone = struct.calcsize("<3h")  # 6 bytes

    while True:
        header = f.read(10)
        if len(header) < 10:
            break
        sid, size, time = struct.unpack("<BBQ", header)
        if size > 192 or sid > 7:
            break

        data = f.read(size)
        if len(data) < size:
            break

        ts = time / 1e6
        name = SID_NAME.get(sid, f"sid{sid}")

        if sid == SENSOR_SID["microphone"]:
            # 96 int16 samples = 48 stereo frames per packet
            n_frames = size // (2 * 2)  # 2 channels, 2 bytes each
            packets[name].append((ts, n_frames))
        elif sid == SENSOR_SID["bone_acc"]:
            if size == sample_size_bone:
                n_samples = 1
            elif (size - 2) % sample_size_bone == 0:
                n_samples = (size - 2) // sample_size_bone
            else:
                n_samples = size // sample_size_bone
            packets[name].append((ts, n_samples))
        else:
            packets[name].append((ts, 1))

    parsed_to = f.tell()
    return packets, parsed_to


def analyze_sensor(name, entries):
    """Analyze packet timing for a single sensor. Returns a stats dict."""
    timestamps = np.array([e[0] for e in entries])
    sample_counts = np.array([e[1] for e in entries])
    total_samples = int(sample_counts.sum())

    stats = {
        "packets": len(timestamps),
        "total_samples": total_samples,
        "time_span_s": 0.0,
    }

    if len(timestamps) < 2:
        return stats

    diffs = np.diff(timestamps)
    span = timestamps[-1] - timestamps[0]
    stats["time_span_s"] = span

    stats["interval_mean_ms"] = diffs.mean() * 1000
    stats["interval_median_ms"] = float(np.median(diffs) * 1000)
    stats["interval_std_ms"] = diffs.std() * 1000
    stats["interval_min_ms"] = diffs.min() * 1000
    stats["interval_max_ms"] = diffs.max() * 1000

    # Gap counts at various thresholds
    for thresh_ms in [5, 10, 50, 100, 500]:
        stats[f"gaps_gt_{thresh_ms}ms"] = int((diffs > thresh_ms / 1000).sum())

    # Largest gaps
    big_mask = diffs > 0.005
    if big_mask.any():
        big_gaps = sorted(diffs[big_mask] * 1000, reverse=True)
        stats["largest_gaps_ms"] = big_gaps[:15]

    # Sensor-specific loss estimates
    if name == "microphone":
        cfg = SENSOR_EXPECTED["microphone"]
        expected_packets = span / (cfg["nominal_interval_ms"] / 1000)
        stats["expected_packets"] = int(expected_packets)
        stats["packet_loss_pct"] = (1 - len(timestamps) / expected_packets) * 100
        dropped = expected_packets - len(timestamps)
        stats["dropped_time_s"] = dropped * cfg["frames_per_packet"] / cfg["sample_rate"]

        # Normal interval check
        normal = (diffs >= 0.0008) & (diffs <= 0.0012)
        stats["normal_interval_pct"] = normal.sum() / len(diffs) * 100

    elif name == "bone_acc":
        cfg = SENSOR_EXPECTED["bone_acc"]
        # Estimate true rate from normal intervals
        normal_mask = diffs < 0.002
        if normal_mask.any():
            normal_rates = sample_counts[:-1][normal_mask] / diffs[normal_mask]
            est_rate = float(np.median(normal_rates))
        else:
            est_rate = cfg["fallback_rate"]
        stats["estimated_rate_hz"] = est_rate
        expected_samples = span * est_rate
        stats["expected_samples"] = int(expected_samples)
        stats["sample_loss_pct"] = (1 - total_samples / expected_samples) * 100
        stats["dropped_time_s"] = (expected_samples - total_samples) / est_rate

        # Sample count distribution
        unique, counts = np.unique(sample_counts, return_counts=True)
        stats["samples_per_packet_dist"] = dict(zip(unique.astype(int).tolist(), counts.tolist()))

    return stats


def print_report(filename, version, timestamp_us, device_id, channel,
                 packets, parsed_to, file_size):
    """Print a formatted analysis report."""
    print(f"\n{'='*60}")
    print(f"File: {filename}")
    print(f"  Version: {version}, Size: {file_size:,} bytes, Parsed: {parsed_to/file_size*100:.1f}%")
    print(f"  Start timestamp: {timestamp_us} us ({timestamp_us/1e6:.2f}s)")
    if device_id is not None:
        print(f"  Device ID: {device_id:#x}, Channel: {channel}")

    # List all sensors present and missing
    present = sorted(packets.keys())
    all_sensors = ["imu", "barometer", "microphone", "ppg", "optical_temp", "bone_acc"]
    missing = [s for s in all_sensors if s not in present]
    print(f"\n  Sensors present: {', '.join(present) if present else 'none'}")
    if missing:
        print(f"  Sensors MISSING: {', '.join(missing)}")

    for name in present:
        entries = packets[name]
        stats = analyze_sensor(name, entries)
        print(f"\n--- {name.upper()} ---")
        print(f"  Packets: {stats['packets']:,}")
        print(f"  Time span: {stats['time_span_s']:.3f}s")

        if stats["time_span_s"] == 0:
            continue

        if stats.get("total_samples") and name in ("bone_acc",):
            print(f"  Total samples: {stats['total_samples']:,}")

        print(f"  Interval (ms): median={stats['interval_median_ms']:.3f}, "
              f"mean={stats['interval_mean_ms']:.3f}, "
              f"std={stats['interval_std_ms']:.3f}")
        print(f"  Interval range (ms): [{stats['interval_min_ms']:.3f}, {stats['interval_max_ms']:.1f}]")

        if "normal_interval_pct" in stats:
            print(f"  Normal interval (0.8-1.2ms): {stats['normal_interval_pct']:.1f}%")

        # Gap summary
        gap_lines = []
        for thresh_ms in [5, 10, 50, 100, 500]:
            count = stats.get(f"gaps_gt_{thresh_ms}ms", 0)
            if count > 0:
                gap_lines.append(f">{thresh_ms}ms: {count}")
        if gap_lines:
            print(f"  Gaps: {', '.join(gap_lines)}")

        if "largest_gaps_ms" in stats:
            gaps_str = ", ".join(f"{g:.1f}" for g in stats["largest_gaps_ms"][:10])
            print(f"  Largest gaps (ms): [{gaps_str}]")

        # Loss estimates
        if name == "microphone":
            print(f"  Expected packets: {stats.get('expected_packets', '?'):,}")
            print(f"  Packet loss: {stats.get('packet_loss_pct', 0):.2f}%")
            print(f"  Dropped time: {stats.get('dropped_time_s', 0):.2f}s")
        elif name == "bone_acc":
            print(f"  Estimated rate: {stats.get('estimated_rate_hz', 0):.1f} Hz")
            print(f"  Expected samples: {stats.get('expected_samples', '?'):,}")
            print(f"  Sample loss: {stats.get('sample_loss_pct', 0):.2f}%")
            print(f"  Dropped time: {stats.get('dropped_time_s', 0):.2f}s")
            if "samples_per_packet_dist" in stats:
                print(f"  Samples/packet: {stats['samples_per_packet_dist']}")

    # Cross-sensor time alignment (microphone as reference, or first sensor)
    if len(present) >= 2:
        print(f"\n--- SENSOR TIME ALIGNMENT ---")

        # Gather start/end/span for each sensor
        sensor_times = {}
        for name in present:
            ts = np.array([e[0] for e in packets[name]])
            sensor_times[name] = (ts[0], ts[-1], ts[-1] - ts[0])

        # Use microphone as reference if present, otherwise first sensor
        ref = "microphone" if "microphone" in sensor_times else present[0]
        ref_start, ref_end, ref_span = sensor_times[ref]
        others = [n for n in present if n != ref]

        print(f"  Reference: {ref} ({ref_span:.3f}s)")
        print(f"  {'Sensor':<15} {'Start vs ref':>14} {'End vs ref':>14} {'Span diff':>12}")
        for name in others:
            start, end, span = sensor_times[name]
            start_off = (start - ref_start) * 1000
            end_off = (end - ref_end) * 1000
            span_diff = (span - ref_span) * 1000
            print(f"  {name:<15} {start_off:+13.1f}ms {end_off:+13.1f}ms {span_diff:+11.1f}ms")

        # Overlap analysis
        overlap_start = max(t[0] for t in sensor_times.values())
        overlap_end = min(t[1] for t in sensor_times.values())
        global_span = max(t[1] for t in sensor_times.values()) - min(t[0] for t in sensor_times.values())
        if overlap_end > overlap_start:
            overlap = overlap_end - overlap_start
            print(f"\n  Common overlap: {overlap:.3f}s "
                  f"({overlap/global_span*100:.1f}% of {global_span:.3f}s total span)")
        else:
            print(f"\n  WARNING: No time overlap between all sensors!")

    # Comparison with baseline (from README known issues)
    if "microphone" in packets or "bone_acc" in packets:
        print(f"\n--- COMPARISON WITH BASELINE (dataset1) ---")
        if "microphone" in packets:
            s = analyze_sensor("microphone", packets["microphone"])
            print(f"  Mic packet loss:  {s.get('packet_loss_pct', 0):.2f}%  (baseline: ~3-4%)")
        if "bone_acc" in packets:
            s = analyze_sensor("bone_acc", packets["bone_acc"])
            print(f"  Bone sample loss: {s.get('sample_loss_pct', 0):.2f}%  (baseline: ~25%)")

    print()


def analyze_file(filepath):
    """Analyze a single .oe file."""
    filepath = Path(filepath)
    file_size = filepath.stat().st_size

    with open(filepath, "rb") as f:
        version, timestamp_us, device_id, channel = parse_file_header(f)
        packets, parsed_to = parse_packets(f, version)

    print_report(
        filepath.name, version, timestamp_us, device_id, channel,
        packets, parsed_to, file_size,
    )


def main():
    parser = argparse.ArgumentParser(description="Analyze packet loss in .oe sensor log files")
    parser.add_argument("files", nargs="+", help="One or more .oe files to analyze")
    args = parser.parse_args()

    for filepath in args.files:
        try:
            analyze_file(filepath)
        except Exception as e:
            print(f"\nERROR processing {filepath}: {e}", file=sys.stderr)


if __name__ == "__main__":
    main()
