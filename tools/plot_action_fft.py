#!/usr/bin/env python3
"""Plot 6x2 FFT subplots from logged action CSV."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def _find_latest_csv() -> Path:
  csv_dir = Path("logs") / "csv"
  files = sorted(
    csv_dir.glob("lerobot_humanoid_no_arms_flat_play_*.csv"),
    key=lambda p: p.stat().st_mtime,
    reverse=True,
  )
  if not files:
    raise FileNotFoundError(
      "No CSV found in logs/csv matching "
      "'lerobot_humanoid_no_arms_flat_play_*.csv'."
    )
  return files[0]


def _action_columns(header: list[str]) -> list[str]:
  action_cols = [h for h in header if h.startswith("action_")]
  action_cols.sort(key=lambda name: int(name.split("_")[1]))
  return action_cols


def _load_actions(csv_path: Path) -> tuple[np.ndarray, list[str]]:
  with csv_path.open(newline="") as f:
    reader = csv.DictReader(f)
    if reader.fieldnames is None:
      raise ValueError(f"CSV header missing in {csv_path}")
    action_cols = _action_columns(reader.fieldnames)
    if len(action_cols) < 12:
      raise ValueError(
        f"Expected at least 12 action columns, found {len(action_cols)} in {csv_path}"
      )

    rows = []
    for row in reader:
      rows.append([float(row[c]) for c in action_cols[:12]])

  if len(rows) < 4:
    raise ValueError(
      f"Need at least 4 timesteps for FFT, found {len(rows)} rows in {csv_path}"
    )
  return np.asarray(rows, dtype=np.float64), action_cols[:12]


def _fft_mag(signal: np.ndarray, sample_rate: float) -> tuple[np.ndarray, np.ndarray]:
  x = signal - np.mean(signal)
  fft_vals = np.fft.rfft(x)
  mag = np.abs(fft_vals) / len(x)
  freq = np.fft.rfftfreq(len(x), d=1.0 / sample_rate)
  return freq, mag


def main() -> None:
  parser = argparse.ArgumentParser(
    description=(
      "Generate a 6x2 FFT plot for action_0..action_11 from "
      "LeRobot no-arms flat-play CSV logs."
    )
  )
  parser.add_argument(
    "--csv",
    type=Path,
    default=None,
    help="Path to CSV log. If omitted, use latest logs/csv file.",
  )
  parser.add_argument(
    "--sample-rate",
    type=float,
    default=1.0,
    help="Action sampling rate in Hz (default: 1.0).",
  )
  parser.add_argument(
    "--output",
    type=Path,
    default=None,
    help="Output image path (default: <csv_stem>_action_fft_6x2.png).",
  )
  parser.add_argument(
    "--max-freq",
    type=float,
    default=None,
    help="Max frequency shown on x-axis (Hz).",
  )
  args = parser.parse_args()

  csv_path = args.csv if args.csv is not None else _find_latest_csv()
  actions, action_cols = _load_actions(csv_path)

  output_path = args.output
  if output_path is None:
    output_path = csv_path.with_name(csv_path.stem + "_action_fft_6x2.png")
  output_path.parent.mkdir(parents=True, exist_ok=True)

  fig, axes = plt.subplots(6, 2, figsize=(14, 18), sharex=False)
  fig.suptitle(f"Action FFT (Left 6 joints | Right 6 joints)\n{csv_path.name}")

  for i in range(6):
    left_idx = i
    right_idx = i + 6

    f_left, m_left = _fft_mag(actions[:, left_idx], args.sample_rate)
    f_right, m_right = _fft_mag(actions[:, right_idx], args.sample_rate)

    ax_left = axes[i, 0]
    ax_right = axes[i, 1]

    ax_left.plot(f_left, m_left, linewidth=1.0)
    ax_right.plot(f_right, m_right, linewidth=1.0)

    ax_left.set_title(f"Left joint {i + 1} ({action_cols[left_idx]})")
    ax_right.set_title(f"Right joint {i + 1} ({action_cols[right_idx]})")
    ax_left.set_ylabel("Magnitude")
    ax_left.set_xlabel("Frequency (Hz)")
    ax_right.set_xlabel("Frequency (Hz)")
    ax_left.tick_params(axis="x", labelbottom=True)
    ax_right.tick_params(axis="x", labelbottom=True)

    if args.max_freq is not None:
      ax_left.set_xlim(0.0, args.max_freq)
      ax_right.set_xlim(0.0, args.max_freq)

    ax_left.grid(True, alpha=0.3)
    ax_right.grid(True, alpha=0.3)

  fig.tight_layout(rect=(0.0, 0.02, 1.0, 0.97))
  fig.savefig(output_path, dpi=180)
  plt.show()
  plt.close(fig)

  print(f"CSV used: {csv_path}")
  print(f"Saved figure: {output_path}")


if __name__ == "__main__":
  main()
