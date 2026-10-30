#!/usr/bin/env python3

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd


SCRIPT_DIR = Path(__file__).resolve().parent


def newest_log(filename):
    candidates = [
        Path.cwd() / filename,
        Path.cwd() / "pid_controller" / filename,
        SCRIPT_DIR / filename,
        SCRIPT_DIR / "pid_controller" / filename,
        SCRIPT_DIR / "pid_controller" / "build-wsl" / filename,
    ]
    existing = {path.resolve() for path in candidates if path.is_file()}
    if not existing:
        raise FileNotFoundError(
            "Could not find {}. Run the controller first or pass --log-dir."
            .format(filename)
        )
    return max(existing, key=lambda path: path.stat().st_mtime)


def read_log(log_dir, filename, columns):
    path = (log_dir / filename) if log_dir else newest_log(filename)
    if not path.is_file():
        raise FileNotFoundError("PID log not found: {}".format(path))

    frame = pd.read_csv(path, sep=r"\s+", header=None, usecols=range(len(columns)))
    frame.columns = columns
    print("Using {} ({} samples)".format(path, len(frame)))
    return frame, path


def plot_steering(frame, output_dir):
    figure, axis = plt.subplots(figsize=(12, 6))
    axis.plot(frame["Iteration"], frame["Steering Error"], label="Steering error [rad]")
    axis.plot(frame["Iteration"], frame["Steering Output"], label="Steering command")
    axis.axhline(0.0, color="black", linewidth=0.8)
    axis.set_title("Steering PID Performance")
    axis.set_xlabel("Controller iteration")
    axis.set_ylabel("Error / command")
    axis.grid(True, alpha=0.3)
    axis.legend()
    figure.tight_layout()
    path = output_dir / "steering_pid.png"
    figure.savefig(path, dpi=160)
    plt.close(figure)
    return path


def plot_throttle(frame, output_dir):
    figure, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    axes[0].plot(frame["Iteration"], frame["Speed Error"], color="tab:blue")
    axes[0].axhline(0.0, color="black", linewidth=0.8)
    axes[0].set_title("Longitudinal PID Performance")
    axes[0].set_ylabel("Speed error [m/s]")
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(frame["Iteration"], frame["Throttle Output"],
                 color="tab:green", label="Throttle")
    axes[1].plot(frame["Iteration"], frame["Brake Output"],
                 color="tab:red", label="Brake")
    axes[1].set_xlabel("Controller iteration")
    axes[1].set_ylabel("Control command")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend()

    figure.tight_layout()
    path = output_dir / "throttle_pid.png"
    figure.savefig(path, dpi=160)
    plt.close(figure)
    return path


def main():
    parser = argparse.ArgumentParser(description="Plot PID controller logs")
    parser.add_argument(
        "--log-dir",
        type=Path,
        help="Directory containing steer_pid_data.txt and throttle_pid_data.txt",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=SCRIPT_DIR / "pid_plots" / "final",
        help="Directory for generated PNG files",
    )
    args = parser.parse_args()

    log_dir = args.log_dir.resolve() if args.log_dir else None
    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    steering, steering_source = read_log(
        log_dir,
        "steer_pid_data.txt",
        ["Iteration", "Steering Error", "Steering Output"],
    )
    throttle, throttle_source = read_log(
        log_dir,
        "throttle_pid_data.txt",
        ["Iteration", "Speed Error", "Brake Output", "Throttle Output"],
    )

    steering_plot = plot_steering(steering, output_dir)
    throttle_plot = plot_throttle(throttle, output_dir)

    print("Steering source: {}".format(steering_source))
    print("Throttle source: {}".format(throttle_source))
    print("Created {}".format(steering_plot))
    print("Created {}".format(throttle_plot))


if __name__ == "__main__":
    main()
