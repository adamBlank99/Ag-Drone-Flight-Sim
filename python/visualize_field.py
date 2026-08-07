#!/usr/bin/env python3

"""Visualize a rectangular agricultural field."""

import argparse
from pathlib import Path


def positive_number(value: str) -> float:
    """Convert a command-line value to a positive floating-point number."""
    number = float(value)
    if number <= 0:
        raise argparse.ArgumentTypeError("value must be greater than zero")
    return number


def visualize_field(
    width: float,
    height: float,
    output_path: Path | None = None,
) -> None:
    """Draw a rectangular field and either display or save the figure."""
    try:
        import matplotlib.pyplot as plt
        from matplotlib.patches import Rectangle
    except ModuleNotFoundError as error:
        raise SystemExit(
            "Matplotlib is required. Install it with: "
            "python3 -m pip install -r python/requirements.txt"
        ) from error

    figure, axes = plt.subplots(figsize=(10, 6))

    field = Rectangle(
        (0, 0),
        width,
        height,
        facecolor="#8fbc5a",
        edgecolor="#285430",
        linewidth=2,
        alpha=0.8,
    )
    axes.add_patch(field)

    margin = max(width, height) * 0.08
    axes.set_xlim(-margin, width + margin)
    axes.set_ylim(-margin, height + margin)
    axes.set_aspect("equal", adjustable="box")
    axes.set_xlabel("X position (m)")
    axes.set_ylabel("Y position (m)")
    axes.set_title(f"Agricultural Field ({width:g} m × {height:g} m)")
    axes.grid(True, linestyle="--", alpha=0.35)

    axes.text(
        width / 2,
        height / 2,
        f"{width:g} m × {height:g} m",
        horizontalalignment="center",
        verticalalignment="center",
        fontsize=13,
        color="#17351d",
    )

    figure.tight_layout()

    if output_path is None:
        plt.show()
    else:
        output_path.parent.mkdir(parents=True, exist_ok=True)
        figure.savefig(output_path, dpi=150, bbox_inches="tight")
        print(f"Saved field visualization to {output_path}")

    plt.close(figure)


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Visualize a rectangular agricultural field."
    )
    parser.add_argument(
        "--width",
        type=positive_number,
        default=100.0,
        help="field width in meters (default: 100)",
    )
    parser.add_argument(
        "--height",
        type=positive_number,
        default=60.0,
        help="field height in meters (default: 60)",
    )
    parser.add_argument(
        "--save",
        type=Path,
        metavar="FILE",
        help="save the plot instead of opening a window",
    )
    return parser.parse_args()


def main() -> None:
    arguments = parse_arguments()
    visualize_field(arguments.width, arguments.height, arguments.save)


if __name__ == "__main__":
    main()
