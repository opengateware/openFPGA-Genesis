#!/usr/bin/env python3

"""Reverse RBF bitstream."""

import argparse
from pathlib import Path
import sys


def reverse_file(src: Path, dst: Path) -> None:
    """Read |src|, reverse its bits, then write to |dst|."""
    table = bytes(int(f"{i:08b}"[::-1], 2) for i in range(256))
    # Since the files tend to be a few MB in size, reading the whole thing
    # into memory shouldn't be a problem.
    dst.write_bytes(src.read_bytes().translate(table))


def get_parser() -> argparse.ArgumentParser:
    """Get CLI parser."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    return parser


def main(argv) -> int:
    parser = get_parser()
    opts = parser.parse_args(argv)
    reverse_file(opts.input, opts.output)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
