"""Shared CLI contract for the foreground launcher and HTTP application."""
import argparse
from pathlib import Path


def argument_parser(description=None):
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--port', default=8765, type=int)
    parser.add_argument('--camera', default='0')
    parser.add_argument('--video')
    parser.add_argument('--output-dir', type=Path)
    parser.add_argument('--diagnostic', action='store_true', help=argparse.SUPPRESS)
    parser.add_argument('--listen-fd', type=int, help=argparse.SUPPRESS)
    return parser
