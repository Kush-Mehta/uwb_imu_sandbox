"""Parser for the UWB tag's text output.

The simulated tag publishes exactly the line the real tag prints on its
serial port, so the same parser works for both:

    [network_count,anchor_id,range_m,rx,fp,quality,delta]

In the simulator rx, fp, quality and delta are normalised, unitless values.
"""
from dataclasses import dataclass
import math
from typing import Optional


@dataclass
class UwbLine:
    network_count: int
    anchor_id: int
    range_m: float
    rx: float
    fp: float
    quality: float
    delta: float


def parse_uwb_line(line: str) -> Optional[UwbLine]:
    """Return the parsed line, or None if it is not a valid ranging line."""
    if not isinstance(line, str):
        return None
    line = line.strip()
    if not (line.startswith('[') and line.endswith(']')):
        return None
    fields = [f.strip() for f in line[1:-1].split(',')]
    if len(fields) != 7:
        return None
    try:
        net = int(fields[0])
        anchor = int(fields[1], 0) if fields[1].lower().startswith('0x') else int(fields[1])
        values = [float(f) for f in fields[2:]]
    except ValueError:
        return None
    if net < 0 or anchor < 0 or not all(math.isfinite(v) for v in values):
        return None
    if values[0] < 0.0:
        return None
    return UwbLine(net, anchor, *values)
