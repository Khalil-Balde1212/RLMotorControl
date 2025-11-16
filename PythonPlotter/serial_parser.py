import numpy as np
from dataclasses import dataclass


@dataclass
class SIData:
    matrix: np.ndarray
    vector: np.ndarray
    conv_rate: float
    curr_error: float
    converged: bool

    def pretty(self) -> str:
        return (
            f"State Transition Matrix:\n{self.matrix}\n"
            f"Response Vector: {self.vector}\n"
            f"Convergence rate: {self.conv_rate:.6f}\n"
            f"Current error: {self.curr_error:.6f}\n"
            f"Converged: {self.converged}\n"
        )


@dataclass
class SDData:
    pos: float
    vel: float
    acc: float
    jerk: float
    current: float
    motor_command: float
    setpoint: float | None = None

    def pretty(self) -> str:
        return (
            f"pos: {self.pos}\n"
            f"setpoint: {self.setpoint}\n"
            f"vel: {self.vel}\n"
            f"acc: {self.acc}\n"
            f"jerk: {self.jerk}\n"
            f"current: {self.current}\n"
            f"motorCommand: {self.motor_command}\n"
        )


def parse_data(line: str):
    try:
        values = line.strip().split(',')
        # Accept exactly 23 tokens, but also accept lines with extra trailing tokens
        # (some devices print additional fields). Only the first 23 are used.
        if len(values) < 23:
            return None
        if len(values) > 23:
            values = values[:23]

        matrix_flat = [float(v) for v in values[:16]]
        matrix = np.array(matrix_flat).reshape(4, 4)

        vector_flat = [float(v) for v in values[16:20]]
        vector = np.array(vector_flat)

        conv_rate = float(values[20])
        curr_error = float(values[21])
        converged = bool(int(values[22]))

        return SIData(matrix, vector, conv_rate, curr_error, converged)
    except Exception:
        return None


def parse_message(line: str):
    if not line:
        return None, None

    tokens = line.split(',', 1)
    if len(tokens) > 1:
        prefix = tokens[0].strip().upper()
        rest = tokens[1]
        if prefix == 'SI':
            parsed = parse_data(rest)
            return 'SI', parsed
        if prefix == 'SD':
            try:
                parts = [p.strip() for p in rest.split(',')]
                if len(parts) < 6:
                    return 'SD', None

                def _to_float(x):
                    try:
                        return float(x)
                    except Exception:
                        import re

                        m = re.search(r'-?\d+\.?\d*', x)
                        if m:
                            try:
                                return float(m.group(0))
                            except Exception:
                                pass
                        return float('nan')

                # support both old 6-val sd and new 7-val with setpoint as 2nd value
                if len(parts) >= 7:
                    vals = [_to_float(p) for p in parts[:7]]
                    return 'SD', SDData(pos=vals[0], vel=vals[2], acc=vals[3], jerk=vals[4], current=vals[5], motor_command=vals[6], setpoint=vals[1])
                else:
                    vals = [_to_float(p) for p in parts[:6]]
                    return 'SD', SDData(pos=vals[0], vel=vals[1], acc=vals[2], jerk=vals[3], current=vals[4], motor_command=vals[5], setpoint=None)
            except Exception:
                return 'SD', None

        # Some devices output the SI tag after the 20 numeric columns rather
        # than at the start of the line. Detect that case and handle it here.
        fields = [t.strip() for t in line.split(',')]
        if 'SI' in [t.upper() for t in fields]:
            try:
                upper = [t.upper() for t in fields]
                si_index = upper.index('SI')
                # expected format: 16 matrix + 4 vector -> 20 numeric tokens; then 'SI', conv_rate, curr_error, converged
                if si_index >= 20 and len(fields) >= si_index + 4:
                    matrix_flat = [float(x) for x in fields[0:16]]
                    matrix = np.array(matrix_flat).reshape(4, 4)
                    vector_flat = [float(x) for x in fields[16:20]]
                    vector = np.array(vector_flat)
                    conv_rate = float(fields[si_index + 1])
                    curr_error = float(fields[si_index + 2])
                    converged = bool(int(fields[si_index + 3]))
                    return 'SI', SIData(matrix, vector, conv_rate, curr_error, converged)
            except Exception:
                # fall through to normal parsing below
                pass

    parsed = parse_data(line)
    if parsed is not None:
        return 'RAW_CSV', parsed

    return None, None


def parse_all_messages(line: str):
    """Parse a line that may contain multiple prefixed messages like 'SD,... SI,...'

    Returns a list of (msg_id, data) tuples.
    """
    import re

    results = []
    # use regex to find all occurrences of prefixes and parse each section
    # This supports lines like: 'SD,val1,...,SI,....'
    pattern = re.compile(r"\b(SI|SD),", flags=re.IGNORECASE)
    it = pattern.finditer(line)
    starts = [m.start() for m in it]
    if not starts:
        # no prefixed tokens, fall back to single parse
        mid, data = parse_message(line)
        if mid is not None:
            results.append((mid, data))
        return results

    # build subsections from each start to the next
    starts.append(len(line))
    for a, b in zip(starts[:-1], starts[1:]):
        section = line[a:b].strip()
        mid, data = parse_message(section)
        if mid is not None:
            results.append((mid, data))

    return results
