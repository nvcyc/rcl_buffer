#!/usr/bin/env python3
#
# Copyright 2026 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
ROS2 Pub/Sub Performance Benchmark Runner.

Tests every combination of topology (1:1, 1:N, multi-topic) and client
library pairing (rclcpp/rclpy) to measure throughput, drop rate, and latency.

Requires:
    pixi shell                          # activate pixi environment
    source install/setup.bash           # source colcon workspace
    python3 <this_script>               # run from workspace root

Usage examples:
    python3 run_benchmark.py                          # all combos, defaults
    python3 run_benchmark.py --clients cpp_cpp py_py  # only pure-language
    python3 run_benchmark.py --topologies 1pub_1sub   # single topology
    python3 run_benchmark.py --rate 10000 --duration 15
"""

import argparse
from dataclasses import dataclass, field
from datetime import datetime
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from typing import Dict, List, Optional, Tuple

PKG = 'perf_pubsub_benchmark'

# ---------------------------------------------------------------------------
# Executable mapping  (name -> filename inside install/PKG/lib/PKG/)
# ---------------------------------------------------------------------------

EXECUTABLES: Dict[str, Dict[str, str]] = {
    'cpp': {
        'pub': 'perf_publisher',
        'sub': 'perf_subscriber',
    },
    'py': {
        'pub': 'perf_publisher_py.py',
        'sub': 'perf_subscriber_py.py',
    },
}

# ---------------------------------------------------------------------------
# Topologies
# ---------------------------------------------------------------------------

TOPOLOGIES = [
    {
        'name': '1pub_1sub',
        'description': '1 Pub -> 1 Sub',
        'topics': [
            {'topic': '/perf_bench/topic_0', 'num_pubs': 1, 'num_subs': 1},
        ],
    },
    {
        'name': '1pub_2sub',
        'description': '1 Pub -> 2 Subs',
        'topics': [
            {'topic': '/perf_bench/topic_0', 'num_pubs': 1, 'num_subs': 2},
        ],
    },
    {
        'name': '1pub_5sub',
        'description': '1 Pub -> 5 Subs',
        'topics': [
            {'topic': '/perf_bench/topic_0', 'num_pubs': 1, 'num_subs': 5},
        ],
    },
    {
        'name': '1pub_10sub',
        'description': '1 Pub -> 10 Subs',
        'topics': [
            {'topic': '/perf_bench/topic_0', 'num_pubs': 1, 'num_subs': 10},
        ],
    },
    {
        'name': '2topics_1pub_1sub',
        'description': '2x (1P->1S)',
        'topics': [
            {'topic': '/perf_bench/topic_0', 'num_pubs': 1, 'num_subs': 1},
            {'topic': '/perf_bench/topic_1', 'num_pubs': 1, 'num_subs': 1},
        ],
    },
    {
        'name': '2topics_1pub_5sub',
        'description': '2x (1P->5S)',
        'topics': [
            {'topic': '/perf_bench/topic_0', 'num_pubs': 1, 'num_subs': 5},
            {'topic': '/perf_bench/topic_1', 'num_pubs': 1, 'num_subs': 5},
        ],
    },
]

# ---------------------------------------------------------------------------
# Client-library combos
# ---------------------------------------------------------------------------

CLIENT_COMBOS = [
    {'name': 'cpp_cpp', 'label': 'cpp->cpp', 'pub': 'cpp', 'sub': 'cpp'},
    {'name': 'py_py',   'label': 'py->py',   'pub': 'py',  'sub': 'py'},
    {'name': 'cpp_py',  'label': 'cpp->py',  'pub': 'cpp', 'sub': 'py'},
    {'name': 'py_cpp',  'label': 'py->cpp',  'pub': 'py',  'sub': 'cpp'},
]

# ---------------------------------------------------------------------------
# Result data classes
# ---------------------------------------------------------------------------


@dataclass
class PubResult:
    topic: str
    pub_id: int
    total_sent: int
    duration_s: float
    msgs_per_sec: float


@dataclass
class SubResult:
    topic: str
    sub_id: int
    total_received: int
    total_expected: int
    dropped: int
    drop_rate_pct: float
    duration_s: float
    msgs_per_sec: float
    latency_min_us: float = 0.0
    latency_mean_us: float = 0.0
    latency_median_us: float = 0.0
    latency_max_us: float = 0.0
    first_latency_us: float = 0.0
    max_is_first: bool = False


@dataclass
class ScenarioResult:
    topology_name: str
    topology_desc: str
    client_combo: str
    client_label: str
    pub_results: List[PubResult] = field(default_factory=list)
    sub_results: List[SubResult] = field(default_factory=list)

    @property
    def avg_pub_rate(self) -> float:
        if not self.pub_results:
            return 0.0
        return sum(p.msgs_per_sec for p in self.pub_results) / len(self.pub_results)

    @property
    def avg_sub_rate(self) -> float:
        if not self.sub_results:
            return 0.0
        return sum(s.msgs_per_sec for s in self.sub_results) / len(self.sub_results)

    @property
    def min_sub_rate(self) -> float:
        return min((s.msgs_per_sec for s in self.sub_results), default=0.0)

    @property
    def max_sub_rate(self) -> float:
        return max((s.msgs_per_sec for s in self.sub_results), default=0.0)

    @property
    def avg_drop_pct(self) -> float:
        if not self.sub_results:
            return 0.0
        return sum(s.drop_rate_pct for s in self.sub_results) / len(self.sub_results)

    @property
    def latency_min(self) -> float:
        return min((s.latency_min_us for s in self.sub_results), default=0.0)

    @property
    def latency_mean(self) -> float:
        if not self.sub_results:
            return 0.0
        return sum(s.latency_mean_us for s in self.sub_results) / len(self.sub_results)

    @property
    def latency_median(self) -> float:
        if not self.sub_results:
            return 0.0
        return (
            sum(s.latency_median_us for s in self.sub_results)
            / len(self.sub_results)
        )

    @property
    def latency_max(self) -> float:
        return max((s.latency_max_us for s in self.sub_results), default=0.0)

    @property
    def max_is_first_flag(self) -> str:
        if not self.sub_results:
            return ''
        if all(s.max_is_first for s in self.sub_results):
            return 'Y'
        if any(s.max_is_first for s in self.sub_results):
            return '~'
        return 'N'


# ---------------------------------------------------------------------------
# Workspace / executable resolution
# ---------------------------------------------------------------------------


def find_workspace_root() -> Optional[Path]:
    """Walk up from the script directory looking for a colcon install dir."""
    start = Path(__file__).resolve().parent
    for p in [start] + list(start.parents):
        if (p / 'install' / PKG).is_dir():
            return p
        if (p / 'pixi.toml').exists() and (p / 'install').is_dir():
            return p
    return None


def resolve_exe(ws: Path, lang: str, role: str) -> List[str]:
    """Return the command list to launch an executable."""
    name = EXECUTABLES[lang][role]
    exe = ws / 'install' / PKG / 'lib' / PKG / name
    if not exe.exists():
        raise FileNotFoundError(
            f'Executable not found: {exe}\n'
            f'  Did you build the package?  pixi run build {PKG}'
        )
    if name.endswith('.py'):
        return [sys.executable, str(exe)]
    return [str(exe)]


# ---------------------------------------------------------------------------
# Output parsing
# ---------------------------------------------------------------------------


def _parse_kv(line: str) -> Optional[dict]:
    idx = line.find('[PERF_RESULT]')
    if idx < 0:
        return None
    tail = line[idx + len('[PERF_RESULT]'):].strip()
    kv: dict = {}
    for token in tail.split():
        if '=' in token:
            k, v = token.split('=', 1)
            kv[k] = v
    return kv


def parse_all_output(combined: str) -> Tuple[List[PubResult], List[SubResult]]:
    pubs: List[PubResult] = []
    subs: List[SubResult] = []
    for line in combined.splitlines():
        kv = _parse_kv(line)
        if kv is None:
            continue
        role = kv.get('role', '')
        if role == 'publisher':
            pubs.append(PubResult(
                topic=kv.get('topic', ''),
                pub_id=int(kv.get('pub_id', 0)),
                total_sent=int(kv.get('total_sent', 0)),
                duration_s=float(kv.get('duration_s', 0)),
                msgs_per_sec=float(kv.get('msgs_per_sec', 0)),
            ))
        elif role == 'subscriber':
            subs.append(SubResult(
                topic=kv.get('topic', ''),
                sub_id=int(kv.get('sub_id', 0)),
                total_received=int(kv.get('total_received', 0)),
                total_expected=int(kv.get('total_expected', 0)),
                dropped=int(kv.get('dropped', 0)),
                drop_rate_pct=float(kv.get('drop_rate_pct', 0)),
                duration_s=float(kv.get('duration_s', 0)),
                msgs_per_sec=float(kv.get('msgs_per_sec', 0)),
                latency_min_us=float(kv.get('latency_min_us', 0)),
                latency_mean_us=float(kv.get('latency_mean_us', 0)),
                latency_median_us=float(kv.get('latency_median_us', 0)),
                latency_max_us=float(kv.get('latency_max_us', 0)),
                first_latency_us=float(kv.get('first_latency_us', 0)),
                max_is_first=kv.get('max_is_first', 'false') == 'true',
            ))
    return pubs, subs


# ---------------------------------------------------------------------------
# Process management
# ---------------------------------------------------------------------------


def _terminate(proc: subprocess.Popen, timeout: float = 5.0):
    if proc.poll() is not None:
        return
    try:
        proc.send_signal(signal.SIGINT)
        proc.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait()


def run_scenario(
    ws: Path,
    topology: dict,
    combo: dict,
    rate_hz: int,
    duration_sec: float,
    msg_size: int,
    domain_id: int,
    reliable: bool,
    qos_depth: int = 1,
) -> ScenarioResult:
    env = os.environ.copy()
    env['ROS_DOMAIN_ID'] = str(domain_id)

    pub_cmd_prefix = resolve_exe(ws, combo['pub'], 'pub')
    sub_cmd_prefix = resolve_exe(ws, combo['sub'], 'sub')

    processes: List[Tuple[str, int, subprocess.Popen]] = []
    result = ScenarioResult(
        topology_name=topology['name'],
        topology_desc=topology['description'],
        client_combo=combo['name'],
        client_label=combo['label'],
    )

    pub_id_counter = 0
    sub_id_counter = 0

    for topic_cfg in topology['topics']:
        topic = topic_cfg['topic']
        for _ in range(topic_cfg['num_subs']):
            sid = sub_id_counter
            sub_id_counter += 1
            cmd = sub_cmd_prefix + [
                '--ros-args',
                '-r', f'__node:=perf_sub_{sid}',
                '-p', f'topic_name:={topic}',
                '-p', f'sub_id:={sid}',
                '-p', 'timeout_sec:=3.0',
                '-p', f'max_duration_sec:={duration_sec + 10.0}',
                '-p', f'reliable:={"true" if reliable else "false"}',
                '-p', f'qos_depth:={qos_depth}',
            ]
            proc = subprocess.Popen(
                cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, env=env,
            )
            processes.append(('sub', sid, proc))

    time.sleep(1.0)

    for topic_cfg in topology['topics']:
        topic = topic_cfg['topic']
        for _ in range(topic_cfg['num_pubs']):
            pid = pub_id_counter
            pub_id_counter += 1
            cmd = pub_cmd_prefix + [
                '--ros-args',
                '-r', f'__node:=perf_pub_{pid}',
                '-p', f'topic_name:={topic}',
                '-p', f'pub_id:={pid}',
                '-p', f'rate_hz:={rate_hz}',
                '-p', f'duration_sec:={duration_sec}',
                '-p', f'msg_size:={msg_size}',
                '-p', 'warmup_sec:=2.0',
                '-p', f'reliable:={"true" if reliable else "false"}',
                '-p', f'qos_depth:={qos_depth}',
            ]
            proc = subprocess.Popen(
                cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, env=env,
            )
            processes.append(('pub', pid, proc))

    deadline = time.time() + duration_sec + 20.0
    for _, _, proc in processes:
        remaining = max(1.0, deadline - time.time())
        try:
            proc.wait(timeout=remaining)
        except subprocess.TimeoutExpired:
            _terminate(proc)

    parts: List[str] = []
    for _, _, proc in processes:
        stdout_bytes = proc.stdout.read() if proc.stdout else b''
        stderr_bytes = proc.stderr.read() if proc.stderr else b''
        parts.append(stdout_bytes.decode('utf-8', errors='replace'))
        parts.append(stderr_bytes.decode('utf-8', errors='replace'))

    pub_results, sub_results = parse_all_output('\n'.join(parts))
    result.pub_results = pub_results
    result.sub_results = sub_results
    return result


# ---------------------------------------------------------------------------
# Formatting
# ---------------------------------------------------------------------------

CT = 16   # topology
CC = 10   # client combo
CN = 12   # numeric rate columns
CP = 8    # pct column
CL = 12   # latency columns (us)
CF = 4    # max_is_first flag

_COLS = [CT, CC, CN, CN, CP, CL, CL, CL, CL, CF]


def _sep() -> str:
    return '+' + '+'.join('-' * (w + 2) for w in _COLS) + '+'


def format_summary_table(results: List[ScenarioResult]) -> str:
    lines = [_sep()]
    header = (
        f'| {"Topology":<{CT}} '
        f'| {"Clients":<{CC}} '
        f'| {"Pub msg/s":>{CN}} '
        f'| {"Sub msg/s":>{CN}} '
        f'| {"Drop %":>{CP}} '
        f'| {"Lat min us":>{CL}} '
        f'| {"Lat med us":>{CL}} '
        f'| {"Lat mean us":>{CL}} '
        f'| {"Lat max us":>{CL}} '
        f'| {"1st?":>{CF}} |'
    )
    lines.append(header)
    lines.append(_sep())

    prev_topo = None
    for r in results:
        if prev_topo is not None and r.topology_name != prev_topo:
            lines.append(_sep())
        prev_topo = r.topology_name

        row = (
            f'| {r.topology_desc:<{CT}} '
            f'| {r.client_label:<{CC}} '
            f'| {r.avg_pub_rate:>{CN}.1f} '
            f'| {r.avg_sub_rate:>{CN}.1f} '
            f'| {r.avg_drop_pct:>{CP}.2f} '
            f'| {r.latency_min:>{CL}.1f} '
            f'| {r.latency_median:>{CL}.1f} '
            f'| {r.latency_mean:>{CL}.1f} '
            f'| {r.latency_max:>{CL}.1f} '
            f'| {r.max_is_first_flag:>{CF}} |'
        )
        lines.append(row)

    lines.append(_sep())
    return '\n'.join(lines)


def format_detailed(results: List[ScenarioResult]) -> str:
    lines: List[str] = []
    for r in results:
        lines.append('')
        lines.append('=' * 72)
        lines.append(
            f'  {r.topology_desc}  [{r.client_label}]'
            f'  ({r.topology_name}/{r.client_combo})'
        )
        lines.append('=' * 72)

        if r.pub_results:
            lines.append('  Publishers:')
            for p in r.pub_results:
                lines.append(
                    f'    [{p.pub_id}] {p.topic}  '
                    f'sent={p.total_sent}  dur={p.duration_s:.3f}s  '
                    f'rate={p.msgs_per_sec:.1f} msg/s'
                )
        if r.sub_results:
            lines.append('  Subscribers:')
            for s in r.sub_results:
                first_tag = ' [max=1st]' if s.max_is_first else ''
                lines.append(
                    f'    [{s.sub_id}] {s.topic}  '
                    f'recv={s.total_received}/{s.total_expected}  '
                    f'drop={s.dropped} ({s.drop_rate_pct:.2f}%)  '
                    f'dur={s.duration_s:.3f}s  rate={s.msgs_per_sec:.1f} msg/s  '
                    f'lat={s.latency_min_us:.0f}/{s.latency_median_us:.0f}/'
                    f'{s.latency_mean_us:.0f}/{s.latency_max_us:.0f} us '
                    f'(min/med/mean/max)  '
                    f'1st={s.first_latency_us:.0f} us{first_tag}'
                )
    return '\n'.join(lines)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def build_scenario_list(topologies, combos):
    """Return (topology, combo) pairs in topology-major order."""
    pairs = []
    for topo in topologies:
        for combo in combos:
            pairs.append((topo, combo))
    return pairs


def main():
    topo_names = [t['name'] for t in TOPOLOGIES]
    combo_names = [c['name'] for c in CLIENT_COMBOS]

    parser = argparse.ArgumentParser(
        description='ROS2 Pub/Sub Performance Benchmark',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            f'Available topologies : {topo_names}\n'
            f'Available client combos: {combo_names}\n'
        ),
    )
    parser.add_argument(
        '--rate', type=int, default=5000,
        help='Target publish rate in Hz (default: 5000)',
    )
    parser.add_argument(
        '--duration', type=float, default=10.0,
        help='Measurement duration per scenario in seconds (default: 10.0)',
    )
    parser.add_argument(
        '--msg-size', type=int, default=256,
        help='Approximate payload size in bytes (default: 256)',
    )
    parser.add_argument(
        '--domain-id', type=int, default=42,
        help='Starting ROS_DOMAIN_ID; incremented per scenario (default: 42)',
    )
    parser.add_argument(
        '--reliable', action='store_true',
        help='Use RELIABLE QoS instead of BEST_EFFORT',
    )
    parser.add_argument(
        '--qos-depth', type=int, default=1,
        help='QoS KEEP_LAST history depth (default: 1)',
    )
    parser.add_argument(
        '--output', type=str, default='/tmp/perf_benchmark_results.txt',
        help='Path to write results file (default: /tmp/perf_benchmark_results.txt)',
    )
    parser.add_argument(
        '--topologies', type=str, nargs='*', default=None,
        help='Run only these topologies (default: all)',
    )
    parser.add_argument(
        '--clients', type=str, nargs='*', default=None,
        help='Run only these client combos (default: all). '
             'Use all or list names like cpp_cpp py_py cpp_py py_cpp',
    )
    parser.add_argument(
        '--workspace', type=str, default=None,
        help='Colcon workspace root (default: auto-detect from script location)',
    )
    args = parser.parse_args()

    # ------------------------------------------------------------------
    # Locate workspace and validate executables
    # ------------------------------------------------------------------
    if args.workspace:
        ws = Path(args.workspace).resolve()
    else:
        ws = find_workspace_root()

    if ws is None or not (ws / 'install' / PKG).is_dir():
        print(
            'Error: cannot find the colcon install directory for '
            f"'{PKG}'.\n"
            'Make sure the package is built and either:\n'
            '  1. Run this script from inside the workspace tree, or\n'
            '  2. Pass --workspace /path/to/workspace\n\n'
            'Quick-start:\n'
            f'  pixi run build {PKG}\n'
            '  pixi shell\n'
            '  source install/setup.bash\n'
            f'  python3 {sys.argv[0]}',
            file=sys.stderr,
        )
        sys.exit(1)

    print(f'Workspace: {ws}')

    # ------------------------------------------------------------------
    # Resolve selections
    # ------------------------------------------------------------------
    topos = TOPOLOGIES
    if args.topologies:
        topos = [t for t in TOPOLOGIES if t['name'] in args.topologies]
        if not topos:
            print(
                f'Error: no matching topologies. Available: {topo_names}',
                file=sys.stderr,
            )
            sys.exit(1)

    combos = CLIENT_COMBOS
    if args.clients and 'all' not in args.clients:
        combos = [c for c in CLIENT_COMBOS if c['name'] in args.clients]
        if not combos:
            print(
                f'Error: no matching client combos. Available: {combo_names}',
                file=sys.stderr,
            )
            sys.exit(1)

    scenario_list = build_scenario_list(topos, combos)
    total = len(scenario_list)

    print(
        '\n'
        '========================================================================\n'
        '  ROS2 Pub/Sub Performance Benchmark\n'
        '========================================================================\n'
        f'  Publish rate : {args.rate} Hz\n'
        f'  Duration     : {args.duration} s per scenario\n'
        f'  Msg size     : {args.msg_size} bytes\n'
        f'  QoS          : {"RELIABLE" if args.reliable else "BEST_EFFORT"} '
        f'depth={args.qos_depth}\n'
        f'  Topologies   : {[t["name"] for t in topos]}\n'
        f'  Clients      : {[c["name"] for c in combos]}\n'
        f'  Total runs   : {total}\n'
        f'  Output file  : {args.output}\n'
        '========================================================================'
    )

    all_results: List[ScenarioResult] = []

    for i, (topo, combo) in enumerate(scenario_list):
        domain_id = args.domain_id + i
        tag = f'{topo["description"]} [{combo["label"]}]'
        print(f'\n--- [{i + 1}/{total}] {tag}  (domain_id={domain_id}) ---')

        try:
            result = run_scenario(
                ws=ws,
                topology=topo,
                combo=combo,
                rate_hz=args.rate,
                duration_sec=args.duration,
                msg_size=args.msg_size,
                domain_id=domain_id,
                reliable=args.reliable,
                qos_depth=args.qos_depth,
            )
        except FileNotFoundError as exc:
            print(f'  SKIP: {exc}', file=sys.stderr)
            continue

        all_results.append(result)

        for p in result.pub_results:
            print(
                f'  PUB[{p.pub_id}] {p.topic}: '
                f'{p.total_sent} msgs, {p.msgs_per_sec:.1f} msg/s'
            )
        for s in result.sub_results:
            first_tag = ' [max=1st]' if s.max_is_first else ''
            print(
                f'  SUB[{s.sub_id}] {s.topic}: '
                f'{s.total_received}/{s.total_expected} msgs, '
                f'{s.msgs_per_sec:.1f} msg/s, drop={s.drop_rate_pct:.2f}%, '
                f'lat={s.latency_min_us:.0f}/{s.latency_median_us:.0f}/'
                f'{s.latency_mean_us:.0f}/{s.latency_max_us:.0f} us '
                f'(min/med/mean/max){first_tag}'
            )

        if i < total - 1:
            print('  (cooldown 3 s)')
            time.sleep(3.0)

    # Final report
    if not all_results:
        print('\nNo results collected.', file=sys.stderr)
        sys.exit(1)

    table = format_summary_table(all_results)
    detailed = format_detailed(all_results)

    print(
        '\n'
        '========================================================================\n'
        '  RESULTS SUMMARY\n'
        '========================================================================'
    )
    print(table)
    print(detailed)

    # Persist
    timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    with open(args.output, 'w') as fout:
        fout.write('ROS2 Pub/Sub Performance Benchmark Results\n')
        fout.write(f'Date: {timestamp}\n')
        fout.write(
            f'Rate: {args.rate} Hz | Duration: {args.duration}s | '
            f'Msg Size: {args.msg_size} bytes | '
            f'QoS: {"RELIABLE" if args.reliable else "BEST_EFFORT"} '
            f'depth={args.qos_depth}\n\n'
        )
        fout.write(table + '\n')
        fout.write(detailed + '\n')

    print(f'\nResults saved to: {args.output}')


if __name__ == '__main__':
    main()
