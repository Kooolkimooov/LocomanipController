"""Sweep the simulated cart mass on HRP5P, leaving the controller's model at 10 kg.

Like testing_harness.py, but: only the MuJoCo mass changes (the URDF and the
reference force stay nominal, so the mass is unknown to the controller), the
masses are given rather than log-spaced, and a run that never finishes -- a
fallen robot leaves the FSM stuck -- is killed instead of hanging.
"""

import argparse
import collections
import os
import shutil
import subprocess
import time
import xml.etree.ElementTree as ET

import yaml

from testing_harness import set_reference_force, set_urdf_mass

BASE_DIR: str = (
    "/home/martin/workspace/src/catkin_locomanip_ws/src/LocomanipController/"
)
MUJOCO_CART_FILE: str = "mujoco/model/Cart.xml"
LOGS_DIR: str = "test_logs_hrp5p/"
LOG_FILE: str = "/tmp/mc-control-LocomanipController-latest.bin"
CONTROLLER_CONFIG: str = "etc/LocomanipController.in.yaml"

#: How long to keep watching after the push ends. LMC::Pause_ now holds for an
#: hour so an mjlab episode can outlive the cycle, so the window that used to be
#: its duration -- the robot's chance to topple once it lets go -- is this.
PAUSE_SETTLE_S: float = 10.0

#: Push-phase hand-wrench settings. Local y is the push axis on both hands (the
#: frames are mirrored, so it maps to world +x on the right and -x on the left);
#: local x is the vertical press. The shipped projection therefore zeroes the
#: push component of the reference and keeps only press and lateral.
ARMS: dict[str, dict] = {
    "shipped": {"enable": True, "alpha": 0.02},
    "off": {"enable": False},
    "slow": {"enable": True, "alpha": 0.005},
    "jvrc-tau": {"enable": True, "alpha": 0.008},
    "full-a0001": {
        "enable": True,
        "alpha": 0.001,
        "forceProjection": [1.0, 1.0, 1.0],
        "momentProjection": [0.0, 0.0, 0.0],
    },
    "full-a001": {
        "enable": True,
        "alpha": 0.01,
        "forceProjection": [1.0, 1.0, 1.0],
        "momentProjection": [0.0, 0.0, 0.0],
    },
    "full-a003": {
        "enable": True,
        "alpha": 0.03,
        "forceProjection": [1.0, 1.0, 1.0],
        "momentProjection": [0.0, 0.0, 0.0],
    },
    "full-a01": {
        "enable": True,
        "alpha": 0.1,
        "forceProjection": [1.0, 1.0, 1.0],
        "momentProjection": [0.0, 0.0, 0.0],
    },
    "xy": {
        "enable": True,
        "alpha": 0.003,
        "forceProjection": [1.0, 1.0, 0.0],
        "momentProjection": [0.0, 0.0, 0.0],
    },
    "yz": {
        "enable": True,
        "alpha": 0.003,
        "forceProjection": [0.0, 1.0, 1.0],
        "momentProjection": [0.0, 0.0, 0.0],
    },
    "world-xy": {
        "enable": True,
        "alpha": 0.003,
        "worldProjection": True,
        "forceProjection": [1.0, 1.0, 0.0],
        "momentProjection": [0.0, 0.0, 0.0],
    },
    "world-x": {
        "enable": True,
        "alpha": 0.003,
        "worldProjection": True,
        "forceProjection": [1.0, 0.0, 0.0],
        "momentProjection": [0.0, 0.0, 0.0],
    },
    "pushz-a001": {
        "enable": True,
        "alpha": 0.01,
        "forceProjection": [0.0, 0.0, 1.0],
        "momentProjection": [0.0, 0.0, 0.0],
    },
    "pushz-a003": {
        "enable": True,
        "alpha": 0.03,
        "forceProjection": [0.0, 0.0, 1.0],
        "momentProjection": [0.0, 0.0, 0.0],
    },
    "shipped-a001": {
        "enable": True,
        "alpha": 0.01,
        "forceProjection": [1.0, 0.0, 1.0],
        "momentProjection": [0.0, 1.0, 0.0],
    },
    "shipped-a003": {
        "enable": True,
        "alpha": 0.03,
        "forceProjection": [1.0, 0.0, 1.0],
        "momentProjection": [0.0, 1.0, 0.0],
    },
    "a0.005": {
        "enable": True,
        "alpha": 0.005,
        "forceProjection": [1.0, 1.0, 1.0],
        "momentProjection": [0.0, 0.0, 0.0],
    },
    "a0.0075": {
        "enable": True,
        "alpha": 0.0075,
        "forceProjection": [1.0, 1.0, 1.0],
        "momentProjection": [0.0, 0.0, 0.0],
    },
    "a0.015": {
        "enable": True,
        "alpha": 0.015,
        "forceProjection": [1.0, 1.0, 1.0],
        "momentProjection": [0.0, 0.0, 0.0],
    },
    "a0.02": {
        "enable": True,
        "alpha": 0.02,
        "forceProjection": [1.0, 1.0, 1.0],
        "momentProjection": [0.0, 0.0, 0.0],
    },
    "a0.05": {
        "enable": True,
        "alpha": 0.05,
        "forceProjection": [1.0, 1.0, 1.0],
        "momentProjection": [0.0, 0.0, 0.0],
    },
    "a0.01": {
        "enable": True,
        "alpha": 0.01,
        "forceProjection": [1.0, 1.0, 1.0],
        "momentProjection": [0.0, 0.0, 0.0],
    },
    "a0.03": {
        "enable": True,
        "alpha": 0.03,
        "forceProjection": [1.0, 1.0, 1.0],
        "momentProjection": [0.0, 0.0, 0.0],
    },
    "a0.04": {
        "enable": True,
        "alpha": 0.04,
        "forceProjection": [1.0, 1.0, 1.0],
        "momentProjection": [0.0, 0.0, 0.0],
    },
    "a0.07": {
        "enable": True,
        "alpha": 0.07,
        "forceProjection": [1.0, 1.0, 1.0],
        "momentProjection": [0.0, 0.0, 0.0],
    },
    "a0.06": {
        "enable": True,
        "alpha": 0.06,
        "forceProjection": [1.0, 1.0, 1.0],
        "momentProjection": [0.0, 0.0, 0.0],
    },
    "a0.085": {
        "enable": True,
        "alpha": 0.085,
        "forceProjection": [1.0, 1.0, 1.0],
        "momentProjection": [0.0, 0.0, 0.0],
    },
    "full": {
        "enable": True,
        "alpha": 0.003,
        "forceProjection": [1.0, 1.0, 1.0],
        "momentProjection": [0.0, 0.0, 0.0],
    },
    "full-lat": {
        "enable": True,
        "alpha": 0.003,
        "forceProjection": [1.0, 0.0, 1.0],
        "momentProjection": [0.0, 0.0, 0.0],
    },
    "shared-full": {
        "enable": True,
        "alpha": 0.003,
        "shared": True,
        "forceProjection": [1.0, 1.0, 1.0],
        "momentProjection": [0.0, 0.0, 0.0],
    },
    "shared": {
        "enable": True,
        "alpha": 0.003,
        "shared": True,
        "forceProjection": [0.0, 0.0, 1.0],
        "momentProjection": [0.0, 0.0, 0.0],
    },
    "shared-a02": {
        "enable": True,
        "alpha": 0.02,
        "shared": True,
        "forceProjection": [0.0, 0.0, 1.0],
        "momentProjection": [0.0, 0.0, 0.0],
    },
    "push-a003": {
        "enable": True,
        "alpha": 0.003,
        "forceProjection": [0.0, 0.0, 1.0],
        "momentProjection": [0.0, 0.0, 0.0],
    },
    "push-a02": {
        "enable": True,
        "alpha": 0.02,
        "forceProjection": [0.0, 0.0, 1.0],
        "momentProjection": [0.0, 0.0, 0.0],
    },
    "push-a05": {
        "enable": True,
        "alpha": 0.05,
        "forceProjection": [0.0, 0.0, 1.0],
        "momentProjection": [0.0, 0.0, 0.0],
    },
    "tuned-y": {
        "enable": True,
        "alpha": 0.008,
        "forceProjection": [0.0, 1.0, 0.0],
        "momentProjection": [0.0, 0.0, 0.0],
    },
    "tuned": {
        "enable": True,
        "alpha": 0.008,
        "forceProjection": [0.0, 0.0, 1.0],
        "momentProjection": [0.0, 0.0, 0.0],
    },
    "fast": {"enable": True, "alpha": 0.05},
    "no-moment": {
        "enable": True,
        "alpha": 0.02,
        "forceProjection": [1.0, 0.0, 1.0],
        "momentProjection": [0.0, 0.0, 0.0],
    },
    "measure-push": {
        "enable": True,
        "alpha": 0.02,
        "forceProjection": [0.0, 1.0, 0.0],
        "momentProjection": [0.0, 0.0, 0.0],
    },
    "press-axis": {
        "enable": True,
        "alpha": 0.02,
        "forceProjection": [1.0, 0.0, 0.0],
        "momentProjection": [0.0, 0.0, 0.0],
    },
    "push-axis-slow": {
        "enable": True,
        "alpha": 0.005,
        "forceProjection": [1.0, 0.0, 0.0],
        "momentProjection": [0.0, 0.0, 0.0],
    },
    "assist": {"enable": True, "mode": "assist", "gain": 400.0, "maxForce": 100.0},
    "assist-strong": {"enable": True, "mode": "assist", "gain": 1000.0, "maxForce": 300.0},
    "assist-max": {"enable": True, "mode": "assist", "gain": 2000.0, "maxForce": 600.0},
    "push-and-vertical": {
        "enable": True,
        "alpha": 0.02,
        "forceProjection": [1.0, 0.0, 1.0],
        "momentProjection": [0.0, 0.0, 0.0],
    },
}


def set_mujoco_mass(mass: float) -> None:
    tree = ET.parse(BASE_DIR + MUJOCO_CART_FILE)
    root = tree.getroot()
    leaf = root.find("worldbody/body[@name='Body']/inertial")
    leaf.set("mass", str(mass))
    tree.write(BASE_DIR + MUJOCO_CART_FILE)


def set_push_reference(mass: float, sign: float) -> None:
    """Write the feed-forward push on the hand-frame push axis, mirrored per hand."""
    path = BASE_DIR + CONTROLLER_CONFIG
    with open(path) as handle:
        config = yaml.safe_load(handle)

    # 10 * m * friction / 2 is the original harness's magnitude: the force needed to
    # slide the cart, split between the hands. Local z is the push axis and points
    # the same way in world on both hands, so the sign is shared, not mirrored.
    per_hand = 0.1 * mass
    wrenches = config["states"]["LMC::PushCart_"]["configs"]["preHandWrenches"]
    wrenches["Left"]["force"] = [0.0, 0.0, -sign * per_hand]
    wrenches["Right"]["force"] = [0.0, 0.0, -sign * per_hand]

    with open(path, "w") as handle:
        yaml.safe_dump(config, handle, sort_keys=False)


def set_adaptation(arm: str) -> None:
    """Write the push state's blend settings; the build installs them."""
    path = BASE_DIR + CONTROLLER_CONFIG
    with open(path) as handle:
        config = yaml.safe_load(handle)
    state = config["states"]["LMC::PushCart_"].setdefault("configs", {})
    state["HandWrenchAdaptation"] = ARMS[arm]
    with open(path, "w") as handle:
        yaml.safe_dump(config, handle, sort_keys=False)


def run_test(timeout: float) -> str:
    """Build, install and simulate; stop on whichever verdict the FSM reaches.

    JVRC1 walks the FSM to Exit even after falling, which is why
    testing_harness.py needs no cap. HRP5P does not: a lost robot ends in
    "QP failed to run()" inside the push state and the executor never advances,
    so mc_mujoco would simulate forever. Watching for either marker ends a
    decided run immediately instead of waiting out a clock.
    """
    process = subprocess.Popen(
        BASE_DIR + "build_and_run",
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )
    verdict = "timeout"
    deadline = time.monotonic() + timeout
    tail: collections.deque[str] = collections.deque(maxlen=15)
    pushed = False
    settled: float | None = None
    assert process.stdout is not None
    for line in process.stdout:
        tail.append(line.rstrip())
        # The robot often topples once it lets go of the handle, which says nothing
        # about whether it could push the load; score that as a completed push.
        if "Starting state LMC::Pause_" in line:
            pushed = True
            settled = time.monotonic() + PAUSE_SETTLE_S
        if "QP failed to run()" in line:
            verdict = "pushed" if pushed else "qp_failed"
            break
        # LMC::Pause_ holds rather than reaching LMC::Exit_, so the settle window
        # is what says the robot survived the release.
        if settled is not None and time.monotonic() > settled:
            verdict = "completed"
            break
        if time.monotonic() > deadline:
            break
    if process.poll() is None:
        process.kill()
        process.wait()
    subprocess.run(["pkill", "-x", "mc_mujoco"], check=False)
    # The next run cannot bind ipc:///tmp/mc_rtc_pub.ipc until this one is reaped,
    # and it aborts rather than waiting.
    while subprocess.run(["pgrep", "-x", "mc_mujoco"], capture_output=True).returncode == 0:
        time.sleep(0.5)
    # An undecided run is a broken one (build error, failed startup); its last
    # lines are the only record of why, since the rest is discarded.
    if verdict == "timeout":
        print("\n".join(tail), flush=True)
    return verdict


def save_log(mass: float, verdict: str) -> str | None:
    if not os.path.exists(LOG_FILE):
        return None
    target = BASE_DIR + LOGS_DIR + f"{mass:.1f}_{verdict}.bin"
    shutil.move(os.path.realpath(LOG_FILE), target)
    return target


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--masses", default="10,30,100,300,600,1000")
    parser.add_argument("--arms", default="shipped")
    parser.add_argument("--push-force", type=float, default=0.0,
                        help="feed-forward push on the hand-frame push axis; +1 or -1")
    parser.add_argument(
        "--hand-force",
        action="store_true",
        help="write preHandWrenches from the true mass (10 * m * friction / 2 per "
        "hand), the feed-forward push an exact mass estimate would give",
    )
    parser.add_argument(
        "--urdf-mass",
        default="true",
        help="'true' keeps the controller's cart model on the simulated mass; a "
        "number pins it, leaving the load unknown to the controller",
    )
    parser.add_argument("--timeout", type=float, default=300.0)
    args = parser.parse_args()

    os.makedirs(BASE_DIR + LOGS_DIR, exist_ok=True)
    for arm in (a.strip() for a in args.arms.split(",")):
      set_adaptation(arm)
      for mass in (float(m) for m in args.masses.split(",")):
        set_mujoco_mass(mass)
        set_urdf_mass(mass if args.urdf_mass == "true" else float(args.urdf_mass))
        # Written per mass, and zeroed otherwise: the value persists in the yaml.
        set_reference_force(mass if args.hand_force else 0.0)
        if args.push_force:
            set_push_reference(mass, args.push_force)
        start = time.monotonic()
        verdict = run_test(args.timeout)
        tag = arm if args.urdf_mass == "true" else f"{arm}-blind"
        tag = f"{tag}-ff" if args.hand_force else tag
        tag = f"{tag}-push{args.push_force:+.0f}" if args.push_force else tag
        log = save_log(mass, f"{tag}_{verdict}")
        print(
            f"arm={tag:16} mass={mass:7.1f} verdict={verdict:9} "
            f"wall={time.monotonic() - start:6.1f}s log={log}",
            flush=True,
        )


if __name__ == "__main__":
    main()
