import os
import subprocess
import shutil
import xml.etree.ElementTree
import yaml
import argparse

MIN_MASS: float = 1.0
MAX_MASS: float = 1000.0

MASS_STEPS: int = 50
MASS_DISCRETIZATION_SPACE: str = "log"

RUNS_REPEAT: int = 1

BASE_DIR: str = (
    "/home/martin/workspace/src/catkin_locomanip_ws/src/LocomanipController/"
)
MUJOCO_CART_FILE: str = "mujoco/model/Cart.xml"
URDF_CART_FILE: str = "description/urdf/Cart.urdf"
CONTROLLER_CONFIG_FILE: str = "etc/LocomanipController.in.yaml"

LOGS_DIR = "test_logs/"


def compute_masses() -> list[float]:
    match MASS_DISCRETIZATION_SPACE:
        case "log":
            masses = [
                MIN_MASS * (MAX_MASS / MIN_MASS) ** (i / (MASS_STEPS - 1))
                for i in range(MASS_STEPS)
            ]
        case "linear":
            masses = [
                MIN_MASS + (MAX_MASS - MIN_MASS) * (i / (MASS_STEPS - 1))
                for i in range(MASS_STEPS)
            ]
        case _:
            raise ValueError(
                f"Invalid MASS_DISCRETIZATION_SPACE: {MASS_DISCRETIZATION_SPACE}"
            )
    return masses


def set_mujoco_mass(mass: float) -> None:
    tree = xml.etree.ElementTree.parse(BASE_DIR + MUJOCO_CART_FILE)
    root = tree.getroot()
    leaf = root.find("worldbody/body[@name='Body']/inertial")
    leaf.set("mass", str(mass))
    tree.write(BASE_DIR + MUJOCO_CART_FILE)


def set_urdf_mass(mass: float) -> None:
    tree = xml.etree.ElementTree.parse(BASE_DIR + URDF_CART_FILE)
    root = tree.getroot()
    leaf = root.find("link[@name='Body']/inertial/mass")
    leaf.set("value", str(mass))
    tree.write(BASE_DIR + URDF_CART_FILE)


def set_reference_force(mass: float) -> None:
    with open(BASE_DIR + CONTROLLER_CONFIG_FILE, "r") as config:
        config_yaml = yaml.safe_load(config)

    tree = xml.etree.ElementTree.parse(BASE_DIR + MUJOCO_CART_FILE)
    root = tree.getroot()
    leaf = root.find("worldbody/body[@name='Body']/geom")
    friction_coefficient = float(leaf.get("friction").split()[0])

    config_yaml["states"]["LMC::PushCart_"]["configs"]["preHandWrenches"]["Left"][
        "force"
    ][0] = -10 * mass * friction_coefficient / 2
    config_yaml["states"]["LMC::PushCart_"]["configs"]["preHandWrenches"]["Right"][
        "force"
    ][0] = 10 * mass * friction_coefficient / 2

    with open(BASE_DIR + CONTROLLER_CONFIG_FILE, "w") as config:
        yaml.dump(config_yaml, config)


def set_mass(mass: float) -> None:
    set_mujoco_mass(mass)
    set_urdf_mass(mass)


def run_test() -> None:
    subprocess.run(BASE_DIR + "build_and_run")


def save_log(mass: float, run_index: int) -> None:
    log_file = os.path.realpath("/tmp/mc-control-LocomanipController-latest.bin")
    shutil.move(log_file, BASE_DIR + LOGS_DIR + f"{mass:.1f}_{run_index}.bin")


def main(ref_mass=None) -> None:
    write_logs: bool = True

    logs_dir_children = list(os.walk(BASE_DIR + LOGS_DIR))
    if len(logs_dir_children) > 0:
        print(logs_dir_children)
        answer: str = input(
            f"Logs directory {
                BASE_DIR + LOGS_DIR
            } is not empty. Do you want to continue and overwrite existing logs? (y/n): "
        )
        if answer.lower() != "y":
            write_logs = False
        elif answer.lower() == "y":
            for _, _, logs_files in logs_dir_children:
                for file in logs_files:
                    os.remove(BASE_DIR + LOGS_DIR + file)

    if not os.path.exists(BASE_DIR + LOGS_DIR):
        os.mkdir(BASE_DIR + LOGS_DIR)

    masses = compute_masses()

    for mass in masses:
        set_mass(mass)
        set_reference_force(ref_mass if ref_mass is not None else mass)
        for i in range(RUNS_REPEAT):
            run_test()
            if write_logs:
                save_log(mass, i)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ref-mass", "-m", type=float)
    args = parser.parse_args()

    main(args.ref_mass)
