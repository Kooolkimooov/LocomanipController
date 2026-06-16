import os
import shutil
import xml.etree.ElementTree

MIN_MASS: float = 100.0
MAX_MASS: float = 1000.0

MASS_STEPS: int = 50
MASS_DISCRETIZATION_SPACE: str = "log"

RUNS_REPEAT: int = 1

BASE_DIR: str = "/home/martin/workspace/src/catkin_locomanip_ws/src/LocomanipController/"
MUJOCO_CART_FILE: str = "mujoco/model/Cart.xml"
URDF_CART_FILE: str = "description/urdf/Cart.urdf"

LOGS_DIR = "/home/martin/workspace/src/catkin_locomanip_ws/test_logs/"


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
                f"Invalid MASS_DISCRETIZATION_SPACE: {
                    MASS_DISCRETIZATION_SPACE}"
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


def set_mass(mass: float) -> None:
    set_mujoco_mass(mass)
    set_urdf_mass(mass)


def run_test() -> None:
    os.system("./build_and_run")


def save_log(mass: float, run_index: int) -> None:
    log_file = os.path.realpath(
        "/tmp/mc-control-LocomanipController-latest.bin")
    shutil.copy(log_file, LOGS_DIR + f"{mass:.1f}_{run_index}.bin")


def main() -> None:
    logs_dir_children = list(os.walk(LOGS_DIR))
    write_logs: bool = True
    if os.path.exists(LOGS_DIR) and len(logs_dir_children) > 0:
        answer: str = input(
            f"Logs directory {
                LOGS_DIR} is not empty. Do you want to continue and overwrite existing logs? (y/n): "
        )
        if answer.lower() != "y":
            write_logs = False
        elif answer.lower() == "y":
            for _, _, logs_files in logs_dir_children:
                for file in logs_files:
                    os.remove(LOGS_DIR + file)

    masses = compute_masses()

    for mass in masses:
        set_mass(mass)
        for i in range(RUNS_REPEAT):
            run_test()
            if write_logs:
                save_log(mass, i)


if __name__ == "__main__":
    main()
