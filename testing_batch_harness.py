from testing_harness import compute_masses, LOGS_DIR, BASE_DIR
import os
import subprocess
import shutil


def main():
    masses = compute_masses()

    for mass in masses:
        save_path = BASE_DIR + LOGS_DIR[:-1] + f"_mass={mass}/"
        if os.path.exists(save_path) and not input(
            f"about to skip {
                mass:.5f} because dir exists (enter to continue), any key to recompute"
        ):
            continue
        subprocess.run(["python3", f"{BASE_DIR}testing_harness.py", "-m", f"{mass}"])
        os.rename(
            BASE_DIR + LOGS_DIR,
            save_path,
        )


if __name__ == "__main__":
    main()
