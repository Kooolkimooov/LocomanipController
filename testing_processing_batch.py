import mc_log_ui
import numpy
from testing_harness import compute_masses, RUNS_REPEAT, LOGS_DIR, BASE_DIR
import pandas
import matplotlib.pyplot as plt

TIME = "t"
ZMP_MIN_X = "CentroidalManager_ZMP_SupportRegion_min_x"
ZMP_MAX_X = "CentroidalManager_ZMP_SupportRegion_max_x"
ZMP_MIN_Y = "CentroidalManager_ZMP_SupportRegion_min_y"
ZMP_MAX_Y = "CentroidalManager_ZMP_SupportRegion_max_y"
ZMP_MEA_X = "CentroidalManager_ZMP_measured_x"
ZMP_MEA_Y = "CentroidalManager_ZMP_measured_y"
ZMP_PLA_X = "CentroidalManager_ZMP_planned_x"
ZMP_PLA_Y = "CentroidalManager_ZMP_planned_y"

CART_POS_X = "obj_FloatingBase_position_x"


def main() -> None:
    import glob
    import os

    masses = compute_masses()

    log_dirs = glob.glob(BASE_DIR + "test_logs_mass=*")
    all_data = []

    for log_dir in log_dirs:
        basename = os.path.basename(log_dir)
        suffix = basename[len("test_logs_mass=") :]
        try:
            expected_mass = float(suffix.replace("_", "."))
        except ValueError:
            continue

        for mass in masses:
            zmp_errors_y = []
            zmp_errors_x = []
            zmp_stds_x = []
            zmp_stds_y = []

            zmp_errors = []
            zmp_stds = []

            zmp_in_supports = []

            cart_errors = []

            for i in range(RUNS_REPEAT):
                print(f"expected mass={expected_mass}, {mass=:.1f}, run_index={i}")

                log_path = os.path.join(log_dir, f"{mass:.1f}_{i}.bin")
                if not os.path.exists(log_path):
                    continue

                log = mc_log_ui.read_log(log_path)

                zmp_mea_x = log.get(ZMP_MEA_X)
                zmp_mea_y = log.get(ZMP_MEA_Y)
                zmp_pla_x = log.get(ZMP_PLA_X)
                zmp_pla_y = log.get(ZMP_PLA_Y)
                zmp_min_x = log.get(ZMP_MIN_X)
                zmp_max_x = log.get(ZMP_MAX_X)
                zmp_min_y = log.get(ZMP_MIN_Y)
                zmp_max_y = log.get(ZMP_MAX_Y)

                cart_pos_x = log.get(CART_POS_X)

                zmp_error_x = numpy.abs(zmp_mea_x - zmp_pla_x)
                zmp_error_y = numpy.abs(zmp_mea_y - zmp_pla_y)

                zmp_error = numpy.sqrt(
                    numpy.square(zmp_error_x) + numpy.square(zmp_error_y)
                )

                cart_error = abs(cart_pos_x[-1] - (cart_pos_x[0] + 1.0))

                zmp_errors_x.append(numpy.nanmean(zmp_error_x))
                zmp_stds_x.append(numpy.nanstd(zmp_error_x))
                zmp_errors_y.append(numpy.nanmean(zmp_error_y))
                zmp_stds_y.append(numpy.nanstd(zmp_error_y))

                zmp_errors.append(numpy.nanmean(zmp_error))
                zmp_stds.append(numpy.nanstd(zmp_error))

                zmp_in_support_x = numpy.logical_and(
                    zmp_mea_x >= zmp_min_x, zmp_mea_x <= zmp_max_x
                )
                zmp_in_support_y = numpy.logical_and(
                    zmp_mea_y >= zmp_min_y, zmp_mea_y <= zmp_max_y
                )
                zmp_in_support = numpy.logical_and(zmp_in_support_x, zmp_in_support_y)

                zmp_in_supports.append(numpy.mean(zmp_in_support))

                cart_errors.append(cart_error)

            if not zmp_errors_x:
                continue

            row = {
                "mass": mass,
                "expected_mass": expected_mass,
                "zmp_error_x": numpy.nanmean(zmp_errors_x),
                "zmp_std_x": numpy.sqrt(numpy.mean(numpy.square(zmp_stds_x))),
                "zmp_error_y": numpy.nanmean(zmp_errors_y),
                "zmp_std_y": numpy.sqrt(numpy.mean(numpy.square(zmp_stds_y))),
                "zmp_error": numpy.nanmean(zmp_errors),
                "zmp_std": numpy.sqrt(numpy.mean(numpy.square(zmp_stds))),
                "zmp_in_support": numpy.nanmean(zmp_in_supports),
                "cart_error": numpy.nanmean(cart_errors),
            }
            all_data.append(row)

    data = pandas.DataFrame(all_data)

    fig = plt.figure(figsize=(10, 15))

    metrics = ["zmp_in_support", "zmp_error", "cart_error"]
    ylabels = [
        "proportion of sequence in zupport region",
        "deviation from planned ZMP (m)",
        "cart error (m)",
    ]

    for idx, (metric, ylabel) in enumerate(zip(metrics, ylabels), 1):
        ax = fig.add_subplot(3, 1, idx, projection="3d")
        pivoted = data.pivot(index="expected_mass", columns="mass", values=metric)
        X, Y = numpy.meshgrid(pivoted.columns, pivoted.index)
        Z = pivoted.values

        ax.plot_surface(X, Y, Z, cmap="viridis")
        ax.set_xscale("log")
        ax.set_yscale("log")
        ax.set_zscale("log")
        ax.set_xlabel("cart mass")
        ax.set_ylabel("expected mass")
        ax.set_zlabel(ylabel)
        ax.set_title(metric)
        ax.view_init(azim=-120)

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
