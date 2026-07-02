import mc_log_ui
import numpy
from testing_harness import compute_masses, RUNS_REPEAT, LOGS_DIR, BASE_DIR
import pandas
import matplotlib.pyplot as plt
import argparse
from scipy.spatial.transform import Rotation

TIME = "t"
ZMP_MIN_X = "CentroidalManager_ZMP_SupportRegion_min_x"
ZMP_MAX_X = "CentroidalManager_ZMP_SupportRegion_max_x"
ZMP_MIN_Y = "CentroidalManager_ZMP_SupportRegion_min_y"
ZMP_MAX_Y = "CentroidalManager_ZMP_SupportRegion_max_y"
ZMP_MEA_X = "CentroidalManager_ZMP_measured_x"
ZMP_MEA_Y = "CentroidalManager_ZMP_measured_y"
ZMP_PLA_X = "CentroidalManager_ZMP_planned_x"
ZMP_PLA_Y = "CentroidalManager_ZMP_planned_y"

CART_POS = "obj_FloatingBase_position_x"
# CART_POS = "obj_FloatingBase_orientation_z"
CART_POS_TARGET_OFFSET = 1.0
# CART_POS_TARGET = -Rotation.from_rotvec([0.0, 0.0, 1.57]).as_quat()[2]


def main(dir: str = None) -> None:
    print(dir, type(dir), LOGS_DIR)
    if dir is not None:
        if dir[-1] != "/":
            dir += "/"
        logs_dir = dir
    else:
        logs_dir = LOGS_DIR

    masses = compute_masses()

    data = pandas.DataFrame()
    data["mass"] = masses

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
            print(f"{mass=:.1f}, run_index={i}")

            log = mc_log_ui.read_log(BASE_DIR + logs_dir + f"{mass:.1f}_{i}.bin")

            zmp_mea_x = log.get(ZMP_MEA_X)
            zmp_mea_y = log.get(ZMP_MEA_Y)
            zmp_pla_x = log.get(ZMP_PLA_X)
            zmp_pla_y = log.get(ZMP_PLA_Y)
            zmp_min_x = log.get(ZMP_MIN_X)
            zmp_max_x = log.get(ZMP_MAX_X)
            zmp_min_y = log.get(ZMP_MIN_Y)
            zmp_max_y = log.get(ZMP_MAX_Y)

            cart_pos = log.get(CART_POS)

            zmp_error_x = numpy.abs(zmp_mea_x - zmp_pla_x)
            zmp_error_y = numpy.abs(zmp_mea_y - zmp_pla_y)

            zmp_error = numpy.sqrt(
                numpy.square(zmp_error_x) + numpy.square(zmp_error_y)
            )

            print(cart_pos[0], cart_pos[-1], CART_POS_TARGET_OFFSET)
            cart_error = numpy.abs(cart_pos[-1] - (cart_pos[0] + CART_POS_TARGET_OFFSET))

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

        data.loc[data["mass"] == mass, "zmp_error_x"] = numpy.nanmean(zmp_error_x)
        data.loc[data["mass"] == mass, "zmp_std_x"] = numpy.sqrt(
            numpy.mean(numpy.square(zmp_stds_x))
        )
        data.loc[data["mass"] == mass, "zmp_error_y"] = numpy.nanmean(zmp_error_y)
        data.loc[data["mass"] == mass, "zmp_std_y"] = numpy.sqrt(
            numpy.mean(numpy.square(zmp_stds_y))
        )

        data.loc[data["mass"] == mass, "zmp_error"] = numpy.nanmean(zmp_error)
        data.loc[data["mass"] == mass, "zmp_std"] = numpy.sqrt(
            numpy.mean(numpy.square(zmp_stds))
        )

        data.loc[data["mass"] == mass, "zmp_in_support"] = numpy.nanmean(
            zmp_in_supports
        )

        data.loc[data["mass"] == mass, "cart_error"] = numpy.nanmean(cart_errors)

    axes = data.plot(
        x="mass",
        y=["zmp_in_support", "zmp_error", "cart_error"],
        logx=True,
        subplots=True,
        legend=False,
    )

    axes[0].set_title("stability metrics for varying cart mass")
    plt.xlabel("cart mass")
    axes[0].set_ylabel("proportion of sequence in zupport region")
    axes[1].set_ylabel("deviation from planned ZMP (m)")
    axes[2].set_ylabel("cart error (m)")
    plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--log-directory", "-d", type=str)
    args = parser.parse_args()

    main(args.log_directory)
