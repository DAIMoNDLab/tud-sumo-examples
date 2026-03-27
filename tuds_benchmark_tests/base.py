import os
import sys
import random
import matplotlib.pyplot as plt
from lxml import etree
from mpl_toolkits.axes_grid1 import make_axes_locatable

import traci
from utils import *

outputs_loc = "outputs/base/"
default_sumoCMD = [
    "sumo",
    "-c",
    SCENARIO_CONFIG,
    "--seed",
    str(RANDOM_SEED),
    "--no-step-log",
    "true",
    "--no-warnings",
    "true",
]

path_tools = os.path.join(os.environ.get("SUMO_HOME"), "tools")
if path_tools in sys.path:
    pass
else:
    sys.path.append(path_tools)

random.seed(RANDOM_SEED)


@include_test
@profile_resources(["traci"])
def run_test_1():
    """
    Simple run of the on-ramp network simulation.
    """

    stats_file = f"{outputs_loc}test_1/stats.xml"
    sumoCMD = ["--statistic-output", stats_file]
    traci.start(default_sumoCMD + sumoCMD)

    curr_step = 0
    while (
        traci.simulation.getMinExpectedNumber() > 0
        and traci.simulation.getTime() < MAX_SIM_DURATION
    ):
        traci.simulationStep()
        curr_step += 1

    traci.close()
    return [stats_file], curr_step


@include_test
@profile_resources(["traci"])
def run_test_2():
    """
    Creating floating-car data from a simulation.
    """

    t_outputs_loc = f"{outputs_loc}test_2/"
    stats_file, fcd_file = f"{t_outputs_loc}stats.xml", f"{t_outputs_loc}fc_data.xml"
    sumoCMD = ["--fcd-output", fcd_file, "--statistic-output", stats_file]
    traci.start(default_sumoCMD + sumoCMD)

    curr_step = 0
    while (
        traci.simulation.getMinExpectedNumber() > 0
        and traci.simulation.getTime() < MAX_SIM_DURATION
    ):
        traci.simulationStep()
        curr_step += 1

    traci.close()
    return [stats_file, fcd_file], curr_step


@include_test
@profile_resources(["traci", "plt", "fig", "ax"])
def run_test_3():
    """
    Generating space-time diagrams.
    """

    t_outputs_loc = f"{outputs_loc}test_3/"
    stats_file, fcd_file = f"{t_outputs_loc}stats.xml", f"{t_outputs_loc}fc_data.xml"
    sumoCMD = ["--fcd-output", fcd_file, "--statistic-output", stats_file]
    traci.start(default_sumoCMD + sumoCMD)

    curr_step = 0
    while (
        traci.simulation.getMinExpectedNumber() > 0
        and traci.simulation.getTime() < MAX_SIM_DURATION
    ):
        traci.simulationStep()
        curr_step += 1

    traci.close()

    upstream_len, weave_len = 171.93, 70.26

    time_vals, dist_vals, speed_vals = [], [], []

    dataTree = etree.parse(fcd_file)
    root = dataTree.getroot()

    for timestep in root.getchildren():
        time = timestep.attrib["time"]
        for veh in timestep.getchildren():
            if "weave" in veh.attrib["lane"]:
                offset = upstream_len
            elif "downstream" in veh.attrib["lane"]:
                offset = upstream_len + weave_len
            elif "upstream" in veh.attrib["lane"]:
                offset = 0
            else:
                continue
            time_vals.append(float(time))
            speed_vals.append(float(veh.attrib["speed"]) * 3.6)
            dist_vals.append(float(veh.attrib["pos"]) + offset)

    plt.style.use("seaborn-v0_8-whitegrid")
    fig, ax = plt.subplots(1, 1)
    plt_data = ax.scatter(x=time_vals, y=dist_vals, c=speed_vals, cmap="hot", s=0.5)
    ax.set_xlim(min(time_vals), max(time_vals))
    ax.set_ylim(0, max(dist_vals))
    divider = make_axes_locatable(ax)
    cax = divider.append_axes("right", size="5%", pad=0.1)
    plt.colorbar(plt_data, cax=cax, label="Vehicle Speed (km/h)")
    ax.set_xlabel("Simulation Time (seconds)")
    ax.set_ylabel("Distance (m)")
    ax.set_title("Base TraCI Implementation", pad=20)
    fig.tight_layout()
    plt.savefig(f"{t_outputs_loc}st_diagram.png", dpi=600)
    plt.close()

    return [stats_file, fcd_file], curr_step


@include_test
@profile_resources(["traci"])
def run_test_4():
    """
    Running on-ramp scenario with ALINEA active.
    """
    stats_file = f"{outputs_loc}test_4/stats.xml"
    sumoCMD = ["--statistic-output", stats_file]
    traci.start(default_sumoCMD + sumoCMD)

    current_r_time, r_time = 0, 0

    active, prev_rate = False, MAX_RATE
    curr_step, occupancy_vals = 0, []
    while (
        traci.simulation.getMinExpectedNumber() > 0
        and traci.simulation.getTime() < MAX_SIM_DURATION
    ):
        traci.simulationStep()
        curr_step += 1

        light_state = traci.trafficlight.getRedYellowGreenState("J2")

        step_occ = (
            sum(
                [
                    traci.inductionloop.getLastStepOccupancy(det_id)
                    for det_id in ["dn_1", "dn_2"]
                ]
            )
            / 2
        )
        occupancy_vals.append(step_occ)

        if curr_step % CONTROL_INTERVAL == 0 and curr_step > 0:
            curr_occupancy = sum(occupancy_vals) / len(occupancy_vals)
            rate = get_metering_rate(prev_rate, curr_occupancy, O_CR, K_R)
            prev_rate, occupancy_vals = rate, []

            max_flow = 1200

            if rate > max_flow:
                traci.trafficlight.setRedYellowGreenState("J2", "G")
                active = False
            else:
                vehicles_per_ci = (rate / 3600) * CONTROL_INTERVAL
                cycle_length = CONTROL_INTERVAL / vehicles_per_ci
                r_time, active = cycle_length - 2, True

        if active:
            if light_state == "G":
                traci.trafficlight.setRedYellowGreenState("J2", "y")
            elif light_state == "y":
                traci.trafficlight.setRedYellowGreenState("J2", "r")
            elif (light_state == "r") & (current_r_time >= r_time):
                traci.trafficlight.setRedYellowGreenState("J2", "G")
                current_r_time = 0
            elif light_state == "r":
                current_r_time += 1

    traci.close()

    return [stats_file], curr_step


@include_test
@profile_resources(["traci"])
def run_test_5():
    """
    On-ramp network scenario with an incident occuring.
    """

    def set_edge_speed(e_id, speed):
        lane_ids = [f"{e_id}_{idx}" for idx in range(traci.edge.getLaneNumber(e_id))]
        for lane in lane_ids:
            traci.lane.setMaxSpeed(lane, speed / 3.6)

    t_outputs_loc = f"{outputs_loc}test_5/"
    stats_file, fcd_file = f"{t_outputs_loc}stats.xml", f"{t_outputs_loc}fc_data.xml"
    sumoCMD = ["--fcd-output", fcd_file, "--statistic-output", stats_file]
    traci.start(default_sumoCMD + sumoCMD)

    curr_step = 0
    while (
        traci.simulation.getMinExpectedNumber() > 0
        and traci.simulation.getTime() < MAX_SIM_DURATION
    ):
        traci.simulationStep()
        curr_step += 1

        if curr_step == INCIDENT_START:
            veh_id = random.choice(traci.edge.getLastStepVehicleIDs(VEHICLE_LOCATION))

            edge_idx = traci.vehicle.getRouteIndex(veh_id)
            next_edge = traci.vehicle.getRoute(veh_id)[edge_idx + 1]
            lane_idx = traci.vehicle.getLaneIndex(veh_id)

            traci.vehicle.setStop(
                vehID=veh_id,
                edgeID=next_edge,
                pos=VEHICLE_POSITION,
                laneIndex=lane_idx,
                duration=INCIDENT_DURATION,
            )
            set_edge_speed(next_edge, 15)

        if curr_step == INCIDENT_START + INCIDENT_DURATION:
            set_edge_speed(next_edge, 80)

    traci.close()
    return [stats_file], curr_step


if __name__ == "__main__":

    all_results = [test() for test in test_funcs]
    save_all_results(all_results, "results/base.csv")
    plot_all_results("results/base.csv", "results/tuds.csv")
