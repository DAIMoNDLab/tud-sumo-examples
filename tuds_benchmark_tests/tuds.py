from tud_sumo.simulation import Simulation
from tud_sumo.plot import Plotter
from utils import *

import random

outputs_loc = "outputs/tuds/"
random.seed(RND_SEED)


@include_test
@profile_resources(["sim"])
def run_test_1():
    """
    Simple run of the on-ramp network simulation.
    """
    sim = Simulation("TUD-SUMO test 1", verbose=False)
    sim.start(SCENARIO_CFG, get_fc_data=False, seed=RND_SEED)

    sim.step_through(n_steps=MAX_SIM_DUR)
    sim.end()

    stats_file = f"{outputs_loc}test_1/stats.txt"
    sim.print_summary(stats_file)
    return [stats_file], sim.curr_step


@include_test
@profile_resources(["sim"])
def run_test_2():
    """
    Creating floating-car data from a simulation.
    """
    sim = Simulation("TUD-SUMO test 2", verbose=False)
    sim.start(SCENARIO_CFG, get_fc_data=True, seed=RND_SEED)

    sim.step_through(n_steps=MAX_SIM_DUR)
    sim.end()

    t_outputs_loc = f"{outputs_loc}test_2/"
    stats_file = f"{t_outputs_loc}stats.txt"
    sim.print_summary(stats_file)

    data_file = f"{t_outputs_loc}fc_data.pkl"
    sim.save_fc_data(data_file)
    return [stats_file, data_file], sim.curr_step


@include_test
@profile_resources(["sim", "plt"])
def run_test_3():
    """
    Generating space-time diagrams.
    """
    sim = Simulation("TUD-SUMO test 3", verbose=False)
    sim.start(SCENARIO_CFG, get_fc_data=False, seed=RND_SEED)

    edges = ["e_upstream", "e_weave", "e_downstream"]
    sim.add_tracked_edges(edges)

    sim.step_through(n_steps=MAX_SIM_DUR)
    sim.end()

    t_outputs_loc = f"{outputs_loc}test_3/"
    stats_file = f"{t_outputs_loc}stats.txt"
    sim.print_summary(stats_file)

    data_file = f"{t_outputs_loc}all_data.pkl"
    sim.save_data(data_file)

    plt = Plotter(data_file)

    plt.plot_space_time_diagram(
        edges,
        fig_title="TUD-SUMO Implementation",
        save_fig=f"{t_outputs_loc}st_diagram.png",
    )

    return [stats_file, data_file], sim.curr_step


@include_test
@profile_resources(["sim"])
def run_test_4():
    """
    Running on-ramp scenario with ALINEA active.
    """
    sim = Simulation("TUD-SUMO test 4", verbose=False)
    sim.start(SCENARIO_CFG, get_fc_data=False, seed=RND_SEED)

    sim.add_tracked_junctions(
        {"J2": {"meter_params": {"min_rate": MIN_RATE, "max_rate": MAX_RATE}}}
    )

    prev_rate = MAX_RATE
    while sim.is_running() and sim.curr_step < MAX_SIM_DUR:

        sim.step_through()

        if sim.curr_step % CTRL_INT == 0 and sim.curr_step > 0:
            curr_occupancy = (
                sim.get_interval_detector_data(
                    ["dn_1", "dn_2"], "occupancies", CTRL_INT
                )
                * 100
            )
            rate = get_metering_rate(prev_rate, curr_occupancy, O_CR, K_R)
            sim.set_tl_metering_rate("J2", rate)
            prev_rate = rate

    sim.end()

    t_outputs_loc = f"{outputs_loc}test_4/"
    stats_file = f"{t_outputs_loc}stats.txt"
    sim.print_summary(stats_file)

    return [stats_file], sim.curr_step


@include_test
@profile_resources(["sim"])
def run_test_5():
    """
    On-ramp network scenario with an incident occuring.
    """
    sim = Simulation("TUD-SUMO test 5", verbose=False)
    sim.start(SCENARIO_CFG, get_fc_data=False, seed=RND_SEED)

    while sim.is_running() and sim.curr_step < MAX_SIM_DUR:
        sim.step_through()

        if sim.curr_step == INCIDENT_START:
            veh_id = random.choice(sim.get_geometry_vals(VEH_LOC, "vehicle_ids"))

            sim.cause_incident(
                INCIDENT_DUR, vehicle_ids=veh_id, edge_speed=15, position=VEH_POS
            )

    sim.end()

    stats_file = f"{outputs_loc}test_5/stats.txt"
    sim.print_summary(stats_file)
    return [stats_file], sim.curr_step


if __name__ == "__main__":

    all_results = [test() for test in test_funcs]
    save_all_results(all_results, "results/tuds.csv")
    plot_all_results("results/base.csv", "results/tuds.csv")
