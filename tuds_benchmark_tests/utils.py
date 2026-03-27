import time, psutil, os, threading, csv, inspect, ast
from functools import wraps
from pathlib import Path
from tud_sumo.simulation import Simulation
import matplotlib.pyplot as plt
import numpy as np

SCENARIO_CONFIG = "scenario/onramp.sumocfg"
RANDOM_SEED, MAX_SIM_DURATION = 1, 2000

# TEST 4
CONTROL_INTERVAL, K_R, O_CR = 60, 80, 8
MIN_RATE, MAX_RATE = 200, 2000

# TEST 5
INCIDENT_START, INCIDENT_DURATION = 1000, 100
VEHICLE_LOCATION, VEHICLE_POSITION = "e_weave", 0.8

# PLOTS
plt_labels = {
    "runtime_s": {"title": "Elapsed Real Time", "y_label": "Seconds (s)"},
    "step_runtime": {"title": "Real Time per Step", "y_label": "Seconds (s)"},
    "cpu_user_s": {"title": "Program CPU Usage", "y_label": "Seconds (s)"},
    "cpu_system_s": {"title": "System CPU Usage", "y_label": "Seconds (s)"},
    "cpu_total_s": {"title": "Total CPU Usage", "y_label": "Seconds (s)"},
    "cpu_avg_percent": {"title": "Average CPU Utilisation", "y_label": "Percent (%)"},
    "peak_rss_mb": {
        "title": "Peak Memory Utilisation",
        "y_label": "Peak Resident Set Size (MB)",
    },
    "output_size": {"title": "Total Output File(s) Size", "y_label": "Output Size (B)"},
    "loc": {"title": "Required Lines of Code", "y_label": "No. of Lines"},
    "api_calls": {"title": "Required Package Calls", "y_label": "No. of Calls"},
    "complexity": {"title": "Code Cyclomatic Complexity", "y_label": "No. of Paths"},
}


test_funcs = []


def include_test(func):
    test_funcs.append(func)
    return func


def profile_resources(modules, sample_interval=0.01):
    def decorator(func):
        @wraps(func)
        def wrapper(*args, **kwargs):

            source = inspect.getsource(func)
            tree = ast.parse(source)
            api_calls, complexity = 0, 1

            def get_full_attr(node):
                if isinstance(node, ast.Name):
                    return node.id
                elif isinstance(node, ast.Attribute):
                    return f"{get_full_attr(node.value)}.{node.attr}"
                return None

            class CallVisitor(ast.NodeVisitor):
                def visit_Call(self, node):
                    nonlocal api_calls
                    full_name = get_full_attr(node.func)
                    if full_name:
                        if any(
                            full_name.startswith(root + ".") or full_name == root
                            for root in modules
                        ):
                            api_calls += 1
                    self.generic_visit(node)

            CallVisitor().visit(tree)

            class CCVisitor(ast.NodeVisitor):
                def visit_If(self, node):
                    nonlocal complexity
                    complexity += 1
                    self.generic_visit(node)

                def visit_For(self, node):
                    nonlocal complexity
                    complexity += 1
                    self.generic_visit(node)

                def visit_While(self, node):
                    nonlocal complexity
                    complexity += 1
                    self.generic_visit(node)

                def visit_Try(self, node):
                    nonlocal complexity
                    complexity += len(node.handlers)
                    if node.finalbody:
                        complexity += 1
                    self.generic_visit(node)

                def visit_BoolOp(self, node):
                    nonlocal complexity
                    if isinstance(node.op, (ast.And, ast.Or)):
                        complexity += len(node.values) - 1
                    self.generic_visit(node)

            CCVisitor().visit(tree)

            # Include the call to the wrapper itself if needed

            process = psutil.Process(os.getpid())

            # --- CPU baseline ---
            cpu_before = process.cpu_times()

            # --- Memory monitoring ---
            peak_rss = 0
            running = True

            def monitor_memory():
                nonlocal peak_rss
                while running:
                    try:
                        rss = process.memory_info().rss
                        peak_rss = max(peak_rss, rss)
                    except psutil.NoSuchProcess:
                        break
                    time.sleep(sample_interval)

            mem_thread = threading.Thread(target=monitor_memory)
            mem_thread.start()

            # --- Runtime ---
            start = time.perf_counter()
            output_files, step_count = func(*args, **kwargs)
            end = time.perf_counter()

            running = False
            mem_thread.join()

            # --- CPU after ---
            cpu_after = process.cpu_times()

            cpu_user = cpu_after.user - cpu_before.user
            cpu_system = cpu_after.system - cpu_before.system
            cpu_total = cpu_user + cpu_system

            wall_time = end - start
            cpu_percent_avg = (cpu_total / wall_time) * 100 if wall_time > 0 else 0.0

            # --- Measure file size ---
            output_size = sum([Path(file).stat().st_size for file in output_files])

            lines, _ = inspect.getsourcelines(func)
            loc = sum(1 for l in lines if l.strip() and not l.lstrip().startswith("#"))

            return {
                "runtime_s": wall_time,  # Wall-clock elapsed time (including all python overhead, waiting, input/output etc.)
                "step_runtime": wall_time / step_count,  # Wall-time per step
                "cpu_user_s": cpu_user,  # CPU time spent executing user-level code/libraries
                "cpu_system_s": cpu_system,  # CPU time spent on system/kernel operations (input/output, memory management etc.)
                "cpu_total_s": cpu_total,  # Sum of user/system CPU time - total processing time consumed by the process
                "cpu_avg_percent": cpu_percent_avg,  # Average CPU utilisation of process over wall-clock runtime
                "peak_rss_mb": peak_rss
                / 1024**2,  # Maximum recorded RSS during execution
                "output_size": output_size,  # Size in bytes of all files generated during the test
                "loc": loc,  # Number of lines of code in the function
                "api_calls": api_calls,  # Total number of function calls to TraCI or Simulation/Plotter objects (including repeated calls)
                "complexity": complexity,  # Code cyclomatic complexity, counting decision points/paths through source code
            }

        return wrapper

    return decorator


def count_api_api_calls(func, package_name):
    source = inspect.getsource(func)
    tree = ast.parse(source)
    api_calls = []

    class CallVisitor(ast.NodeVisitor):
        def visit_Call(self, node):
            # e.g., package_name.func(...)
            if isinstance(node.func, ast.Attribute):
                if isinstance(node.func.value, ast.Name):
                    if node.func.value.id == package_name:
                        api_calls.append(node.func.attr)
            self.generic_visit(node)

    CallVisitor().visit(tree)
    return len(set(api_calls))  # count distinct API api_calls


def save_all_results(all_results, filename):

    headers, rows = [
        "test_no",
        "runtime_s",
        "step_runtime",
        "cpu_user_s",
        "cpu_system_s",
        "cpu_total_s",
        "cpu_avg_percent",
        "peak_rss_mb",
        "output_size",
        "loc",
        "api_calls",
        "complexity",
    ], []

    for results_idx, results_set in enumerate(all_results):
        rows.append(
            [
                results_set[hdr] if hdr != "test_no" else results_idx + 1
                for hdr in headers
            ]
        )

    with open(filename, "w") as fp:
        writer = csv.writer(fp)
        writer.writerows([headers] + rows)


def get_metering_rate(
    prev_rate: int | float,
    curr_occupancy: int | float,
    occupancy_0: int | float,
    K_r: int,
) -> int | float:

    rate = prev_rate + (K_r * (occupancy_0 - curr_occupancy))
    return min(max(rate, MIN_RATE), MAX_RATE)


def plot_all_results(base_res, tuds_res, output_loc="results/"):

    if not os.path.exists(base_res) or not os.path.exists(tuds_res):
        return

    keys = None
    base_data, tuds_data = {}, {}

    for res_file, data in zip([base_res, tuds_res], [base_data, tuds_data]):
        with open(res_file, "r") as fp:
            reader = csv.reader(fp)
            header = True

            for row in reader:
                if keys == None:
                    keys, header = row[1:], False
                    for key in keys:
                        data[key] = []
                elif header:
                    header = False
                    continue
                else:
                    for idx, key in enumerate(keys):
                        data[key].append(float(row[idx + 1]))

        keys = None

    plot_barchart(
        base_data,
        tuds_data,
        ["complexity", "loc", "api_calls", "peak_rss_mb", "runtime_s", "cpu_avg_percent"],
        "TUD-SUMO and Standard TraCI Benchmark Comparison",
        f"{output_loc}benchmark_comparison.png"
    )
    plot_barchart(
        base_data,
        tuds_data,
        ["complexity", "loc", "api_calls"],
        "Standard TraCI and TUD-SUMO User Complexity",
        f"{output_loc}user_complexity.png",
    )
    plot_barchart(
        base_data,
        tuds_data,
        ["cpu_avg_percent", "cpu_total_s", "peak_rss_mb"],
        "Standard TraCI and TUD-SUMO Computational Load",
        f"{output_loc}computational_load.png",
    )
    plot_barchart(
        base_data,
        tuds_data,
        ["runtime_s"],
        "Standard TraCI and TUD-SUMO Elapsed Real Time",
        f"{output_loc}runtime.png",
    )
    plot_barchart(
        base_data,
        tuds_data,
        ["output_size"],
        "Standard TraCI and TUD-SUMO Total Output File Size",
        f"{output_loc}output_size.png",
        [2, 3],
    )


def plot_barchart(base_data, tuds_data, data_keys, title, savefig=None, test_nos=None):
    n_tests = len(base_data["runtime_s"])

    plt.style.use("seaborn-v0_8-whitegrid")
    if len(data_keys) != 6: fig, axes = plt.subplots(1, len(data_keys), figsize=(len(data_keys) * 5, 5))
    else:
        fig, axes = plt.subplots(3, 2, figsize=(10, 15))
        axes = [item for sublist in axes for item in sublist]
    if test_nos == None:
        test_labels = [str(chr(i + 96)).upper() for i in range(1, n_tests + 1)]
    else:
        test_labels = [str(chr(i + 96)).upper() for i in test_nos]
    colours = ["#348133", "#05A6D7"]

    if len(data_keys) == 1:
        axes = [axes]

    for ax_idx, (data_key, ax) in enumerate(zip(data_keys, axes)):
        x = np.arange(len(test_labels))
        width = 0.25
        multiplier = 0

        for package, results, c in zip(
            ["Standard TraCI", "TUD-SUMO"],
            [base_data[data_key], tuds_data[data_key]],
            colours,
        ):
            offset = width * multiplier
            if test_nos == None:
                r = results
            else:
                r = [results[no - 1] for no in test_nos]
            ax.bar(x + offset, r, width, color=c, label=package, zorder=5)
            multiplier += 1

        ax.set_ylabel(plt_labels[data_key]["y_label"])
        if len(data_keys) > 1: ax.set_title(f"{str(chr(ax_idx + 97))}) {plt_labels[data_key]['title']}")
        else: ax.set_title(plt_labels[data_key]['title'])
        if len(data_keys) != 6 or ax_idx in [4, 5]: ax.set_xlabel("Test")
        ax.set_xticks(x + width / 2, test_labels)
        ax.grid(True, 'both', color='grey', linestyle='dashed', linewidth=0.5, zorder=1)

        handles, labels = ax.get_legend_handles_labels()

    fig.suptitle(title, fontweight="bold")
    fig.legend(handles, labels, loc="lower center", bbox_to_anchor=(0.5, 0.01), ncol=3)
    if len(data_keys) != 6: fig.tight_layout(rect=[0, 0.07, 1, 0.99])
    else: fig.tight_layout(rect=[0, 0.02, 1, 0.99])

    if savefig == None:
        plt.show(block=True)
    else:
        plt.savefig(savefig, dpi=600)
    plt.close()
