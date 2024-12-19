import argparse
import sys, os
import json
import numpy as np

from util.logging import print_value, print_success

from compare import compute_comparison_metrics

def load_default_settings(scenarios):
    table = dict()
    table["output_name"] = "comparison"
    table["simulations"] = scenarios
    table["clean_method_name"] = clean_method_name
    table["clean_simulation_name"] = clean_simulation_name
    table["method_order_list"] = method_order_list
    table["caption"] = lambda num_experiments, num_scenarios: "A comparative table."
    table["double_highlight"] = []
    return table

def highlight(value, decimals):
    if value == "-":
        return value
    
    value = f'{value:.{decimals}f}'
    return "\\textbf{" + str(value) + "}"


def no_highlight(value, decimals):
    return f'{value:.{decimals}f}'


def find_highlighted_value(text):
    highlight_split = text.split("\\textbf{")
    if len(highlight_split) == 1:
        return -1
    else:
        value = float(highlight_split[1].split("}")[0])
        return value


def include_method(text, method, metrics):
    result = text(metrics) if metrics['name'] == method else "-"
    return result


def exclude_method(text, method, metrics):
    result = text if metrics['name'] != method else "-"
    return result


def clean_simulation_name(simulation):
    return simulation

def clean_method_name(name):
    return name.replace("_", "\\_")

def method_order_list():
    return []

def get_method_id(planner, planners) -> int:
    if planner not in planners:
        raise IOError(f"Planner {planner} is missing from method_order_list()")

    return planners.index(planner)

class LatexTable:

    def __init__(self, name, filename, table_settings):
        self.headers = []
        self.data_lambdas = []
        self.alignments = []
        self.highlight_select = []
        self.table_settings = table_settings

        self.output_name = table_settings["output_name"]
        self.clean_simulation_name = table_settings["clean_simulation_name"]

        self.name = name
        self.filename = filename
        self.set_header()
        self.set_footer()

    def write_next(self, f, i, m):
        if i < len(self.headers) - 1:
            f.write(" & ")
        else:
            if m in self.table_settings["double_highlight"]:
                f.write(" \\\\\\Xhline{1.2pt}\n")
            else:
                f.write(" \\\\\\hline\n")

    def write_table(self, metrics, simulation, full_caption=False):
        f = open(self.filename, "w")

        self.header_lambda(f, metrics, simulation, full_caption)
        self.write_data(f, metrics)
        self.footer_lambda(f)

        f.close()

    def write_header(self, f, metrics, simulation, full_caption):
        d = metrics[0]

        f.write('\\begin{table*}\n'
                '\\small\\sf\\centering\n')
        f.write(self.table_settings["caption"](d["num_experiments"], len(metrics)) + "\n")

        f.write('\\begin{tabular}{|')
        for i in range(len(self.headers)):
            if i < len(self.headers) - 1:
                f.write(f'{self.alignments[i]}' + "|")
            else:
                f.write(f'{self.alignments[i]}' + "|}")
        f.write("\n")

        f.write("\\hline")
        for i, header in enumerate(self.headers):
            if "\\textbf" not in header:
                f.write('\\textbf{' + header + "}")
            else:
                f.write(header)
            self.write_next(f, i, 0)

    def set_header(self, header_lambda=None):
        if header_lambda is None:
            self.header_lambda = self.write_header
        else:
            self.header_lambda = lambda f, metrics, simulation, full_caption: header_lambda(self, f, metrics, simulation, full_caption)

    def set_footer(self, footer_lambda=None):
        if footer_lambda is None:
            self.footer_lambda = self.close_table
        else:
            self.footer_lambda = lambda f: footer_lambda(self, f)

    def get_highlighted(self, metrics):
        # First figure out which is best
        values = np.zeros((len(metrics), len(self.data_lambdas)))
        highlighted = []
        for i, data_text in enumerate(self.data_lambdas):
            for m, method_metrics in enumerate(metrics):
                values[m, i] = find_highlighted_value(data_text(method_metrics, highlight))

            select_value = self.highlight_select[i](values[:, i])
            highlighted.append(list(np.where(values[:, i] == select_value)[0]))  # There may be more than one

        return highlighted

    def write_data(self, f, metrics):
        highlighted = self.get_highlighted(metrics)

        for m, method_metrics in enumerate(metrics):
            for i, data_text in enumerate(self.data_lambdas):
                if m in highlighted[i]:
                    f.write(data_text(method_metrics, highlight))
                else:
                    f.write(data_text(method_metrics, no_highlight))

                self.write_next(f, i, m)

    def close_table(self, f):
        f.write("\\end{tabular}}\n"
                "\\label{tab:" + self.output_name + "}\n"
                                                    "\\end{table*}\n")

    def add_data(self, header_name, data_lambda, align="c", highlight_select=min):
        self.headers.append(header_name)
        self.data_lambdas.append(data_lambda)
        self.alignments.append(align)
        self.highlight_select.append(highlight_select)


class CombinedLatexTable(LatexTable):

    def __init__(self, filename, table_settings):
        super().__init__(filename.split(".")[0], filename, table_settings)

        self.simulation_list = table_settings["simulations"]

    def write_next(self, f, i, m):
        if i < len(self.headers) - 1:
            f.write(" & ")
        else:
            if m:
                f.write(" \\\\\\hline\n")
            else:
                f.write(" \\\\\n")

    def write_table(self, metrics, simulation, full_caption=False):
        f = open(self.filename, "w")

        self.header_lambda(f, metrics, simulation, full_caption)
        self.write_data(f, metrics)
        self.footer_lambda(f)

        f.close()

    def write_header(self, f, metrics, simulation, full_caption):
        d = metrics[0]
        f.write('\\begin{table*}\n'
                '\\centering\n')

        caption = self.table_settings["caption"](d[0]["num experiments"], len(metrics))
        f.write(caption)
        f.write("\n")

        f.write('\\begin{tabular}{|l|')
        for i in range(len(self.headers)):
            if i < len(self.headers) - 1:
                f.write(f'{self.alignments[i]}' + "|")
            else:
                f.write(f'{self.alignments[i]}' + "|}")
        f.write("\n")

        f.write("\\hline\\textbf{\#} & ")
        for i, header in enumerate(self.headers):
            if "\\textbf" not in header:
                f.write('\\textbf{' + header + "}")
            else:
                f.write(header)
                self.write_next(f, i, 1)

    def write_data(self, f, metrics):
        for simulation_id, simulation_metrics in enumerate(metrics):  # For all simulations read

            highlighted = self.get_highlighted(simulation_metrics)

            for m, method_metrics in enumerate(simulation_metrics):
                for i, data_text in enumerate(self.data_lambdas):
                    if m == 0 and i == 0:
                        simulation_name = clean_simulation_name(self.simulation_list[simulation_id])
                        f.write("\\multirow{" + str(len(simulation_metrics)) + "}{*}{" + simulation_name + "} & ")
                    elif i == 0:
                        f.write("&")

                    if m in highlighted[i]:
                        f.write(data_text(method_metrics, highlight))
                    else:
                        f.write(data_text(method_metrics, no_highlight))

                    self.write_next(f, i, int(m == len(simulation_metrics) - 1))


def create_table(base_folder, scenario, experiments, table_settings, config):

    comparison_metrics = compute_comparison_metrics(base_folder, scenario, experiments, config)

    table_folder = f"{base_folder}/tables/{scenario}/"
    table_name = "comparison"
    os.makedirs(table_folder, exist_ok=True)
   
    table = LatexTable(scenario, table_folder + table_name + ".tex", table_settings)
    table_settings["add_table_data"](table)

    table.write_table(comparison_metrics, scenario, True)
    print("Saved table as: " + table_folder + table_name + ".tex")
    return comparison_metrics