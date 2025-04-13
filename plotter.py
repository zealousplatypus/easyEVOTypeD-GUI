import pandas as pd
import matplotlib.pyplot as plt
import io

# Process the easyEVO output csv format into multiple dataframes per experiment
def generate_dfs(file_path):
    with open(file_path, 'r') as file:
        lines = file.readlines()

    # Use the first line as the template for the header
    header_template = lines[0].strip()

    # Identify header rows based on matching the first line
    header_indices = [i for i, line in enumerate(lines) if line.strip() == header_template]

    # Extract header names from the first header row
    header = header_template.split(',')

    # Create a list of DataFrames, each representing an experiment
    experiments = []
    for i in range(len(header_indices)):
        start_idx = header_indices[i] + 1
        end_idx = header_indices[i + 1] if i + 1 < len(header_indices) else len(lines)
        experiment_data = ''.join(lines[start_idx:end_idx])
        experiment_df = pd.read_csv(io.StringIO(experiment_data), header=None, names=header)
        if not experiment_df.empty:
            experiments.append(experiment_df)
    return experiments


# plot the ODs over time of the current experiment, with time in hours
def plot_OD(experiments, ax, experiment_number, start_time_hours=None, end_time_hours=None):
    experiment = experiments[experiment_number]
    # Convert upTime from seconds to the specified hours scale
    ax.plot(experiment['upTime'] / 3600.0, experiment['OD940'], label='OD940')
    ax.set_xlabel('Time (hours)')
    ax.set_ylabel('OD940')
    ax.legend()

    ax.set_xlim([start_time_hours, end_time_hours])

def read_and_plot_OD(file_path, ax, n, start_time_hours=None, end_time_hours=None):
    # Load the CSV file
    experiments = generate_dfs(file_path)
    plot_OD(experiments, ax, n, start_time_hours, end_time_hours)
