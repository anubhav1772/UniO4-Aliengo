import numpy as np
import matplotlib.pyplot as plt

def load_data(csv_file):
    data = np.loadtxt(csv_file, delimiter=',')
    return data

def plot_12_motors(data):
    time = np.arange(len(data))
    num_motors = 12

    fig, axes = plt.subplots(nrows=4, ncols=3, figsize=(18, 12))
    axes = axes.flatten()

    for i in range(num_motors):
        #des_col = 2 * i
        #act_col = 2 * i + 1

        # desired = data[:, des_col]
        #actual = data[:, act_col]
        desired = data[:, i]
        actual = np.zeros(desired.shape)
        error = desired - actual

        min_error = np.min(error)
        max_error = np.max(error)
        min_idx = np.argmin(error)
        max_idx = np.argmax(error)

        ax = axes[i]
        ax.plot(time, desired, label='Desired', linestyle='--', linewidth=1.5)
        ax.plot(time, actual, label='Actual', linewidth=1.5)
        ax.plot(time, error, label='Error', linestyle=':', linewidth=1.2)

        # Highlight min and max error points on the error plot
        ax.plot(time[min_idx], error[min_idx], 'ro', label='Min Error' if i == 0 else "")
        ax.plot(time[max_idx], error[max_idx], 'go', label='Max Error' if i == 0 else "")

        # Annotate the values
        ax.annotate(f"{min_error:.2f}", (time[min_idx], error[min_idx]),
                    textcoords="offset points", xytext=(-10, -10), ha='center', color='red')
        ax.annotate(f"{max_error:.2f}", (time[max_idx], error[max_idx]),
                    textcoords="offset points", xytext=(10, 10), ha='center', color='green')

        ax.set_title(f'Motor {i+1}')
        ax.set_xlabel('Time Step')
        ax.set_ylabel('Angle (rad)')
        ax.grid(True)

    handles, labels = ax.get_legend_handles_labels()
    fig.legend(handles, labels, loc='upper center', ncol=4, fontsize='small')
    plt.tight_layout()
    plt.suptitle('Desired vs Actual Positions for 12 Motors', fontsize=16, y=1.02)
    plt.subplots_adjust(top=0.93)  # Adjust space for suptitle and legend
    plt.show()

if __name__ == "__main__":
    csv_file = "data/data_13_5.csv"  # Replace with your CSV file name
    data = load_data(csv_file)
    # if data.shape[1] != 24:
    #     raise ValueError("Expected 24 columns (12 motors × 2: desired + actual).")
    plot_12_motors(data)