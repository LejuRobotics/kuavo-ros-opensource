import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import savgol_filter
import math


class EvalMetric:
    def __init__(self):
        self.latency = []    # latency of the retargeting process
        self.velocities = [] # velocities alongside time
        self.stamps = []
        self.lowest_height = []

    def add_latency(self, delta_t: float):
        self.latency.append(delta_t)
    
    def add_velocity(self, vel: np.ndarray):
        self.velocities.append(vel.copy())

    def add_timestamp(self, stamp: int | float):
        self.stamps.append(stamp)

    def add_lowest_height(self, lowest_height: float):
        self.lowest_height.append(lowest_height)

    def plot_latency_hist(self, bins=50, show=True, save_path=None):
        """
        Plot a histogram of latencies (in milliseconds).

        Parameters:
        - bins: int or sequence, number of bins for the histogram (default 50)
        - show: bool, whether to call plt.show() (default True)
        - save_path: str or None, if provided, save the figure to this path
        """
        if not self.latency:
            print("No latency data to plot.")
            return

        # Convert to milliseconds
        data_ms = [v * 1000.0 for v in self.latency]

        plt.figure(figsize=(8, 6))
        plt.hist(data_ms, bins=bins, color="#4c72b0", edgecolor="black", alpha=0.75)
        plt.title("gmr optimisation latency")
        plt.xlabel("ms")
        plt.ylabel("Frequency")
        plt.grid(True, linestyle="--", alpha=0.5)

        if save_path is not None:
            plt.savefig(save_path, bbox_inches="tight")
            print(f"Latency histogram saved to: {save_path}")

        if show:
            plt.show()
        else:
            plt.close()

    def plot_velocities(self):
        """
        Smart velocity plotting with automatic subplot layout.
        """
        velocities = np.array(self.velocities)  # Shape: (N, nv)
        N, nv = velocities.shape
        
        if nv == 0:
            print("No velocity data to plot.")
            return
        
        t = np.arange(N) * 0.01  # Assuming 100 Hz sampling rate

        # Automatically create multiple figures if needed
        n_figures = math.ceil(nv / 8)
   
        for fig_idx in range(n_figures):
            start_idx = fig_idx * 8
            end_idx = min(start_idx + 8, nv)
            n_plots = end_idx - start_idx
            
            # Dynamic grid: try to make it roughly square or wide
            cols = min(n_plots, 4)  # Max 4 columns for readability
            rows = math.ceil(n_plots / cols)
            
            fig, axes = plt.subplots(rows, cols, 
                                    figsize=(12, 3 * rows),
                                    sharex=True,
                                    constrained_layout=True)
            
            # Handle case when only one row or one subplot
            if n_plots == 1:
                axes = [axes]
            elif rows == 1:
                axes = axes  # already 1D array
            else:
                axes = axes.flatten()  # make it 1D for easy indexing
            
            # Plot each velocity dimension in its subplot
            for i in range(n_plots):
                dim_idx = start_idx + i
                ax = axes[i]
                ax.plot(t, velocities[:, dim_idx], label=f'v{dim_idx}', linewidth=1.2)
                ax.set_ylabel(f'Velocity {dim_idx}')
                ax.grid(True, alpha=0.3)
                if i >= n_plots - cols:  # bottom row
                    ax.set_xlabel('Time (s)')
            
            # Hide unused subplots
            for i in range(n_plots, len(axes)):
                axes[i].set_visible(False)
            
            # Add an overall title for the figure
            fig.suptitle(f'Joint/Link Velocities {start_idx}–{end_idx-1}', 
                        fontsize=14, fontweight='bold')
            
            plt.show()

    def plot_timestamp(self, bins=50, show=True, save_path=None):
        """
        Plot a histogram of latencies (in milliseconds).

        Parameters:
        - bins: int or sequence, number of bins for the histogram (default 50)
        - show: bool, whether to call plt.show() (default True)
        - save_path: str or None, if provided, save the figure to this path
        """
        if not self.latency:
            print("No latency data to plot.")
            return

        # Convert to milliseconds
        data_ms = [v * 1000.0 for v in self.latency]
        stamps = np.array(self.stamps)
        delta_stamps = np.diff(stamps)

        plt.figure(figsize=(8, 6))
        plt.hist(delta_stamps, bins=bins, color="#4c72b0", edgecolor="black", alpha=0.75)
        plt.title("PICO frame delta stamps")
        plt.xlabel("ms")
        plt.ylabel("Frequency")
        plt.grid(True, linestyle="--", alpha=0.5)

        if save_path is not None:
            plt.savefig(save_path, bbox_inches="tight")
            print(f"Latency histogram saved to: {save_path}")

        if show:
            plt.show()
        else:
            plt.close()
    
    def plot_lowest_height(self, show=True, save_path=None):
        """
        Plot a histogram of lowest heights.
        """
        lowest_heights = np.array(self.lowest_height) # N, 
        t = np.arange(len(lowest_heights)) * 0.01  # Assuming 100 Hz sampling rate
        plt.figure(figsize=(8, 6))
        plt.plot(t, lowest_heights)
        plt.title("Lowest Body Height")
        plt.xlabel("Time (s)")
        plt.ylabel("Height (m)")
        plt.grid(True, linestyle="--", alpha=0.5)
        plt.show()
