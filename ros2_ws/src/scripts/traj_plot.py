#!/usr/bin/env python3
"""
Description: Plots both LiDAR trajectory and odometry with orientation arrows
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
import os

def quaternion_to_yaw(qx, qy, qz, qw):
    """
    Convert quaternion to yaw angle (rotation around Z-axis)
    """
    # Create rotation object from quaternion
    rot = R.from_quat([qx, qy, qz, qw])
    # Get euler angles (roll, pitch, yaw)
    euler = rot.as_euler('xyz', degrees=False)
    return euler[2]  # Return yaw

def load_trajectory_data(file_path):
    """
    Load trajectory data from text file
    Returns: timestamp, positions, quaternions, orientations
    """
    data = np.loadtxt(file_path)
    timestamps = data[:, 0]
    positions = data[:, 1:4]  # x, y, z
    quaternions = data[:, 4:8]  # qx, qy, qz, qw
    
    # Convert quaternions to yaw angles
    yaw_angles = []
    for i in range(len(quaternions)):
        yaw = quaternion_to_yaw(quaternions[i, 0], quaternions[i, 1], 
                               quaternions[i, 2], quaternions[i, 3])
        yaw_angles.append(yaw)
    
    return timestamps, positions, quaternions, np.array(yaw_angles)

def plot_single_trajectory(ax, positions, quaternions, yaw_angles, color='blue', label='Trajectory', arrow_color='red', show_markers=True):
    """
    Plot a single trajectory on given axes
    """
    # Standard axes: X horizontal, Y vertical with actual distances
    x = - positions[:, 0]  # X coordinate for horizontal axis
    y = - positions[:, 1]  # Y coordinate for vertical axis
    
    # Plot trajectory line
    ax.plot(x, y, color=color, linewidth=2, alpha=0.8, label=label, zorder=2)
    
    # Add orientation arrows with better visibility and spacing
    total_length = np.sum(np.sqrt(np.diff(x)**2 + np.diff(y)**2))
    arrow_spacing = max(0.15, total_length / 25)  # Reduced minimum spacing for higher density
    
    # Select arrow positions based on distance
    arrow_indices = [0]  # Always include start
    cumulative_dist = 0
    for i in range(1, len(x)):
        dist = np.sqrt((x[i] - x[i-1])**2 + (y[i] - y[i-1])**2)
        cumulative_dist += dist
        if cumulative_dist >= arrow_spacing:
            arrow_indices.append(i)
            cumulative_dist = 0
    
    # Add end point if not already included
    if arrow_indices[-1] != len(x) - 1:
        arrow_indices.append(len(x) - 1)
    
    for i in arrow_indices:
        # Calculate arrow components - FIX DIRECTION
        arrow_length = 0.2  # Increased size for better visibility
        
        # Convert quaternion to rotation matrix for proper direction
        qx, qy, qz, qw = quaternions[i]
        rot = R.from_quat([qx, qy, qz, qw])
        
        # Get the forward direction (typically X-axis in body frame)
        forward_vector = rot.apply([1, 0, 0])  # Forward direction in world frame
        
        # For standard axes
        dx = arrow_length * forward_vector[0]  # X component for horizontal (reversed)
        dy = arrow_length * forward_vector[1]  # Y component for vertical (reversed)
        
        # Draw larger, more visible orientation arrow
        ax.arrow(x[i], y[i], dx, dy, 
                head_width=0.12, head_length=0.1,
                fc=arrow_color, ec=arrow_color, linewidth=2, alpha=0.8, zorder=3)
    
    return x, y

def plot_traj_only_2d(traj_data, save_path=None):
    # Set up the plot with professional styling
    plt.style.use('seaborn-v0_8-whitegrid')
    fig, ax = plt.subplots(figsize=(12, 10))
    
    traj_timestamps, traj_positions, traj_quaternions, traj_yaw_angles = traj_data
    
    # Plot only TRAJ trajectory
    x, y = plot_single_trajectory(ax, traj_positions, traj_quaternions, traj_yaw_angles, 
                                 color='blue', label='LiDAR Trajectory', arrow_color='red', show_markers=False)
    
    # Add start and end markers
    ax.plot(x[0], y[0], 'go', markersize=12, label='Start Point', zorder=4)
    ax.annotate('START', 
                xy=(x[0], y[0]), 
                xytext=(x[0]+0.1, y[0]+0.1),
                fontsize=12, fontweight='bold', color='green',
                arrowprops=dict(arrowstyle='->', color='green', lw=2),
                zorder=5)
    
    ax.plot(x[-1], y[-1], 'ro', markersize=12, label='End Point', zorder=4)
    ax.annotate('END', 
                xy=(x[-1], y[-1]), 
                xytext=(x[-1]+0.1, y[-1]+0.1),
                fontsize=12, fontweight='bold', color='red',
                arrowprops=dict(arrowstyle='->', color='red', lw=2),
                zorder=5)
    
    # Customize the plot
    ax.set_xlabel('X Position (m)', fontsize=14, fontweight='bold')
    ax.set_ylabel('Y Position (m)', fontsize=14, fontweight='bold')
    ax.set_title('LiDAR Outdoor Trajectory', fontsize=16, fontweight='bold', pad=20)
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=12, loc='upper right')
    # Equal aspect ratio already set above
    
    # Add trajectory statistics
    traj_length = np.sum(np.sqrt(np.diff(x)**2 + np.diff(y)**2))
    traj_duration = traj_timestamps[-1] - traj_timestamps[0]
    
    stats_text = f'Trajectory Statistics:\n'
    stats_text += f'• Duration: {traj_duration:.2f}s\n'
    stats_text += f'• Length: {traj_length:.2f}m\n'
    stats_text += f'• Points: {len(x)}'
    
    ax.text(0.02, 0.98, stats_text, 
            transform=ax.transAxes, fontsize=11,
            verticalalignment='top', bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.9))
    
    # Make plot fill entire canvas with minimal margins
    plt.subplots_adjust(left=0.05, right=0.98, top=0.95, bottom=0.08)
    
    # Use equal aspect ratio for actual distance scaling
    ax.set_aspect('equal', adjustable='box')
    
    # Save or show
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"TRAJ trajectory plot saved to: {save_path}")
    else:
        plt.show()
    
    return fig, ax

def main():
    # Data file path
    traj_file = "/tmp/dump/traj_lidar.txt"
    
    # Check if file exists
    if not os.path.exists(traj_file):
        print(f"Error: Trajectory file not found at {traj_file}")
        return
    
    # Load trajectory data
    print("Loading trajectory data...")
    traj_data = load_trajectory_data(traj_file)
    traj_timestamps, traj_positions, traj_quaternions, traj_yaw_angles = traj_data
    
    print(f"Loaded {len(traj_positions)} trajectory points")
    print(f"TRAJ time range: {traj_timestamps[0]:.2f}s to {traj_timestamps[-1]:.2f}s")
    
    # Generate output filename
    output_file = "/tmp/lidar_outdoor_trajectory_final.png"
    
    # Create the trajectory plot
    print("Generating trajectory plot...")
    plot_traj_only_2d(traj_data, save_path=output_file)
    
    print("Trajectory visualization completed!")

if __name__ == "__main__":
    main()