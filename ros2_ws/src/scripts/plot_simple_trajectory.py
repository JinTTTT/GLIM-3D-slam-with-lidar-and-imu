#!/usr/bin/env python3
"""
Simple 2D trajectory plot for GLIM traj_lidar results
Shows only the optimized LiDAR trajectory with start/end positions
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import argparse
import os

def quaternion_to_yaw(qx, qy, qz, qw):
    """Convert quaternion to yaw angle in radians"""
    return np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy**2 + qz**2))

def load_trajectory(filename='traj_lidar.txt'):
    """Load and convert trajectory to 2D format"""
    if not os.path.exists(filename):
        print(f"Error: {filename} not found!")
        return None
    
    try:
        # Load TUM format data
        data = np.loadtxt(filename)
        if data.shape[1] != 8:
            print(f"Error: Expected 8 columns (TUM format), got {data.shape[1]}")
            return None
        
        # Extract X, Y positions
        timestamps = data[:, 0]
        x, y = data[:, 1], data[:, 2]
        
        # Convert quaternion to yaw (optional, for info)
        qx, qy, qz, qw = data[:, 4], data[:, 5], data[:, 6], data[:, 7]
        yaw = quaternion_to_yaw(qx, qy, qz, qw)
        
        return {
            'x': x,
            'y': y,
            'yaw': yaw,
            'timestamps': timestamps
        }
        
    except Exception as e:
        print(f"Error loading {filename}: {e}")
        return None

def plot_trajectory(traj_data, save_path='traj_lidar_simple.png'):
    """Create simple 2D trajectory plot"""
    
    x, y = traj_data['x'], traj_data['y']
    
    # Calculate trajectory statistics
    total_distance = np.sum(np.sqrt(np.diff(x)**2 + np.diff(y)**2))
    displacement = np.sqrt((x[-1] - x[0])**2 + (y[-1] - y[0])**2)
    
    # Create single plot
    plt.figure(figsize=(10, 8))
    
    # Plot trajectory path
    plt.plot(x, y, 'b-', linewidth=2.5, alpha=0.8, label='LiDAR Trajectory')
    
    # Mark start position
    plt.scatter(x[0], y[0], color='green', s=150, marker='o', 
               label=f'Start ({x[0]:.3f}, {y[0]:.3f})', zorder=5, edgecolor='darkgreen', linewidth=2)
    
    # Mark end position  
    plt.scatter(x[-1], y[-1], color='red', s=150, marker='s', 
               label=f'End ({x[-1]:.3f}, {y[-1]:.3f})', zorder=5, edgecolor='darkred', linewidth=2)
    
    # Formatting
    plt.xlabel('X (meters)', fontsize=12)
    plt.ylabel('Y (meters)', fontsize=12)
    plt.title(f'GLIM LiDAR Trajectory\n{len(x)} poses, {total_distance:.2f}m path, {displacement:.2f}m displacement', 
              fontsize=14, fontweight='bold')
    plt.grid(True, alpha=0.3)
    plt.axis('equal')
    plt.legend(fontsize=11)
    
    # Add some padding around the trajectory
    x_margin = (max(x) - min(x)) * 0.1
    y_margin = (max(y) - min(y)) * 0.1
    plt.xlim(min(x) - x_margin, max(x) + x_margin)
    plt.ylim(min(y) - y_margin, max(y) + y_margin)
    
    plt.tight_layout()
    
    # Save plot
    plt.savefig(save_path, dpi=300, bbox_inches='tight')
    print(f"📊 Plot saved to: {save_path}")
    
    plt.show()
    
    # Print summary
    print(f"\n📈 Trajectory Summary:")
    print(f"   📍 Start: ({x[0]:.3f}, {y[0]:.3f}) meters")
    print(f"   🏁 End: ({x[-1]:.3f}, {y[-1]:.3f}) meters")
    print(f"   📏 Path length: {total_distance:.3f} meters")
    print(f"   📐 Direct displacement: {displacement:.3f} meters")
    print(f"   📊 Number of poses: {len(x)}")
    print(f"   🔄 Path efficiency: {displacement/total_distance*100:.1f}%")

def main():
    parser = argparse.ArgumentParser(description='Plot simple LiDAR trajectory')
    parser.add_argument('--input', '-i', default='traj_lidar.txt',
                       help='Input trajectory file (default: traj_lidar.txt)')
    parser.add_argument('--save', '-s', default='traj_lidar_simple.png',
                       help='Output plot file (default: traj_lidar_simple.png)')
    
    args = parser.parse_args()
    
    print("🎯 Simple GLIM LiDAR Trajectory Plot")
    print("=" * 40)
    
    # Load trajectory data
    print(f"📂 Loading trajectory from: {args.input}")
    traj_data = load_trajectory(args.input)
    
    if traj_data is None:
        return 1
    
    # Create plot
    plot_trajectory(traj_data, args.save)
    
    return 0

if __name__ == "__main__":
    main() 