#!/usr/bin/env python3
"""
Simple conversion of traj_lidar.txt to 2D CSV format
Only extracts essential navigation data: timestamp, x, y, yaw
"""

import numpy as np
import pandas as pd
import os

def quaternion_to_yaw(qx, qy, qz, qw):
    """Convert quaternion to yaw angle in radians"""
    return np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy**2 + qz**2))

def convert_traj_lidar_to_2d(input_file='traj_lidar.txt', output_file='traj_lidar_2d.csv'):
    """Convert traj_lidar.txt to simplified 2D CSV format"""
    
    if not os.path.exists(input_file):
        print(f"❌ Error: {input_file} not found!")
        return False
    
    try:
        print(f"📂 Loading {input_file}...")
        
        # Load TUM format data
        data = np.loadtxt(input_file)
        if data.shape[1] != 8:
            print(f"❌ Error: Expected 8 columns (TUM format), got {data.shape[1]}")
            return False
        
        # Extract data
        timestamps = data[:, 0]
        x, y = data[:, 1], data[:, 2]
        qx, qy, qz, qw = data[:, 4], data[:, 5], data[:, 6], data[:, 7]
        
        # Convert quaternion to yaw
        yaw_rad = quaternion_to_yaw(qx, qy, qz, qw)
        yaw_deg = np.degrees(yaw_rad)
        
        # Create simplified DataFrame
        df = pd.DataFrame({
            'timestamp': timestamps,
            'x': x,
            'y': y,
            'yaw_rad': yaw_rad,
            'yaw_deg': yaw_deg
        })
        
        # Save to CSV
        df.to_csv(output_file, index=False, float_format='%.6f')
        
        # Calculate statistics
        total_distance = np.sum(np.sqrt(np.diff(x)**2 + np.diff(y)**2))
        duration = timestamps[-1] - timestamps[0]
        displacement = np.sqrt((x[-1] - x[0])**2 + (y[-1] - y[0])**2)
        
        print(f"✅ Conversion successful!")
        print(f"   📊 {len(df)} poses over {duration:.1f} seconds")
        print(f"   📏 Path length: {total_distance:.2f} meters")
        print(f"   📐 Displacement: {displacement:.2f} meters")
        print(f"   📍 Start: ({x[0]:.3f}, {y[0]:.3f})")
        print(f"   🏁 End: ({x[-1]:.3f}, {y[-1]:.3f})")
        print(f"   💾 Saved to: {output_file}")
        
        return True
        
    except Exception as e:
        print(f"❌ Error converting {input_file}: {e}")
        return False

def main():
    print("🎯 Simple GLIM LiDAR Trajectory Converter")
    print("Converting traj_lidar.txt to 2D CSV format")
    print("=" * 45)
    
    success = convert_traj_lidar_to_2d()
    
    if success:
        print("\n✨ Conversion complete!")
        print("📄 CSV format: timestamp, x, y, yaw_rad, yaw_deg")
        print("🚀 Next: python3 plot_simple_trajectory.py")
    else:
        print("\n❌ Conversion failed!")
        return 1
    
    return 0

if __name__ == "__main__":
    main() 