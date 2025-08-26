import os
import argparse
import struct

import numpy as np
import dpkt
import rosbag2_py
import rclpy
from rclpy.serialization import serialize_message
from ouster.sdk import client, pcap
from sensor_msgs.msg import PointCloud2, PointField, Imu
from std_msgs.msg import Header

def apply_coordinate_transform(points, transform_matrix):
    """
    Apply coordinate transformation matrix to points
    transform_matrix: 4x4 homogeneous transformation matrix
    """
    # Convert to homogeneous coordinates
    points_homo = np.column_stack([points, np.ones(len(points))])
    
    # Apply transformation
    transformed_homo = (transform_matrix @ points_homo.T).T
    
    # Convert back to 3D coordinates
    return transformed_homo[:, :3]

def calculate_exact_timing_os0_32(points):
    """
    Calculate exact timing for Ouster OS-0-32 at 10Hz
    Based on your metadata: 2048 columns, 10Hz rotation = 0.1s per scan
    """
    n_points = len(points)
    timing = np.zeros(n_points, dtype=np.float32)
    
    # Exact specifications for your Ouster OS-0-32
    SCAN_DURATION = 0.1          # 10 Hz = 0.1 seconds per rotation
    COLS_PER_FRAME = 2048        # From your metadata
    BEAMS_PER_COL = 32           # OS-0-32 = 32 beams
    TIME_PER_COL = SCAN_DURATION / COLS_PER_FRAME  # 48.828 microseconds
    
    print(f"DEBUG: n_points={n_points}, expected={COLS_PER_FRAME * BEAMS_PER_COL}")
    print(f"DEBUG: TIME_PER_COL={TIME_PER_COL*1e6:.3f} microseconds")
    
    # Expected points for full scan: 32 beams × 2048 columns = 65,536 points
    if n_points == COLS_PER_FRAME * BEAMS_PER_COL:
        print("DEBUG: Using column-based timing (exact)")
        # Column-based timing (most accurate)
        for i in range(n_points):
            col_idx = i // BEAMS_PER_COL  # Which azimuth column (0-2047)
            timing[i] = col_idx * TIME_PER_COL
    else:
        print("DEBUG: Using linear timing fallback")
        # Fallback: linear timing for different point counts
        for i in range(n_points):
            timing[i] = (i / n_points) * SCAN_DURATION
    
    # Debug: Show some timing values
    if n_points > 0:
        print(f"DEBUG: timing[0]={timing[0]:.6f}s, timing[100]={timing[100]:.6f}s")
        if n_points > 1000:
            print(f"DEBUG: timing[1000]={timing[1000]:.6f}s, timing[-1]={timing[-1]:.6f}s")
    
    return timing

def create_pointcloud2(points, intensity, reflectivity, nearir, timestamp_ns, frame_id="os_sensor", transform_to_lidar=False):
    """
    Build a sensor_msgs/PointCloud2 message containing fields:
      x y z intensity reflectivity nearir t
    points: (N,3) float32
    intensity: (N,) uint16
    reflectivity: (N,) uint8
    nearir: (N,) uint16
    transform_to_lidar: if True, apply sensor->lidar coordinate transformation
    """
    N = points.shape[0]
    
    # Apply coordinate transformation if requested
    if transform_to_lidar:
        # Sensor -> Lidar transformation (inverse of lidar_to_sensor)
        # lidar_to_sensor = [-1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1, 36.18, 0, 0, 0, 1]
        sensor_to_lidar = np.array([
            [-1, 0, 0, 0],
            [0, -1, 0, 0],  
            [0, 0, 1, -36.18],
            [0, 0, 0, 1]
        ])
        points = apply_coordinate_transform(points, sensor_to_lidar)
        print(f"Applied sensor->lidar transformation (180° rotation)")
    
    # Calculate exact timing for each point (RELATIVE to frame start)
    point_timing = calculate_exact_timing_os0_32(points)
    
    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.UINT16, count=1),
        PointField(name="reflectivity", offset=14, datatype=PointField.UINT8, count=1),
        PointField(name="nearir", offset=15, datatype=PointField.UINT16, count=1),
        PointField(name="t", offset=17, datatype=PointField.FLOAT32, count=1),  # RELATIVE TIMING
    ]
    point_step = 21  # Updated: 4+4+4+2+1+2+4 = 21 bytes per point
    row_step = point_step * N

    buffer = bytearray(N * point_step)
    for i in range(N):
        x, y, z = points[i]
        it = int(intensity[i])
        rf = int(reflectivity[i])
        ni = int(nearir[i])
        t = float(point_timing[i])  # Relative timing in seconds
        struct.pack_into("<fffHBHf", buffer, i * point_step, x, y, z, it, rf, ni, t)

    sec = int(timestamp_ns // 1_000_000_000)
    nanosec = int(timestamp_ns % 1_000_000_000)
    header = Header()
    header.stamp.sec = sec
    header.stamp.nanosec = nanosec
    header.frame_id = frame_id

    pc2 = PointCloud2()
    pc2.header = header
    pc2.height = 1
    pc2.width = N
    pc2.fields = fields
    pc2.is_bigendian = False
    pc2.point_step = point_step
    pc2.row_step = row_step
    pc2.is_dense = False
    pc2.data = bytes(buffer)
    return pc2

def create_imu_msg(ax, ay, az, wx, wy, wz, timestamp_ns, frame_id="os_imu"):
    sec = int(timestamp_ns // 1_000_000_000)
    nanosec = int(timestamp_ns % 1_000_000_000)
    header = Header()
    header.stamp.sec = sec
    header.stamp.nanosec = nanosec
    header.frame_id = frame_id

    imu = Imu()
    imu.header = header
    
    imu.orientation_covariance = [-1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0]
    imu.orientation.x = 0.0
    imu.orientation.y = 0.0
    imu.orientation.z = 0.0
    imu.orientation.w = 0.0
    
    imu.angular_velocity.x = wx
    imu.angular_velocity.y = wy
    imu.angular_velocity.z = wz
    imu.angular_velocity_covariance = [0.0006, 0.0, 0.0, 0.0, 0.0006, 0.0, 0.0, 0.0, 0.0006]
    
    imu.linear_acceleration.x = ax
    imu.linear_acceleration.y = ay
    imu.linear_acceleration.z = az
    imu.linear_acceleration_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
    
    return imu

def main():
    parser = argparse.ArgumentParser(
        description="Convert PCAP → ROS 2 bag with PointCloud2 and Imu topics."
    )
    parser.add_argument("pcap_path", help="Path to input PCAP file")
    parser.add_argument("--transform", action="store_true", help="Apply sensor->lidar coordinate transformation")
    args = parser.parse_args()
    pcap_path = args.pcap_path
    transform_to_lidar = args.transform

    rclpy.init()

    bag_name = "pcap_ros2bag_glim"
    storage_opts = rosbag2_py._storage.StorageOptions(uri=bag_name, storage_id="sqlite3")
    converter_opts = rosbag2_py._storage.ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )
    writer = rosbag2_py.SequentialWriter()
    writer.open(storage_opts, converter_opts)

    print(f"=== COORDINATE FRAME TEST ===")
    print(f"Transform to lidar frame: {transform_to_lidar}")
    if transform_to_lidar:
        print("  - Will apply 180° rotation (sensor->lidar)")
        print("  - X will point backward, Y at 90° (lidar frame)")
    else:
        print("  - Using sensor coordinate frame (default xyz_lut)")
        print("  - X will point forward, Y left (sensor frame)")
    print("================================")

    writer.create_topic(
        rosbag2_py._storage.TopicMetadata(
            name="/os_cloud_node/points", type="sensor_msgs/msg/PointCloud2", serialization_format="cdr"
        )
    )
    writer.create_topic(
        rosbag2_py._storage.TopicMetadata(
            name="/os_cloud_node/imu", type="sensor_msgs/msg/Imu", serialization_format="cdr"
        )
    )

    scan_source = pcap.PcapScanSource(pcap_path).single_source(0)
    metadata = scan_source.metadata
    xyz_lut = client.XYZLut(metadata)
    pkt_fmt = client.PacketFormat(metadata)

    cols_frame = metadata.format.columns_per_frame
    cols_packet = metadata.format.columns_per_packet
    packets_per_scan = cols_frame // cols_packet

    lidar_timestamps = []

    with open(pcap_path, "rb") as f_pcap:
        dpkt_reader = dpkt.pcap.Reader(f_pcap)
        ouster_reader = pcap.PcapMultiPacketReader(pcap_path).single_source(0)

        for (ts_sec, _), packet in zip(dpkt_reader, ouster_reader):
            pcap_ts_ns = int(ts_sec * 1e9)

            if isinstance(packet, client.LidarPacket):
                lidar_timestamps.append(pcap_ts_ns)

            elif isinstance(packet, client.ImuPacket):
                ax = pkt_fmt.imu_la_x(packet.buf)
                ay = pkt_fmt.imu_la_y(packet.buf)
                az = pkt_fmt.imu_la_z(packet.buf)
                wx = pkt_fmt.imu_av_x(packet.buf)
                wy = pkt_fmt.imu_av_y(packet.buf)
                wz = pkt_fmt.imu_av_z(packet.buf)

                imu_msg = create_imu_msg(ax, ay, az, wx, wy, wz, pcap_ts_ns)
                serialized_imu = serialize_message(imu_msg)
                writer.write("/os_cloud_node/imu", serialized_imu, pcap_ts_ns)

    total_lidar_packets = len(lidar_timestamps)
    if total_lidar_packets < packets_per_scan:
        raise RuntimeError("Not enough LidarPackets for a single scan.")

    num_scans = total_lidar_packets // packets_per_scan

    scan_source = pcap.PcapScanSource(pcap_path).single_source(0)

    for idx, scan in enumerate(scan_source):
        if idx >= num_scans:
            break

        start_idx = idx * packets_per_scan
        last_idx = start_idx + packets_per_scan - 1
        first_ts = lidar_timestamps[start_idx]
        last_ts = lidar_timestamps[last_idx]
        mid_ts = (first_ts + last_ts) // 2

        xyz = xyz_lut(scan.field(client.ChanField.RANGE))
        points = xyz.reshape(-1, 3)

        # Diagnostic: print some point coordinates for first scan
        if idx == 0:
            print(f"\nFirst scan diagnostics (BEFORE transformation):")
            print(f"Original points from xyz_lut (sensor frame):")
            print(f"  Point 0: {points[0]}")
            print(f"  Point 1000: {points[1000]}")
            print(f"  Point -1: {points[-1]}")
            
            # CRITICAL: Check raw range data
            range_data = scan.field(client.ChanField.RANGE)
            print(f"\nRAW RANGE DATA DIAGNOSTICS:")
            print(f"  Range data shape: {range_data.shape}")
            print(f"  Range data type: {range_data.dtype}")
            print(f"  Range min: {np.min(range_data)}")
            print(f"  Range max: {np.max(range_data)}")
            print(f"  Range mean: {np.mean(range_data)}")
            print(f"  Non-zero ranges: {np.count_nonzero(range_data)}/{range_data.size}")
            print(f"  Sample range values: {range_data.flatten()[:10]}")
            
            # Find indices with non-zero ranges
            range_flat = range_data.flatten()
            nonzero_indices = np.nonzero(range_flat)[0]
            if len(nonzero_indices) > 0:
                print(f"\nNON-ZERO RANGE ANALYSIS:")
                print(f"  First 5 non-zero range indices: {nonzero_indices[:5]}")
                print(f"  Their range values: {range_flat[nonzero_indices[:5]]}")
                print(f"  Corresponding XYZ points:")
                for i, idx in enumerate(nonzero_indices[:5]):
                    print(f"    Index {idx}: range={range_flat[idx]}mm → xyz={points[idx]}")
                
                # Check if ALL xyz points are zero despite valid ranges
                xyz_nonzero = np.count_nonzero(points)
                print(f"  Non-zero XYZ coordinates: {xyz_nonzero}/{points.size}")
                if xyz_nonzero == 0:
                    print(f"  ❌ CRITICAL: xyz_lut converts valid ranges to ALL ZEROS!")
                    print(f"  This is the root cause of the mapping problem")
                else:
                    print(f"  ✅ xyz_lut is working correctly")
            
            if np.count_nonzero(range_data) == 0:
                print(f"  ❌ PROBLEM: All range values are ZERO!")
                print(f"  This explains why xyz_lut returns all zeros")
            else:
                print(f"  ✅ Range data looks valid")

        refl = scan.field(client.ChanField.REFLECTIVITY).flatten()
        intensity = scan.field(client.ChanField.SIGNAL).flatten()
        nearir = scan.field(client.ChanField.NEAR_IR).flatten()

        pc2_msg = create_pointcloud2(points, intensity, refl, nearir, mid_ts, transform_to_lidar=transform_to_lidar)
        serialized_pc2 = serialize_message(pc2_msg)
        writer.write("/os_cloud_node/points", serialized_pc2, mid_ts)

        print(f"[{idx}] Wrote PointCloud2 at timestamp {mid_ts}")

    print(f"Finished writing bag: {bag_name}")
    rclpy.shutdown()

if __name__ == "__main__":
    main()
