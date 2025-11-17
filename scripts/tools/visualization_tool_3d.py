"""
Visualize lidar data collected from the drone simulation.

Usage:
    python visualize_lidar.py --data_dir lidar_data/run_20251116_234500
    python visualize_lidar.py --data_dir lidar_data/run_20251116_234500 --frame 10
    python visualize_lidar.py --data_dir lidar_data/run_20251116_234500 --animation
"""

import argparse
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
import json
import os
from pathlib import Path


def load_frame(data_dir, frame_id):
    """Load a single frame of lidar data."""
    frame_path = os.path.join(data_dir, "numpy", f"frame_{frame_id:06d}.npz")
    if not os.path.exists(frame_path):
        raise FileNotFoundError(f"Frame {frame_id} not found at {frame_path}")
    
    data = np.load(frame_path)
    return {
        'frame_id': int(data['frame_id']),
        'sim_time': float(data['sim_time']),
        'drone_position': data['drone_position'],
        'drone_quaternion': data['drone_quaternion'],
        'sensor_position': data['sensor_position'],
        'hit_positions': data['hit_positions'],
        'valid_mask': data['valid_mask'],
        'valid_hits': data['valid_hits']
    }


def load_metadata(data_dir):
    """Load metadata for the entire run."""
    metadata_path = os.path.join(data_dir, "metadata.json")
    with open(metadata_path, 'r') as f:
        return json.load(f)


def plot_single_frame(data_dir, frame_id, save_path=None):
    """Visualize a single frame of lidar data."""
    frame = load_frame(data_dir, frame_id)
    
    fig = plt.figure(figsize=(16, 12))
    
    # 3D point cloud view
    ax1 = fig.add_subplot(2, 2, 1, projection='3d')
    points = frame['valid_hits']
    sensor_pos = frame['sensor_position']
    drone_pos = frame['drone_position']
    
    # Calculate distances for color mapping
    distances = np.linalg.norm(points - sensor_pos, axis=1)
    
    # Plot point cloud colored by distance
    scatter = ax1.scatter(points[:, 0], points[:, 1], points[:, 2], 
                         c=distances, cmap='jet', s=1, alpha=0.6)
    
    # Plot sensor and drone position
    ax1.scatter(*sensor_pos, color='red', s=100, marker='^', label='Sensor')
    ax1.scatter(*drone_pos, color='green', s=100, marker='o', label='Drone')
    
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_zlabel('Z (m)')
    ax1.set_title(f'3D Point Cloud - Frame {frame_id}\nTime: {frame["sim_time"]:.2f}s')
    ax1.legend()
    plt.colorbar(scatter, ax=ax1, label='Distance (m)')
    
    # Top-down view (XY plane)
    ax2 = fig.add_subplot(2, 2, 2)
    ax2.scatter(points[:, 0], points[:, 1], c=distances, cmap='jet', s=1, alpha=0.6)
    ax2.scatter(sensor_pos[0], sensor_pos[1], color='red', s=100, marker='^', label='Sensor')
    ax2.scatter(drone_pos[0], drone_pos[1], color='green', s=100, marker='o', label='Drone')
    ax2.set_xlabel('X (m)')
    ax2.set_ylabel('Y (m)')
    ax2.set_title('Top-Down View (XY)')
    ax2.axis('equal')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    # Side view (XZ plane)
    ax3 = fig.add_subplot(2, 2, 3)
    ax3.scatter(points[:, 0], points[:, 2], c=distances, cmap='jet', s=1, alpha=0.6)
    ax3.scatter(sensor_pos[0], sensor_pos[2], color='red', s=100, marker='^', label='Sensor')
    ax3.scatter(drone_pos[0], drone_pos[2], color='green', s=100, marker='o', label='Drone')
    ax3.set_xlabel('X (m)')
    ax3.set_ylabel('Z (m)')
    ax3.set_title('Side View (XZ)')
    ax3.axis('equal')
    ax3.legend()
    ax3.grid(True, alpha=0.3)
    
    # Distance histogram
    ax4 = fig.add_subplot(2, 2, 4)
    ax4.hist(distances, bins=50, alpha=0.7, edgecolor='black')
    ax4.axvline(distances.mean(), color='red', linestyle='--', label=f'Mean: {distances.mean():.2f}m')
    ax4.axvline(np.median(distances), color='green', linestyle='--', label=f'Median: {np.median(distances):.2f}m')
    ax4.set_xlabel('Distance (m)')
    ax4.set_ylabel('Count')
    ax4.set_title('Distance Distribution')
    ax4.legend()
    ax4.grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"Saved visualization to {save_path}")
    else:
        plt.show()


def plot_trajectory(data_dir, save_path=None):
    """Plot the drone trajectory over time."""
    metadata = load_metadata(data_dir)
    
    positions = np.array([f['drone_position'] for f in metadata['frames']])
    times = np.array([f['sim_time'] for f in metadata['frames']])
    
    fig = plt.figure(figsize=(15, 10))
    
    # 3D trajectory
    ax1 = fig.add_subplot(2, 2, 1, projection='3d')
    scatter = ax1.scatter(positions[:, 0], positions[:, 1], positions[:, 2], 
                         c=times, cmap='viridis', s=20)
    ax1.plot(positions[:, 0], positions[:, 1], positions[:, 2], 
            'b-', alpha=0.3, linewidth=1)
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_zlabel('Z (m)')
    ax1.set_title('3D Trajectory')
    plt.colorbar(scatter, ax=ax1, label='Time (s)')
    
    # Top view
    ax2 = fig.add_subplot(2, 2, 2)
    scatter = ax2.scatter(positions[:, 0], positions[:, 1], c=times, cmap='viridis', s=20)
    ax2.plot(positions[:, 0], positions[:, 1], 'b-', alpha=0.3, linewidth=1)
    ax2.set_xlabel('X (m)')
    ax2.set_ylabel('Y (m)')
    ax2.set_title('Top View (XY)')
    ax2.axis('equal')
    ax2.grid(True, alpha=0.3)
    plt.colorbar(scatter, ax=ax2, label='Time (s)')
    
    # Position vs time
    ax3 = fig.add_subplot(2, 2, 3)
    ax3.plot(times, positions[:, 0], label='X', linewidth=2)
    ax3.plot(times, positions[:, 1], label='Y', linewidth=2)
    ax3.plot(times, positions[:, 2], label='Z', linewidth=2)
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Position (m)')
    ax3.set_title('Position Components vs Time')
    ax3.legend()
    ax3.grid(True, alpha=0.3)
    
    # Statistics over time
    ax4 = fig.add_subplot(2, 2, 4)
    valid_hits = [f['statistics']['valid_hits'] for f in metadata['frames']]
    hit_rates = [f['statistics']['hit_rate_percent'] for f in metadata['frames']]
    
    ax4_twin = ax4.twinx()
    line1 = ax4.plot(times, valid_hits, 'b-', label='Valid Hits', linewidth=2)
    line2 = ax4_twin.plot(times, hit_rates, 'r-', label='Hit Rate %', linewidth=2)
    
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Valid Hits', color='b')
    ax4_twin.set_ylabel('Hit Rate (%)', color='r')
    ax4.set_title('Lidar Statistics vs Time')
    ax4.tick_params(axis='y', labelcolor='b')
    ax4_twin.tick_params(axis='y', labelcolor='r')
    ax4.grid(True, alpha=0.3)
    
    lines = line1 + line2
    labels = [l.get_label() for l in lines]
    ax4.legend(lines, labels, loc='upper left')
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"Saved trajectory plot to {save_path}")
    else:
        plt.show()


def create_animation(data_dir, output_path=None):
    """Create an animation of the lidar data over time."""
    metadata = load_metadata(data_dir)
    num_frames = len(metadata['frames'])
    
    print(f"Creating animation with {num_frames} frames...")
    
    fig = plt.figure(figsize=(16, 8))
    ax1 = fig.add_subplot(1, 2, 1, projection='3d')
    ax2 = fig.add_subplot(1, 2, 2)
    
    def update(frame_idx):
        ax1.clear()
        ax2.clear()
        
        frame = load_frame(data_dir, frame_idx)
        points = frame['valid_hits']
        sensor_pos = frame['sensor_position']
        
        if len(points) == 0:
            return
        
        distances = np.linalg.norm(points - sensor_pos, axis=1)
        
        # 3D view
        ax1.scatter(points[:, 0], points[:, 1], points[:, 2], 
                   c=distances, cmap='jet', s=1, alpha=0.6, vmin=0, vmax=10)
        ax1.scatter(*sensor_pos, color='red', s=100, marker='^')
        ax1.set_xlabel('X (m)')
        ax1.set_ylabel('Y (m)')
        ax1.set_zlabel('Z (m)')
        ax1.set_xlim(-10, 10)
        ax1.set_ylim(-10, 10)
        ax1.set_zlim(0, 15)
        ax1.set_title(f'Frame {frame_idx} | Time: {frame["sim_time"]:.2f}s')
        
        # Top-down view
        ax2.scatter(points[:, 0], points[:, 1], c=distances, cmap='jet', 
                   s=1, alpha=0.6, vmin=0, vmax=10)
        ax2.scatter(sensor_pos[0], sensor_pos[1], color='red', s=100, marker='^')
        ax2.set_xlabel('X (m)')
        ax2.set_ylabel('Y (m)')
        ax2.set_xlim(-10, 10)
        ax2.set_ylim(-10, 10)
        ax2.set_aspect('equal')
        ax2.grid(True, alpha=0.3)
        ax2.set_title('Top-Down View')
    
    anim = FuncAnimation(fig, update, frames=num_frames, interval=100, repeat=True)
    
    if output_path:
        print(f"Saving animation to {output_path} (this may take a while)...")
        anim.save(output_path, writer='pillow', fps=10)
        print(f"Animation saved!")
    else:
        plt.show()


def print_summary(data_dir):
    """Print summary statistics for the run."""
    metadata = load_metadata(data_dir)
    
    print("\n" + "="*60)
    print("LIDAR DATA SUMMARY")
    print("="*60)
    print(f"Run timestamp: {metadata['start_time']}")
    print(f"Total frames: {len(metadata['frames'])}")
    
    if len(metadata['frames']) > 0:
        first_frame = metadata['frames'][0]
        last_frame = metadata['frames'][-1]
        
        print(f"Time range: {first_frame['sim_time']:.2f}s - {last_frame['sim_time']:.2f}s")
        print(f"Duration: {last_frame['sim_time'] - first_frame['sim_time']:.2f}s")
        
        # Calculate statistics
        valid_hits = [f['statistics']['valid_hits'] for f in metadata['frames']]
        hit_rates = [f['statistics']['hit_rate_percent'] for f in metadata['frames']]
        
        print(f"\nValid hits per frame:")
        print(f"  Min: {min(valid_hits)}")
        print(f"  Max: {max(valid_hits)}")
        print(f"  Mean: {np.mean(valid_hits):.0f}")
        
        print(f"\nHit rate:")
        print(f"  Min: {min(hit_rates):.1f}%")
        print(f"  Max: {max(hit_rates):.1f}%")
        print(f"  Mean: {np.mean(hit_rates):.1f}%")
        
        # Analyze distance statistics
        frames_with_stats = [f for f in metadata['frames'] if 'distance_mean' in f['statistics']]
        if frames_with_stats:
            distances_mean = [f['statistics']['distance_mean'] for f in frames_with_stats]
            distances_min = [f['statistics']['distance_min'] for f in frames_with_stats]
            distances_max = [f['statistics']['distance_max'] for f in frames_with_stats]
            
            print(f"\nDistance statistics:")
            print(f"  Overall min: {min(distances_min):.2f}m")
            print(f"  Overall max: {max(distances_max):.2f}m")
            print(f"  Average mean distance: {np.mean(distances_mean):.2f}m")
    
    print("="*60 + "\n")


def main():
    parser = argparse.ArgumentParser(description="Visualize lidar data from drone simulation")
    parser.add_argument("--data_dir", type=str, required=True, 
                       help="Path to lidar data directory")
    parser.add_argument("--frame", type=int, default=None,
                       help="Visualize specific frame (default: first frame)")
    parser.add_argument("--trajectory", action="store_true",
                       help="Plot trajectory")
    parser.add_argument("--animation", action="store_true",
                       help="Create animation")
    parser.add_argument("--save", type=str, default=None,
                       help="Save output to file")
    parser.add_argument("--summary", action="store_true",
                       help="Print summary statistics")
    
    args = parser.parse_args()
    
    if not os.path.exists(args.data_dir):
        print(f"Error: Directory not found: {args.data_dir}")
        return
    
    # Print summary
    if args.summary or (not args.frame and not args.trajectory and not args.animation):
        print_summary(args.data_dir)
    
    # Visualize specific frame
    if args.frame is not None:
        plot_single_frame(args.data_dir, args.frame, args.save)
    elif not args.trajectory and not args.animation and not args.summary:
        # Default: show first frame
        plot_single_frame(args.data_dir, 0, args.save)
    
    # Plot trajectory
    if args.trajectory:
        save_path = args.save if args.save else None
        plot_trajectory(args.data_dir, save_path)
    
    # Create animation
    if args.animation:
        output_path = args.save if args.save else "lidar_animation.gif"
        create_animation(args.data_dir, output_path)


if __name__ == "__main__":
    main()