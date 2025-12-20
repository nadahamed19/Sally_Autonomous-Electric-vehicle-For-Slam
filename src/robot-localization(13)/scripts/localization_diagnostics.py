#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
from collections import deque
import time

class LocalizationDiagnostics:
    """Diagnostics and monitoring for localization system"""
    
    def __init__(self, history_size=1000):
        self.history_size = history_size
        
        # Data storage
        self.timestamps = deque(maxlen=history_size)
        self.positions = deque(maxlen=history_size)
        self.orientations = deque(maxlen=history_size)
        self.velocities = deque(maxlen=history_size)
        self.covariances = deque(maxlen=history_size)
        
        # Error tracking
        self.position_errors = deque(maxlen=history_size)
        self.orientation_errors = deque(maxlen=history_size)
        
        # Performance metrics
        self.update_rates = deque(maxlen=100)
        self.last_update_time = time.time()
    
    def update(self, state, covariance, ground_truth=None):
        """Update diagnostics with new state estimate"""
        current_time = time.time()
        
        # Store data
        self.timestamps.append(current_time)
        self.positions.append([state[0], state[1]])
        self.orientations.append(state[2])
        self.velocities.append([state[3], state[4], state[5]])
        self.covariances.append(np.diag(covariance)[:3])  # x, y, theta variances
        
        # Calculate update rate
        dt = current_time - self.last_update_time
        if dt > 0:
            self.update_rates.append(1.0 / dt)
        self.last_update_time = current_time
        
        # Calculate errors if ground truth is available
        if ground_truth is not None:
            pos_error = np.sqrt((state[0] - ground_truth[0])**2 + 
                               (state[1] - ground_truth[1])**2)
            ori_error = abs(self.normalize_angle(state[2] - ground_truth[2]))
            
            self.position_errors.append(pos_error)
            self.orientation_errors.append(ori_error)
    
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        return np.arctan2(np.sin(angle), np.cos(angle))
    
    def get_statistics(self):
        """Get current performance statistics"""
        stats = {}
        
        if len(self.update_rates) > 0:
            stats['update_rate'] = {
                'mean': np.mean(self.update_rates),
                'std': np.std(self.update_rates),
                'min': np.min(self.update_rates),
                'max': np.max(self.update_rates)
            }
        
        if len(self.position_errors) > 0:
            stats['position_error'] = {
                'mean': np.mean(self.position_errors),
                'std': np.std(self.position_errors),
                'rms': np.sqrt(np.mean(np.array(self.position_errors)**2))
            }
        
        if len(self.orientation_errors) > 0:
            stats['orientation_error'] = {
                'mean': np.mean(self.orientation_errors),
                'std': np.std(self.orientation_errors),
                'rms': np.sqrt(np.mean(np.array(self.orientation_errors)**2))
            }
        
        if len(self.covariances) > 0:
            latest_cov = self.covariances[-1]
            stats['uncertainty'] = {
                'position_std': np.sqrt(latest_cov[0] + latest_cov[1]),
                'orientation_std': np.sqrt(latest_cov[2])
            }
        
        return stats
    
    def plot_trajectory(self, save_path=None):
        """Plot robot trajectory"""
        if len(self.positions) < 2:
            print("Not enough data to plot trajectory")
            return
        
        positions = np.array(self.positions)
        
        plt.figure(figsize=(10, 8))
        plt.plot(positions[:, 0], positions[:, 1], 'b-', linewidth=2, label='Estimated Trajectory')
        plt.scatter(positions[0, 0], positions[0, 1], color='green', s=100, label='Start', zorder=5)
        plt.scatter(positions[-1, 0], positions[-1, 1], color='red', s=100, label='End', zorder=5)
        
        plt.xlabel('X Position (m)')
        plt.ylabel('Y Position (m)')
        plt.title('Robot Trajectory')
        plt.legend()
        plt.grid(True, alpha=0.3)
        plt.axis('equal')
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
        plt.show()
    
    def plot_errors(self, save_path=None):
        """Plot localization errors over time"""
        if len(self.position_errors) == 0:
            print("No error data available")
            return
        
        times = np.array(self.timestamps[-len(self.position_errors):])
        times = times - times[0]  # Relative time
        
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))
        
        # Position errors
        ax1.plot(times, self.position_errors, 'r-', linewidth=2)
        ax1.set_ylabel('Position Error (m)')
        ax1.set_title('Localization Errors')
        ax1.grid(True, alpha=0.3)
        
        # Orientation errors
        ax2.plot(times, np.degrees(self.orientation_errors), 'b-', linewidth=2)
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Orientation Error (deg)')
        ax2.grid(True, alpha=0.3)
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
        plt.show()
    
    def print_summary(self):
        """Print performance summary"""
        stats = self.get_statistics()
        
        print("\n=== Localization Performance Summary ===")
        
        if 'update_rate' in stats:
            print(f"Update Rate: {stats['update_rate']['mean']:.1f} ± {stats['update_rate']['std']:.1f} Hz")
        
        if 'position_error' in stats:
            print(f"Position Error (RMS): {stats['position_error']['rms']:.3f} m")
        
        if 'orientation_error' in stats:
            print(f"Orientation Error (RMS): {np.degrees(stats['orientation_error']['rms']):.1f} deg")
        
        if 'uncertainty' in stats:
            print(f"Current Uncertainty - Position: {stats['uncertainty']['position_std']:.3f} m, "
                  f"Orientation: {np.degrees(stats['uncertainty']['orientation_std']):.1f} deg")
        
        print("=" * 45)

# Example usage
if __name__ == "__main__":
    # Create diagnostics instance
    diagnostics = LocalizationDiagnostics()
    
    # Simulate some data
    for i in range(100):
        # Simulate state [x, y, theta, vx, vy, omega]
        state = np.array([i * 0.1, np.sin(i * 0.1), i * 0.01, 0.1, 0.0, 0.01])
        
        # Simulate covariance
        covariance = np.eye(6) * 0.01
        
        # Simulate ground truth (with some error)
        ground_truth = [state[0] + np.random.normal(0, 0.02), 
                       state[1] + np.random.normal(0, 0.02),
                       state[2] + np.random.normal(0, 0.01)]
        
        diagnostics.update(state, covariance, ground_truth)
        time.sleep(0.01)  # Simulate real-time updates
    
    # Print summary and plot results
    diagnostics.print_summary()
    diagnostics.plot_trajectory()
    diagnostics.plot_errors()
