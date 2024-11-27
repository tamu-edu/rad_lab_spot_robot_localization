import pandas as pd
import matplotlib.pyplot as plt
import os

def plot_and_save(csv_file):
    # Read the CSV file into a Pandas DataFrame
    data = pd.read_csv(csv_file)

    # Convert timestamp to a time offset (start at 0 seconds)
    data['time'] = data['timestamp'] - data['timestamp'].iloc[0]

    # Create a directory for saving plots
    save_dir = "imu_twist_plots"
    os.makedirs(save_dir, exist_ok=True)

    # Plot and save IMU Euler angles
    plt.figure(figsize=(12, 6))
    plt.plot(data['time'], data['imu_roll'], label='Roll (rad)', linestyle='-', marker='o')
    plt.plot(data['time'], data['imu_pitch'], label='Pitch (rad)', linestyle='-', marker='o')
    plt.plot(data['time'], data['imu_yaw'], label='Yaw (rad)', linestyle='-', marker='o')
    plt.title("IMU Orientation (Euler Angles)")
    plt.xlabel("Time (s)")
    plt.ylabel("Angle (radians)")
    plt.legend()
    plt.grid()
    plt.savefig(os.path.join(save_dir, "imu_orientation_euler_angles.png"))
    plt.close()

    # Plot and save IMU angular velocity
    plt.figure(figsize=(12, 6))
    plt.plot(data['time'], data['imu_angular_velocity_x'], label='Angular Velocity X', linestyle='-', marker='o')
    plt.plot(data['time'], data['imu_angular_velocity_y'], label='Angular Velocity Y', linestyle='-', marker='o')
    plt.plot(data['time'], data['imu_angular_velocity_z'], label='Angular Velocity Z', linestyle='-', marker='o')
    plt.title("IMU Angular Velocity")
    plt.xlabel("Time (s)")
    plt.ylabel("Angular Velocity (rad/s)")
    plt.legend()
    plt.grid()
    plt.savefig(os.path.join(save_dir, "imu_angular_velocity.png"))
    plt.close()

    # Plot and save IMU linear acceleration
    plt.figure(figsize=(12, 6))
    plt.plot(data['time'], data['imu_linear_acceleration_x'], label='Linear Acceleration X', linestyle='-', marker='o')
    plt.plot(data['time'], data['imu_linear_acceleration_y'], label='Linear Acceleration Y', linestyle='-', marker='o')
    plt.plot(data['time'], data['imu_linear_acceleration_z'], label='Linear Acceleration Z', linestyle='-', marker='o')
    plt.title("IMU Linear Acceleration")
    plt.xlabel("Time (s)")
    plt.ylabel("Acceleration (m/s²)")
    plt.legend()
    plt.grid()
    plt.savefig(os.path.join(save_dir, "imu_linear_acceleration.png"))
    plt.close()

    # Plot and save Twist linear velocities
    plt.figure(figsize=(12, 6))
    plt.plot(data['time'], data['twist_linear_x'], label='Linear Velocity X', linestyle='-', marker='o')
    plt.plot(data['time'], data['twist_linear_y'], label='Linear Velocity Y', linestyle='-', marker='o')
    plt.plot(data['time'], data['twist_linear_z'], label='Linear Velocity Z', linestyle='-', marker='o')
    plt.title("Twist Linear Velocities")
    plt.xlabel("Time (s)")
    plt.ylabel("Velocity (m/s)")
    plt.legend()
    plt.grid()
    plt.savefig(os.path.join(save_dir, "twist_linear_velocities.png"))
    plt.close()

    # Plot and save Twist angular velocities
    plt.figure(figsize=(12, 6))
    plt.plot(data['time'], data['twist_angular_x'], label='Angular Velocity X', linestyle='-', marker='o')
    plt.plot(data['time'], data['twist_angular_y'], label='Angular Velocity Y', linestyle='-', marker='o')
    plt.plot(data['time'], data['twist_angular_z'], label='Angular Velocity Z', linestyle='-', marker='o')
    plt.title("Twist Angular Velocities")
    plt.xlabel("Time (s)")
    plt.ylabel("Angular Velocity (rad/s)")
    plt.legend()
    plt.grid()
    plt.savefig(os.path.join(save_dir, "twist_angular_velocities.png"))
    plt.close()

    print(f"Plots saved in directory: {save_dir}")


if __name__ == "__main__":
    # Path to the CSV file
    csv_file = 'imu_twist_data_20241118_133712.csv'  # Replace with the actual file name

    try:
        plot_and_save(csv_file)
    except FileNotFoundError:
        print(f"File {csv_file} not found. Ensure the CSV file path is correct.")
