#!/usr/bin/env python3

import matplotlib.pyplot as plt
import pandas as pd

# Function to plot velocity data in one giant figure with 6 subplots
def plot_velocity_data():
    # Read the CSV file
    vel_data = pd.read_csv('vel_data.csv')

    # Create a figure with 6 subplots for both linear and angular velocity data
    fig, axs = plt.subplots(6, 1, figsize=(10, 12))  # Adjusted the height
    fig.suptitle('/mcu/state/vel', fontsize=16)

    # Plot vx, vy, vz on separate subplots
    axs[0].plot(vel_data['time'], vel_data['vx'], label='vx', color='r')
    axs[0].set_title('vx')
    axs[0].set_ylabel('vx')

    axs[1].plot(vel_data['time'], vel_data['vy'], label='vy', color='g')
    axs[1].set_title('vy')
    axs[1].set_ylabel('vy')

    axs[2].plot(vel_data['time'], vel_data['vz'], label='vz', color='b')
    axs[2].set_title('vz')
    axs[2].set_ylabel('vz')

    # Plot vroll, vpitch, vyaw on separate subplots
    axs[3].plot(vel_data['time'], vel_data['vroll'], label='vroll', color='r')
    axs[3].set_title('vroll')
    axs[3].set_ylabel('vroll')

    axs[4].plot(vel_data['time'], vel_data['vpitch'], label='vpitch', color='g')
    axs[4].set_title('vpitch')
    axs[4].set_ylabel('vpitch')

    axs[5].plot(vel_data['time'], vel_data['vyaw'], label='vyaw', color='b')
    axs[5].set_title('vyaw')
    axs[5].set_ylabel('vyaw')
    axs[5].set_xlabel('Time (ns)')

    # Rotate the x-axis labels for all subplots
    for ax in axs:
        ax.tick_params(axis='x', rotation=45)

    plt.tight_layout()
    plt.subplots_adjust(top=0.93)  # Add some space for the figure title

    # Save the figure to a file
    plt.savefig('velocity_data_plot.png')

    # # Show the plot
    # plt.show()

# Function to plot IMU orientation and angular velocity data in one figure
def plot_imu_orientation_angular_velocity():
    # Read the CSV file
    imu_data = pd.read_csv('imu_data.csv')

    # Create a figure with 6 subplots for orientation and angular velocity data
    fig, axs = plt.subplots(6, 1, figsize=(10, 12))
    fig.suptitle('/mcu/state/imu - Orientation and Angular Velocity', fontsize=16)

    # Plot roll, pitch, yaw on separate subplots
    axs[0].plot(imu_data['time'], imu_data['roll'], label='roll', color='r')
    axs[0].set_title('roll')
    axs[0].set_ylabel('roll')

    axs[1].plot(imu_data['time'], imu_data['pitch'], label='pitch', color='g')
    axs[1].set_title('pitch')
    axs[1].set_ylabel('pitch')

    axs[2].plot(imu_data['time'], imu_data['yaw'], label='yaw', color='b')
    axs[2].set_title('yaw')
    axs[2].set_ylabel('yaw')

    # Plot angular velocity components (x, y, z) on separate subplots
    axs[3].plot(imu_data['time'], imu_data['angular_velocity_x'], label='Angular Velocity X', color='r')
    axs[3].set_title('vroll')
    axs[3].set_ylabel('vroll')

    axs[4].plot(imu_data['time'], imu_data['angular_velocity_y'], label='Angular Velocity Y', color='g')
    axs[4].set_title('vpitch')
    axs[4].set_ylabel('vpitch')

    axs[5].plot(imu_data['time'], imu_data['angular_velocity_z'], label='Angular Velocity Z', color='b')
    axs[5].set_title('vyaw')
    axs[5].set_ylabel('Angular Velocity Z (rad/s)')
    axs[5].set_xlabel('vyaw')

    # Rotate the x-axis labels for all subplots
    for ax in axs:
        ax.tick_params(axis='x', rotation=45)

    plt.tight_layout()
    plt.subplots_adjust(top=0.93)  # Add some space for the figure title

    # Save the figure to a file
    plt.savefig('imu_orientation_angular_velocity_plot.png')

    # # Show the plot
    # plt.show()

# Function to plot IMU linear acceleration data in a separate figure
def plot_imu_acceleration():
    # Read the CSV file
    imu_data = pd.read_csv('imu_data.csv')

    # Create a figure with 3 subplots for linear acceleration data
    fig, axs = plt.subplots(3, 1, figsize=(10, 12))
    fig.suptitle('/mcu/state/imu - Linear Acceleration', fontsize=16)

    # Plot linear acceleration components (x, y, z) on separate subplots
    axs[0].plot(imu_data['time'], imu_data['linear_acceleration_x'], label='ax', color='r')
    axs[0].set_title('ax')
    axs[0].set_ylabel('ax')

    axs[1].plot(imu_data['time'], imu_data['linear_acceleration_y'], label='ay', color='g')
    axs[1].set_title('ay')
    axs[1].set_ylabel('ay')

    axs[2].plot(imu_data['time'], imu_data['linear_acceleration_z'], label='az', color='b')
    axs[2].set_title('az')
    axs[2].set_ylabel('az')
    axs[2].set_xlabel('Time (ns)')

    # Rotate the x-axis labels for all subplots
    for ax in axs:
        ax.tick_params(axis='x', rotation=45)

    plt.tight_layout()
    plt.subplots_adjust(top=0.93)  # Add some space for the figure title

    # Save the figure to a file
    plt.savefig('imu_acceleration_plot.png')

    # # Show the plot
    # plt.show()

def plot_odometry_translation(overlay=False, file_to_overlay='odom_data.csv'):
    # Read the CSV file
    odom_data = pd.read_csv('odom_data.csv')

    # Create a figure with 6 subplots for orientation and angular velocity data
    fig, axs = plt.subplots(6, 1, figsize=(10, 12))
    fig.suptitle('/odometry/filtered - Fused Translation Output', fontsize=16)

    if overlay:
        odom_overlay_data = pd.read_csv(file_to_overlay)

    # Plot roll, pitch, yaw on separate subplots
    axs[0].plot(odom_data['time'], odom_data['x'], label='x', color='r')
    if overlay:
        axs[0].plot(odom_overlay_data['time'], odom_overlay_data['x'], label='with_imu_modification', color='r', linestyle='--')
    axs[0].set_title('x')
    axs[0].set_ylabel('x')

    axs[1].plot(odom_data['time'], odom_data['y'], label='y', color='g')
    if overlay:
        axs[1].plot(odom_overlay_data['time'], odom_overlay_data['y'], label='with_imu_modification', color='g', linestyle='--')
    axs[1].set_title('y')
    axs[1].set_ylabel('y')

    axs[2].plot(odom_data['time'], odom_data['z'], label='z', color='b')
    if overlay:
        axs[2].plot(odom_overlay_data['time'], odom_overlay_data['z'], label='with_imu_modification', color='b', linestyle='--')
    axs[2].set_title('z')
    axs[2].set_ylabel('z')

    # Plot angular velocity components (x, y, z) on separate subplots
    axs[3].plot(odom_data['time'], odom_data['vx'], label='vx', color='r')
    if overlay:
        axs[3].plot(odom_overlay_data['time'], odom_overlay_data['vx'], label='with_imu_modification', color='r', linestyle='--')
    axs[3].set_title('vx')
    axs[3].set_ylabel('vx')

    axs[4].plot(odom_data['time'], odom_data['vy'], label='vy', color='g')
    if overlay:
        axs[4].plot(odom_overlay_data['time'], odom_overlay_data['vy'], label='with_imu_modification', color='g', linestyle='--')
    axs[4].set_title('vy')
    axs[4].set_ylabel('vy')

    axs[5].plot(odom_data['time'], odom_data['vz'], label='vz', color='b')
    if overlay:
        axs[5].plot(odom_overlay_data['time'], odom_overlay_data['vz'], label='with_imu_modification', color='b', linestyle='--')
    axs[5].set_title('vz')
    axs[5].set_ylabel('vz')
    axs[5].set_xlabel('Time (ns)')

    # Rotate the x-axis labels for all subplots
    for ax in axs:
        ax.tick_params(axis='x', rotation=45)

    plt.tight_layout()
    plt.subplots_adjust(top=0.93)  # Add some space for the figure title

    # Save the figure to a file
    plt.savefig('odometry_translation_plot.png')

def plot_odometry_orientation(overlay=False, file_to_overlay='odom_data.csv'):
    # Read the CSV file
    odom_data = pd.read_csv('odom_data.csv')

    # Create a figure with 6 subplots for orientation and angular velocity data
    fig, axs = plt.subplots(6, 1, figsize=(10, 12))
    fig.suptitle('/odometry/filtered - Fused Orientation Output', fontsize=16)

    if overlay:
        odom_overlay_data = pd.read_csv(file_to_overlay)

    # Plot roll, pitch, yaw on separate subplots
    axs[0].plot(odom_data['time'], odom_data['roll'], label='roll', color='r')
    if overlay:
        axs[0].plot(odom_overlay_data['time'], odom_overlay_data['roll'], label='with_imu_modification', color='r', linestyle='--')
    axs[0].set_title('roll')
    axs[0].set_ylabel('roll')

    axs[1].plot(odom_data['time'], odom_data['pitch'], label='pitch', color='g')
    if overlay:
        axs[1].plot(odom_overlay_data['time'], odom_overlay_data['pitch'], label='with_imu_modification', color='g', linestyle='--')
    axs[1].set_title('pitch')
    axs[1].set_ylabel('pitch')

    axs[2].plot(odom_data['time'], odom_data['yaw'], label='z', color='b')
    if overlay:
        axs[2].plot(odom_overlay_data['time'], odom_overlay_data['yaw'], label='with_imu_modification', color='b', linestyle='--')
    axs[2].set_title('yaw')
    axs[2].set_ylabel('yaw')

    # Plot angular velocity components (x, y, z) on separate subplots
    axs[3].plot(odom_data['time'], odom_data['vroll'], label='vroll', color='r')
    if overlay:
        axs[3].plot(odom_overlay_data['time'], odom_overlay_data['vroll'], label='with_imu_modification', color='r', linestyle='--')
    axs[3].set_title('vroll')
    axs[3].set_ylabel('vroll')

    axs[4].plot(odom_data['time'], odom_data['vpitch'], label='vpitch', color='g')
    if overlay:
        axs[4].plot(odom_overlay_data['time'], odom_overlay_data['vpitch'], label='with_imu_modification', color='g', linestyle='--')
    axs[4].set_title('vpitch')
    axs[4].set_ylabel('vpitch')

    axs[5].plot(odom_data['time'], odom_data['vyaw'], label='vyaw', color='b')
    if overlay:
        axs[5].plot(odom_overlay_data['time'], odom_overlay_data['vyaw'], label='with_imu_modification', color='', linestyle='--')
    axs[5].set_title('vyaw')
    axs[5].set_ylabel('vyaw')
    axs[5].set_xlabel('Time (ns)')

    # Rotate the x-axis labels for all subplots
    for ax in axs:
        ax.tick_params(axis='x', rotation=45)

    plt.tight_layout()
    plt.subplots_adjust(top=0.93)  # Add some space for the figure title

    # Save the figure to a file
    plt.savefig('odometry_orientation_plot.png')

    
if __name__ == '__main__':

    overlay_odom_data = False
    overlay_file = 'odom_data_imu_vroll.csv'

    # Plot and save velocity data
    plot_velocity_data()

    # Plot and save IMU orientation and angular velocity data
    plot_imu_orientation_angular_velocity()

    # Plot and save IMU linear acceleration data
    plot_imu_acceleration()

    # Plot and save odometry translation data
    plot_odometry_translation()

    # Plot and save odometry orientation data
    plot_odometry_orientation()

