import csv
import ast  # For safely evaluating string literals
import pandas as pd

# Pulse Sitting Down and Angles When Sitting Down (Radians) for each joint
pulse_sitting_down = {
    'front_left_shoulder': 1510,
    'front_right_shoulder': 1570,
    'rear_left_shoulder': 1630,
    'rear_right_shoulder': 1570,
    'front_left_leg': 2490,
    'front_right_leg': 620,
    'rear_left_leg': 2290,
    'rear_right_leg': 710,
    'front_left_foot': 410,
    'front_right_foot': 2730,
    'rear_left_foot': 460,
    'rear_right_foot': 2540
}

angle_sitting_down = {
    'front_left_shoulder': 1.5893254712295857e-08,
    'front_right_shoulder': 1.5893254712295857e-08,
    'rear_left_shoulder': 1.5893254712295857e-08,
    'rear_right_shoulder': 1.5893254712295857e-08,
    'front_left_leg': -1.235984206199646,
    'front_right_leg': -1.235984206199646,
    'rear_left_leg': -1.235984206199646,
    'rear_right_leg': -1.235984206199646,
    'front_left_foot': 2.512333393096924,
    'front_right_foot': 2.512333393096924,
    'rear_left_foot': 2.512333393096924,
    'rear_right_foot': 2.512333393096924
}

# Function to compute pulse position
def compute_pulse_position(joint_names, positions):
    joint_names = ast.literal_eval(joint_names)
    positions = ast.literal_eval(positions)
    
    pulse_positions = []
    for joint, pos in zip(joint_names, positions):
        if "shoulder" in joint:
            pulse_positions.append(pulse_sitting_down[joint])
        else:
            pulse_positions.append(pos * pulse_sitting_down[joint] / angle_sitting_down[joint])
    return pulse_positions

# Load the data
input_file = '/home/tototime/ros2_spok_ws/src/S7_G7_Perrichet_Sibenaler/software/spok_rob/spok_rob/joint_trajectory_data.csv'
output_file = '/home/tototime/ros2_spok_ws/src/S7_G7_Perrichet_Sibenaler/software/spok_rob/spok_rob/joint_trajectory_data_modified.csv'

data = pd.read_csv(input_file)

# Compute the pulse position for each row
data['pulse_position'] = data.apply(
    lambda row: compute_pulse_position(row['joint_names'], row['positions']), axis=1
)

# Save the modified data to a new file
data.to_csv(output_file, index=False)

print(f"Modified file saved as {output_file}")
