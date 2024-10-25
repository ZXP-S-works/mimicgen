import mujoco
import mujoco_py  # Import the MuJoCo Python library
import numpy as np
from scipy.spatial.transform import Rotation as R  # For converting rotations to quaternions
# Load a basic environment XML file
model = mujoco_py.load_model_from_path("path/to/your_model.xml")
sim = mujoco_py.MjSim(model)
# Function to add or modify a box with customizable size, pose, and orientation
def create_or_update_box(sim, body_name="dynamic_box", pos=(0, 0, 1), size=(0.1, 0.1, 0.1), rgba=(1, 1, 1, 1), euler=(0, 0, 0)):
    # Convert Euler angles to a quaternion
    quat = R.from_euler('xyz', euler, degrees=True).as_quat()
    # Check if body exists; if not, add it
    if body_name not in sim.model.body_names:
        # Add a new body and geom
        box_body_id = sim.model.body_name2id(body_name)
        sim.model.body_pos[box_body_id] = np.array(pos)
        sim.model.geom_size[box_body_id] = np.array(size)
        sim.model.geom_rgba[box_body_id] = np.array(rgba)
        sim.model.body_quat[box_body_id] = np.array(quat)  # Set the rotation
    else:
        # Update existing body position, size, color, and rotation
        box_body_id = sim.model.body_name2id(body_name)
        sim.model.body_pos[box_body_id] = np.array(pos)
        sim.model.geom_size[box_body_id] = np.array(size)
        sim.model.geom_rgba[box_body_id] = np.array(rgba)
        sim.model.body_quat[box_body_id] = np.array(quat)  # Update the rotation
# Set the box properties
pos = (1.0, 1.0, 1.0)     # Position (x, y, z)
size = (0.5, 0.5, 0.5)     # Size in x, y, z dimensions
rgba = (1, 1, 1, 1)        # Color: white with full opacity
euler = (45, 30, 0)        # Rotation in degrees (roll, pitch, yaw)
# Create or update the box in the simulation
create_or_update_box(sim, body_name="dynamic_box", pos=pos, size=size, rgba=rgba, euler=euler)
# Run the simulation
sim.step()