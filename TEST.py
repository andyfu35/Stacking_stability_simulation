import pybullet as p
import pybullet_data
import time

# Connect to PyBullet simulation environment
p.connect(p.GUI)

# Set gravity
p.setGravity(0, 0, -9.81)

# Load PyBullet internal data resources
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Create custom gray ground
ground_collision_shape = p.createCollisionShape(
    p.GEOM_BOX,
    halfExtents=[5, 5, 0.01]  # Ground size (length, width, height)
)
ground_visual_shape = p.createVisualShape(
    p.GEOM_BOX,
    halfExtents=[5, 5, 0.01],
    rgbaColor=[0.7, 0.7, 0.7, 1]  # Gray color
)
ground_id = p.createMultiBody(
    baseMass=0,  # Set ground as static object
    baseCollisionShapeIndex=ground_collision_shape,
    baseVisualShapeIndex=ground_visual_shape,
    basePosition=[0, 0, 0]
)

# Set friction for ground
p.changeDynamics(ground_id, -1, lateralFriction=0.5)

# Box parameters (length, width, height)
box_length = 0.213
box_width = 0.255
box_height = 0.113

# Set box mass
box_mass = 10.0  # Mass of each part, set to 10 kg

# Create collision shape for the box
box_collision_shape = p.createCollisionShape(
    p.GEOM_BOX,
    halfExtents=[box_length / 2, box_width / 2, box_height / 8]  # Height of each part is one-fourth of the original
)

# Create visual shape for the box
box_visual_shape = p.createVisualShape(
    p.GEOM_BOX,
    halfExtents=[box_length / 2, box_width / 2, box_height / 8],
    rgbaColor=[0.8, 0.6, 0.4, 1]  # Light brown color
)

# Stack multiple boxes, adding one layer every second
num_boxes = 100
box_segments = []
current_box = 0
start_time = time.time()
interval = 2  # Add a new box every 2 seconds

# Run the simulation and add a new box every second
while current_box < num_boxes:
    # Get current time
    current_time = time.time()

    # Add a new box every 'interval' seconds
    if current_time - start_time >= interval:
        # Calculate the base height of each layer and leave enough gap to avoid overlap
        z_base_position = (box_height / 2) + current_box * (box_height + 0.02)

        # Create 4 parts of the box (stacked vertically)
        segment_ids = []
        for j in range(4):
            # Calculate the offset for each part to ensure correct stacking
            z_offset = (box_height / 4) * (j - 1.5)
            box_part = p.createMultiBody(
                baseMass=box_mass,
                baseCollisionShapeIndex=box_collision_shape,
                baseVisualShapeIndex=box_visual_shape,
                basePosition=[0, 0, z_base_position + z_offset]
            )
            segment_ids.append(box_part)

            # Set friction for each box part
            p.changeDynamics(box_part, -1, lateralFriction=0.4)

        # Connect each part with four joints
        for k in range(3):  # Connect each adjacent part
            # Create four constraint points at each corner
            corner_offsets = [
                [box_length / 2, box_width / 2, box_height / 8],  # Front right top corner
                [-box_length / 2, box_width / 2, box_height / 8],  # Front left top corner
                [box_length / 2, -box_width / 2, box_height / 8],  # Back right top corner
                [-box_length / 2, -box_width / 2, box_height / 8]  # Back left top corner
            ]

            for offset in corner_offsets:
                p.createConstraint(
                    parentBodyUniqueId=segment_ids[k],
                    parentLinkIndex=-1,
                    childBodyUniqueId=segment_ids[k + 1],
                    childLinkIndex=-1,
                    jointType=p.JOINT_POINT2POINT,
                    jointAxis=[0, 0, 0],
                    parentFramePosition=offset,
                    childFramePosition=[offset[0], offset[1], -box_height / 8]
                )

        box_segments.append(segment_ids)
        current_box += 1

        # Reset the timer
        start_time = current_time

    # Run the simulation
    p.stepSimulation()
    time.sleep(1. / 240.)

# Run for a while to observe the stacking effect
time.sleep(100)

# Disconnect the simulation
p.disconnect()
