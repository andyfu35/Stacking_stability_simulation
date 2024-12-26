import pybullet as p
import trimesh
import numpy as np


def create_box(box_size, box_pos, box_weight):
   l_box, w_box, h_box = (size / 2 for size in box_size)
   x_pos, y_pos, z_pos = box_pos
   # Create the collision shape of the box
   collision_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=(l_box, w_box, h_box))
   # Create the visual shape of the box with brown color
   visual_shape = p.createVisualShape(
       p.GEOM_BOX,
       halfExtents=(l_box, w_box, h_box),
       rgbaColor=[0.72, 0.53, 0.04, 1.0]  # Brown color, opaque
   )
   # Create the multi-body object representing the box
   box_id = p.createMultiBody(
       baseMass=box_weight,  # Set the mass of the box
       baseCollisionShapeIndex=collision_shape,
       baseVisualShapeIndex=visual_shape,
       basePosition=[x_pos, y_pos, z_pos]  # Set the position of the box
   )
   # Adjust the dynamics properties of the box
   p.changeDynamics(
       box_id, -1,
       ccdSweptSphereRadius=0.02,  # Radius for continuous collision detection (CCD)
       contactProcessingThreshold=0.0,  # Threshold for contact processing
       lateralFriction=0.6,  # Lateral friction coefficient
       spinningFriction=0.1,  # Spinning friction coefficient
       rollingFriction=0.1  # Rolling friction coefficient
   )

# def create_container(container_size, degree):
#    output_path = "/tmp/hollow_box_with_opening.obj"
#    container_l, container_w, container_h = container_size
#    # Create the outer box (external dimensions of the container)
#    outer_box = trimesh.creation.box(extents=(container_l, container_w, container_h))
#    # Create the inner box (to hollow out the container)
#    inner_box = trimesh.creation.box(extents=(container_l - 0.2, container_w - 0.1, container_h - 0.1))
#    inner_box.apply_translation([0, 0, 0])  # Center the inner box inside the outer box
#    # Create the right-side opening box
#    opening_box = trimesh.creation.box(extents=(0.1, container_w, container_h))
#    opening_box.apply_translation([(container_l - 0.1) / 2, 0, 0])  # Position the opening box on the right side
#    # Perform boolean operations: hollow out the outer box and add the opening
#    hollow_box = outer_box.difference(inner_box).difference(opening_box)
#    # Save the resulting hollow box with an opening to a file
#    hollow_box.export(output_path)
#    # Calculate the quaternion for rotation around the Y-axis
#    angle_in_radians = np.deg2rad(degree)  # Convert degrees to radians
#    rotation_quaternion = p.getQuaternionFromEuler([0, angle_in_radians, 0])  # Rotation around the Y-axis
#    # Load the hollow box into PyBullet
#    visual_shape_id = p.createVisualShape(
#        shapeType=p.GEOM_MESH,
#        fileName=output_path,
#        rgbaColor=[0, 0, 1, 0.3]  # Blue color with full opacity
#    )
#    collision_shape_id = p.createCollisionShape(
#        shapeType=p.GEOM_MESH,
#        fileName=output_path
#    )
#    p.createMultiBody(
#        baseMass=0,  # Set mass to 0 to make it static
#        baseCollisionShapeIndex=collision_shape_id,
#        baseVisualShapeIndex=visual_shape_id,
#        basePosition=[0, 0, 1],  # Position the hollow box above the ground
#        baseOrientation=rotation_quaternion  # Apply the initial rotation
#    )/
# create_container((6.0, 2.35, 2.36), 3)
# create_box((0.256, 0.213, 0.113), (0, 0, 0), 5)

def create_wall(wall_size, wall_position):
    container_color = (0, 0, 1, 0.1)
    p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=p.createCollisionShape(
            p.GEOM_BOX,
            halfExtents=wall_size
        ),
        baseVisualShapeIndex=p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=wall_size,
            rgbaColor=container_color
        ),
        basePosition=wall_position
    )

def create_container(container_size):
    wall_thickness = 0.01
    container_l, container_w, container_h = (half_size / 2 for half_size in container_size)
    wall_size_ = [
        (wall_thickness, container_w+wall_thickness, container_h+wall_thickness),
        (container_l, container_w, wall_thickness),
        (container_l, container_w, wall_thickness),
        (container_l, wall_thickness, container_h),
        (container_l, wall_thickness, container_h)
    ]
    wall_position_ = [
        (-wall_thickness/2, 0, container_h),
        (container_l, 0, -wall_thickness/2),
        (container_l, 0, container_h*2 + wall_thickness/2),
        (container_l, container_w + wall_thickness/2, container_h),
        (container_l, -container_w - wall_thickness / 2, container_h)
    ]
    for wall in range(len(wall_size_)):
        create_wall(wall_size_[wall], wall_position_[wall])

