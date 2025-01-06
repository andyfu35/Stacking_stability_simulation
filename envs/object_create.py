import pybullet as p
import trimesh
import numpy as np


def create_box(box_size, box_pos):
   l_box, w_box, h_box, weight = (size / 2 * 10 for size in box_size)
   x_pos, y_pos, z_pos = (box_pos * 10 for box_pos in box_pos)
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
       baseMass=weight * 200,  # Set the mass of the box
       baseCollisionShapeIndex=collision_shape,
       baseVisualShapeIndex=visual_shape,
       basePosition=[x_pos, y_pos, z_pos]  # Set the position of the box
   )
   # Adjust the dynamics properties of the box
   p.changeDynamics(
       box_id, -1,
       ccdSweptSphereRadius=0.01,  # Radius for continuous collision detection (CCD)
       contactProcessingThreshold=0.0,  # Threshold for contact processing
       lateralFriction=0.5,  # Lateral friction coefficient
       spinningFriction=0.2,  # Spinning friction coefficient
       rollingFriction=0.2  # Rolling friction coefficient
   )

def create_wall(wall_size, wall_position):
    container_color = (0, 0, 1, 0.1)
    wall_id = p.createMultiBody(
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
    p.changeDynamics(
        wall_id, -1,
        lateralFriction=1.0,  # Lateral friction coefficient
        spinningFriction=1.0,  # Spinning friction coefficient
        rollingFriction=1.0  # Rolling friction coefficient
    )

def create_container(container_size):
    wall_thickness = 1.0
    container_l, container_w, container_h = (half_size / 2  * 10 for half_size in container_size)
    wall_size_ = [
        (wall_thickness, container_w + wall_thickness * 2, container_h + wall_thickness * 2),
        (container_l, container_w + wall_thickness * 2, wall_thickness),
        (container_l, container_w + wall_thickness * 2, wall_thickness),
        (container_l, wall_thickness, container_h),
        (container_l, wall_thickness, container_h)
    ]
    wall_position_ = [
        (-wall_thickness, 0, container_h),
        (container_l, 0, -wall_thickness),
        (container_l, 0, container_h*2 + wall_thickness),
        (container_l, container_w + wall_thickness, container_h),
        (container_l, -container_w - wall_thickness, container_h)
    ]
    for wall in range(len(wall_size_)):
        create_wall(wall_size_[wall], wall_position_[wall])

