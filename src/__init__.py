
import bpy
from sys import float_info
from math import degrees, radians, acos, pi, sin, atan2, cos
from mathutils import Euler, Matrix, Vector, Quaternion
from time import perf_counter as now

bl_info = {
    "name": "Source Engine Procedural Bones",
    "category": "Animation",
    "description": "A panel that helps create procedural bones for source engine models.",
    "author": "NameIsJakob",
    "version": (2, 3, 2),
    "blender": (2, 80, 0),
    "location": "View3D > Src Proc Bones",
    "tracker_url": "https://github.com/NameIsJakob/srcprocbones/issues",
}


# region Simulation

def simulate_quaternion_procedural(previewer, quaternion_procedural):
    if len(quaternion_procedural.triggers) == 0:
        return

    target_bone = previewer.pose.bones.get(quaternion_procedural.target_bone)
    if target_bone is None:
        return
    control_bone = previewer.pose.bones.get(quaternion_procedural.control_bone)
    if control_bone is None:
        return

    if target_bone is None or control_bone is None:
        return

    if target_bone == control_bone:
        return

    if target_bone.parent is None or control_bone.parent is None:
        return

    weights = [i for i in range(32)]
    scale = 0.0

    transpose_parent = control_bone.parent.bone.matrix_local.inverted_safe()
    parent_space = transpose_parent @ control_bone.bone.matrix_local
    current_rotation = parent_space @ control_bone.matrix_basis
    for index, trigger in enumerate(quaternion_procedural.triggers):
        if abs(trigger.tolerance) <= float_info.epsilon:
            return

        dot = abs(Euler(trigger.trigger_angle).to_quaternion().dot(current_rotation.to_quaternion()))
        dot = min(max(dot, -1.0), 1.0)
        weights[index] = 1.0 - (2.0 * acos(dot) * (1.0 / trigger.tolerance))
        weights[index] = max(0.0, weights[index])
        scale += weights[index]

    if scale <= 0.001:
        for trigger in quaternion_procedural.triggers:
            trigger.weight = 0.0

        active_trigger = quaternion_procedural.triggers[0]
        active_trigger.weight = 1.0

        target_rotation = Euler(active_trigger.target_angle).to_matrix()
        target_bone_inverted = target_bone.bone.matrix_local.inverted_safe()

        if not quaternion_procedural.override_position:
            base_position = (target_bone.parent.bone.matrix_local.inverted_safe() @ target_bone.bone.matrix_local).to_translation()
            target_translation = Matrix.Translation(base_position + Vector(active_trigger.target_position))
            target_bone.matrix_basis = target_bone_inverted @ target_bone.parent.bone.matrix_local @ target_translation @ target_rotation.to_4x4()
            return

        override_position = Vector(quaternion_procedural.position_override)
        distance = quaternion_procedural.distance / 100.0
        control_bone_position = (control_bone.parent.bone.matrix_local.inverted_safe() @ control_bone.bone.matrix_local).to_translation() * distance
        target_translation = Matrix.Translation(override_position + control_bone_position + Vector(active_trigger.target_position))
        target_bone.matrix_basis = target_bone_inverted @ target_bone.parent.bone.matrix_local @ target_translation @ target_rotation.to_4x4()
        return

    scale = 1.0 / scale

    quat = Quaternion((0.0, 0.0, 0.0, 0.0))
    pos = Vector((0.0, 0.0, 0.0))

    for index, trigger in enumerate(quaternion_procedural.triggers):
        if weights[index] == 0.0:
            trigger.weight = 0.0
            continue

        s = weights[index] * scale
        trigger.weight = s
        target_quaternion = Euler(trigger.target_angle).to_quaternion()
        target_position = Vector(trigger.target_position)

        quat.x += s * target_quaternion.x
        quat.y += s * target_quaternion.y
        quat.z += s * target_quaternion.z
        quat.w += s * target_quaternion.w

        pos.x += s * target_position.x
        pos.y += s * target_position.y
        pos.z += s * target_position.z

    target_rotation = quat.normalized().to_matrix()
    target_bone_transposed = target_bone.bone.matrix_local.inverted_safe()

    if not quaternion_procedural.override_position:
        base_position = (target_bone.parent.bone.matrix_local.inverted_safe() @ target_bone.bone.matrix_local).to_translation()
        target_translation = Matrix.Translation(base_position + pos)
        target_bone.matrix_basis = target_bone_transposed @ target_bone.parent.bone.matrix_local @ target_translation @ target_rotation.to_4x4()
        return

    override_position = Vector(quaternion_procedural.position_override)
    distance = quaternion_procedural.distance / 100.0
    control_bone_position = (control_bone.parent.bone.matrix_local.inverted_safe() @ control_bone.bone.matrix_local).to_translation() * distance
    target_translation = Matrix.Translation(override_position + control_bone_position + pos)
    target_bone.matrix_basis = target_bone_transposed @ target_bone.parent.bone.matrix_local @ target_translation @ target_rotation.to_4x4()


def simulate_aimat_procedural(previewer, aimat_procedural):
    # The math for this is the same but gets different results from source.
    procedural_bone = previewer.pose.bones.get(aimat_procedural.procedural_bone)
    if procedural_bone is None or procedural_bone.parent is None:
        return

    aim_target = aimat_procedural.aim_target
    if aim_target is None:
        return

    aim_direction = Vector(aimat_procedural.aim_vector).normalized()
    if aim_direction.length <= float_info.epsilon:
        aimat_procedural.aim_vector = Vector((1.0, 0.0, 0.0))
    up_direction = Vector(aimat_procedural.up_vector).normalized()
    if up_direction.length <= float_info.epsilon:
        aimat_procedural.up_vector = Vector((0.0, 0.0, 1.0))

    procedural_position = procedural_bone.matrix.translation
    target_position = aim_target.matrix_world.translation
    target_direction = (target_position - procedural_position).normalized()

    aim_axis = aim_direction.cross(target_direction).normalized()
    aim_angle = acos(aim_direction.dot(target_direction))
    aim_rotation = Matrix.Rotation(aim_angle, 4, aim_axis)

    temp_up = aim_rotation @ up_direction
    temp_up_direction = target_direction * target_direction.dot(temp_up)
    up = (temp_up - temp_up_direction).normalized()

    temp_parent_up = procedural_bone.bone.matrix_local @ up_direction
    temp_parent_up_direction = target_direction * target_direction.dot(temp_parent_up)
    parent_up = (temp_parent_up - temp_parent_up_direction).normalized()

    up_axis = up.cross(parent_up).normalized()
    up_angle = up.dot(parent_up)
    up_rotation = Matrix.Rotation(acos(up_angle), 4, up_axis) if 1.0 - abs(up_angle) > float_info.epsilon else Matrix.Rotation(0, 4, parent_up)

    procedural_bone.matrix = Matrix.Translation(procedural_position) @ up_rotation @ aim_rotation


def simulate_jiggle_procedural(previewer, active_jiggle_procedural, current_time):
    target_bone = previewer.pose.bones.get(active_jiggle_procedural.target_bone)
    if target_bone is None:
        return

    jiggle_bone_data_space_name = active_jiggle_procedural.target_bone + " Jiggle Bone Data Space"
    jiggle_bone_data_space = bpy.data.objects.get(jiggle_bone_data_space_name)
    if not jiggle_bone_data_space:
        jiggle_bone_data_space = bpy.data.objects.new(jiggle_bone_data_space_name, None)

    jiggle_bone_data_name = active_jiggle_procedural.target_bone + " Jiggle Bone Data"
    jiggle_bone_data = bpy.data.objects.get(jiggle_bone_data_name)
    if not jiggle_bone_data:
        jiggle_bone_data = bpy.data.objects.new(jiggle_bone_data_name, None)
        jiggle_bone_data.parent = jiggle_bone_data_space

    location_constraint_name = active_jiggle_procedural.target_bone + " Jiggle Bone Location"
    location_constraint = target_bone.constraints.get(location_constraint_name)
    if not location_constraint:
        location_constraint = target_bone.constraints.new("COPY_LOCATION")
        location_constraint.show_expanded = False
        location_constraint.name = location_constraint_name
        location_constraint.target = jiggle_bone_data
        location_constraint.use_offset = True
        location_constraint.target_space = "LOCAL"
        location_constraint.owner_space = "LOCAL"

    rotation_constraint_name = active_jiggle_procedural.target_bone + " Jiggle Bone Rotation"
    rotation_constraint = target_bone.constraints.get(rotation_constraint_name)
    if not rotation_constraint:
        rotation_constraint = target_bone.constraints.new("COPY_ROTATION")
        rotation_constraint.show_expanded = False
        rotation_constraint.name = rotation_constraint_name
        rotation_constraint.target = jiggle_bone_data
        if bpy.app.version[:2] == (2, 80):
            rotation_constraint.use_offset = True
        else:
            rotation_constraint.mix_mode = "ADD"
        rotation_constraint.target_space = "LOCAL"
        rotation_constraint.owner_space = "LOCAL"

    scale_constraint_name = active_jiggle_procedural.target_bone + " Jiggle Bone Scale"
    scale_constraint = target_bone.constraints.get(scale_constraint_name)
    if not scale_constraint:
        scale_constraint = target_bone.constraints.new("COPY_SCALE")
        scale_constraint.show_expanded = False
        scale_constraint.name = scale_constraint_name
        scale_constraint.target = jiggle_bone_data
        scale_constraint.use_offset = True
        scale_constraint.target_space = "LOCAL"
        scale_constraint.owner_space = "LOCAL"

    goal_matrix = Matrix()
    bone_matrix = Matrix()

    if target_bone.parent:
        parent_bind = target_bone.parent.bone.matrix_local
        parent_bind_inverted = parent_bind.inverted_safe()
        parent_transforms = parent_bind_inverted @ target_bone.parent.matrix

        target_bind = target_bone.bone.matrix_local
        target_parent_transforms = parent_bind_inverted @ target_bind
        target_transforms = target_parent_transforms @ target_bone.matrix_basis

        goal_matrix = (parent_bind @ parent_transforms @ target_transforms).normalized()
        bone_matrix = (parent_bind @ parent_transforms @ target_transforms).normalized()
    else:
        goal_matrix = (target_bone.bone.matrix_local @ target_bone.matrix_basis).normalized()
        bone_matrix = (target_bone.bone.matrix_local @ target_bone.matrix_basis).normalized()

    jiggle_bone_data_space.matrix_world = goal_matrix

    goal_base_position = goal_matrix.to_translation()

    goal_left = goal_matrix.col[0].to_3d()
    goal_up = goal_matrix.col[1].to_3d()
    goal_forward = goal_matrix.col[2].to_3d()

    goal_tip = goal_base_position + active_jiggle_procedural.length * goal_forward

    time_tolerance = 0.5
    if current_time - active_jiggle_procedural.last_update > time_tolerance:
        active_jiggle_procedural.last_update = current_time
        active_jiggle_procedural.base_position = goal_base_position
        active_jiggle_procedural.last_base_position = goal_base_position
        active_jiggle_procedural.base_velocity = (0.0, 0.0, 0.0)
        active_jiggle_procedural.base_acceleration = (0.0, 0.0, 0.0)
        active_jiggle_procedural.tip_position = goal_tip
        active_jiggle_procedural.tip_velocity = (0.0, 0.0, 0.0)
        active_jiggle_procedural.tip_acceleration = (0.0, 0.0, 0.0)
        active_jiggle_procedural.last_boing_position = goal_base_position
        active_jiggle_procedural.boing_direction = Vector((0.0, 0.0, 1.0))
        active_jiggle_procedural.boing_velocity_direction = Vector()
        active_jiggle_procedural.boing_speed = 0.0
        active_jiggle_procedural.boing_time = 0.0

        jiggle_bone_data.matrix_local = Matrix()

    source_procedural_previewing_data = bpy.context.scene.source_procedural_previewing_data
    delta_time = 1.0 / source_procedural_previewing_data.update_rate
    if current_time - active_jiggle_procedural.last_update > delta_time:
        delta_time = current_time - active_jiggle_procedural.last_update
    active_jiggle_procedural.last_update = current_time

    if active_jiggle_procedural.tip_flex_type == "RIGID" or active_jiggle_procedural.tip_flex_type == "FLEXIBLE":
        def tip_position():
            return Vector(active_jiggle_procedural.tip_position)

        def tip_velocity():
            return Vector(active_jiggle_procedural.tip_velocity)

        def tip_acceleration():
            return Vector(active_jiggle_procedural.tip_acceleration)

        active_jiggle_procedural.tip_acceleration[2] -= active_jiggle_procedural.tip_mass

        if active_jiggle_procedural.tip_flex_type == "FLEXIBLE":
            error = goal_tip - tip_position()

            local_error = Vector()
            local_error.x = goal_left.dot(error)
            local_error.y = goal_up.dot(error)
            local_error.z = goal_forward.dot(error)

            local_velocity = Vector()
            local_velocity.x = goal_left.dot(tip_velocity())
            local_velocity.y = goal_up.dot(tip_velocity())

            yaw_acceleration = active_jiggle_procedural.yaw_stiffness * local_error.x - active_jiggle_procedural.yaw_damping * local_velocity.x

            pitch_acceleration = active_jiggle_procedural.pitch_stiffness * local_error.y - active_jiggle_procedural.pitch_damping * local_velocity.y

            if active_jiggle_procedural.along_constraint == "CONSTRAINT":
                active_jiggle_procedural.tip_acceleration = tip_acceleration() + yaw_acceleration * goal_left + pitch_acceleration * goal_up
            else:
                local_velocity.z = goal_forward.dot(tip_velocity())

                along_acceleration = active_jiggle_procedural.along_stiffness * local_error.z - active_jiggle_procedural.along_damping * local_velocity.z

                active_jiggle_procedural.tip_acceleration = tip_acceleration() + yaw_acceleration * goal_left + pitch_acceleration * goal_up + along_acceleration * goal_forward

        active_jiggle_procedural.tip_velocity = tip_velocity() + tip_acceleration() * delta_time
        active_jiggle_procedural.tip_position = tip_position() + tip_velocity() * delta_time

        active_jiggle_procedural.tip_acceleration = Vector()

        if active_jiggle_procedural.yaw_constraint == "CONSTRAINT" or active_jiggle_procedural.pitch_constraint == "CONSTRAINT":
            along = tip_position() - goal_base_position
            local_along = Vector()
            local_along.x = goal_left.dot(along)
            local_along.y = goal_up.dot(along)
            local_along.z = goal_forward.dot(along)

            local_velocity = Vector()
            local_velocity.x = goal_left.dot(tip_velocity())
            local_velocity.y = goal_up.dot(tip_velocity())
            local_velocity.z = goal_forward.dot(tip_velocity())

            if active_jiggle_procedural.yaw_constraint == "CONSTRAINT":
                yaw_error = atan2(local_along.x, local_along.z)

                is_at_limit = False
                yaw = 0.0

                if yaw_error < active_jiggle_procedural.yaw_minimum:
                    is_at_limit = True
                    yaw = active_jiggle_procedural.yaw_minimum
                elif yaw_error > active_jiggle_procedural.yaw_maximum:
                    is_at_limit = True
                    yaw = active_jiggle_procedural.yaw_maximum

                if is_at_limit:
                    sin_yaw = sin(yaw)
                    cos_yaw = cos(yaw)

                    yaw_matrix = Matrix(((cos_yaw, 0.0, sin_yaw, 0.0),
                                         (0.0, 1.0, 0.0, 0.0),
                                         (-sin_yaw, 0.0, cos_yaw, 0.0),
                                         (0.0, 0.0, 0.0, 1.0)))

                    limit_matrix = goal_matrix @ yaw_matrix
                    limit_left = limit_matrix.col[0].to_3d()
                    limit_up = limit_matrix.col[1].to_3d()
                    limit_forward = limit_matrix.col[2].to_3d()
                    limit_along = Vector((limit_left.dot(along), limit_up.dot(along), limit_forward.dot(along)))

                    active_jiggle_procedural.tip_position = goal_base_position + limit_along.y * limit_up + limit_along.z * limit_forward
                    if active_jiggle_procedural.use_friction:
                        limit_velocity = Vector((0.0, limit_up.dot(tip_velocity()), limit_forward.dot(tip_velocity())))
                        active_jiggle_procedural.tip_acceleration = tip_acceleration() - active_jiggle_procedural.yaw_friction * \
                            (limit_velocity.y * limit_up + limit_velocity.z * limit_forward)
                        active_jiggle_procedural.tip_velocity = -active_jiggle_procedural.yaw_bounce * \
                            limit_velocity.x * limit_left + limit_velocity.y * limit_up + limit_velocity.z * limit_forward
                    else:
                        active_jiggle_procedural.tip_velocity = Vector()

                    along = tip_position() - goal_base_position
                    local_along = Vector((goal_left.dot(along), goal_up.dot(along), goal_forward.dot(along)))
                    local_velocity = Vector((goal_left.dot(tip_velocity()), goal_up.dot(tip_velocity()), goal_forward.dot(tip_velocity())))

            if active_jiggle_procedural.pitch_constraint == "CONSTRAINT":
                pitch_error = atan2(local_along.y, local_along.z)

                is_at_limit = False
                pitch = 0.0

                if pitch_error < active_jiggle_procedural.pitch_minimum:
                    is_at_limit = True
                    pitch = active_jiggle_procedural.pitch_minimum
                elif pitch_error > active_jiggle_procedural.pitch_maximum:
                    is_at_limit = True
                    pitch = active_jiggle_procedural.pitch_maximum

                if is_at_limit:
                    sin_pitch = sin(pitch)
                    cos_pitch = cos(pitch)

                    pitch_matrix = Matrix(((1.0, 0.0, 0.0, 0.0),
                                           (0.0, cos_pitch, sin_pitch, 0.0),
                                           (0.0, -sin_pitch, cos_pitch, 0.0),
                                           (0.0, 0.0, 0.0, 1.0)))

                    limit_matrix = goal_matrix @ pitch_matrix
                    limit_left = limit_matrix.col[0].to_3d()
                    limit_up = limit_matrix.col[1].to_3d()
                    limit_forward = limit_matrix.col[2].to_3d()
                    limit_along = Vector((limit_left.dot(along), limit_up.dot(along), limit_forward.dot(along)))

                    active_jiggle_procedural.tip_position = goal_base_position + limit_along.x * limit_left + limit_along.z * limit_forward
                    if active_jiggle_procedural.use_friction:
                        limit_velocity = Vector((0.0, limit_up.dot(tip_velocity()), limit_forward.dot(tip_velocity())))
                        active_jiggle_procedural.tip_acceleration = tip_acceleration() - active_jiggle_procedural.pitch_friction * \
                            (limit_velocity.x * limit_left + limit_velocity.z * limit_forward)
                        active_jiggle_procedural.tip_velocity = limit_velocity.x * limit_left - \
                            active_jiggle_procedural.pitch_bounce * limit_velocity.y * limit_up + limit_velocity.z * limit_forward
                    else:
                        active_jiggle_procedural.tip_velocity = Vector()

        forward = (tip_position() - goal_base_position).normalized()

        if active_jiggle_procedural.angle_constraint == "CONSTRAINT":
            dot = forward.dot(goal_forward)

            if dot > -1.0 and dot < 1.0:
                angle_between = acos(dot)
                if dot < 0.0:
                    angle_between = 2.0 * pi - angle_between

                if angle_between > active_jiggle_procedural.angle_limit:
                    max_between = active_jiggle_procedural.length * sin(active_jiggle_procedural.angle_limit)
                    delta = (goal_tip - tip_position()).normalized()
                    active_jiggle_procedural.tip_position = goal_tip - max_between * delta
                    forward = (tip_position() - goal_base_position).normalized()

        if active_jiggle_procedural.along_constraint == "CONSTRAINT":
            active_jiggle_procedural.tip_position = goal_base_position + active_jiggle_procedural.length * forward
            active_jiggle_procedural.tip_velocity = tip_velocity() - tip_velocity().dot(forward) * forward

        left = goal_up.cross(forward).normalized()
        up = forward.cross(left)

        bone_matrix[0][0] = left.x
        bone_matrix[1][0] = left.y
        bone_matrix[2][0] = left.z
        bone_matrix[0][1] = up.x
        bone_matrix[1][1] = up.y
        bone_matrix[2][1] = up.z
        bone_matrix[0][2] = forward.x
        bone_matrix[1][2] = forward.y
        bone_matrix[2][2] = forward.z

        bone_matrix[0][3] = goal_base_position.x
        bone_matrix[1][3] = goal_base_position.y
        bone_matrix[2][3] = goal_base_position.z

    if active_jiggle_procedural.base_flex_type == "SPRING":
        def base_position():
            return Vector(active_jiggle_procedural.base_position)

        def last_base_position():
            return Vector(active_jiggle_procedural.last_base_position)

        def base_velocity():
            return Vector(active_jiggle_procedural.base_velocity)

        def base_acceleration():
            return Vector(active_jiggle_procedural.base_acceleration)

        active_jiggle_procedural.base_acceleration[2] -= active_jiggle_procedural.base_mass

        error = goal_base_position - base_position()
        active_jiggle_procedural.base_acceleration = base_acceleration() + active_jiggle_procedural.base_stiffness * error - \
            active_jiggle_procedural.base_damping * base_velocity()

        active_jiggle_procedural.base_velocity = base_velocity() + base_acceleration() * delta_time
        active_jiggle_procedural.base_position = base_position() + base_velocity() * delta_time

        active_jiggle_procedural.base_acceleration = Vector()

        error = base_position() - goal_base_position
        local_error = Vector()
        local_error.x = goal_left.dot(error)
        local_error.y = goal_up.dot(error)
        local_error.z = goal_forward.dot(error)

        local_velocity = Vector()
        local_velocity.x = goal_left.dot(base_velocity())
        local_velocity.y = goal_up.dot(base_velocity())
        local_velocity.z = goal_forward.dot(base_velocity())

        if local_error.x < active_jiggle_procedural.base_minimum_left:
            local_error.x = active_jiggle_procedural.base_minimum_left
            active_jiggle_procedural.base_acceleration = base_acceleration() - active_jiggle_procedural.base_friction_left * \
                (local_velocity.y * goal_up + local_velocity.z * goal_forward)
        elif local_error.x > active_jiggle_procedural.base_maximum_left:
            local_error.x = active_jiggle_procedural.base_maximum_left
            active_jiggle_procedural.base_acceleration = base_acceleration() - active_jiggle_procedural.base_friction_left * \
                (local_velocity.y * goal_up + local_velocity.z * goal_forward)

        if local_error.y < active_jiggle_procedural.base_minimum_up:
            local_error.y = active_jiggle_procedural.base_minimum_up
            active_jiggle_procedural.base_acceleration = base_acceleration() - active_jiggle_procedural.base_friction_up * \
                (local_velocity.x * goal_left + local_velocity.z * goal_forward)
        elif local_error.y > active_jiggle_procedural.base_maximum_up:
            local_error.y = active_jiggle_procedural.base_maximum_up
            active_jiggle_procedural.base_acceleration = base_acceleration() - active_jiggle_procedural.base_friction_up * \
                (local_velocity.x * goal_left + local_velocity.z * goal_forward)

        if local_error.z < active_jiggle_procedural.base_minimum_forward:
            local_error.z = active_jiggle_procedural.base_minimum_forward
            active_jiggle_procedural.base_acceleration = base_acceleration() - active_jiggle_procedural.base_friction_forward * \
                (local_velocity.x * goal_left + local_velocity.y * goal_up)
        elif local_error.z > active_jiggle_procedural.base_maximum_forward:
            local_error.z = active_jiggle_procedural.base_maximum_forward
            active_jiggle_procedural.base_acceleration = base_acceleration() - active_jiggle_procedural.base_friction_forward * \
                (local_velocity.x * goal_left + local_velocity.y * goal_up)

        active_jiggle_procedural.base_position = goal_base_position + local_error.x * goal_left + local_error.y * goal_up + local_error.z * goal_forward

        active_jiggle_procedural.base_velocity = (base_position() - last_base_position()) / delta_time
        active_jiggle_procedural.last_base_position = base_position()

        bone_matrix[0][3] = active_jiggle_procedural.base_position[0]
        bone_matrix[1][3] = active_jiggle_procedural.base_position[1]
        bone_matrix[2][3] = active_jiggle_procedural.base_position[2]

    if active_jiggle_procedural.base_flex_type == "BOING":
        def last_boing_position():
            return Vector(active_jiggle_procedural.last_boing_position)

        def boing_direction():
            return Vector(active_jiggle_procedural.boing_direction)

        def boing_velocity_direction():
            return Vector(active_jiggle_procedural.boing_velocity_direction)

        velocity = (goal_base_position - last_boing_position()).normalized()
        speed = (goal_base_position - last_boing_position()).length
        active_jiggle_procedural.last_boing_position = goal_base_position

        if speed < 0.00001:
            velocity = Vector((0.0, 0.0, 1.0))
            speed = 0.0
        else:
            speed /= delta_time

        active_jiggle_procedural.boing_time += delta_time

        minimum_speed = 5.0
        minimum_reboing_time = 0.5

        if (speed > minimum_speed or active_jiggle_procedural.boing_speed > minimum_speed) and active_jiggle_procedural.boing_time > minimum_reboing_time:
            if abs(active_jiggle_procedural.boing_speed - speed) > active_jiggle_procedural.boing_impact_speed or velocity.dot(boing_velocity_direction()) < cos(active_jiggle_procedural.boing_impact_angle):
                active_jiggle_procedural.boing_time = 0.0
                active_jiggle_procedural.boing_direction = -velocity

        active_jiggle_procedural.boing_velocity_direction = velocity
        active_jiggle_procedural.boing_speed = speed

        damping = 1.0 - (active_jiggle_procedural.boing_damping_rate * active_jiggle_procedural.boing_time)

        if damping < 0.01:
            bone_matrix = goal_matrix.copy()
            bone_matrix[0][3] = goal_base_position.x
            bone_matrix[1][3] = goal_base_position.y
            bone_matrix[2][3] = goal_base_position.z
        else:
            damping *= damping
            damping *= damping

            flex = active_jiggle_procedural.boing_amplitude * cos(active_jiggle_procedural.boing_frequency * active_jiggle_procedural.boing_time) * damping

            squash = 1.0 + flex
            stretch = 1.0 - flex

            bone_matrix = goal_matrix.copy()
            bone_matrix[0][3] = 0.0
            bone_matrix[1][3] = 0.0
            bone_matrix[2][3] = 0.0

            boing_side = boing_direction().cross(Vector((1.0, 0.0, 0.0)) if abs(boing_direction().x) < 0.9 else Vector((0.0, 0.0, 1.0))).normalized()
            boing_other_side = boing_direction().cross(boing_side)

            to_boing = Matrix((
                (boing_side.x, boing_other_side.x, boing_direction().x),
                (boing_side.y, boing_other_side.y, boing_direction().y),
                (boing_side.z, boing_other_side.z, boing_direction().z)
            ))

            boing = Matrix((
                (squash, 0.0, 0.0),
                (0.0, squash, 0.0),
                (0.0, 0.0, stretch)
            ))

            from_boing = to_boing.transposed()

            bone_matrix @= (to_boing @ boing @ from_boing).to_4x4()

            bone_matrix[0][3] = goal_base_position.x
            bone_matrix[1][3] = goal_base_position.y
            bone_matrix[2][3] = goal_base_position.z

    jiggle_bone_data.matrix_world = bone_matrix


def previewing():
    source_procedural_previewing_data = bpy.context.scene.source_procedural_previewing_data

    for previewer in source_procedural_previewing_data.previewers:
        source_procedural_bone_data = previewer.armature.source_procedural_bone_data

        for quaternion_procedural in source_procedural_bone_data.quaternion_procedurals:
            if not quaternion_procedural.preview:
                continue

            simulate_quaternion_procedural(previewer.armature, quaternion_procedural)

        for aimat_procedural in source_procedural_bone_data.aimat_procedurals:
            if not aimat_procedural.preview:
                continue

            simulate_aimat_procedural(previewer.armature, aimat_procedural)

        for jiggle_procedural in source_procedural_bone_data.jiggle_procedurals:
            if not jiggle_procedural.preview:
                continue

            simulate_jiggle_procedural(previewer.armature, jiggle_procedural, now())

    return 1.0 / source_procedural_previewing_data.update_rate


def update_previewing(self, context):
    active_armature = context.object
    source_procedural_previewing_data = context.scene.source_procedural_previewing_data

    is_previewer = any(previewer.armature == active_armature for previewer in source_procedural_previewing_data.previewers)

    if self.previewing and not is_previewer:
        previewer = source_procedural_previewing_data.previewers.add()
        previewer.armature = active_armature

    if not self.previewing and is_previewer:
        for index, previewer in enumerate(source_procedural_previewing_data.previewers):
            if previewer.armature != active_armature:
                continue
            source_procedural_previewing_data.previewers.remove(index)
            break

    if len(source_procedural_previewing_data.previewers) > 0 and not bpy.app.timers.is_registered(previewing):
        bpy.app.timers.register(previewing)
        return

    if len(source_procedural_previewing_data.previewers) == 0 and bpy.app.timers.is_registered(previewing):
        bpy.app.timers.unregister(previewing)

# endregion

# region Properties


class QuaternionProceduralTriggerProperty(bpy.types.PropertyGroup):
    name: bpy.props.StringProperty(default="New Trigger", name="Name", description="The name of the trigger")
    weight: bpy.props.FloatProperty(options={"SKIP_SAVE"})
    tolerance: bpy.props.FloatProperty(default=radians(90.0), precision=6, soft_min=0.0, unit="ROTATION",
                                       description="The angle cone the control bone angle to the angle trigger to activate the trigger")
    trigger_angle: bpy.props.FloatVectorProperty(precision=6, unit="ROTATION", name="Trigger Angle",
                                                 description="The angle the control bone should be at to activate the trigger")
    target_angle: bpy.props.FloatVectorProperty(precision=6, unit="ROTATION", name="Target Angle",
                                                description="The angle the target bone will be set to when the trigger is active")
    target_position: bpy.props.FloatVectorProperty(precision=6, name="Target Position", description="The added offset position when the trigger is active")


def copy_quaternion_procedural_trigger(source, destination):
    destination.name = source.name
    destination.tolerance = source.tolerance
    destination.trigger_angle = source.trigger_angle
    destination.target_angle = source.target_angle
    destination.target_position = source.target_position


class QuaternionProceduralProperty(bpy.types.PropertyGroup):
    name: bpy.props.StringProperty(default="New Quaternion Procedural", name="Name", description="The name of the quaternion procedural")
    target_bone: bpy.props.StringProperty(name="Target Bone", description="The bone that will be procedurally animated")
    control_bone: bpy.props.StringProperty(name="Control Bone", description="The bone that will drive the target bone")
    override_position: bpy.props.BoolProperty(default=False, description="Allows to override the position of the target bone")
    position_override: bpy.props.FloatVectorProperty(precision=6, name="Position", description="The offset position of the target bone's parent")
    distance: bpy.props.FloatProperty(soft_min=0.0, soft_max=100.0,
                                      description="The percentage of the control bone's position offset from its parent added to the target bone's position")
    triggers: bpy.props.CollectionProperty(type=QuaternionProceduralTriggerProperty)
    active_trigger: bpy.props.IntProperty(name="Active Trigger")
    preview: bpy.props.BoolProperty(default=True)


def copy_quaternion_procedural(source, destination):
    destination.override_position = source.override_position
    destination.position_override = source.position_override
    destination.distance = source.distance
    destination.triggers.clear()
    for trigger in source.triggers:
        destination.triggers.add()
        copy_quaternion_procedural_trigger(trigger, destination.triggers[len(destination.triggers) - 1])


class AimatProceduralProperty(bpy.types.PropertyGroup):
    name: bpy.props.StringProperty(default="New Aimat Procedural", name="Name", description="The name of the aimat procedural")
    procedural_bone: bpy.props.StringProperty(name="Procedural Bone", description="The bone that will be procedurally animated")
    aim_target: bpy.props.PointerProperty(name="Aim Target", description="The attachment the procedural bone will target",
                                          type=bpy.types.Object, poll=lambda self, object: object.type == 'EMPTY')
    aim_vector: bpy.props.FloatVectorProperty(default=(1.0, 0.0, 0.0), precision=6, name="Aim Vector",
                                              description="The direction the bone will point towards the target")
    up_vector: bpy.props.FloatVectorProperty(default=(0.0, 0.0, 1.0), precision=6, name="Up Vector",
                                             description="The direction of up for the bone")
    preview: bpy.props.BoolProperty(default=True)


def copy_aimat_procedural(source, destination):
    destination.name = source.name
    destination.aim_target = source.aim_target
    destination.aim_vector = source.aim_vector
    destination.up_vector = source.up_vector


class JiggleProceduralProperty(bpy.types.PropertyGroup):
    name: bpy.props.StringProperty(default="New Jiggle Procedural", name="Name", description="The name of the jiggle procedural")
    target_bone: bpy.props.StringProperty(name="Target Bone", description="The bone that will be procedurally animated")
    tip_flex_type: bpy.props.EnumProperty(items=[("NONE", "None", "No Rotational Jiggle"), ("RIGID", "Rigid", "A Simple Rotational Jiggle"), ("FLEXIBLE", "Flexible", "A Complex Rotational Jiggle")],
                                          default="FLEXIBLE", name="Tip Flex Type", description="The rotational type of the jiggle procedural")
    length: bpy.props.FloatProperty(soft_min=0.0, default=10.0, name="Length", description="The distance from the base to the flex tip")
    tip_mass: bpy.props.FloatProperty(soft_min=0.0, name="Tip Mass", description="An acceleration down at in/s²")
    yaw_stiffness: bpy.props.FloatProperty(soft_min=0.0, soft_max=1000.0, default=100.0, name="Yaw Stiffness",
                                           description="The speed of the jiggle on the Y axis")
    yaw_damping: bpy.props.FloatProperty(soft_min=0.0, soft_max=1000.0, name="Yaw Damping", description="A velocity reduction of the jiggle on the Y axis")
    pitch_stiffness: bpy.props.FloatProperty(soft_min=0.0, soft_max=1000.0, default=100.0, name="Pitch Stiffness",
                                             description="The speed of the jiggle on the X axis")
    pitch_damping: bpy.props.FloatProperty(soft_min=0.0, soft_max=1000.0, name="Pitch Damping", description="A velocity reduction of the jiggle on the X axis")
    along_constraint: bpy.props.EnumProperty(items=[("NONE", "None", "The Tip Length Is Allow To Jiggle"), ("CONSTRAINT", "Constraint", "The Tip Length Is Locked")],
                                             default="CONSTRAINT", name="Along Constraint", description="Specify if the tip can jiggle")
    along_stiffness: bpy.props.FloatProperty(soft_min=0.0, soft_max=1000.0, default=100.0, name="Pitch Stiffness",
                                             description="The speed of the jiggle on the Z axis")
    along_damping: bpy.props.FloatProperty(soft_min=0.0, soft_max=1000.0, name="Pitch Damping", description="A velocity reduction of the jiggle on the Z axis")
    angle_constraint: bpy.props.EnumProperty(items=[("NONE", "None", "No Angle Constraint"), ("CONSTRAINT", "Constraint",
                                             "Jiggle Will Be Constraint By An Angle")], name="Angle Constraint", description="Constraint jiggle bone by an angle")
    angle_limit: bpy.props.FloatProperty(soft_min=0.0, unit="ROTATION", name="Angle Limit", description="The angle to constrain the jiggle bone")
    use_friction: bpy.props.BoolProperty(name="Use Friction", description="Use the friction and bounce values")
    yaw_constraint: bpy.props.EnumProperty(items=[("NONE", "None", "Jiggle Not Constrained On The X"), ("CONSTRAINT", "Constraint",
                                           "Jiggle Constrained On The X")], name="Yaw Constraint", description="Specify if the yaw is constrained by 2 angles")
    yaw_minimum: bpy.props.FloatProperty(soft_max=0.0, unit="ROTATION", name="Yaw Minimum", description="The minimum angle the yaw can rotate")
    yaw_maximum: bpy.props.FloatProperty(soft_min=0.0, unit="ROTATION", name="Yaw Maximum", description="The maximum angle the yaw can rotate")
    yaw_friction: bpy.props.FloatProperty(soft_min=0.0, name="Yaw Friction", description="A acceleration reduction on the X")
    yaw_bounce: bpy.props.FloatProperty(soft_min=0.0, name="Yaw Bounce", description="Velocity set on the X in the opposite direction")
    pitch_constraint: bpy.props.EnumProperty(items=[("NONE", "None", "Jiggle Not Constrained On The Y"), ("CONSTRAINT", "Constraint",
                                             "Jiggle Constrained On The Y")], name="Pitch Constraint", description="Specify if the pitch is constrained by 2 angles")
    pitch_minimum: bpy.props.FloatProperty(soft_max=0.0, unit="ROTATION", name="Pitch Minimum", description="The minimum angle the pitch can rotate")
    pitch_maximum: bpy.props.FloatProperty(soft_min=0.0, unit="ROTATION", name="Pitch Maximum", description="The maximum angle the pitch can rotate")
    pitch_friction: bpy.props.FloatProperty(soft_min=0.0, name="Pitch Friction", description="A acceleration reduction on the Y")
    pitch_bounce: bpy.props.FloatProperty(soft_min=0.0, name="Pitch Bounce", description="Velocity set on the Y in the opposite direction")
    base_flex_type: bpy.props.EnumProperty(items=[("NONE", "None", "No Base Jiggle"), ("SPRING", "Spring", "Translational Jiggling"),
                                           ("BOING", "Boing", "Boing Jiggling")], name="Base Flex Type", description="Flex type for the base of the jiggle procedural")
    base_mass: bpy.props.FloatProperty(name="Base Mass", description="An acceleration down at in/s²")
    base_stiffness: bpy.props.FloatProperty(soft_min=0.0, soft_max=1000.0, default=100.0, name="Base Stiffness", description="The speed of the jiggle")
    base_damping: bpy.props.FloatProperty(soft_min=0.0, soft_max=1000.0, name="Base Damping", description="A velocity reduction of the jiggle")
    base_minimum_left: bpy.props.FloatProperty(soft_max=0.0, default=-100.0, name="Base Minimum Left", description="The minimum offset the jiggle on the X")
    base_maximum_left: bpy.props.FloatProperty(soft_min=0.0, default=100.0, name="Base Maximum Left", description="The maximum offset the jiggle on the X")
    base_friction_left: bpy.props.FloatProperty(soft_min=0.0, name="Base Friction Left",
                                                description="When at limit, apply acceleration in opposite direction on the X")
    base_minimum_up: bpy.props.FloatProperty(soft_max=0.0, default=-100.0, name="Base Minimum Up", description="The minimum offset the jiggle on the Y")
    base_maximum_up: bpy.props.FloatProperty(soft_min=0.0, default=100.0, name="Base Maximum Up", description="The maximum offset the jiggle on the Y")
    base_friction_up: bpy.props.FloatProperty(soft_min=0.0, name="Base Friction Up",
                                              description="When at limit, apply acceleration in opposite direction on the Y")
    base_minimum_forward: bpy.props.FloatProperty(soft_max=0.0, default=-100.0, name="Base Minimum Forward",
                                                  description="The minimum offset the jiggle on the Z")
    base_maximum_forward: bpy.props.FloatProperty(soft_min=0.0, default=100.0, name="Base Minimum Forward",
                                                  description="The maximum offset the jiggle on the Z")
    base_friction_forward: bpy.props.FloatProperty(soft_min=0.0, name="Base Friction Forward",
                                                   description="When at limit, apply acceleration in opposite direction on the Z")
    boing_impact_speed: bpy.props.FloatProperty(soft_min=0.0, default=100.0, name="Boing Impact Speed",
                                                description="The speed of the jiggle to create a impact")
    boing_impact_angle: bpy.props.FloatProperty(soft_min=0.0, unit="ROTATION", default=radians(
        45.0), name="Boing Impact Angle", description="The angle difference of velocity to create a impact")
    boing_damping_rate: bpy.props.FloatProperty(soft_min=0.0, default=0.25, name="Boing Damping Rate", description="The speed the jiggle settles")
    boing_frequency: bpy.props.FloatProperty(soft_min=0.0, default=30.0, name="Boing Frequency", description="The speed of the squash and stretch")
    boing_amplitude: bpy.props.FloatProperty(soft_min=0.0, default=0.35, name="Boing Amplitude", description="The strength of the squash and stretch")
    preview: bpy.props.BoolProperty(default=True)
    # Run Time Data
    last_update: bpy.props.FloatProperty(options={"SKIP_SAVE"})
    base_position: bpy.props.FloatVectorProperty(options={"SKIP_SAVE"})
    last_base_position: bpy.props.FloatVectorProperty(options={"SKIP_SAVE"})
    base_velocity: bpy.props.FloatVectorProperty(options={"SKIP_SAVE"})
    base_acceleration: bpy.props.FloatVectorProperty(options={"SKIP_SAVE"})
    tip_position: bpy.props.FloatVectorProperty(options={"SKIP_SAVE"})
    tip_velocity: bpy.props.FloatVectorProperty(options={"SKIP_SAVE"})
    tip_acceleration: bpy.props.FloatVectorProperty(options={"SKIP_SAVE"})
    last_boing_position: bpy.props.FloatVectorProperty(options={"SKIP_SAVE"})
    boing_direction: bpy.props.FloatVectorProperty(options={"SKIP_SAVE"})
    boing_velocity_direction: bpy.props.FloatVectorProperty(options={"SKIP_SAVE"})
    boing_speed: bpy.props.FloatProperty(options={"SKIP_SAVE"})
    boing_time: bpy.props.FloatProperty(options={"SKIP_SAVE"})


def copy_jiggle_procedural(source, destination):
    destination.tip_flex_type = source.tip_flex_type
    destination.length = source.length
    destination.tip_mass = source.tip_mass
    destination.yaw_stiffness = source.yaw_stiffness
    destination.yaw_damping = source.yaw_damping
    destination.pitch_stiffness = source.pitch_stiffness
    destination.pitch_damping = source.pitch_damping
    destination.along_constraint = source.along_constraint
    destination.along_stiffness = source.along_stiffness
    destination.along_damping = source.along_damping
    destination.angle_constraint = source.angle_constraint
    destination.angle_limit = source.angle_limit
    destination.use_friction = source.use_friction
    destination.yaw_constraint = source.yaw_constraint
    destination.yaw_minimum = source.yaw_minimum
    destination.yaw_maximum = source.yaw_maximum
    destination.yaw_friction = source.yaw_friction
    destination.yaw_bounce = source.yaw_bounce
    destination.pitch_constraint = source.pitch_constraint
    destination.pitch_minimum = source.pitch_minimum
    destination.pitch_maximum = source.pitch_maximum
    destination.pitch_friction = source.pitch_friction
    destination.pitch_bounce = source.pitch_bounce
    destination.base_flex_type = source.base_flex_type
    destination.base_mass = source.base_mass
    destination.base_stiffness = source.base_stiffness
    destination.base_damping = source.base_damping
    destination.base_minimum_left = source.base_minimum_left
    destination.base_maximum_left = source.base_maximum_left
    destination.base_friction_left = source.base_friction_left
    destination.base_minimum_up = source.base_minimum_up
    destination.base_maximum_up = source.base_maximum_up
    destination.base_friction_up = source.base_friction_up
    destination.base_minimum_forward = source.base_minimum_forward
    destination.base_maximum_forward = source.base_maximum_forward
    destination.base_friction_forward = source.base_friction_forward
    destination.boing_impact_speed = source.boing_impact_speed
    destination.boing_impact_angle = source.boing_impact_angle
    destination.boing_damping_rate = source.boing_damping_rate
    destination.boing_frequency = source.boing_frequency
    destination.boing_amplitude = source.boing_amplitude


class SourceProceduralBoneDataProperty(bpy.types.PropertyGroup):
    quaternion_procedurals: bpy.props.CollectionProperty(type=QuaternionProceduralProperty)
    active_quaternion_procedural: bpy.props.IntProperty(name="Active Quaternion Procedural")
    aimat_procedurals: bpy.props.CollectionProperty(type=AimatProceduralProperty)
    active_aimat_procedural: bpy.props.IntProperty(name="Active Aimat Procedural")
    jiggle_procedurals: bpy.props.CollectionProperty(type=JiggleProceduralProperty)
    active_jiggle_procedural: bpy.props.IntProperty(name="Active Jiggle Procedural")
    previewing: bpy.props.BoolProperty(update=update_previewing, options={"SKIP_SAVE"})


class PreviewingArmatureObject(bpy.types.PropertyGroup):
    armature: bpy.props.PointerProperty(type=bpy.types.Object)


class SourceProceduralPreviewingDataProperty(bpy.types.PropertyGroup):
    previewers: bpy.props.CollectionProperty(type=PreviewingArmatureObject, options={"SKIP_SAVE"})
    update_rate: bpy.props.IntProperty(name="Update Rate", default=60, min=1, soft_min=20, soft_max=300)
    quaternion_procedural_trigger_copy: bpy.props.PointerProperty(type=QuaternionProceduralTriggerProperty)
    quaternion_procedural_copy: bpy.props.PointerProperty(type=QuaternionProceduralProperty)
    aimat_procedural_copy: bpy.props.PointerProperty(type=AimatProceduralProperty)
    jiggle_procedural_copy: bpy.props.PointerProperty(type=JiggleProceduralProperty)

# endregion


def get_string_after_dot(input_string):
    parts = input_string.split(".", 1)
    if len(parts) > 1:
        return parts[1]
    return input_string

# region Quaternion Procedural Operators


class AddQuaternionProceduralOperator(bpy.types.Operator):
    bl_idname = "source_procedural.quaternion_add"
    bl_label = "Add Quaternion Procedural"
    bl_description = "Adds a new quaternion procedural"

    def execute(self, context):
        source_procedural_bone_data = context.object.source_procedural_bone_data

        source_procedural_bone_data.quaternion_procedurals.add()
        source_procedural_bone_data.active_quaternion_procedural = len(source_procedural_bone_data.quaternion_procedurals) - 1

        return {"FINISHED"}


class RemoveQuaternionProceduralOperator(bpy.types.Operator):
    bl_idname = "source_procedural.quaternion_remove"
    bl_label = "Remove Quaternion Procedural"
    bl_description = "Removes the selected quaternion procedural"

    @classmethod
    def poll(cls, context):
        source_procedural_bone_data = context.object.source_procedural_bone_data

        return len(source_procedural_bone_data.quaternion_procedurals) != 0

    def execute(self, context):
        source_procedural_bone_data = context.object.source_procedural_bone_data
        active_quaternion_procedural = source_procedural_bone_data.quaternion_procedurals[source_procedural_bone_data.active_quaternion_procedural]

        active_quaternion_procedural.preview = False
        source_procedural_bone_data.quaternion_procedurals.remove(source_procedural_bone_data.active_quaternion_procedural)

        if source_procedural_bone_data.active_quaternion_procedural > 0:
            source_procedural_bone_data.active_quaternion_procedural -= 1

        return {"FINISHED"}


class CreateQuaternionProceduralCommandOperator(bpy.types.Operator):
    bl_idname = "source_procedural.quaternion_create_command"
    bl_label = "Create Quaternion Procedural"
    bl_description = "Creates the selected quaternion procedural vrd command to the clipboard"

    @classmethod
    def poll(cls, context):
        source_procedural_bone_data = context.object.source_procedural_bone_data
        active_quaternion_procedural = source_procedural_bone_data.quaternion_procedurals[source_procedural_bone_data.active_quaternion_procedural]

        return len(active_quaternion_procedural.triggers) > 0

    def execute(self, context):
        source_procedural_bone_data = context.object.source_procedural_bone_data
        active_quaternion_procedural = source_procedural_bone_data.quaternion_procedurals[source_procedural_bone_data.active_quaternion_procedural]
        target_bone = context.object.pose.bones[active_quaternion_procedural.target_bone]
        control_bone = context.object.pose.bones[active_quaternion_procedural.control_bone]

        current_position = (target_bone.parent.bone.matrix_local.inverted_safe() @ target_bone.bone.matrix_local).to_translation()

        if active_quaternion_procedural.override_position:
            current_position = Vector(active_quaternion_procedural.position_override)

        target_bone_name = get_string_after_dot(target_bone.name)
        target_bone_parent_name = get_string_after_dot(target_bone.parent.name)
        control_bone_parent_name = get_string_after_dot(control_bone.parent.name)
        control_bone_name = get_string_after_dot(control_bone.name)
        procedural_string = f"<helper> {target_bone_name} {target_bone_parent_name} {control_bone_parent_name} {control_bone_name}\n"

        procedural_string += f"<display> 0 0 0 {active_quaternion_procedural.distance if active_quaternion_procedural.override_position else 0}\n"

        procedural_string += f"<basepos> {current_position.x} {current_position.y} {current_position.z}\n"

        procedural_string += "<rotateaxis> 0 0 0\n"

        procedural_string += "<jointorient> 0 0 0\n"

        for trigger in active_quaternion_procedural.triggers:
            trigger_angle = " ".join([str(degrees(r)) for r in trigger.trigger_angle])
            target_angle = " ".join([str(degrees(r)) for r in trigger.target_angle])
            target_position = " ".join([str(p) for p in trigger.target_position])
            procedural_string += f"<trigger> {degrees(trigger.tolerance)} {trigger_angle} {target_angle} {target_position}\n"

        context.window_manager.clipboard = procedural_string

        return {"FINISHED"}


class CreateAllEnabledQuaternionProceduralCommandOperator(bpy.types.Operator):
    bl_idname = "source_procedural.quaternion_create_all_command_enabled"
    bl_label = "Create All Enabled Quaternion Procedural"
    bl_description = "Creates all the enabled quaternion procedural vrd commands to the clipboard"

    @classmethod
    def poll(cls, context):
        source_procedural_bone_data = context.object.source_procedural_bone_data
        active_count = sum(1 for quaternion_procedural in source_procedural_bone_data.quaternion_procedurals if quaternion_procedural.preview)
        return active_count > 0

    def execute(self, context):
        source_procedural_bone_data = context.object.source_procedural_bone_data
        active_quaternion_procedural_index = source_procedural_bone_data.active_quaternion_procedural
        quaternion_procedurals = source_procedural_bone_data.quaternion_procedurals

        all_procedural_strings = []

        for (procedural_index, quaternion_procedural) in enumerate(quaternion_procedurals):
            if not quaternion_procedural.preview:
                continue

            target_bone = context.object.pose.bones.get(quaternion_procedural.target_bone)
            control_bone = context.object.pose.bones.get(quaternion_procedural.control_bone)

            if target_bone is None or control_bone is None:
                continue

            if target_bone.parent is None or control_bone.parent is None:
                continue

            if not len(quaternion_procedural.triggers):
                continue

            source_procedural_bone_data.active_quaternion_procedural = procedural_index
            bpy.ops.source_procedural.quaternion_create_command()
            all_procedural_strings.append(context.window_manager.clipboard)

        source_procedural_bone_data.active_quaternion_procedural = active_quaternion_procedural_index

        if len(all_procedural_strings):
            context.window_manager.clipboard = "\n".join(all_procedural_strings)

        return {"FINISHED"}


class CopyQuaternionProceduralOperator(bpy.types.Operator):
    bl_idname = "source_procedural.quaternion_copy"
    bl_label = "Copy Quaternion Procedural"
    bl_description = "Copy the quaternion procedural parameters"

    @classmethod
    def poll(cls, context):
        return len(context.object.source_procedural_bone_data.quaternion_procedurals)

    def execute(self, context):
        quaternion_procedural_copy = context.scene.source_procedural_previewing_data.quaternion_procedural_copy
        source_procedural_bone_data = context.object.source_procedural_bone_data
        active_quaternion_procedural = source_procedural_bone_data.quaternion_procedurals[source_procedural_bone_data.active_quaternion_procedural]
        copy_quaternion_procedural(active_quaternion_procedural, quaternion_procedural_copy)
        return {"FINISHED"}


class PasteQuaternionProceduralOperator(bpy.types.Operator):
    bl_idname = "source_procedural.quaternion_paste"
    bl_label = "Paste Quaternion Procedural"
    bl_description = "Paste the quaternion procedural parameters"

    @classmethod
    def poll(cls, context):
        return len(context.object.source_procedural_bone_data.quaternion_procedurals)

    def execute(self, context):
        quaternion_procedural_copy = context.scene.source_procedural_previewing_data.quaternion_procedural_copy
        source_procedural_bone_data = context.object.source_procedural_bone_data
        active_quaternion_procedural = source_procedural_bone_data.quaternion_procedurals[source_procedural_bone_data.active_quaternion_procedural]
        copy_quaternion_procedural(quaternion_procedural_copy, active_quaternion_procedural)
        return {"FINISHED"}


class RemoveAllQuaternionProceduralOperator(bpy.types.Operator):
    bl_idname = "source_procedural.quaternion_remove_all"
    bl_label = "Remove All Quaternion Procedural"
    bl_description = "Removes all quaternion procedurals"

    @classmethod
    def poll(cls, context):
        return len(context.object.source_procedural_bone_data.quaternion_procedurals)

    def execute(self, context):
        source_procedural_bone_data = context.object.source_procedural_bone_data
        source_procedural_bone_data.quaternion_procedurals.clear()
        return {"FINISHED"}


# endregion

# region Quaternion Procedural Trigger Operators


class AddQuaternionProceduralTriggerOperator(bpy.types.Operator):
    bl_idname = "source_procedural.quaternion_trigger_add"
    bl_label = "Add Quaternion Procedural Trigger"
    bl_description = "Adds a new trigger"

    @classmethod
    def poll(cls, context):
        source_procedural_bone_data = context.object.source_procedural_bone_data
        active_quaternion_procedural = source_procedural_bone_data.quaternion_procedurals[source_procedural_bone_data.active_quaternion_procedural]

        return len(active_quaternion_procedural.triggers) < 32

    def execute(self, context):
        source_procedural_bone_data = context.object.source_procedural_bone_data
        active_quaternion_procedural = source_procedural_bone_data.quaternion_procedurals[source_procedural_bone_data.active_quaternion_procedural]

        active_quaternion_procedural.triggers.add()
        active_quaternion_procedural.active_trigger = len(active_quaternion_procedural.triggers) - 1

        return {"FINISHED"}


class RemoveQuaternionProceduralTriggerOperator(bpy.types.Operator):
    bl_idname = "source_procedural.quaternion_trigger_remove"
    bl_label = "Remove Quaternion Procedural Trigger"
    bl_description = "Removes the selected trigger"

    @classmethod
    def poll(cls, context):
        source_procedural_bone_data = context.object.source_procedural_bone_data
        active_quaternion_procedural = source_procedural_bone_data.quaternion_procedurals[source_procedural_bone_data.active_quaternion_procedural]

        return len(active_quaternion_procedural.triggers) != 0

    def execute(self, context):
        source_procedural_bone_data = context.object.source_procedural_bone_data
        active_quaternion_procedural = source_procedural_bone_data.quaternion_procedurals[source_procedural_bone_data.active_quaternion_procedural]

        active_quaternion_procedural.triggers.remove(active_quaternion_procedural.active_trigger)

        if active_quaternion_procedural.active_trigger > 0:
            active_quaternion_procedural.active_trigger -= 1

        return {"FINISHED"}


class MoveUpQuaternionProceduralTriggerOperator(bpy.types.Operator):
    bl_idname = "source_procedural.quaternion_trigger_move_up"
    bl_label = "Move Up Quaternion Procedural Trigger"
    bl_description = "Moves the selected trigger up"

    @classmethod
    def poll(cls, context):
        source_procedural_bone_data = context.object.source_procedural_bone_data
        active_quaternion_procedural = source_procedural_bone_data.quaternion_procedurals[source_procedural_bone_data.active_quaternion_procedural]

        return active_quaternion_procedural.active_trigger > 0

    def execute(self, context):
        source_procedural_bone_data = context.object.source_procedural_bone_data
        active_quaternion_procedural = source_procedural_bone_data.quaternion_procedurals[source_procedural_bone_data.active_quaternion_procedural]

        active_quaternion_procedural.triggers.move(active_quaternion_procedural.active_trigger, active_quaternion_procedural.active_trigger - 1)
        active_quaternion_procedural.active_trigger -= 1

        return {"FINISHED"}


class MoveDownQuaternionProceduralTriggerOperator(bpy.types.Operator):
    bl_idname = "source_procedural.quaternion_trigger_move_down"
    bl_label = "Move Down Quaternion Procedural Trigger"
    bl_description = "Moves the selected trigger down"

    @classmethod
    def poll(cls, context):
        source_procedural_bone_data = context.object.source_procedural_bone_data
        active_quaternion_procedural = source_procedural_bone_data.quaternion_procedurals[source_procedural_bone_data.active_quaternion_procedural]

        return active_quaternion_procedural.active_trigger < len(active_quaternion_procedural.triggers) - 1

    def execute(self, context):
        source_procedural_bone_data = context.object.source_procedural_bone_data
        active_quaternion_procedural = source_procedural_bone_data.quaternion_procedurals[source_procedural_bone_data.active_quaternion_procedural]

        active_quaternion_procedural.triggers.move(active_quaternion_procedural.active_trigger, active_quaternion_procedural.active_trigger + 1)
        active_quaternion_procedural.active_trigger += 1

        return {"FINISHED"}


class SetTriggerQuaternionProceduralTriggerOperator(bpy.types.Operator):
    bl_idname = "source_procedural.quaternion_trigger_set_trigger"
    bl_label = "Set Trigger Quaternion Procedural Trigger"
    bl_description = "Sets the trigger angle of the selected trigger to the current angle of the control bone"

    @classmethod
    def poll(cls, context):
        return context.mode == "POSE"

    def execute(self, context):
        source_procedural_bone_data = context.object.source_procedural_bone_data
        active_quaternion_procedural = source_procedural_bone_data.quaternion_procedurals[source_procedural_bone_data.active_quaternion_procedural]
        control_bone = context.object.pose.bones[active_quaternion_procedural.control_bone]

        parent_space = control_bone.parent.bone.matrix_local.inverted_safe() @ control_bone.bone.matrix_local
        current_rotation = parent_space @ control_bone.matrix_basis
        active_quaternion_procedural.triggers[active_quaternion_procedural.active_trigger].trigger_angle = current_rotation.to_euler()

        return {"FINISHED"}


class SetAngleQuaternionProceduralTriggerOperator(bpy.types.Operator):
    bl_idname = "source_procedural.quaternion_trigger_set_angle"
    bl_label = "Set Angle Quaternion Procedural Trigger"
    bl_description = "Sets the target angle of the selected trigger to the current angle of the target bone"

    @classmethod
    def poll(cls, context):
        return context.mode == "POSE"

    def execute(self, context):
        source_procedural_bone_data = context.object.source_procedural_bone_data
        active_quaternion_procedural = source_procedural_bone_data.quaternion_procedurals[source_procedural_bone_data.active_quaternion_procedural]
        target_bone = context.object.pose.bones[active_quaternion_procedural.target_bone]

        parent_space = target_bone.parent.bone.matrix_local.inverted_safe() @ target_bone.bone.matrix_local
        current_rotation = parent_space @ target_bone.matrix_basis
        active_quaternion_procedural.triggers[active_quaternion_procedural.active_trigger].target_angle = current_rotation.to_euler()

        return {"FINISHED"}


class SetPositionQuaternionProceduralTriggerOperator(bpy.types.Operator):
    bl_idname = "source_procedural.quaternion_trigger_set_position"
    bl_label = "Set Position Quaternion Procedural Trigger"
    bl_description = "Sets the target position of the selected trigger to the current position of the target bone"

    @classmethod
    def poll(cls, context):
        return context.mode == "POSE"

    def execute(self, context):
        source_procedural_bone_data = context.object.source_procedural_bone_data
        active_quaternion_procedural = source_procedural_bone_data.quaternion_procedurals[source_procedural_bone_data.active_quaternion_procedural]
        control_bone = context.object.pose.bones[active_quaternion_procedural.control_bone]
        target_bone = context.object.pose.bones[active_quaternion_procedural.target_bone]

        base_position = (target_bone.parent.bone.matrix_local.inverted_safe() @ target_bone.bone.matrix_local).to_translation()
        current_position = (target_bone.parent.bone.matrix_local.inverted_safe() @ target_bone.bone.matrix_local @ target_bone.matrix_basis).to_translation()

        if not active_quaternion_procedural.override_position:
            active_quaternion_procedural.triggers[active_quaternion_procedural.active_trigger].target_position = current_position - base_position
            return {"FINISHED"}

        override_position = Vector(active_quaternion_procedural.position_override)
        distance = active_quaternion_procedural.distance / 100.0
        control_bone_position = (control_bone.parent.bone.matrix_local.inverted_safe() @ control_bone.bone.matrix_local).to_translation() * distance

        active_quaternion_procedural.triggers[active_quaternion_procedural.active_trigger].target_position = current_position - \
            override_position - control_bone_position
        return {"FINISHED"}


class PreviewQuaternionProceduralTriggerOperator(bpy.types.Operator):
    bl_idname = "source_procedural.quaternion_trigger_preview"
    bl_label = "Preview Quaternion Procedural Trigger"
    bl_description = "Sets the trigger angle, target angle, and target position of the selected trigger"

    @classmethod
    def poll(cls, context):
        return context.mode == "POSE"

    def execute(self, context):
        source_procedural_bone_data = context.object.source_procedural_bone_data
        active_quaternion_procedural = source_procedural_bone_data.quaternion_procedurals[source_procedural_bone_data.active_quaternion_procedural]
        active_trigger = active_quaternion_procedural.triggers[active_quaternion_procedural.active_trigger]
        control_bone = context.object.pose.bones[active_quaternion_procedural.control_bone]
        target_bone = context.object.pose.bones[active_quaternion_procedural.target_bone]

        trigger_rotation = Euler(active_trigger.trigger_angle).to_matrix()
        control_bone.matrix_basis = (control_bone.bone.matrix_local.to_3x3().inverted_safe() @
                                     control_bone.parent.bone.matrix_local.to_3x3() @ trigger_rotation).to_4x4()

        target_rotation = Euler(active_trigger.target_angle).to_matrix()
        target_bone_inverted = target_bone.bone.matrix_local.inverted_safe()

        if not active_quaternion_procedural.override_position:
            base_position = (target_bone.parent.bone.matrix_local.inverted_safe() @ target_bone.bone.matrix_local).to_translation()
            target_translation = Matrix.Translation(base_position + Vector(active_trigger.target_position))
            target_bone.matrix_basis = target_bone_inverted @ target_bone.parent.bone.matrix_local @ target_translation @ target_rotation.to_4x4()
            return {"FINISHED"}

        override_position = Vector(active_quaternion_procedural.position_override)
        distance = active_quaternion_procedural.distance / 100.0
        control_bone_position = (control_bone.parent.bone.matrix_local.inverted_safe() @ control_bone.bone.matrix_local).to_translation() * distance
        target_translation = Matrix.Translation(override_position + control_bone_position + Vector(active_trigger.target_position))
        target_bone.matrix_basis = target_bone_inverted @ target_bone.parent.bone.matrix_local @ target_translation @ target_rotation.to_4x4()
        return {"FINISHED"}


class CopyQuaternionProceduralTriggerOperator(bpy.types.Operator):
    bl_idname = "source_procedural.quaternion_trigger_copy"
    bl_label = "Copy Quaternion Procedural Trigger"
    bl_description = "Copy the quaternion procedural trigger parameters"

    @classmethod
    def poll(cls, context):
        source_procedural_bone_data = context.object.source_procedural_bone_data
        active_quaternion_procedural = source_procedural_bone_data.quaternion_procedurals[source_procedural_bone_data.active_quaternion_procedural]
        return len(active_quaternion_procedural.triggers)

    def execute(self, context):
        quaternion_procedural_trigger_copy = context.scene.source_procedural_previewing_data.quaternion_procedural_trigger_copy
        source_procedural_bone_data = context.object.source_procedural_bone_data
        active_quaternion_procedural = source_procedural_bone_data.quaternion_procedurals[source_procedural_bone_data.active_quaternion_procedural]
        active_quaternion_procedural_trigger = active_quaternion_procedural.triggers[active_quaternion_procedural.active_trigger]
        copy_quaternion_procedural_trigger(active_quaternion_procedural_trigger, quaternion_procedural_trigger_copy)
        return {"FINISHED"}


class PasteQuaternionProceduralTriggerOperator(bpy.types.Operator):
    bl_idname = "source_procedural.quaternion_trigger_paste"
    bl_label = "Paste Quaternion Procedural Trigger"
    bl_description = "Paste the quaternion procedural trigger parameters"

    @classmethod
    def poll(cls, context):
        source_procedural_bone_data = context.object.source_procedural_bone_data
        active_quaternion_procedural = source_procedural_bone_data.quaternion_procedurals[source_procedural_bone_data.active_quaternion_procedural]
        return len(active_quaternion_procedural.triggers)

    def execute(self, context):
        quaternion_procedural_trigger_copy = context.scene.source_procedural_previewing_data.quaternion_procedural_trigger_copy
        source_procedural_bone_data = context.object.source_procedural_bone_data
        active_quaternion_procedural = source_procedural_bone_data.quaternion_procedurals[source_procedural_bone_data.active_quaternion_procedural]
        active_quaternion_procedural_trigger = active_quaternion_procedural.triggers[active_quaternion_procedural.active_trigger]
        copy_quaternion_procedural_trigger(quaternion_procedural_trigger_copy, active_quaternion_procedural_trigger)
        return {"FINISHED"}


class RemoveAllQuaternionProceduralTriggerOperator(bpy.types.Operator):
    bl_idname = "source_procedural.quaternion_trigger_remove_all"
    bl_label = "Remove All Quaternion Procedural Triggers"
    bl_description = "Removes all quaternion procedural triggers"

    @classmethod
    def poll(cls, context):
        source_procedural_bone_data = context.object.source_procedural_bone_data
        active_quaternion_procedural = source_procedural_bone_data.quaternion_procedurals[source_procedural_bone_data.active_quaternion_procedural]
        return len(active_quaternion_procedural.triggers)

    def execute(self, context):
        source_procedural_bone_data = context.object.source_procedural_bone_data
        active_quaternion_procedural = source_procedural_bone_data.quaternion_procedurals[source_procedural_bone_data.active_quaternion_procedural]
        active_quaternion_procedural.triggers.clear()
        return {"FINISHED"}

# endregion

# region Aimat Procedural Operators


class AddAimatProceduralOperator(bpy.types.Operator):
    bl_idname = "source_procedural.aimat_add"
    bl_label = "Add Aimat Procedural"
    bl_description = "Adds a new Aimat procedural"

    def execute(self, context):
        source_procedural_bone_data = context.object.source_procedural_bone_data

        source_procedural_bone_data.aimat_procedurals.add()
        source_procedural_bone_data.active_aimat_procedural = len(source_procedural_bone_data.aimat_procedurals) - 1

        return {"FINISHED"}


class RemoveAimatProceduralOperator(bpy.types.Operator):
    bl_idname = "source_procedural.aimat_remove"
    bl_label = "Remove Aimat Procedural"
    bl_description = "Removes the selected aimat procedural"

    @classmethod
    def poll(cls, context):
        source_procedural_bone_data = context.object.source_procedural_bone_data

        return len(source_procedural_bone_data.aimat_procedurals) != 0

    def execute(self, context):
        source_procedural_bone_data = context.object.source_procedural_bone_data
        active_aimat_procedural = source_procedural_bone_data.aimat_procedurals[source_procedural_bone_data.active_aimat_procedural]

        active_aimat_procedural.preview = False
        source_procedural_bone_data.aimat_procedurals.remove(source_procedural_bone_data.active_aimat_procedural)

        if source_procedural_bone_data.active_aimat_procedural > 0:
            source_procedural_bone_data.active_aimat_procedural -= 1

        return {"FINISHED"}


class CreateAimatProceduralCommandOperator(bpy.types.Operator):
    bl_idname = "source_procedural.aimat_create_command"
    bl_label = "Create Aimat Procedural"
    bl_description = "Creates the selected Aimat procedural vrd command to the clipboard"

    def execute(self, context):
        source_procedural_bone_data = context.object.source_procedural_bone_data
        active_aimat_procedural = source_procedural_bone_data.aimat_procedurals[source_procedural_bone_data.active_aimat_procedural]
        procedural_bone = context.object.pose.bones[active_aimat_procedural.procedural_bone]

        procedural_bone_name = get_string_after_dot(procedural_bone.name)
        procedural_bone_parent_name = get_string_after_dot(procedural_bone.parent.name)
        aim_vector = " ".join([str(p) for p in active_aimat_procedural.aim_vector])
        up_vector = " ".join([str(p) for p in active_aimat_procedural.up_vector])
        current_position = (procedural_bone.parent.bone.matrix_local.inverted_safe() @ procedural_bone.bone.matrix_local).to_translation()
        procedural_string = f"<aimconstraint> {procedural_bone_name} {procedural_bone_parent_name} {active_aimat_procedural.aim_target.name}\n"
        procedural_string += f"<aimvector> {aim_vector}\n"
        procedural_string += f"<upvector> {up_vector}\n"
        procedural_string += f"<basepos> {current_position.x} {current_position.y} {current_position.z}\n"

        context.window_manager.clipboard = procedural_string

        return {"FINISHED"}


class CreateAllEnabledAimatProceduralCommandOperator(bpy.types.Operator):
    bl_idname = "source_procedural.aimat_create_all_command_enabled"
    bl_label = "Create All Enabled Aimat Procedural"
    bl_description = "Creates all enabled aimat procedural vrd commands to the clipboard"

    @classmethod
    def poll(cls, context):
        source_procedural_bone_data = context.object.source_procedural_bone_data
        active_count = sum(1 for aimat_procedural in source_procedural_bone_data.aimat_procedurals if aimat_procedural.preview)
        return active_count > 0

    def execute(self, context):
        source_procedural_bone_data = context.object.source_procedural_bone_data
        active_aimat_procedural_index = source_procedural_bone_data.active_aimat_procedural
        aimat_procedurals = source_procedural_bone_data.aimat_procedurals

        all_procedural_strings = []

        for (procedural_index, aimat_procedural) in enumerate(aimat_procedurals):
            if not aimat_procedural.preview:
                continue

            procedural_bone = context.object.pose.bones.get(aimat_procedural.procedural_bone)
            if procedural_bone is None or procedural_bone.parent is None:
                continue

            aim_target = aimat_procedural.aim_target
            if aim_target is None:
                continue

            source_procedural_bone_data.active_aimat_procedural = procedural_index
            bpy.ops.source_procedural.aimat_create_command()
            all_procedural_strings.append(context.window_manager.clipboard)

        source_procedural_bone_data.active_aimat_procedural = active_aimat_procedural_index

        if len(all_procedural_strings):
            context.window_manager.clipboard = "\n".join(all_procedural_strings)

        return {"FINISHED"}


class CopyAimatProceduralOperator(bpy.types.Operator):
    bl_idname = "source_procedural.aimat_copy"
    bl_label = "Copy Aimat Procedural"
    bl_description = "Copy the Aimat procedural parameters"

    @classmethod
    def poll(cls, context):
        return len(context.object.source_procedural_bone_data.aimat_procedurals)

    def execute(self, context):
        aimat_procedural_copy = context.scene.source_procedural_previewing_data.aimat_procedural_copy
        source_procedural_bone_data = context.object.source_procedural_bone_data
        active_aimat_procedural = source_procedural_bone_data.aimat_procedurals[source_procedural_bone_data.active_aimat_procedural]
        copy_aimat_procedural(active_aimat_procedural, aimat_procedural_copy)
        return {"FINISHED"}


class PasteAimatProceduralOperator(bpy.types.Operator):
    bl_idname = "source_procedural.aimat_paste"
    bl_label = "Paste Aimat Procedural"
    bl_description = "Paste the Aimat procedural parameters"

    @classmethod
    def poll(cls, context):
        return len(context.object.source_procedural_bone_data.quaternion_procedurals)

    def execute(self, context):
        aimat_procedural_copy = context.scene.source_procedural_previewing_data.aimat_procedural_copy
        source_procedural_bone_data = context.object.source_procedural_bone_data
        active_aimat_procedural = source_procedural_bone_data.aimat_procedurals[source_procedural_bone_data.active_aimat_procedural]
        copy_aimat_procedural(aimat_procedural_copy, active_aimat_procedural)
        return {"FINISHED"}


class RemoveAllAimatProceduralOperator(bpy.types.Operator):
    bl_idname = "source_procedural.aimat_remove_all"
    bl_label = "Remove All Aimat Procedural"
    bl_description = "Removes all Aimat procedurals"

    @classmethod
    def poll(cls, context):
        return len(context.object.source_procedural_bone_data.aimat_procedurals)

    def execute(self, context):
        source_procedural_bone_data = context.object.source_procedural_bone_data
        source_procedural_bone_data.aimat_procedurals.clear()
        return {"FINISHED"}


# endregion

# region Jiggle Procedural Operators


class AddJiggleProceduralOperator(bpy.types.Operator):
    bl_idname = "source_procedural.jiggle_add"
    bl_label = "Add Jiggle Procedural"
    bl_description = "Adds a new jiggle procedural"

    def execute(self, context):
        source_procedural_bone_data = context.object.source_procedural_bone_data

        source_procedural_bone_data.jiggle_procedurals.add()
        source_procedural_bone_data.active_jiggle_procedural = len(source_procedural_bone_data.jiggle_procedurals) - 1

        return {"FINISHED"}


class RemoveJiggleProceduralOperator(bpy.types.Operator):
    bl_idname = "source_procedural.jiggle_remove"
    bl_label = "Remove Jiggle Procedural"
    bl_description = "Removes the selected jiggle procedural"

    @classmethod
    def poll(cls, context):
        source_procedural_bone_data = context.object.source_procedural_bone_data

        return len(source_procedural_bone_data.jiggle_procedurals) != 0

    def execute(self, context):
        source_procedural_bone_data = context.object.source_procedural_bone_data
        active_jiggle_procedural = source_procedural_bone_data.jiggle_procedurals[source_procedural_bone_data.active_jiggle_procedural]

        active_jiggle_procedural.preview = False
        source_procedural_bone_data.jiggle_procedurals.remove(source_procedural_bone_data.active_jiggle_procedural)

        if source_procedural_bone_data.active_jiggle_procedural > 0:
            source_procedural_bone_data.active_jiggle_procedural -= 1

        return {"FINISHED"}


class CreateJiggleProceduralCommandOperator(bpy.types.Operator):
    bl_idname = "source_procedural.jiggle_create_command"
    bl_label = "Create Jiggle Procedural"
    bl_description = "Creates the selected jiggle procedural qc command to the clipboard"

    def execute(self, context):
        source_procedural_bone_data = context.object.source_procedural_bone_data
        active_jiggle_procedural = source_procedural_bone_data.jiggle_procedurals[source_procedural_bone_data.active_jiggle_procedural]
        target_bone = context.object.pose.bones[active_jiggle_procedural.target_bone]

        procedural_string = f"$jigglebone \"{target_bone.name}\" {{\n"

        if active_jiggle_procedural.tip_flex_type == "RIGID":
            procedural_string += "\tis_rigid {\n"
            procedural_string += f"\t\tlength {active_jiggle_procedural.length}\n"
            procedural_string += f"\t\ttip_mass {active_jiggle_procedural.tip_mass}\n"

        if active_jiggle_procedural.tip_flex_type == "FLEXIBLE":
            procedural_string += "\tis_flexible {\n"
            procedural_string += f"\t\tlength {active_jiggle_procedural.length}\n"
            procedural_string += f"\t\ttip_mass {active_jiggle_procedural.tip_mass}\n"
            procedural_string += f"\t\tyaw_stiffness {active_jiggle_procedural.yaw_stiffness}\n"
            procedural_string += f"\t\tyaw_damping {active_jiggle_procedural.yaw_damping}\n"
            procedural_string += f"\t\tpitch_stiffness {active_jiggle_procedural.pitch_stiffness}\n"
            procedural_string += f"\t\tpitch_damping {active_jiggle_procedural.pitch_damping}\n"

            if active_jiggle_procedural.along_constraint == "NONE":
                procedural_string += "\t\tallow_length_flex\n"
                procedural_string += f"\t\talong_stiffness {active_jiggle_procedural.along_stiffness}\n"
                procedural_string += f"\t\talong_damping {active_jiggle_procedural.along_damping}\n"

        if active_jiggle_procedural.tip_flex_type == "RIGID" or active_jiggle_procedural.tip_flex_type == "FLEXIBLE":
            if active_jiggle_procedural.angle_constraint == "CONSTRAINT":
                procedural_string += f"\t\tangle_constraint {degrees(active_jiggle_procedural.angle_limit)}\n"

            if active_jiggle_procedural.yaw_constraint == "CONSTRAINT":
                procedural_string += f"\t\tyaw_constraint {degrees(active_jiggle_procedural.yaw_minimum)} {degrees(active_jiggle_procedural.yaw_maximum)}\n"
                if active_jiggle_procedural.use_friction:
                    procedural_string += f"\t\tyaw_friction {active_jiggle_procedural.yaw_friction}\n"
                    procedural_string += f"\t\tyaw_bounce {active_jiggle_procedural.yaw_bounce}\n"

            if active_jiggle_procedural.pitch_constraint == "CONSTRAINT":
                procedural_string += f"\t\tpitch_constraint {degrees(active_jiggle_procedural.pitch_minimum)} {degrees(active_jiggle_procedural.pitch_maximum)}\n"
                if active_jiggle_procedural.use_friction:
                    procedural_string += f"\t\tpitch_friction {active_jiggle_procedural.pitch_friction}\n"
                    procedural_string += f"\t\tpitch_bounce {active_jiggle_procedural.pitch_bounce}\n"

            procedural_string += "\t}\n"

        if active_jiggle_procedural.base_flex_type == "SPRING":
            procedural_string += "\thas_base_spring {\n"

            procedural_string += f"\t\tbase_mass {active_jiggle_procedural.base_mass}\n"
            procedural_string += f"\t\tstiffness {active_jiggle_procedural.base_stiffness}\n"
            procedural_string += f"\t\tdamping {active_jiggle_procedural.base_damping}\n"
            procedural_string += f"\t\tleft_constraint {active_jiggle_procedural.base_minimum_left} {active_jiggle_procedural.base_maximum_left}\n"
            procedural_string += f"\t\tleft_friction {active_jiggle_procedural.base_friction_left}\n"
            procedural_string += f"\t\tup_constraint {active_jiggle_procedural.base_minimum_up} {active_jiggle_procedural.base_maximum_up}\n"
            procedural_string += f"\t\tup_friction {active_jiggle_procedural.base_friction_up}\n"
            procedural_string += f"\t\tforward_constraint {active_jiggle_procedural.base_minimum_forward} {active_jiggle_procedural.base_maximum_forward}\n"
            procedural_string += f"\t\tforward_friction {active_jiggle_procedural.base_friction_forward}\n"

            procedural_string += "\t}\n"

        if active_jiggle_procedural.base_flex_type == "BOING":
            procedural_string += "\tis_boing {\n"

            procedural_string += f"\t\timpact_speed {active_jiggle_procedural.boing_impact_speed}\n"
            procedural_string += f"\t\timpact_angle {degrees(active_jiggle_procedural.boing_impact_angle)}\n"
            procedural_string += f"\t\tdamping_rate {active_jiggle_procedural.boing_damping_rate}\n"
            procedural_string += f"\t\tfrequency {active_jiggle_procedural.boing_frequency}\n"
            procedural_string += f"\t\tamplitude {active_jiggle_procedural.boing_amplitude}\n"

            procedural_string += "\t}\n"

        procedural_string += "}\n"

        context.window_manager.clipboard = procedural_string

        return {"FINISHED"}


class CreateAllEnabledJiggleProceduralCommandOperator(bpy.types.Operator):
    bl_idname = "source_procedural.jiggle_create_all_command_enabled"
    bl_label = "Create All Enabled Jiggle Procedural"
    bl_description = "Creates all enabled jiggle procedural qc commands to the clipboard"

    @classmethod
    def poll(cls, context):
        source_procedural_bone_data = context.object.source_procedural_bone_data
        active_count = sum(1 for jiggle_procedural in source_procedural_bone_data.jiggle_procedurals if jiggle_procedural.preview)
        return active_count > 0

    def execute(self, context):
        source_procedural_bone_data = context.object.source_procedural_bone_data
        active_jiggle_procedural_index = source_procedural_bone_data.active_jiggle_procedural
        jiggle_procedurals = source_procedural_bone_data.jiggle_procedurals

        all_procedural_strings = []

        for (procedural_index, jiggle_procedural) in enumerate(jiggle_procedurals):
            if not jiggle_procedural.preview:
                continue

            target_bone = context.object.pose.bones.get(jiggle_procedural.target_bone)
            if target_bone is None:
                continue

            source_procedural_bone_data.active_jiggle_procedural = procedural_index
            bpy.ops.source_procedural.jiggle_create_command()
            all_procedural_strings.append(context.window_manager.clipboard)

        source_procedural_bone_data.active_jiggle_procedural = active_jiggle_procedural_index

        if len(all_procedural_strings):
            context.window_manager.clipboard = "\n".join(all_procedural_strings)

        return {"FINISHED"}


class ClearJiggleBoneDataOperator(bpy.types.Operator):
    bl_idname = "source_procedural.jiggle_clear"
    bl_label = "Clear All Jiggle Preview Data"
    bl_description = "Clears all jiggle procedural preview data"

    def execute(self, context):
        for bone in context.object.pose.bones:
            constraint_postfixes = (" Jiggle Bone Location", " Jiggle Bone Rotation", " Jiggle Bone Scale")
            constraints_to_remove = [constraint for constraint in bone.constraints if constraint.name.endswith(constraint_postfixes)]
            for constraint_name in constraints_to_remove:
                bone.constraints.remove(constraint_name)

        return {"FINISHED"}


class CopyJiggleProceduralOperator(bpy.types.Operator):
    bl_idname = "source_procedural.jiggle_copy"
    bl_label = "Copy Jiggle Procedural"
    bl_description = "Copy the jiggle procedural parameters"

    @classmethod
    def poll(cls, context):
        return len(context.object.source_procedural_bone_data.jiggle_procedurals)

    def execute(self, context):
        jiggle_procedural_copy = context.scene.source_procedural_previewing_data.jiggle_procedural_copy
        source_procedural_bone_data = context.object.source_procedural_bone_data
        active_jiggle_procedural = source_procedural_bone_data.jiggle_procedurals[source_procedural_bone_data.active_jiggle_procedural]
        copy_jiggle_procedural(active_jiggle_procedural, jiggle_procedural_copy)
        return {"FINISHED"}


class PasteJiggleProceduralOperator(bpy.types.Operator):
    bl_idname = "source_procedural.jiggle_paste"
    bl_label = "Paste Jiggle Procedural"
    bl_description = "Paste the jiggle procedural parameters"

    @classmethod
    def poll(cls, context):
        return len(context.object.source_procedural_bone_data.jiggle_procedurals)

    def execute(self, context):
        jiggle_procedural_copy = context.scene.source_procedural_previewing_data.jiggle_procedural_copy
        source_procedural_bone_data = context.object.source_procedural_bone_data
        active_jiggle_procedural = source_procedural_bone_data.jiggle_procedurals[source_procedural_bone_data.active_jiggle_procedural]
        copy_jiggle_procedural(jiggle_procedural_copy, active_jiggle_procedural)
        return {"FINISHED"}


class AddSelectedBonesJiggleProceduralOperator(bpy.types.Operator):
    bl_idname = "source_procedural.jiggle_add_selected_bones"
    bl_label = "Add All Selected Bones"
    bl_description = "Adds jiggle procedural for all selected bones"

    @classmethod
    def poll(cls, context):
        return context.mode == "POSE" and len(context.selected_pose_bones)

    def execute(self, context):
        for selected_bone in bpy.context.selected_pose_bones:
            bpy.ops.source_procedural.jiggle_add()
            source_procedural_bone_data = context.object.source_procedural_bone_data
            active_jiggle_procedural = source_procedural_bone_data.jiggle_procedurals[source_procedural_bone_data.active_jiggle_procedural]
            active_jiggle_procedural.name = selected_bone.name + " Jiggle"
            active_jiggle_procedural.target_bone = selected_bone.name

        return {"FINISHED"}


class AddSelectedBonesFromActiveJiggleProceduralOperator(bpy.types.Operator):
    bl_idname = "source_procedural.jiggle_add_selected__bones_from_active"
    bl_label = "Add All Selected Bones From Active"
    bl_description = "Adds jiggle procedural for all selected bones with the active jiggle procedural"

    @classmethod
    def poll(cls, context):
        return context.mode == "POSE" and len(context.object.source_procedural_bone_data.jiggle_procedurals)

    def execute(self, context):
        source_procedural_bone_data = context.object.source_procedural_bone_data
        active_jiggle_procedural = source_procedural_bone_data.jiggle_procedurals[source_procedural_bone_data.active_jiggle_procedural]

        for selected_bone in bpy.context.selected_pose_bones:
            if selected_bone == context.active_pose_bone:
                continue

            bpy.ops.source_procedural.jiggle_add()
            source_procedural_bone_data = context.object.source_procedural_bone_data
            created_jiggle_procedural = source_procedural_bone_data.jiggle_procedurals[source_procedural_bone_data.active_jiggle_procedural]
            created_jiggle_procedural.name = selected_bone.name + " Jiggle"
            created_jiggle_procedural.target_bone = selected_bone.name
            copy_jiggle_procedural(active_jiggle_procedural, created_jiggle_procedural)

        return {"FINISHED"}


class RemoveAllJiggleProceduralOperator(bpy.types.Operator):
    bl_idname = "source_procedural.jiggle_remove_all"
    bl_label = "Remove All Jiggle Procedural"
    bl_description = "Removes all jiggle procedurals"

    @classmethod
    def poll(cls, context):
        return len(context.object.source_procedural_bone_data.jiggle_procedurals)

    def execute(self, context):
        source_procedural_bone_data = context.object.source_procedural_bone_data
        source_procedural_bone_data.jiggle_procedurals.clear()
        return {"FINISHED"}

# endregion

# region UI


class PreviewingOptionsPanel(bpy.types.Panel):
    bl_category = "Src Proc Bones"
    bl_label = "Previewing Options"
    bl_idname = "VIEW3D_PT_PreviewingOptions"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_options = {"HIDE_HEADER"}

    @classmethod
    def poll(cls, context):
        return context.object is not None and context.object.type == "ARMATURE"

    def draw(self, context):
        layout = self.layout
        source_procedural_bone_data = context.object.source_procedural_bone_data

        previewing_text = "Disable Previewing" if source_procedural_bone_data.previewing else "Enable Previewing"
        previewing_icon = "PLAY" if source_procedural_bone_data.previewing else "PAUSE"
        layout.prop(source_procedural_bone_data, "previewing", text=previewing_text, icon_only=True, icon=previewing_icon)

        source_procedural_previewing_data = context.scene.source_procedural_previewing_data
        layout.prop(source_procedural_previewing_data, "update_rate")

        layout.operator(ClearJiggleBoneDataOperator.bl_idname)


class QuaternionProceduralContextMenu(bpy.types.Menu):
    bl_idname = "OBJECT_MT_QuaternionProceduralContextMenu"
    bl_label = "Jiggle Context"

    def draw(self, context):
        layout = self.layout
        layout.operator(CreateAllEnabledQuaternionProceduralCommandOperator.bl_idname)
        layout.operator(CopyQuaternionProceduralOperator.bl_idname)
        layout.operator(PasteQuaternionProceduralOperator.bl_idname)
        layout.operator(RemoveAllQuaternionProceduralOperator.bl_idname)


class QuaternionProceduralList(bpy.types.UIList):
    bl_idname = "OBJECT_UL_QuaternionProceduralList"

    def draw_item(self, context, layout, data, item, icon, active_data, active_propname):
        layout.prop(item, "name", text="", emboss=False, icon_value=icon)
        layout.prop(item, "preview", text="", icon="PLAY" if item.preview else "PAUSE")


class QuaternionProceduralTriggerContextMenu(bpy.types.Menu):
    bl_idname = "OBJECT_MT_QuaternionProceduralTriggerContextMenu"
    bl_label = "Jiggle Context"

    def draw(self, context):
        layout = self.layout
        layout.operator(CopyQuaternionProceduralTriggerOperator.bl_idname)
        layout.operator(PasteQuaternionProceduralTriggerOperator.bl_idname)
        layout.operator(RemoveAllQuaternionProceduralTriggerOperator.bl_idname)


class QuaternionProceduralTriggerList(bpy.types.UIList):
    bl_idname = "OBJECT_UL_QuaternionProceduralTriggerList"

    def draw_item(self, context, layout, data, item, icon, active_data, active_propname):
        layout.prop(item, "name", text="", emboss=False, icon_value=icon)

        source_procedural_bone_data = context.object.source_procedural_bone_data
        active_quaternion_procedural = source_procedural_bone_data.quaternion_procedurals[source_procedural_bone_data.active_quaternion_procedural]
        source_procedural_previewing_data = context.scene.source_procedural_previewing_data
        if active_quaternion_procedural.preview and len(source_procedural_previewing_data.previewers):
            layout.label(text=f"Weight: {item.weight * 100:.2f}%")


class QuaternionProceduralPanel(bpy.types.Panel):
    bl_category = "Src Proc Bones"
    bl_label = "Quaternion Procedurals"
    bl_idname = "VIEW3D_PT_QuaternionProceduralPanel"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_options = {"DEFAULT_CLOSED"}

    @classmethod
    def poll(cls, context):
        return context.object is not None and context.object.type == "ARMATURE"

    def draw(self, context):
        layout = self.layout
        source_procedural_bone_data = context.object.source_procedural_bone_data

        row = layout.row(align=True)
        row.template_list(QuaternionProceduralList.bl_idname, "", source_procedural_bone_data,
                          "quaternion_procedurals", source_procedural_bone_data, "active_quaternion_procedural")

        col = row.column(align=True)
        col.operator(AddQuaternionProceduralOperator.bl_idname, text="", icon="ADD")
        col.operator(RemoveQuaternionProceduralOperator.bl_idname, text="", icon="REMOVE")
        col.separator()
        col.menu(QuaternionProceduralContextMenu.bl_idname, text="", icon="DOWNARROW_HLT")

        if len(source_procedural_bone_data.quaternion_procedurals) == 0:
            return

        active_quaternion_procedural = source_procedural_bone_data.quaternion_procedurals[source_procedural_bone_data.active_quaternion_procedural]

        col = layout.column(align=True)
        col.prop(active_quaternion_procedural, "name", text="")

        target_bone = context.object.pose.bones.get(active_quaternion_procedural.target_bone)
        control_bone = context.object.pose.bones.get(active_quaternion_procedural.control_bone)

        box = col.box()
        row = box.row(align=True)
        if target_bone is None:
            row.alert = True
            label_row = box.row()
            label_row.alert = True
            label_row.label(text="Target Bone Not Found!")
        elif control_bone is None:
            row.alert = True
            label_row = box.row()
            label_row.alert = True
            label_row.label(text="Control Bone Not Found!")
        elif target_bone == control_bone:
            row.alert = True
            label_row = box.row()
            label_row.alert = True
            label_row.label(text="Target Bone Is The Same As Control Bone!")
        elif target_bone.parent is None:
            row.alert = True
            label_row = box.row()
            label_row.alert = True
            label_row.label(text="Target Bone Has No Parent!")
        elif control_bone.parent is None:
            row.alert = True
            label_row = box.row()
            label_row.alert = True
            label_row.label(text="Control Bone Has No Parent!")

        row.label(text="Target Bone:")
        row.prop_search(active_quaternion_procedural, "target_bone", context.object.pose, "bones", text="")
        row.label(text="Control Bone:")
        row.prop_search(active_quaternion_procedural, "control_bone", context.object.pose, "bones", text="")

        if row.alert:
            return

        col = box.column(align=True)
        row = col.row(align=True)
        row.prop(active_quaternion_procedural, "override_position", text="Override Position")
        if active_quaternion_procedural.override_position:
            row.prop(active_quaternion_procedural, "position_override", text="")
            col.prop(active_quaternion_procedural, "distance", text="Distance")

        preview_text = "Disallow Previewing" if active_quaternion_procedural.preview else "Allow Previewing"
        preview_icon = "PLAY" if active_quaternion_procedural.preview else "PAUSE"
        box.prop(active_quaternion_procedural, "preview", text=preview_text, icon_only=True, icon=preview_icon)

        box.operator(CreateQuaternionProceduralCommandOperator.bl_idname, text="Create Procedural")

        row = box.row(align=True)
        row.template_list(QuaternionProceduralTriggerList.bl_idname, "", active_quaternion_procedural,
                          "triggers", active_quaternion_procedural, "active_trigger")
        col = row.column(align=True)
        col.operator(AddQuaternionProceduralTriggerOperator.bl_idname, text="", icon="ADD")
        col.operator(RemoveQuaternionProceduralTriggerOperator.bl_idname, text="", icon="REMOVE")
        col.separator()
        col.menu(QuaternionProceduralTriggerContextMenu.bl_idname, text="", icon="DOWNARROW_HLT")
        col.separator()
        col.operator(MoveUpQuaternionProceduralTriggerOperator.bl_idname, text="", icon="TRIA_UP")
        col.operator(MoveDownQuaternionProceduralTriggerOperator.bl_idname, text="", icon="TRIA_DOWN")

        if len(active_quaternion_procedural.triggers) == 0:
            return

        active_trigger = active_quaternion_procedural.triggers[active_quaternion_procedural.active_trigger]

        col = box.box().column(align=True)
        col.prop(active_trigger, "name", text="")

        row = col.row(align=True)
        if abs(active_trigger.tolerance) <= float_info.epsilon:
            row.alert = True
            label_row = col.row()
            label_row.alert = True
            label_row.label(text="Tolerance Is Too Small!")
        row.prop(active_trigger, "tolerance", text="Tolerance")

        row = col.row(align=True)
        row.prop(active_trigger, "trigger_angle", text="Trigger")
        row.operator(SetTriggerQuaternionProceduralTriggerOperator.bl_idname, text="Set")

        row = col.row(align=True)
        row.prop(active_trigger, "target_angle", text="Angle")
        row.operator(SetAngleQuaternionProceduralTriggerOperator.bl_idname, text="Set")

        row = col.row(align=True)
        row.prop(active_trigger, "target_position", text="Position")
        row.operator(SetPositionQuaternionProceduralTriggerOperator.bl_idname, text="Set")

        col.operator(PreviewQuaternionProceduralTriggerOperator.bl_idname, text="Preview Trigger")


class AimatProceduralContextMenu(bpy.types.Menu):
    bl_idname = "OBJECT_MT_AimatProceduralContextMenu"
    bl_label = "Aimat Context"

    def draw(self, context):
        layout = self.layout

        layout.operator(CreateAllEnabledAimatProceduralCommandOperator.bl_idname)
        layout.operator(CopyAimatProceduralOperator.bl_idname)
        layout.operator(PasteAimatProceduralOperator.bl_idname)
        layout.operator(RemoveAllAimatProceduralOperator.bl_idname)


class AimatProceduralList(bpy.types.UIList):
    bl_idname = "OBJECT_UL_AimatProceduralList"

    def draw_item(self, context, layout, data, item, icon, active_data, active_propname):
        layout.prop(item, "name", text="", emboss=False, icon_value=icon)
        layout.prop(item, "preview", text="", icon="PLAY" if item.preview else "PAUSE")


class AimatProceduralPanel(bpy.types.Panel):
    bl_category = "Src Proc Bones"
    bl_label = "Aimat Procedurals"
    bl_idname = "VIEW3D_PT_AimatProceduralPanel"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_options = {"DEFAULT_CLOSED"}

    @classmethod
    def poll(cls, context):
        return context.object is not None and context.object.type == "ARMATURE"

    def draw(self, context):
        layout = self.layout
        source_procedural_bone_data = context.object.source_procedural_bone_data

        row = layout.row(align=True)
        row.template_list(AimatProceduralList.bl_idname, "", source_procedural_bone_data,
                          "aimat_procedurals", source_procedural_bone_data, "active_aimat_procedural")

        col = row.column(align=True)
        col.operator(AddAimatProceduralOperator.bl_idname, text="", icon="ADD")
        col.operator(RemoveAimatProceduralOperator.bl_idname, text="", icon="REMOVE")
        col.separator()
        col.menu(AimatProceduralContextMenu.bl_idname, text="", icon="DOWNARROW_HLT")

        if len(source_procedural_bone_data.aimat_procedurals) == 0:
            return

        active_aimat_procedural = source_procedural_bone_data.aimat_procedurals[source_procedural_bone_data.active_aimat_procedural]

        col = layout.column(align=True)
        col.prop(active_aimat_procedural, "name", text="")

        box = col.box()
        col = box.column(align=True)

        col.prop_search(active_aimat_procedural, "procedural_bone", context.object.pose, "bones", text="")
        col.prop(active_aimat_procedural, "aim_target", text="")

        procedural_bone = context.object.pose.bones.get(active_aimat_procedural.procedural_bone)
        aim_target = active_aimat_procedural.aim_target

        row = box.row(align=True)
        if procedural_bone is None:
            row.alert = True
            label_row = box.row()
            label_row.alert = True
            label_row.label(text="Procedural Bone Not Found!")
        elif aim_target is None:
            row.alert = True
            label_row = box.row()
            label_row.alert = True
            label_row.label(text="Aim Target Not Found!")
        elif procedural_bone.parent is None:
            row.alert = True
            label_row = box.row()
            label_row.alert = True
            label_row.label(text="Procedural Bone Has No Parent!")

        if row.alert:
            return

        preview_text = "Disallow Previewing" if active_aimat_procedural.preview else "Allow Previewing"
        preview_icon = "PLAY" if active_aimat_procedural.preview else "PAUSE"
        box.prop(active_aimat_procedural, "preview", text=preview_text, icon_only=True, icon=preview_icon)
        box.operator(CreateAimatProceduralCommandOperator.bl_idname, text="Create Procedural")

        box.prop(active_aimat_procedural, "aim_vector")
        box.prop(active_aimat_procedural, "up_vector")


class JiggleProceduralContextMenu(bpy.types.Menu):
    bl_idname = "OBJECT_MT_JiggleProceduralContextMenu"
    bl_label = "Jiggle Context"

    def draw(self, context):
        layout = self.layout

        layout.operator(CreateAllEnabledJiggleProceduralCommandOperator.bl_idname)
        layout.operator(CopyJiggleProceduralOperator.bl_idname)
        layout.operator(PasteJiggleProceduralOperator.bl_idname)
        layout.operator(AddSelectedBonesJiggleProceduralOperator.bl_idname)
        layout.operator(AddSelectedBonesFromActiveJiggleProceduralOperator.bl_idname)
        layout.operator(RemoveAllJiggleProceduralOperator.bl_idname)


class JiggleProceduralList(bpy.types.UIList):
    bl_idname = "OBJECT_UL_JiggleProceduralList"

    def draw_item(self, context, layout, data, item, icon, active_data, active_propname):
        layout.prop(item, "name", text="", emboss=False, icon_value=icon)
        layout.prop(item, "preview", text="", icon="PLAY" if item.preview else "PAUSE")


class JiggleProceduralPanel(bpy.types.Panel):
    bl_category = "Src Proc Bones"
    bl_label = "Jiggle Procedurals"
    bl_idname = "VIEW3D_PT_JiggleProceduralPanel"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_options = {"DEFAULT_CLOSED"}

    @classmethod
    def poll(cls, context):
        return context.object is not None and context.object.type == "ARMATURE"

    def draw(self, context):
        layout = self.layout
        source_procedural_bone_data = context.object.source_procedural_bone_data

        row = layout.row(align=True)
        row.template_list(JiggleProceduralList.bl_idname, "", source_procedural_bone_data,
                          "jiggle_procedurals", source_procedural_bone_data, "active_jiggle_procedural")

        col = row.column(align=True)
        col.operator(AddJiggleProceduralOperator.bl_idname, text="", icon="ADD")
        col.operator(RemoveJiggleProceduralOperator.bl_idname, text="", icon="REMOVE")
        col.separator()
        col.menu(JiggleProceduralContextMenu.bl_idname, text="", icon="DOWNARROW_HLT")

        if len(source_procedural_bone_data.jiggle_procedurals) == 0:
            return

        active_jiggle_procedural = source_procedural_bone_data.jiggle_procedurals[source_procedural_bone_data.active_jiggle_procedural]

        col = layout.column(align=True)
        col.prop(active_jiggle_procedural, "name", text="")

        box = col.box()
        row = box.row(align=True)
        row.prop_search(active_jiggle_procedural, "target_bone", context.object.pose, "bones", text="")

        target_bone = context.object.pose.bones.get(active_jiggle_procedural.target_bone)

        if target_bone is None:
            return

        preview_text = "Disallow Previewing" if active_jiggle_procedural.preview else "Allow Previewing"
        preview_icon = "PLAY" if active_jiggle_procedural.preview else "PAUSE"
        box.prop(active_jiggle_procedural, "preview", text=preview_text, icon_only=True, icon=preview_icon)
        box.operator(CreateJiggleProceduralCommandOperator.bl_idname, text="Create Procedural")

        col = box.column(align=True)
        col.label(text="Tip Flex Type")
        col.prop(active_jiggle_procedural, "tip_flex_type", text="")

        if active_jiggle_procedural.tip_flex_type == "RIGID" or active_jiggle_procedural.tip_flex_type == "FLEXIBLE":
            col = col.box().column(align=True)

            col.prop(active_jiggle_procedural, "length")
            col.prop(active_jiggle_procedural, "tip_mass")

            if active_jiggle_procedural.tip_flex_type == "FLEXIBLE":
                col.prop(active_jiggle_procedural, "yaw_stiffness")
                col.prop(active_jiggle_procedural, "yaw_damping")
                col.prop(active_jiggle_procedural, "pitch_stiffness")
                col.prop(active_jiggle_procedural, "pitch_damping")

                col.label(text="Along Constraint")
                col.prop(active_jiggle_procedural, "along_constraint", text="")
                if active_jiggle_procedural.along_constraint == "NONE":
                    col.prop(active_jiggle_procedural, "along_stiffness")
                    col.prop(active_jiggle_procedural, "along_damping")

            col.label(text="Angle Constraint")
            col.prop(active_jiggle_procedural, "angle_constraint", text="")
            if active_jiggle_procedural.angle_constraint == "CONSTRAINT":
                col.prop(active_jiggle_procedural, "angle_limit")

            if active_jiggle_procedural.yaw_constraint == "CONSTRAINT" or active_jiggle_procedural.pitch_constraint == "CONSTRAINT":
                col.prop(active_jiggle_procedural, "use_friction")

            col.label(text="Yaw Constraint")
            col.prop(active_jiggle_procedural, "yaw_constraint", text="")
            if active_jiggle_procedural.yaw_constraint == "CONSTRAINT":
                row = col.row(align=True)
                row.prop(active_jiggle_procedural, "yaw_minimum")
                row.prop(active_jiggle_procedural, "yaw_maximum")
                if active_jiggle_procedural.use_friction:
                    col.prop(active_jiggle_procedural, "yaw_friction")
                    col.prop(active_jiggle_procedural, "yaw_bounce")

            col.label(text="Pitch Constraint")
            col.prop(active_jiggle_procedural, "pitch_constraint", text="")
            if active_jiggle_procedural.pitch_constraint == "CONSTRAINT":
                row = col.row(align=True)
                row.prop(active_jiggle_procedural, "pitch_minimum")
                row.prop(active_jiggle_procedural, "pitch_maximum")
                if active_jiggle_procedural.use_friction:
                    col.prop(active_jiggle_procedural, "pitch_friction")
                    col.prop(active_jiggle_procedural, "pitch_bounce")

        col = box.column(align=True)
        col.label(text="Base Flex Type")
        col.prop(active_jiggle_procedural, "base_flex_type", text="")
        if active_jiggle_procedural.base_flex_type == "SPRING":
            col = col.box().column(align=True)

            col.prop(active_jiggle_procedural, "base_mass")
            col.prop(active_jiggle_procedural, "base_stiffness")
            col.prop(active_jiggle_procedural, "base_damping")
            col.label(text="Left Constraint")
            row = col.row(align=True)
            row.prop(active_jiggle_procedural, "base_minimum_left", text="")
            row.prop(active_jiggle_procedural, "base_maximum_left", text="")
            col.prop(active_jiggle_procedural, "base_friction_left")
            col.label(text="Up Constraint")
            row = col.row(align=True)
            row.prop(active_jiggle_procedural, "base_minimum_up", text="")
            row.prop(active_jiggle_procedural, "base_maximum_up", text="")
            col.prop(active_jiggle_procedural, "base_friction_up")
            col.label(text="Forward Constraint")
            row = col.row(align=True)
            row.prop(active_jiggle_procedural, "base_minimum_forward", text="")
            row.prop(active_jiggle_procedural, "base_maximum_forward", text="")
            col.prop(active_jiggle_procedural, "base_friction_forward")

        if active_jiggle_procedural.base_flex_type == "BOING":
            col = col.box().column(align=True)

            col.prop(active_jiggle_procedural, "boing_impact_speed")
            col.prop(active_jiggle_procedural, "boing_impact_angle")
            col.prop(active_jiggle_procedural, "boing_damping_rate")
            col.prop(active_jiggle_procedural, "boing_frequency")
            col.prop(active_jiggle_procedural, "boing_amplitude")


# endregion


def register():
    # Properties
    bpy.utils.register_class(QuaternionProceduralTriggerProperty)
    bpy.utils.register_class(QuaternionProceduralProperty)
    bpy.utils.register_class(AimatProceduralProperty)
    bpy.utils.register_class(JiggleProceduralProperty)
    bpy.utils.register_class(SourceProceduralBoneDataProperty)
    bpy.utils.register_class(PreviewingArmatureObject)
    bpy.utils.register_class(SourceProceduralPreviewingDataProperty)

    # Quaternion Procedural Operators
    bpy.utils.register_class(AddQuaternionProceduralOperator)
    bpy.utils.register_class(RemoveQuaternionProceduralOperator)
    bpy.utils.register_class(CreateQuaternionProceduralCommandOperator)
    bpy.utils.register_class(CreateAllEnabledQuaternionProceduralCommandOperator)
    bpy.utils.register_class(CopyQuaternionProceduralOperator)
    bpy.utils.register_class(PasteQuaternionProceduralOperator)
    bpy.utils.register_class(RemoveAllQuaternionProceduralOperator)

    # Quaternion Procedural Trigger Operators
    bpy.utils.register_class(AddQuaternionProceduralTriggerOperator)
    bpy.utils.register_class(RemoveQuaternionProceduralTriggerOperator)
    bpy.utils.register_class(MoveUpQuaternionProceduralTriggerOperator)
    bpy.utils.register_class(MoveDownQuaternionProceduralTriggerOperator)
    bpy.utils.register_class(SetTriggerQuaternionProceduralTriggerOperator)
    bpy.utils.register_class(SetAngleQuaternionProceduralTriggerOperator)
    bpy.utils.register_class(SetPositionQuaternionProceduralTriggerOperator)
    bpy.utils.register_class(PreviewQuaternionProceduralTriggerOperator)
    bpy.utils.register_class(CopyQuaternionProceduralTriggerOperator)
    bpy.utils.register_class(PasteQuaternionProceduralTriggerOperator)
    bpy.utils.register_class(RemoveAllQuaternionProceduralTriggerOperator)

    # Aimat Procedural Operators
    bpy.utils.register_class(AddAimatProceduralOperator)
    bpy.utils.register_class(RemoveAimatProceduralOperator)
    bpy.utils.register_class(CreateAimatProceduralCommandOperator)
    bpy.utils.register_class(CreateAllEnabledAimatProceduralCommandOperator)
    bpy.utils.register_class(CopyAimatProceduralOperator)
    bpy.utils.register_class(PasteAimatProceduralOperator)
    bpy.utils.register_class(RemoveAllAimatProceduralOperator)

    # Jiggle Procedural Operators
    bpy.utils.register_class(AddJiggleProceduralOperator)
    bpy.utils.register_class(RemoveJiggleProceduralOperator)
    bpy.utils.register_class(CreateJiggleProceduralCommandOperator)
    bpy.utils.register_class(CreateAllEnabledJiggleProceduralCommandOperator)
    bpy.utils.register_class(ClearJiggleBoneDataOperator)
    bpy.utils.register_class(CopyJiggleProceduralOperator)
    bpy.utils.register_class(PasteJiggleProceduralOperator)
    bpy.utils.register_class(AddSelectedBonesJiggleProceduralOperator)
    bpy.utils.register_class(AddSelectedBonesFromActiveJiggleProceduralOperator)
    bpy.utils.register_class(RemoveAllJiggleProceduralOperator)

    # UI
    bpy.utils.register_class(PreviewingOptionsPanel)
    bpy.utils.register_class(QuaternionProceduralContextMenu)
    bpy.utils.register_class(QuaternionProceduralList)
    bpy.utils.register_class(QuaternionProceduralTriggerContextMenu)
    bpy.utils.register_class(QuaternionProceduralTriggerList)
    bpy.utils.register_class(QuaternionProceduralPanel)
    bpy.utils.register_class(AimatProceduralContextMenu)
    bpy.utils.register_class(AimatProceduralList)
    bpy.utils.register_class(AimatProceduralPanel)
    bpy.utils.register_class(JiggleProceduralContextMenu)
    bpy.utils.register_class(JiggleProceduralList)
    bpy.utils.register_class(JiggleProceduralPanel)

    bpy.types.Object.source_procedural_bone_data = bpy.props.PointerProperty(type=SourceProceduralBoneDataProperty)
    bpy.types.Scene.source_procedural_previewing_data = bpy.props.PointerProperty(type=SourceProceduralPreviewingDataProperty)


def unregister():
    # Properties
    bpy.utils.unregister_class(QuaternionProceduralTriggerProperty)
    bpy.utils.unregister_class(QuaternionProceduralProperty)
    bpy.utils.unregister_class(AimatProceduralProperty)
    bpy.utils.unregister_class(JiggleProceduralProperty)
    bpy.utils.unregister_class(SourceProceduralBoneDataProperty)
    bpy.utils.unregister_class(PreviewingArmatureObject)
    bpy.utils.unregister_class(SourceProceduralPreviewingDataProperty)

    # Quaternion Procedural Operators
    bpy.utils.unregister_class(AddQuaternionProceduralOperator)
    bpy.utils.unregister_class(RemoveQuaternionProceduralOperator)
    bpy.utils.unregister_class(CreateQuaternionProceduralCommandOperator)
    bpy.utils.unregister_class(CreateAllEnabledQuaternionProceduralCommandOperator)
    bpy.utils.unregister_class(CopyQuaternionProceduralOperator)
    bpy.utils.unregister_class(PasteQuaternionProceduralOperator)
    bpy.utils.unregister_class(RemoveAllQuaternionProceduralOperator)

    # Quaternion Procedural Trigger Operators
    bpy.utils.unregister_class(AddQuaternionProceduralTriggerOperator)
    bpy.utils.unregister_class(RemoveQuaternionProceduralTriggerOperator)
    bpy.utils.unregister_class(MoveUpQuaternionProceduralTriggerOperator)
    bpy.utils.unregister_class(MoveDownQuaternionProceduralTriggerOperator)
    bpy.utils.unregister_class(SetTriggerQuaternionProceduralTriggerOperator)
    bpy.utils.unregister_class(SetAngleQuaternionProceduralTriggerOperator)
    bpy.utils.unregister_class(SetPositionQuaternionProceduralTriggerOperator)
    bpy.utils.unregister_class(PreviewQuaternionProceduralTriggerOperator)
    bpy.utils.unregister_class(CopyQuaternionProceduralTriggerOperator)
    bpy.utils.unregister_class(PasteQuaternionProceduralTriggerOperator)
    bpy.utils.unregister_class(RemoveAllQuaternionProceduralTriggerOperator)

    # Aimat Procedural Operators
    bpy.utils.unregister_class(AddAimatProceduralOperator)
    bpy.utils.unregister_class(RemoveAimatProceduralOperator)
    bpy.utils.unregister_class(CreateAimatProceduralCommandOperator)
    bpy.utils.unregister_class(CreateAllEnabledAimatProceduralCommandOperator)
    bpy.utils.unregister_class(CopyAimatProceduralOperator)
    bpy.utils.unregister_class(PasteAimatProceduralOperator)
    bpy.utils.unregister_class(RemoveAllAimatProceduralOperator)

    # Jiggle Procedural Operators
    bpy.utils.unregister_class(AddJiggleProceduralOperator)
    bpy.utils.unregister_class(RemoveJiggleProceduralOperator)
    bpy.utils.unregister_class(CreateJiggleProceduralCommandOperator)
    bpy.utils.unregister_class(CreateAllEnabledJiggleProceduralCommandOperator)
    bpy.utils.unregister_class(ClearJiggleBoneDataOperator)
    bpy.utils.unregister_class(CopyJiggleProceduralOperator)
    bpy.utils.unregister_class(PasteJiggleProceduralOperator)
    bpy.utils.unregister_class(AddSelectedBonesJiggleProceduralOperator)
    bpy.utils.unregister_class(AddSelectedBonesFromActiveJiggleProceduralOperator)
    bpy.utils.unregister_class(RemoveAllJiggleProceduralOperator)

    # UI
    bpy.utils.unregister_class(PreviewingOptionsPanel)
    bpy.utils.unregister_class(QuaternionProceduralContextMenu)
    bpy.utils.unregister_class(QuaternionProceduralList)
    bpy.utils.unregister_class(QuaternionProceduralTriggerContextMenu)
    bpy.utils.unregister_class(QuaternionProceduralTriggerList)
    bpy.utils.unregister_class(QuaternionProceduralPanel)
    bpy.utils.unregister_class(AimatProceduralContextMenu)
    bpy.utils.unregister_class(AimatProceduralList)
    bpy.utils.unregister_class(AimatProceduralPanel)
    bpy.utils.unregister_class(JiggleProceduralContextMenu)
    bpy.utils.unregister_class(JiggleProceduralList)
    bpy.utils.unregister_class(JiggleProceduralPanel)

    if bpy.app.timers.is_registered(previewing):
        bpy.app.timers.unregister(previewing)

    del bpy.types.Object.source_procedural_bone_data
    del bpy.types.Scene.source_procedural_previewing_data
