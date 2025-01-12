
import bpy
from sys import float_info
from math import degrees, radians, acos
from mathutils import Euler, Matrix, Vector, Quaternion

bl_info = {
    "name": "Source Engine Procedural Bones",
    "category": "Animation",
    "description": "A panel that helps create procedural bones for source engine models.",
    "author": "Jakobg1215",
    "version": (2, 0, 2),
    "blender": (2, 80, 0),
    "location": "View3D > Src Proc Bones",
    "tracker_url": "https://github.com/Jakobg1215/srcprocbones/issues",
}

# region Properties


class QuaternionProceduralTriggerProperty(bpy.types.PropertyGroup):
    name: bpy.props.StringProperty(default="New Trigger", name="Name", description="The name of the trigger")
    tolerance: bpy.props.FloatProperty(default=radians(90), precision=6, soft_min=0, unit='ROTATION',
                                       description="The angle cone the control bone angle to the angle trigger to activate the trigger")
    trigger_angle: bpy.props.FloatVectorProperty(precision=6, unit='ROTATION', name="Trigger Angle",
                                                 description="The angle the control bone should be at to activate the trigger")
    target_angle: bpy.props.FloatVectorProperty(precision=6, unit='ROTATION', name="Target Angle",
                                                description="The angle the target bone will be set to when the trigger is active")
    target_position: bpy.props.FloatVectorProperty(precision=6, name="Target Position", description="The added offset position when the trigger is active")


class QuaternionProceduralProperty(bpy.types.PropertyGroup):
    name: bpy.props.StringProperty(default="New Quaternion Procedural", name="Name", description="The name of the quaternion procedural")
    target_bone: bpy.props.StringProperty(name="Target Bone", description="The bone that will be procedurally animated")
    control_bone: bpy.props.StringProperty(name="Control Bone", description="The bone that will drive the target bone")
    override_position: bpy.props.BoolProperty(default=False, description="Allows to override the position of the target bone")
    position_override: bpy.props.FloatVectorProperty(precision=6, name="Position", description="The offset position of the target bone's parent")
    distance: bpy.props.FloatProperty(soft_min=0, soft_max=100,
                                      description="The percentage of the control bone's position offset from its parent added to the target bone's position")
    triggers: bpy.props.CollectionProperty(type=QuaternionProceduralTriggerProperty)
    active_trigger: bpy.props.IntProperty(name="Active Trigger")
    preview: bpy.props.BoolProperty()


class SourceProceduralBoneDataProperty(bpy.types.PropertyGroup):
    quaternion_procedurals: bpy.props.CollectionProperty(type=QuaternionProceduralProperty)
    active_quaternion_procedural: bpy.props.IntProperty(name="Active Quaternion Procedural")

# endregion


def matrix_transpose(matrix):
    translation = Vector((matrix[0][3], matrix[1][3], matrix[2][3]))
    row_x = Vector((matrix[0][0], matrix[1][0], matrix[2][0]))
    row_y = Vector((matrix[0][1], matrix[1][1], matrix[2][1]))
    row_z = Vector((matrix[0][2], matrix[1][2], matrix[2][2]))

    return Matrix(([matrix[0][0], matrix[1][0], matrix[2][0], -translation.dot(row_x)],
                   [matrix[0][1], matrix[1][1], matrix[2][1], -translation.dot(row_y)],
                   [matrix[0][2], matrix[1][2], matrix[2][2], -translation.dot(row_z)],
                   [matrix[0][3], matrix[1][3], matrix[2][3], matrix[3][3]]))


# region Quaternion Procedural Operators


class AddQuaternionProceduralOperator(bpy.types.Operator):
    bl_idname = "source_procedural.quaternion_add"
    bl_label = "Add Quaternion Procedural"
    bl_description = "Adds a new quaternion procedural"

    def execute(self, context):
        source_procedural_bone_data = context.object.source_procedural_bone_data

        source_procedural_bone_data.quaternion_procedurals.add()
        source_procedural_bone_data.active_quaternion_procedural = len(source_procedural_bone_data.quaternion_procedurals) - 1

        return {'FINISHED'}


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

        return {'FINISHED'}


class PreviewQuaternionProceduralOperator(bpy.types.Operator):
    bl_idname = "source_procedural.quaternion_preview"
    bl_label = "Preview Quaternion Procedural"
    bl_description = "Previews the selected quaternion procedural"

    def __init__(self):
        self.active_quaternion_procedural = None
        self.timer = None

    @classmethod
    def poll(cls, context):
        source_procedural_bone_data = context.object.source_procedural_bone_data
        active_quaternion_procedural = source_procedural_bone_data.quaternion_procedurals[source_procedural_bone_data.active_quaternion_procedural]

        return len(active_quaternion_procedural.triggers) > 0

    def execute(self, context):
        source_procedural_bone_data = context.object.source_procedural_bone_data
        active_quaternion_procedural = source_procedural_bone_data.quaternion_procedurals[source_procedural_bone_data.active_quaternion_procedural]

        if not active_quaternion_procedural.preview:
            active_quaternion_procedural.preview = True
            self.active_quaternion_procedural = active_quaternion_procedural
            self.timer = context.window_manager.event_timer_add(1/context.scene.render.fps, window=context.window)
            context.window_manager.modal_handler_add(self)
            return {'RUNNING_MODAL'}

        active_quaternion_procedural.preview = False
        self.cancel(context)

        return {'FINISHED'}

    def modal(self, context, event):
        if event.type != 'TIMER':
            return {'PASS_THROUGH'}

        if context.object is None or context.object.type != 'ARMATURE':
            self.cancel(context)
            return {'FINISHED'}

        active_quaternion_procedural = self.active_quaternion_procedural

        if not active_quaternion_procedural.preview:
            self.cancel(context)
            return {'FINISHED'}

        if len(active_quaternion_procedural.triggers) == 0:
            self.cancel(context)
            return {'FINISHED'}

        target_bone = context.object.pose.bones.get(active_quaternion_procedural.target_bone)
        if target_bone is None:
            self.cancel(context)
            return {'FINISHED'}
        control_bone = context.object.pose.bones.get(active_quaternion_procedural.control_bone)
        if control_bone is None:
            self.cancel(context)
            return {'FINISHED'}

        if target_bone is None or control_bone is None:
            self.cancel(context)
            return {'FINISHED'}

        if target_bone == control_bone:
            self.cancel(context)
            return {'FINISHED'}

        if target_bone.parent is None or control_bone.parent is None:
            self.cancel(context)
            return {'FINISHED'}

        weights = [i for i in range(32)]
        scale = 0

        transpose_parent = matrix_transpose(control_bone.parent.bone.matrix_local)
        parent_space = transpose_parent @ control_bone.bone.matrix_local
        current_rotation = parent_space @ control_bone.matrix_basis
        for index, trigger in enumerate(active_quaternion_procedural.triggers):
            if trigger.tolerance <= float_info.epsilon:
                self.report({'ERROR'}, f"\"{trigger.name}\" Tolerance Is Too Small")
                self.cancel(context)
                return {'FINISHED'}

            dot = abs(Euler(trigger.trigger_angle).to_quaternion().dot(current_rotation.to_quaternion()))
            dot = min(max(dot, -1), 1)
            weights[index] = 1 - (2 * acos(dot) * (1 / trigger.tolerance))
            weights[index] = max(0, weights[index])
            scale += weights[index]

        if scale <= 0.001:
            active_trigger = active_quaternion_procedural.triggers[0]
            target_rotation = Euler(active_trigger.target_angle).to_matrix()
            target_bone_transposed = matrix_transpose(target_bone.bone.matrix_local)

            if not active_quaternion_procedural.override_position:
                base_position = (matrix_transpose(target_bone.parent.bone.matrix_local) @ target_bone.bone.matrix_local).to_translation()
                target_translation = Matrix.Translation(base_position + Vector(active_trigger.target_position))
                target_bone.matrix_basis = target_bone_transposed @ target_bone.parent.bone.matrix_local @ target_translation @ target_rotation.to_4x4()
                return {'PASS_THROUGH'}

            override_position = Vector(active_quaternion_procedural.position_override)
            distance = active_quaternion_procedural.distance / 100
            control_bone_position = (matrix_transpose(control_bone.parent.bone.matrix_local) @ control_bone.bone.matrix_local).to_translation() * distance
            target_translation = Matrix.Translation(override_position + control_bone_position + Vector(active_trigger.target_position))
            target_bone.matrix_basis = target_bone_transposed @ target_bone.parent.bone.matrix_local @ target_translation @ target_rotation.to_4x4()
            return {'PASS_THROUGH'}

        scale = 1.0 / scale

        quat = Quaternion((0, 0, 0, 0))
        pos = Vector((0, 0, 0))

        def quaternion_align(p, q, qt):
            a = 0
            b = 0

            for i in range(4):
                a += (p[i]-q[i])*(p[i]-q[i])
                b += (p[i]+q[i])*(p[i]+q[i])

            if a > b:
                for i in range(4):
                    print(i)
                    qt[i] = -q[i]
                return

            if not qt is q:
                for i in range(4):
                    qt[i] = q[i]

        for index, trigger in enumerate(active_quaternion_procedural.triggers):
            if weights[index] == 0:
                continue

            s = weights[index] * scale
            target_quaternion = Euler(trigger.target_angle).to_quaternion()
            target_position = Vector(trigger.target_position)

            quaternion_align(target_quaternion, quat, quat)

            quat.x += s * target_quaternion.x
            quat.y += s * target_quaternion.y
            quat.z += s * target_quaternion.z
            quat.w += s * target_quaternion.w

            pos.x += s * target_position.x
            pos.y += s * target_position.y
            pos.z += s * target_position.z

        target_rotation = quat.to_matrix()
        target_bone_transposed = matrix_transpose(target_bone.bone.matrix_local)

        if not active_quaternion_procedural.override_position:
            base_position = (matrix_transpose(target_bone.parent.bone.matrix_local) @ target_bone.bone.matrix_local).to_translation()
            target_translation = Matrix.Translation(base_position + pos)
            target_bone.matrix_basis = target_bone_transposed @ target_bone.parent.bone.matrix_local @ target_translation @ target_rotation.to_4x4()
            return {'PASS_THROUGH'}

        override_position = Vector(active_quaternion_procedural.position_override)
        distance = active_quaternion_procedural.distance / 100
        control_bone_position = (matrix_transpose(control_bone.parent.bone.matrix_local) @ control_bone.bone.matrix_local).to_translation() * distance
        target_translation = Matrix.Translation(override_position + control_bone_position + pos)
        target_bone.matrix_basis = target_bone_transposed @ target_bone.parent.bone.matrix_local @ target_translation @ target_rotation.to_4x4()
        return {'PASS_THROUGH'}

    def cancel(self, context):
        if self.timer is not None:
            context.window_manager.event_timer_remove(self.timer)
            self.timer = None

        if self.active_quaternion_procedural is not None:
            self.active_quaternion_procedural.preview = False
            self.active_quaternion_procedural = None


class CopyQuaternionProceduralOperator(bpy.types.Operator):
    bl_idname = "source_procedural.quaternion_copy"
    bl_label = "Copy Quaternion Procedural"
    bl_description = "Copies the selected quaternion procedural to the clipboard"

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

        current_position = (matrix_transpose(target_bone.parent.bone.matrix_local) @ target_bone.bone.matrix_local).to_translation()

        if active_quaternion_procedural.override_position:
            current_position = Vector(active_quaternion_procedural.position_override)

        procedural_string = ""

        def get_string_after_dot(input_string):
            parts = input_string.split('.', 1)
            if len(parts) > 1:
                return parts[1]
            return input_string

        procedural_string += "<helper> " + get_string_after_dot(target_bone.name) + " " + \
            get_string_after_dot(target_bone.parent.name) + " " + \
            get_string_after_dot(control_bone.parent.name) + " " + \
            get_string_after_dot(control_bone.name) + "\n"

        procedural_string += "<display> 0 0 0 " + str(active_quaternion_procedural.distance if active_quaternion_procedural.override_position else 0) + "\n"

        procedural_string += "<basepos> " + str(current_position.x) + " " + str(current_position.y) + " " + str(current_position.z) + "\n"

        procedural_string += "<rotateaxis> 0 0 0\n"

        procedural_string += "<jointorient> 0 0 0\n"

        for trigger in active_quaternion_procedural.triggers:
            procedural_string += "<trigger> " + str(degrees(trigger.tolerance)) + " " + \
                " ".join([str(degrees(r)) for r in trigger.trigger_angle]) + " " + \
                " ".join([str(degrees(r)) for r in trigger.target_angle]) + " " + \
                " ".join([str(p) for p in trigger.target_position]) + "\n"

        context.window_manager.clipboard = procedural_string

        return {'FINISHED'}

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

        return {'FINISHED'}


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

        return {'FINISHED'}


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

        return {'FINISHED'}


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

        return {'FINISHED'}


class SetTriggerQuaternionProceduralTriggerOperator(bpy.types.Operator):
    bl_idname = "source_procedural.quaternion_trigger_set_trigger"
    bl_label = "Set Trigger Quaternion Procedural Trigger"
    bl_description = "Sets the trigger angle of the selected trigger to the current angle of the control bone"

    @classmethod
    def poll(cls, context):
        return context.mode == 'POSE'

    def execute(self, context):
        source_procedural_bone_data = context.object.source_procedural_bone_data
        active_quaternion_procedural = source_procedural_bone_data.quaternion_procedurals[source_procedural_bone_data.active_quaternion_procedural]
        control_bone = context.object.pose.bones[active_quaternion_procedural.control_bone]

        transpose_parent = matrix_transpose(control_bone.parent.bone.matrix_local)
        parent_space = transpose_parent @ control_bone.bone.matrix_local
        current_rotation = parent_space @ control_bone.matrix_basis
        active_quaternion_procedural.triggers[active_quaternion_procedural.active_trigger].trigger_angle = current_rotation.to_euler()

        return {'FINISHED'}


class SetAngleQuaternionProceduralTriggerOperator(bpy.types.Operator):
    bl_idname = "source_procedural.quaternion_trigger_set_angle"
    bl_label = "Set Angle Quaternion Procedural Trigger"
    bl_description = "Sets the target angle of the selected trigger to the current angle of the target bone"

    @classmethod
    def poll(cls, context):
        return context.mode == 'POSE'

    def execute(self, context):
        source_procedural_bone_data = context.object.source_procedural_bone_data
        active_quaternion_procedural = source_procedural_bone_data.quaternion_procedurals[source_procedural_bone_data.active_quaternion_procedural]
        target_bone = context.object.pose.bones[active_quaternion_procedural.target_bone]

        transpose_parent = matrix_transpose(target_bone.parent.bone.matrix_local)
        parent_space = transpose_parent @ target_bone.bone.matrix_local
        current_rotation = parent_space @ target_bone.matrix_basis
        active_quaternion_procedural.triggers[active_quaternion_procedural.active_trigger].target_angle = current_rotation.to_euler()

        return {'FINISHED'}


class SetPositionQuaternionProceduralTriggerOperator(bpy.types.Operator):
    bl_idname = "source_procedural.quaternion_trigger_set_position"
    bl_label = "Set Position Quaternion Procedural Trigger"
    bl_description = "Sets the target position of the selected trigger to the current position of the target bone"

    @classmethod
    def poll(cls, context):
        return context.mode == 'POSE'

    def execute(self, context):
        source_procedural_bone_data = context.object.source_procedural_bone_data
        active_quaternion_procedural = source_procedural_bone_data.quaternion_procedurals[source_procedural_bone_data.active_quaternion_procedural]
        control_bone = context.object.pose.bones[active_quaternion_procedural.control_bone]
        target_bone = context.object.pose.bones[active_quaternion_procedural.target_bone]

        base_position = (matrix_transpose(target_bone.parent.bone.matrix_local) @ target_bone.bone.matrix_local).to_translation()
        current_position = ((matrix_transpose(target_bone.parent.bone.matrix_local) @
                            target_bone.bone.matrix_local) @ target_bone.matrix_basis).to_translation()

        if not active_quaternion_procedural.override_position:
            active_quaternion_procedural.triggers[active_quaternion_procedural.active_trigger].target_position = current_position - base_position
            return {'FINISHED'}

        override_position = Vector(active_quaternion_procedural.position_override)
        distance = active_quaternion_procedural.distance / 100
        control_bone_position = (matrix_transpose(control_bone.parent.bone.matrix_local) @ control_bone.bone.matrix_local).to_translation() * distance

        active_quaternion_procedural.triggers[active_quaternion_procedural.active_trigger].target_position = current_position - \
            override_position - control_bone_position
        return {'FINISHED'}


class PreviewQuaternionProceduralTriggerOperator(bpy.types.Operator):
    bl_idname = "source_procedural.quaternion_trigger_preview"
    bl_label = "Preview Quaternion Procedural Trigger"
    bl_description = "Sets the trigger angle, target angle, and target position of the selected trigger"

    @classmethod
    def poll(cls, context):
        return context.mode == 'POSE'

    def execute(self, context):
        source_procedural_bone_data = context.object.source_procedural_bone_data
        active_quaternion_procedural = source_procedural_bone_data.quaternion_procedurals[source_procedural_bone_data.active_quaternion_procedural]
        active_trigger = active_quaternion_procedural.triggers[active_quaternion_procedural.active_trigger]
        control_bone = context.object.pose.bones[active_quaternion_procedural.control_bone]
        target_bone = context.object.pose.bones[active_quaternion_procedural.target_bone]

        trigger_rotation = Euler(active_trigger.trigger_angle).to_matrix()
        control_bone_transposed = matrix_transpose(control_bone.bone.matrix_local).to_3x3()
        control_bone.matrix_basis = (control_bone_transposed @ control_bone.parent.bone.matrix_local.to_3x3() @ trigger_rotation).to_4x4()

        target_rotation = Euler(active_trigger.target_angle).to_matrix()
        target_bone_transposed = matrix_transpose(target_bone.bone.matrix_local)

        if not active_quaternion_procedural.override_position:
            base_position = (matrix_transpose(target_bone.parent.bone.matrix_local) @ target_bone.bone.matrix_local).to_translation()
            target_translation = Matrix.Translation(base_position + Vector(active_trigger.target_position))
            target_bone.matrix_basis = target_bone_transposed @ target_bone.parent.bone.matrix_local @ target_translation @ target_rotation.to_4x4()
            return {'FINISHED'}

        override_position = Vector(active_quaternion_procedural.position_override)
        distance = active_quaternion_procedural.distance / 100
        control_bone_position = (matrix_transpose(control_bone.parent.bone.matrix_local) @ control_bone.bone.matrix_local).to_translation() * distance
        target_translation = Matrix.Translation(override_position + control_bone_position + Vector(active_trigger.target_position))
        target_bone.matrix_basis = target_bone_transposed @ target_bone.parent.bone.matrix_local @ target_translation @ target_rotation.to_4x4()
        return {'FINISHED'}

# endregion

# region UI


class QuaternionProceduralList(bpy.types.UIList):
    bl_idname = "OBJECT_UL_QuaternionProcedural"

    def draw_item(self, context, layout, data, item, icon, active_data, active_propname):
        layout.prop(item, "name", text="", emboss=False, icon_value=icon)


class QuaternionProceduralTriggerList(bpy.types.UIList):
    bl_idname = "OBJECT_UL_QuaternionProceduralTrigger"

    def draw_item(self, context, layout, data, item, icon, active_data, active_propname):
        layout.prop(item, "name", text="", emboss=False, icon_value=icon)


class ProceduralBonePanel(bpy.types.Panel):
    bl_category = "Src Proc Bones"
    bl_label = "Quaternion Procedurals"
    bl_idname = "VIEW3D_PT_ProceduralBone"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'

    @classmethod
    def poll(cls, context):
        return context.object is not None and context.object.type == 'ARMATURE'

    def draw(self, context):
        layout = self.layout
        source_procedural_bone_data = context.object.source_procedural_bone_data

        row = layout.row(align=True)
        row.template_list(QuaternionProceduralList.bl_idname, "", source_procedural_bone_data,
                          "quaternion_procedurals", source_procedural_bone_data, "active_quaternion_procedural")

        col = row.column(align=True)
        col.operator(AddQuaternionProceduralOperator.bl_idname, text="", icon='ADD')
        col.operator(RemoveQuaternionProceduralOperator.bl_idname, text="", icon='REMOVE')

        if len(source_procedural_bone_data.quaternion_procedurals) == 0:
            return

        active_quaternion_procedural = source_procedural_bone_data.quaternion_procedurals[source_procedural_bone_data.active_quaternion_procedural]

        col = layout.column(align=True)
        col.prop(active_quaternion_procedural, "name", text="")

        box = col.box()
        row = box.row(align=True)
        row.label(text="Target Bone:")
        row.prop_search(active_quaternion_procedural, "target_bone", context.object.pose, "bones", text="")
        row.label(text="Control Bone:")
        row.prop_search(active_quaternion_procedural, "control_bone", context.object.pose, "bones", text="")

        target_bone = context.object.pose.bones.get(active_quaternion_procedural.target_bone)
        control_bone = context.object.pose.bones.get(active_quaternion_procedural.control_bone)

        if target_bone is None or control_bone is None:
            return

        if target_bone == control_bone:
            return

        if target_bone.parent is None or control_bone.parent is None:
            return

        col = box.column(align=True)
        row = col.row(align=True)
        row.prop(active_quaternion_procedural, "override_position", text="Override Position")
        if active_quaternion_procedural.override_position:
            row.prop(active_quaternion_procedural, "position_override", text="")
            col.prop(active_quaternion_procedural, "distance", text="Distance")

        # rotateaxis

        # box.label(text="rotateaxis")

        # jointorient

        # box.label(text="jointorient")

        box.operator(PreviewQuaternionProceduralOperator.bl_idname, text="Preview Procedural", depress=active_quaternion_procedural.preview)

        box.operator(CopyQuaternionProceduralOperator.bl_idname, text="Copy Procedural")

        row = box.row(align=True)
        row.template_list(QuaternionProceduralTriggerList.bl_idname, "", active_quaternion_procedural,
                          "triggers", active_quaternion_procedural, "active_trigger")
        col = row.column(align=True)
        col.operator(AddQuaternionProceduralTriggerOperator.bl_idname, text="", icon='ADD')
        col.operator(RemoveQuaternionProceduralTriggerOperator.bl_idname, text="", icon='REMOVE')
        col.separator()
        col.operator(MoveUpQuaternionProceduralTriggerOperator.bl_idname, text="", icon='TRIA_UP')
        col.operator(MoveDownQuaternionProceduralTriggerOperator.bl_idname, text="", icon='TRIA_DOWN')

        if len(active_quaternion_procedural.triggers) == 0:
            return

        active_trigger = active_quaternion_procedural.triggers[active_quaternion_procedural.active_trigger]

        col = box.box().column(align=True)
        col.prop(active_trigger, "name", text="")
        col.prop(active_trigger, "tolerance", text="Tolerance")

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

# endregion


def register():
    # Properties
    bpy.utils.register_class(QuaternionProceduralTriggerProperty)
    bpy.utils.register_class(QuaternionProceduralProperty)
    bpy.utils.register_class(SourceProceduralBoneDataProperty)

    # Quaternion Procedural Operators
    bpy.utils.register_class(AddQuaternionProceduralOperator)
    bpy.utils.register_class(RemoveQuaternionProceduralOperator)
    bpy.utils.register_class(PreviewQuaternionProceduralOperator)
    bpy.utils.register_class(CopyQuaternionProceduralOperator)

    # Quaternion Procedural Trigger Operators
    bpy.utils.register_class(AddQuaternionProceduralTriggerOperator)
    bpy.utils.register_class(RemoveQuaternionProceduralTriggerOperator)
    bpy.utils.register_class(MoveUpQuaternionProceduralTriggerOperator)
    bpy.utils.register_class(MoveDownQuaternionProceduralTriggerOperator)
    bpy.utils.register_class(SetTriggerQuaternionProceduralTriggerOperator)
    bpy.utils.register_class(SetAngleQuaternionProceduralTriggerOperator)
    bpy.utils.register_class(SetPositionQuaternionProceduralTriggerOperator)
    bpy.utils.register_class(PreviewQuaternionProceduralTriggerOperator)

    # UI
    bpy.utils.register_class(QuaternionProceduralList)
    bpy.utils.register_class(QuaternionProceduralTriggerList)
    bpy.utils.register_class(ProceduralBonePanel)

    bpy.types.Object.source_procedural_bone_data = bpy.props.PointerProperty(type=SourceProceduralBoneDataProperty)


def unregister():
    # Properties
    bpy.utils.unregister_class(QuaternionProceduralTriggerProperty)
    bpy.utils.unregister_class(QuaternionProceduralProperty)
    bpy.utils.unregister_class(SourceProceduralBoneDataProperty)

    # Quaternion Procedural Operators
    bpy.utils.unregister_class(AddQuaternionProceduralOperator)
    bpy.utils.unregister_class(RemoveQuaternionProceduralOperator)
    bpy.utils.unregister_class(PreviewQuaternionProceduralOperator)
    bpy.utils.unregister_class(CopyQuaternionProceduralOperator)

    # Quaternion Procedural Trigger Operators
    bpy.utils.unregister_class(AddQuaternionProceduralTriggerOperator)
    bpy.utils.unregister_class(RemoveQuaternionProceduralTriggerOperator)
    bpy.utils.unregister_class(MoveUpQuaternionProceduralTriggerOperator)
    bpy.utils.unregister_class(MoveDownQuaternionProceduralTriggerOperator)
    bpy.utils.unregister_class(SetTriggerQuaternionProceduralTriggerOperator)
    bpy.utils.unregister_class(SetAngleQuaternionProceduralTriggerOperator)
    bpy.utils.unregister_class(SetPositionQuaternionProceduralTriggerOperator)
    bpy.utils.unregister_class(PreviewQuaternionProceduralTriggerOperator)

    # UI
    bpy.utils.unregister_class(QuaternionProceduralList)
    bpy.utils.unregister_class(QuaternionProceduralTriggerList)
    bpy.utils.unregister_class(ProceduralBonePanel)

    del bpy.types.Object.source_procedural_bone_data
