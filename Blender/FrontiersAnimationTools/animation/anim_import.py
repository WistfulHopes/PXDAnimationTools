import bpy
import mathutils
import math
import os
import io
import time
import numpy as np
from struct import unpack
from bpy_extras.io_utils import ImportHelper
from mathutils import (Quaternion,
                       Matrix,
                       Vector)
from bpy.props import (BoolProperty,
                       StringProperty,
                       EnumProperty,
                       CollectionProperty
                       )
from ..FrontiersAnimDecompress.process_buffer import decompress
from .console_output import BatchProgress

SINE_RMS = 1 / math.sqrt(2)
ROOT_OBJ_ROTATE = mathutils.Quaternion((SINE_RMS, SINE_RMS, 0.0, 0.0))
ROOT_BONE_ROTATE = mathutils.Quaternion((0.5, -0.5, -0.5, -0.5))

class ArmatureData:
    def __init__(self, arm_obj):
        self.object = arm_obj
        self.pose_bones = arm_obj.pose.bones
        self.bone = self.pose_bones[0].bone
        self.bone_names = [pbone.name for pbone in self.pose_bones]
        self.bone_count = len(self.pose_bones)
        self.local_matrices = {pbone.name: pbone.bone.matrix_local for pbone in self.pose_bones}

        self.bone_parents = {}
        for pbone in self.pose_bones:
            if pbone.parent:
                self.bone_parents[pbone.name] = pbone.parent.name
            else:
                self.bone_parents[pbone.name] = None

        self.bone_children = {}
        for pbone in self.pose_bones:
            children = []
            if pbone.children:
                children = [child.name for child in pbone.children]
            self.bone_children[pbone.name] = children

        self.bone_paths = {}
        for pbone in self.pose_bones:
            path_loc = pbone.path_from_id("location")
            path_rot = pbone.path_from_id("rotation_quaternion")
            path_scl = pbone.path_from_id("scale")
            self.bone_paths[pbone.name] = [path_loc, path_rot, path_scl]

    def prepare_settings(self):
        self.object.rotation_mode = 'QUATERNION'
        for pbone in self.pose_bones:
            pbone.bone.inherit_scale = 'ALIGNED'
            pbone.rotation_mode = 'QUATERNION'


class ActionData:
    def __init__(self, arm_data: ArmatureData, action_name):
        self.arm_data = arm_data
        self.object = arm_data.object
        self.object.animation_data_create()
        self.action_name =action_name
        self.action = bpy.data.actions.new(action_name)
        self.action.use_frame_range = True
        self.fcurves = {}
        self.fcurves_root = None
        self.fc_container = self.get_fc_container(self.action)

        for name in arm_data.bone_names:
            path_loc, path_rot, path_scl = arm_data.bone_paths[name]

            fc_loc = [self.fc_container.fcurves.new(path_loc, index=i_loc) for i_loc in range(3)]
            fc_rot = [self.fc_container.fcurves.new(path_rot, index=i_rot) for i_rot in range(4)]
            fc_scl = [self.fc_container.fcurves.new(path_scl, index=i_scl) for i_scl in range(3)]

            for fc in fc_loc + fc_scl:
                fc.color_mode = 'AUTO_RGB'

            for fc in fc_rot:
                fc.color_mode = 'AUTO_YRGB'

            self.fcurves[name] = [fc_loc, fc_rot, fc_scl]

        self.object.animation_data.action = self.action

    def make_root_curves(self):
        fc_loc = [self.fc_container.fcurves.ensure("location", index=i_loc) for i_loc in range(3)]
        fc_rot = [self.fc_container.fcurves.ensure("rotation_quaternion", index=i_rot) for i_rot in range(4)]
        fc_scl = [self.fc_container.fcurves.ensure("scale", index=i_scl) for i_scl in range(3)]

        for fc in fc_loc + fc_scl:
            fc.color_mode = 'AUTO_RGB'

        for fc in fc_rot:
            fc.color_mode = 'AUTO_YRGB'

        self.fcurves_root = [fc_loc, fc_rot, fc_scl]

    def get_fc_container(self, action):
        if is_version_at_least(4,5):
            layer = action.layers.new("Layer")
            slot = action.slots.new(id_type='OBJECT', name=f"{self.object.name}")
            strip = layer.strips.new(type='KEYFRAME')
            channelbag = strip.channelbag(slot, ensure=True)
            return channelbag
        else:
            return action

    def add_kps_uncompressed(bone_point_counts={}): # TODO
        pass
        # for name in arm_data.bone_names:
        #     fc_loc, fc_rot, fc_scl = fcurves[name]
        #     num_loc, num_rot, num_scl = bone_point_counts[name]
        # [fc.keyframe_points.add(count=num_loc) for fc in fc_loc]
        # [fc.keyframe_points.add(count=num_rot) for fc in fc_rot]
        # [fc.keyframe_points.add(count=num_scl) for fc in fc_scl]



    def create_np_array(self, track_count, frame_count, is_quat=False):
        # if is_quat:
        #     channels = 4
        # else:
        #     channels = 3
        # return np.empty((track_count, channels, frame_count*2), dtype=np.float32)
        pass

    def update_np_arrays(self, arm_data: ArmatureData, matrix_map_basis, frame):
        # frame_f = float(frame)
        # np_frame_i  = frame*2
        # np_val_i = np_frame_i+1
        #
        # for i in range(arm_data.bone_count):
        #     name = arm_data.bone_names[i]
        #     matrix = matrix_map_basis[name]
        #     loc, rot, scl = matrix.decompose()
        #
        #     for j, val in enumerate(loc):
        #         np_loc[i][j][np_frame_i] = frame_f
        #         np_loc[i][j][np_val_i] = val
        #
        #     for j, val in enumerate(rot):
        #         np_rot[i][j][np_frame_i] = frame_f
        #         np_rot[i][j][np_val_i] = val
        #
        #     for j, val in enumerate(scl):
        #         np_scl[i][j][np_frame_i] = frame_f
        #         np_scl[i][j][np_val_i] = val
        pass

    def add_kps_compressed(self, count):
        [fc.keyframe_points.add(count=count) for fc in self.fc_container.fcurves]

    def update_curves(self):
        [fc.update() for fc in self.fc_container.fcurves]

    def make_points_linear(self):
        for fc in self.fc_container.fcurves:
            for kp in fc.keyframe_points:
                kp.interpolation = 'LINEAR'


def is_version_at_least(major, minor):
    if bpy.app.version[0] > major:
        return True
    elif bpy.app.version[0] == major and bpy.app.version[1] >= minor:
        return True
    return False


# Get global matrix of raw bone tracks, which are relative to parent track's local space.
# In HE2, scales are inhereted from parents, but locations are not, unlike Blender where scale
# affects locations. We calculate the global location and rotation matrices by multiplying the
# local matrices from the root to the current bone, then insert the final scale at the end.
def get_matrix_map_global(arm_data: ArmatureData, matrix_map_local, scale_map):
    bone_names = arm_data.bone_names
    bone_parents = arm_data.bone_parents
    bone_children = arm_data.bone_children

    matrix_map_global = {name:mathutils.Matrix() for name in bone_names}

    # Recursively multiply matrices (and scales) leading up to each bone
    def rec(name, parent_name):

        matrix = matrix_map_global[name]
        if parent_name:
            matrix @= matrix_map_global[parent_name]
            scale_map[name] *= scale_map[parent_name]

        matrix @= matrix_map_local[name]

        if bone_children[name]:
            for child in bone_children[name]:
                rec(child, name)

    for name in bone_names:
        if not bone_parents[name]:
            rec(name, None)

    # Then finally apply scales, as all locations are now final
    for name in bone_names:
        matrix = matrix_map_global[name]
        scale = scale_map[name]
        matrix @= mathutils.Matrix.Diagonal(scale).to_4x4()

    return matrix_map_global


# Once global matrices of all bones are obtained, we can now go back and recalculate
# the pose bone's matrix_basis, which will be its local transforms as a matrix. Constructing
# this

def get_matrix_map_basis(arm_data: ArmatureData,matrix_map_global,frame,truth_table=None,is_compressed=False):

    bone_names = arm_data.bone_names
    bone_parents = arm_data.bone_parents
    bone_children = arm_data.bone_children
    local_matrices = arm_data.local_matrices
    bone = arm_data.bone
    matrix_map_basis = {}

    # Calculate local matrices from global
    # Based on example in Blender docs:
    # https://docs.blender.org/api/current/bpy.types.Bone.html#bpy.types.Bone.convert_local_to_pose
    def rec(name, parent_matrix):
        parent_name = bone_parents[name]
        matrix_local = local_matrices[name]
        if bone_parents[name]:
            parent_matrix_local = local_matrices[parent_name]

        if name in matrix_map_global:
            # Compute and assign local matrix, using the new parent matrix
            matrix = matrix_map_global[name]
            if bone_parents[name]:
                matrix_out = bone.convert_local_to_pose( matrix,
                                              matrix_local,
                                              parent_matrix=parent_matrix,
                                              parent_matrix_local=parent_matrix_local,
                                              invert=True )

            else:
                matrix_out = bone.convert_local_to_pose( matrix,
                                                      matrix_local,
                                                      invert=True)

        # If a keyframe isn't present
        else:
            # Compute the updated pose matrix from local and new parent matrix
            matrix_basis = bone_stuff.pbones[name].matrix_basis
            if bone_parents[name]:
                matrix = bone.convert_local_to_pose(matrix_basis,
                                                          matrix_local,
                                                          parent_matrix=parent_matrix,
                                                          parent_matrix_local=parent_matrix_local)
            else:
                matrix = bone.convert_local_to_pose(matrix_basis, matrix_local)

        matrix_map_basis[name] = matrix_out

        # Recursively process children, passing the new matrix through
        if bone_children[name]:
            for child in bone_children[name]:
                rec(child, matrix)

    # Scan all bone trees from their roots
    for name in bone_names:
        if not bone_parents[name]:
            rec(name, None)

    return matrix_map_basis

# Parse keyframes into nested list for uncompressed animations
def get_uncompressed_frame_table(anim_file, frame_count, track_count, table_offset):
    # track_table[frame index][track index][loc/rot/scale]
    # avoid bones/dictionaries so function may be used for root motion
    frame_table = []
    for frame in range(frame_count):
        tmp_track_table = []
        for track in range(track_count):
            tmp_track_table.append([None, None, None])  # Location, Rotation, Scale
        frame_table.append(tmp_track_table)

    for track in range(track_count):
        anim_file.seek(table_offset + 0x48 * track)

        loc_count = int.from_bytes(anim_file.read(8), byteorder='little')
        loc_frame_offset = int.from_bytes(anim_file.read(8), byteorder='little') + 0x40
        loc_data_offset = int.from_bytes(anim_file.read(8), byteorder='little') + 0x40

        rot_count = int.from_bytes(anim_file.read(8), byteorder='little')
        rot_frame_offset = int.from_bytes(anim_file.read(8), byteorder='little') + 0x40
        rot_data_offset = int.from_bytes(anim_file.read(8), byteorder='little') + 0x40

        scale_count = int.from_bytes(anim_file.read(8), byteorder='little')
        scale_frame_offset = int.from_bytes(anim_file.read(8), byteorder='little') + 0x40
        scale_data_offset = int.from_bytes(anim_file.read(8), byteorder='little') + 0x40

        for i in range(loc_count):
            anim_file.seek(loc_frame_offset + 0x2 * i)
            tmp_frame = int.from_bytes(anim_file.read(2), byteorder='little')
            anim_file.seek(loc_data_offset + 0x10 * i)
            tmp_loc = unpack('<fff', anim_file.read(0xC))
            frame_table[tmp_frame][track][0] = tmp_loc

        for i in range(rot_count):
            anim_file.seek(rot_frame_offset + 0x2 * i)
            tmp_frame = int.from_bytes(anim_file.read(2), byteorder='little')
            anim_file.seek(rot_data_offset + 0x10 * i)
            tmp_rot = unpack('<ffff', anim_file.read(0x10))
            frame_table[tmp_frame][track][1] = tmp_rot

        for i in range(scale_count):
            anim_file.seek(scale_frame_offset + 0x2 * i)
            tmp_frame = int.from_bytes(anim_file.read(2), byteorder='little')
            anim_file.seek(scale_data_offset + 0x10 * i)
            tmp_scale = unpack('<fff', anim_file.read(0xC))
            frame_table[tmp_frame][track][2] = tmp_scale

    return frame_table


class PXDAnimParam:
    def __init__(self, file, name):
        self.name = name
        for ext in [".outanim", ".anm", ".pxd"]:
            self.name = self.name.replace(ext, "")

        file.seek(8)
        file_size = int.from_bytes(file.read(4), byteorder='little')

        file.seek(0x40)
        magic = file.read(4)
        if magic != b'NAXP':
            self.error = f"Not a valid PXD animation file"
            return
        version = int.from_bytes(file.read(4), byteorder='little')
        if version != 512:
            self.error = "Unsupported PXD version"
            return
        flag_additive = int.from_bytes(file.read(1), byteorder='little')
        flag_compressed = int.from_bytes(file.read(1), byteorder='little')

        if flag_additive == 1:
            self.is_additive = True
        else:
            self.is_additive = False

        if flag_compressed == 8:
            self.is_compressed = True
        else:
            self.is_compressed = False

        file.seek(0x58)
        self.duration = unpack('<f', file.read(4))[0]
        self.frame_count = int.from_bytes(file.read(4), byteorder='little')
        if self.duration != 0.0:
            self.frame_rate = (self.frame_count - 1) / self.duration
        else:
            self.frame_rate = 30.0
        self.track_count = int.from_bytes(file.read(8), byteorder='little')
        self.main_offset = int.from_bytes(file.read(8), byteorder='little')
        if self.main_offset:
            self.main_offset += 0x40
        else:
            self.main_offset = None

        self.root_offset = int.from_bytes(file.read(8), byteorder='little')
        if self.root_offset:
            self.root_offset += 0x40

        # Animations compressed with old FrontiersAnimDecompress had non-existent root chunk offsets beyond EOF
        if (self.root_offset > (file_size - 0x40)) or (not self.root_offset):
            self.root_offset = None

        file.seek(0)
        self.error = None


class FrontiersAnimImport(bpy.types.Operator, ImportHelper):
    bl_idname = "import_anim.frontiers_anim"
    bl_label = "Import"
    bl_description = "Imports compressed Hedgehog Engine 2 PXD animation"
    bl_options = {'PRESET', 'UNDO'}
    filename_ext = ".anm.pxd"
    filter_glob: StringProperty(
        default="*.anm.pxd",
        options={'HIDDEN'},
    )
    filepath: StringProperty(subtype='FILE_PATH', )
    files: CollectionProperty(type=bpy.types.PropertyGroup)

    bool_yx_skel: BoolProperty(
        name="Use YX Bone Orientation",
        description="Enable if your skeleton was reoriented for Blender's YX orientation instead of HE2's XZ",
        default=True,
    )

    bool_root_motion: BoolProperty(
        name="Import Root Motion",
        description="Import root motion animation onto skeleton object's transform",
        default=True,
    )

    bool_keyframe_needed: BoolProperty(
        name="Insert Needed Keyframes Only",
        description="Refrains from inserting keyframes if values are exact same as previous frame (not working)",
        default=False,
    )

    enum_loop_check: EnumProperty(
        items=[
            ("loop_auto", "Auto", "Pad the animation if \"_loop\" is in the file name", 1),
            ("loop_yes", "Yes", "Force pad the animation", 2),
            ("loop_no", "No", "Import file contents like normal", 3),
        ],
        name="Pad loop",
        description="(NOTE: Does not work on uncompressed animations)\n"
                    "(NOTE: May or may not cause issues with 360deg rotations)\n\n"
                    "Imports the animation with copies of the animation before and after the export range. Useful for advanced users trying to do things like smoothly looping physics animations",
        default="loop_no",
    )

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.bool_skel_conv = False
        self.frame_count_loop = 0
        self.pad_loop = False

    def draw(self, context):
        layout = self.layout
        ui_scene_box = layout.box()
        ui_scene_box.label(text="Animation Settings", icon='ACTION')

        ui_scene_row_loop = ui_scene_box.row()
        ui_scene_row_loop.label(text="Pad Loop:")
        ui_scene_row_loop.prop(self, "enum_loop_check", text="")

        ui_scene_row_root_motion = ui_scene_box.row()
        ui_scene_row_root_motion.prop(self, "bool_root_motion", )

        # Currently not working as expected, meant to only insert keyframes if local transform is different
        # ui_scene_row_needed = ui_scene_box.row()
        # ui_scene_row_needed.prop(self, "bool_keyframe_needed")

        ui_bone_box = layout.box()
        ui_bone_box.label(text="Armature Settings", icon='ARMATURE_DATA')

        ui_orientation_row = ui_bone_box.row()
        ui_orientation_row.prop(self, "bool_yx_skel", )

    @classmethod
    def poll(cls, context):
        obj = context.active_object
        if obj and obj.type == 'ARMATURE':
            return True
        else:
            return False

    def execute(self, context):
        # Scene check and setup
        arm_active = context.active_object
        scene_active = context.scene

        if not arm_active:
            self.report({'INFO'}, f"No active armature. Please select an armature.")
            return {'CANCELLED'}
        if arm_active.type != 'ARMATURE':
            self.report({'INFO'}, f"Active object \"{arm_active.name}\" is not an armature. Please select an armature.")
            return {'CANCELLED'}

        arm_data = ArmatureData(arm_active)
        arm_data.prepare_settings()

        # Status logging
        self.progress = BatchProgress(self, num_items=len(self.files), method='IMPORT')

        for f, file in enumerate(self.files):
            # Begin import
            anim_file = open(os.path.join(os.path.dirname(self.filepath), file.name), "rb")
            anim_param = PXDAnimParam(anim_file, file.name)
            if not anim_param.is_compressed:    # TODO
                self.progress.update_error(error=f"{file.name} compressed animation import is currently broken. File skipped. Sorry! :(")
                continue

            self.progress.update_frame_count(anim_param.frame_count)
            self.progress.resume(frame_num=-1, name=file.name, item_num=f)

            if (not anim_param) or anim_param.error:
                self.progress.update_error(name=file_name, error=anim_param.error)
                continue

            scene_active.render.fps = int(round(anim_param.frame_rate))

            if anim_param.frame_rate != 0.0:
                scene_active.render.fps_base = scene_active.render.fps / anim_param.frame_rate # in case calculated frame rate ends up as non-integer value
            else:
                scene_active.render.fps_base = 1.0

            if arm_data.bone_count != anim_param.track_count:
                self.report(
                    {'WARNING'},
                    f"Bone count of \"{arm_active.data.name}\" ({arm_data.bone_count}) does not match track count of \"{file.name}\" ({anim_param.track_count}). Results may not turn out as expected."
                )

            action_data = ActionData(arm_data, anim_param.name)
            action_active = action_data.action

            if self.enum_loop_check == "loop_yes" or (self.enum_loop_check == "loop_auto" and "_loop" in anim_param.name):
                self.pad_loop = True

            # frame_count_loop used ubiquitously in case of padding
            if self.pad_loop and anim_param.is_compressed:
                self.frame_count_loop = 3 * (anim_param.frame_count - 1) + 1
                # Weird Blender behavior requires this to be set later
                # action_active.frame_start = anim_param.frame_count - 1
                # action_active.frame_end = self.frame_count_loop - anim_param.frame_count
                action_active.use_cyclic = True
            else:
                self.frame_count_loop = anim_param.frame_count
                scene_active.frame_start = action_active.frame_start = 0
                scene_active.frame_end = action_active.frame_end = self.frame_count_loop - 1

            action_active.pxd_export = True
            action_active.pxd_fps = anim_param.frame_rate
            action_active.pxd_root = self.bool_root_motion
            action_active.pxd_compress = anim_param.is_compressed
            action_active.pxd_additive = anim_param.is_additive

            if anim_param.is_compressed:
                import_action = self.import_compressed(arm_data, action_data, anim_file, anim_param)
            else:
                import_action = self.import_uncompressed(arm_active, anim_file, anim_param)
            anim_file.close()
            del anim_file
            if not import_action:
                self.progress.update_error(error=f"{file.name} compressed animation import couldn't be processed. File skipped.")
                continue

            # Keyframes become invisible if this is set earlier than anim import.
            if self.pad_loop and anim_param.is_compressed:
                scene_active.frame_start = action_active.frame_start = anim_param.frame_count - 1
                scene_active.frame_end = action_active.frame_end = self.frame_count_loop - anim_param.frame_count

        self.progress.finish()
        return {'FINISHED'}

    def import_compressed(self,
                          arm_data: ArmatureData,
                          action_data: ActionData,
                          anim_file,
                          anim_data):

        arm_active = arm_data.object
        frame_count = anim_data.frame_count
        track_count = anim_data.track_count
        main_offset = anim_data.main_offset
        root_offset = anim_data.root_offset
        bone_names = arm_data.bone_names
        bone_parents = arm_data.bone_parents
        bone_count = arm_data.bone_count


        anim_file.seek(main_offset)
        main_buffer_length = int.from_bytes(anim_file.read(4), byteorder='little')
        anim_file.seek(main_offset)
        main_buffer_compressed = anim_file.read(main_buffer_length)
        main_buffer = decompress(main_buffer_compressed)

        if not len(main_buffer.getvalue()):
            self.progress.update_error(error=f"{anim_data.name} buffer failed to initialize. File skipped.")
            return False
        del main_buffer_compressed

        if self.bool_root_motion and (root_offset is not None):
            anim_file.seek(root_offset, 0)
            root_buffer_length = int.from_bytes(anim_file.read(4), byteorder='little')
            anim_file.seek(root_offset, 0)
            root_buffer_compressed = anim_file.read(root_buffer_length)
            root_buffer = decompress(root_buffer_compressed)
            if not len(root_buffer.getvalue()):
                self.report({'WARNING'},f"{anim_data.name} root buffer failed to initialize. Importing without root motion.")
                root_buffer = None
            del root_buffer_compressed
        else:
            root_buffer = None

        # Nice for sanity check, but not necessary
        duration_acl = unpack('<f', main_buffer.read(0x4))[0]
        frame_rate_acl = unpack('<f', main_buffer.read(0x4))[0]
        frame_count_acl = int.from_bytes(main_buffer.read(4), byteorder='little')
        track_count_acl = int.from_bytes(main_buffer.read(4), byteorder='little')

        # +1 for potential root motion
        np_loc = np.empty((track_count+1, 3, self.frame_count_loop*2), dtype=np.float32)
        np_rot = np.empty((track_count+1, 4, self.frame_count_loop*2), dtype=np.float32)
        np_scl = np.empty((track_count+1, 3, self.frame_count_loop*2), dtype=np.float32)
        root_i = track_count

        for frame in range(self.frame_count_loop):
            # self.progress.resume(frame_num=frame)
            if self.pad_loop:
                main_buffer.seek(0x10 + (0x30 * track_count * (frame % (frame_count - 1))))
            else:
                main_buffer.seek(0x10 + (0x30 * track_count * frame))

            matrix_map_local = {}
            scale_map = {}

            for i in range(bone_count):
                name = arm_data.bone_names[i]
                if i in range(track_count):
                    r0, r1, r2, r3 = unpack('<ffff', main_buffer.read(0x10))
                    p0, p1, p2, __ = unpack('<ffff', main_buffer.read(0x10))
                    s0, s1, s2, __ = unpack('<ffff', main_buffer.read(0x10))

                    if self.bool_yx_skel:
                        tmp_rot = mathutils.Quaternion((r3, r2, r0, r1))
                        tmp_loc = mathutils.Vector((p2, p0, p1))
                        if not arm_data.bone_parents[name]:
                            tmp_rot @= ROOT_BONE_ROTATE
                    else:
                        tmp_rot = mathutils.Quaternion((r3, r0, r1, r2))
                        tmp_loc = mathutils.Vector((p0, p1, p2))

                    matrix = mathutils.Matrix.LocRotScale(tmp_loc, tmp_rot, None)
                    matrix_map_local[name] = matrix

                    if (s0, s1, s2) != (0.0, 0.0, 0.0):
                        if self.bool_yx_skel:
                            tmp_scale = mathutils.Vector((s2, s0, s1))
                        else:
                            tmp_scale = mathutils.Vector((s0, s1, s2))
                    else:
                        tmp_scale = mathutils.Vector((1.0, 1.0, 1.0))

                    scale_map[name] = tmp_scale

                else:
                    matrix_map_local[name] = mathutils.Matrix()
                    scale_map[name] = mathutils.Vector((1.0, 1.0, 1.0))

            matrix_map_global = get_matrix_map_global(arm_data, matrix_map_local, scale_map)
            matrix_map_basis = get_matrix_map_basis(arm_data, matrix_map_global, frame, is_compressed=True)

            frame_f = float(frame)
            np_frame_i  = frame*2
            np_val_i = np_frame_i+1
            for i in range(track_count):
                name = bone_names[i]
                mat = matrix_map_basis[name]
                loc, rot, scl = mat.decompose()

                for j, val in enumerate(loc):
                    np_loc[i][j][np_frame_i] = frame_f
                    np_loc[i][j][np_val_i] = val

                for j, val in enumerate(rot):
                    np_rot[i][j][np_frame_i] = frame_f
                    np_rot[i][j][np_val_i] = val

                for j, val in enumerate(scl):
                    np_scl[i][j][np_frame_i] = frame_f
                    np_scl[i][j][np_val_i] = val

            ### TODO ###

            if root_buffer:
                action_data.make_root_curves()

                if self.pad_loop:
                    root_buffer.seek(0x10 + (0x30 * (frame % (frame_count - 1))))
                else:
                    root_buffer.seek(0x10 + (0x30 * frame))

                r0, r1, r2, r3 = unpack('<ffff', root_buffer.read(0x10))
                p0, p1, p2, __ = unpack('<ffff', root_buffer.read(0x10))
                s0, s1, s2, __ = unpack('<ffff', root_buffer.read(0x10))

                rot = ROOT_OBJ_ROTATE.copy()
                rot @= mathutils.Quaternion((r3, r0, r1, r2))
                loc = mathutils.Vector((p0, -p2, p1))
                if (s0, s1, s2) != (0.0, 0.0, 0.0):
                    scl = mathutils.Vector((s0, s1, s2))
                else:
                    scl = mathutils.Vector((1.0, 1.0, 1.0))

                for j, val in enumerate(loc):
                    np_loc[root_i][j][np_frame_i] = frame_f
                    np_loc[root_i][j][np_val_i] = val

                for j, val in enumerate(rot):
                    np_rot[root_i][j][np_frame_i] = frame_f
                    np_rot[root_i][j][np_val_i] = val

                for j, val in enumerate(scl):
                    np_scl[root_i][j][np_frame_i] = frame_f
                    np_scl[root_i][j][np_val_i] = val

            elif self.bool_root_motion and not root_buffer:
                self.report({'INFO'}, "No root motion chunk found.")

        action_data.add_kps_compressed(self.frame_count_loop)

        for i in range(track_count):
            name = bone_names[i]
            fc_loc, fc_rot, fc_scl = action_data.fcurves[name]
            [fc.keyframe_points.foreach_set('co', np_loc[i][j]) for j, fc in enumerate(fc_loc)]
            [fc.keyframe_points.foreach_set('co', np_rot[i][j]) for j, fc in enumerate(fc_rot)]
            [fc.keyframe_points.foreach_set('co', np_scl[i][j]) for j, fc in enumerate(fc_scl)]

        if action_data.fcurves_root:
            fc_loc, fc_rot, fc_scl = action_data.fcurves_root
            [fc.keyframe_points.foreach_set('co', np_loc[root_i][j]) for j, fc in enumerate(fc_loc)]
            [fc.keyframe_points.foreach_set('co', np_rot[root_i][j]) for j, fc in enumerate(fc_rot)]
            [fc.keyframe_points.foreach_set('co', np_scl[root_i][j]) for j, fc in enumerate(fc_scl)]

        action_data.update_curves()

        return True

    def import_uncompressed(self, arm_active, anim_file, anim_data):    # TODO
        pass
        # frame_count = anim_data.frame_count
        # track_count = anim_data.track_count
        # bone_count = len(arm_active.data.bones)
        # main_offset = anim_data.main_offset
        # root_offset = anim_data.root_offset
        #
        # # Carry over local transformation if there's no new keyframe.
        # # Needed for global transformation conversion to correct locations as a result of scaling.
        # matrix_basis_carry = {}
        # for pbone in arm_active.pose.bones:
        #     matrix_basis_carry[pbone.name] = mathutils.Matrix()
        #
        # frame_table = get_uncompressed_frame_table(anim_file, frame_count, track_count, main_offset)
        #
        # root_basis_carry = mathutils.Matrix() @ mathutils.Quaternion((SINE_RMS, SINE_RMS, 0.0, 0.0)).to_matrix().to_4x4()
        # if self.bool_root_motion:
        #     if root_offset:
        #         root_frame_table = get_uncompressed_frame_table(anim_file, frame_count, 1, root_offset)
        #     else:
        #         self.report({'INFO'}, "No root motion chunk found. Skipping root motion import")
        #
        # for frame in range(frame_count):
        #     self.progress.resume(frame_num=frame)
        #     track_table = frame_table[frame]
        #     if self.bool_root_motion and root_offset:
        #         root_table = root_frame_table[frame][0]
        #     matrix_map_local = {}
        #     scale_map = {}
        #
        #     # Need track_table status as dictionary for get_matrix_map_basis function.
        #     truth_table = {}
        #     for pbone in arm_active.pose.bones:
        #         truth_table[pbone.name] = [False, False, False]
        #
        #     for i in range(bone_count):
        #         pbone = arm_active.pose.bones[i]
        #         if i in range(track_count):
        #             bone_table = track_table[i]
        #             bone_key = truth_table[pbone.name]
        #             tmp_loc, tmp_rot, tmp_scale = matrix_basis_carry[pbone.name].decompose()
        #
        #             if bone_table[0]:  # Location
        #                 p0, p1, p2 = bone_table[0]
        #                 if self.bool_yx_skel:
        #                     tmp_loc = mathutils.Vector((p2, p0, p1))
        #                 else:
        #                     tmp_loc = mathutils.Vector((p0, p1, p2))
        #                 bone_key[0] = True
        #
        #             if bone_table[1]:  # Rotation
        #                 r0, r1, r2, r3 = bone_table[1]
        #                 if self.bool_yx_skel:
        #                     tmp_rot = mathutils.Quaternion((r3, r2, r0, r1))
        #                     if not pbone.parent:
        #                         tmp_rot @= mathutils.Quaternion((0.5, -0.5, -0.5, -0.5))
        #                 else:
        #                     tmp_rot = mathutils.Quaternion((r3, r0, r1, r2))
        #                 bone_key[1] = True
        #
        #             if bone_table[2]:  # Scale
        #                 s0, s1, s2 = bone_table[2]
        #                 if (s0, s1, s2) != (0.0, 0.0, 0.0):
        #                     if self.bool_yx_skel:
        #                         tmp_scale = mathutils.Vector((s2, s0, s1))
        #                     else:
        #                         tmp_scale = mathutils.Vector((s0, s1, s2))
        #                 else:
        #                     tmp_scale = mathutils.Vector((1.0, 1.0, 1.0))
        #                 bone_key[2] = True
        #
        #             matrix_basis_carry[pbone.name] = mathutils.Matrix.LocRotScale(tmp_loc, tmp_rot, tmp_scale)
        #             matrix = mathutils.Matrix.LocRotScale(tmp_loc, tmp_rot, mathutils.Vector((1.0, 1.0, 1.0)))
        #             matrix_map_local[pbone.name] = matrix
        #             scale_map[pbone.name] = tmp_scale
        #         else:
        #             matrix_map_local.update({pbone.name: mathutils.Matrix()})
        #             scale_map.update({pbone.name: mathutils.Vector((1.0, 1.0, 1.0))})
        #
        #     matrix_map_global = get_matrix_map_global(arm_active, matrix_map_local, scale_map)
        #     get_matrix_map_basis(arm_active, matrix_map_global, frame, truth_table=truth_table)
        #
        #     if self.bool_root_motion and root_offset:
        #         # Always reorient for Z-up space, should work regardless if pose-space of skeleton is Y-up or Z-up
        #         tmp_loc, tmp_rot, tmp_scale = root_basis_carry.decompose()
        #         if root_table[0]:  # Location
        #             p0, p1, p2 = root_table[0]
        #             tmp_loc = mathutils.Vector((p0, -p2, p1))
        #             arm_active.location = tmp_loc
        #             arm_active.keyframe_insert('location', frame=frame)
        #
        #         if root_table[1]:  # Rotation
        #             r0, r1, r2, r3 = root_table[1]
        #             tmp_rot = mathutils.Quaternion((RMS, RMS, 0.0, 0.0))
        #             tmp_rot @= mathutils.Quaternion((r3, r0, r1, r2))
        #             arm_active.rotation_quaternion = tmp_rot
        #             arm_active.keyframe_insert('rotation_quaternion', frame=frame)
        #
        #         if root_table[2]:  # Scale
        #             s0, s1, s2 = root_table[2]
        #             if (s0, s1, s2) != (0.0, 0.0, 0.0):
        #                 tmp_scale = mathutils.Vector((s0, s1, s2))
        #             else:
        #                 tmp_scale = mathutils.Vector((1.0, 1.0, 1.0))
        #                 arm_active.scale = tmp_scale
        #             arm_active.keyframe_insert('scale', frame=frame)
        #
        #         root_basis_carry = mathutils.Matrix.LocRotScale(tmp_loc, tmp_rot, tmp_scale)

        return True

    def menu_func_import(self, context):
        self.layout.operator(
            FrontiersAnimImport.bl_idname,
            text="Hedgehog Engine 2 Animation (.anm.pxd)",
            icon='ACTION',
        )
