import bpy
import sys
import os
import math
import importlib

# Get file paths from command-line args
obj_path = sys.argv[-2]
fbx_path = sys.argv[-1]

bpy.context.scene.unit_settings.system = "METRIC"
bpy.context.scene.unit_settings.scale_length = 1.0  

# Clear the default scene
bpy.ops.wm.read_factory_settings(use_empty=True)

# Import the OBJ
bpy.ops.wm.obj_import(filepath=obj_path)

bpy.context.scene.unit_settings.scale_length = 0.01
# Export as FBX

# Apply rotation: -90 degrees around Z (to convert Y→X)
for obj in bpy.context.scene.objects:
    bpy.context.view_layer.objects.active = obj
    #obj.rotation_euler[0] = math.pi / 2
    # Original (0, 90, 0); (-90, 0, 0)
    bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
    obj.rotation_euler = (-math.pi / 2, 0, -math.pi / 2)  # 90° around Z
    obj.scale = (100,100,100)
    print(obj)


# Export to FBX with Z-up, X-forward (Unreal default)
bpy.ops.export_scene.fbx(
    filepath=fbx_path,
    axis_forward='X',  # Unreal expects -Z as forward
    axis_up='Z',        # Unreal expects Y as up
    apply_unit_scale=True,
    apply_scale_options='FBX_SCALE_ALL',
    global_scale=1.0
)