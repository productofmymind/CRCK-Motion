from pathlib import Path

import bpy

from . import autopose
from .autopose import LIMBS, is_ped_rig

ASSET = Path(__file__).parent / "assets" / "freemode_m.blend"
RIG_OBJECTS = ["control_rig", "GAME_RIG", "skel"]


class CRCKMotionSettings(bpy.types.PropertyGroup):
    body_follows: bpy.props.FloatProperty(
        name="Body Follows", default=0.5, min=0, max=1,
        description="How much the torso moves to help a dragged hand or the hips. 0: only the limb")
    reach: bpy.props.IntProperty(
        name="Reach", default=6, min=0, max=60,
        description="With Auto Keying on, how many frames either side an edit carries into existing keys")
    falloff: bpy.props.EnumProperty(name="Falloff", items=[
        ("SMOOTH", "Smooth", "Ease out from the edited frame"),
        ("HOLD", "Hold", "Full strength over most of the Reach"),
        ("LINEAR", "Linear", "Fade evenly"),
        ("SHARP", "Sharp", "Strong at the edited frame, falls off fast"),
    ])
    intensity: bpy.props.FloatProperty(name="Intensity", default=1.0, min=0, max=1, subtype="FACTOR")


def find_game_skel(rig):
    """The armature whose bones copy this control rig (the one Sollumz exports)."""
    for ob in bpy.data.objects:
        if ob.type == "ARMATURE" and ob is not rig:
            for pb in ob.pose.bones:
                if any(getattr(c, "target", None) is rig for c in pb.constraints):
                    return ob
    return None


class CRCK_OT_add_rig(bpy.types.Operator):
    """Append the Freemode male control rig and game skeleton (no mesh: import your ped with Sollumz)"""
    bl_idname = "crck_motion.add_rig"
    bl_label = "Add Freemode Rig"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        if context.mode != "OBJECT":
            bpy.ops.object.mode_set(mode="OBJECT")
        with bpy.data.libraries.load(str(ASSET), link=False) as (src, dst):
            dst.objects = [n for n in src.objects if n in RIG_OBJECTS]
        coll = bpy.data.collections.new("Freemode_M")
        context.scene.collection.children.link(coll)
        rig = None
        for ob in dst.objects:
            coll.objects.link(ob)
            if ob.type == "ARMATURE" and is_ped_rig(ob) and find_game_skel(ob):
                rig = ob
        for ob in dst.objects:
            ob.select_set(False)
            if ob.type == "ARMATURE" and ob is not rig:
                ob.hide_set(True)
        context.view_layer.objects.active = rig
        rig.select_set(True)
        return {"FINISHED"}


class CRCK_OT_key_pose(bpy.types.Operator):
    """Key every control Autopose moves at the current frame"""
    bl_idname = "crck_motion.key_pose"
    bl_label = "Key Pose"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        return is_ped_rig(context.active_object)

    def execute(self, context):
        autopose.key_pose(context.active_object, context.scene.frame_current)
        return {"FINISHED"}


class CRCK_OT_toggle_pin(bpy.types.Operator):
    """Pinned: stays put in the world. Carried: rides with the body"""
    bl_idname = "crck_motion.toggle_pin"
    bl_label = "Toggle Pin"
    bl_options = {"REGISTER", "UNDO"}

    handle: bpy.props.StringProperty()

    @classmethod
    def poll(cls, context):
        return is_ped_rig(context.active_object)

    def execute(self, context):
        autopose.toggle_pin(context.active_object, self.handle)
        return {"FINISHED"}


class CRCK_OT_bake(bpy.types.Operator):
    """Bake the control rig's animation onto the game skeleton (constraints kept), ready for Sollumz export"""
    bl_idname = "crck_motion.bake"
    bl_label = "Bake to Game Skeleton"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        return is_ped_rig(context.active_object)

    def execute(self, context):
        rig, scene = context.active_object, context.scene
        skel = find_game_skel(rig)
        if skel is None:
            self.report({"ERROR"}, "No armature copies this rig")
            return {"CANCELLED"}
        hidden = skel.hide_get(), skel.hide_viewport
        skel.hide_set(False)
        skel.hide_viewport = False
        if not skel.visible_get():
            skel.hide_set(hidden[0])
            skel.hide_viewport = hidden[1]
            self.report({"ERROR"}, f"'{skel.name}' is in a hidden collection; show it and bake again")
            return {"CANCELLED"}
        bpy.ops.object.mode_set(mode="OBJECT")
        for ob in context.selected_objects:
            ob.select_set(False)
        skel.select_set(True)
        context.view_layer.objects.active = skel
        bpy.ops.object.mode_set(mode="POSE")
        bpy.ops.pose.select_all(action="SELECT")
        bpy.ops.nla.bake(frame_start=scene.frame_start, frame_end=scene.frame_end, only_selected=True,
                         visual_keying=True, clear_constraints=False, use_current_action=False,
                         bake_types={"POSE"})
        src = rig.animation_data.action if rig.animation_data else None
        if src and skel.animation_data and skel.animation_data.action:
            skel.animation_data.action.name = f"{src.name}_game"
        bpy.ops.object.mode_set(mode="OBJECT")
        skel.select_set(False)
        skel.hide_set(hidden[0])
        skel.hide_viewport = hidden[1]
        rig.select_set(True)
        context.view_layer.objects.active = rig
        bpy.ops.object.mode_set(mode="POSE")
        self.report({"INFO"}, f"Baked frames {scene.frame_start}-{scene.frame_end} onto '{skel.name}'")
        return {"FINISHED"}


class CRCK_PT_panel(bpy.types.Panel):
    bl_label = "CRCK Motion"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "CRCK Motion"

    def draw(self, context):
        layout, s, rig = self.layout, context.scene.crck_motion, context.active_object
        if not is_ped_rig(rig):
            layout.label(text="Select a Freemode control rig, or:")
            layout.operator(CRCK_OT_add_rig.bl_idname, icon="ARMATURE_DATA")
            return

        running = autopose._running is not None
        layout.operator(autopose.CRCK_OT_autopose.bl_idname, text="Stop Autopose" if running else "Autopose",
                        icon="HAND", depress=running)
        layout.prop(s, "body_follows", slider=True)

        col = layout.column(align=True)
        col.label(text="Pinned (P over a handle)")
        pins = autopose.pinned(rig)
        row = col.row(align=True)
        for key in LIMBS:
            op = row.operator(CRCK_OT_toggle_pin.bl_idname, text=key.replace("_", " ").title(), depress=key in pins)
            op.handle = key
        if "crck_look" in rig:
            col.operator(CRCK_OT_toggle_pin.bl_idname, text="Look-at Off", icon="HIDE_ON").handle = "look"

        box = layout.box()
        row = box.row(align=True)
        row.operator(CRCK_OT_key_pose.bl_idname, icon="KEY_HLT")
        row.prop(context.scene.tool_settings, "use_keyframe_insert_auto", text="", icon="RECORD_OFF")
        box.prop(s, "reach")
        box.prop(s, "falloff")
        box.prop(s, "intensity", slider=True)

        layout.operator(CRCK_OT_bake.bl_idname, icon="EXPORT")


classes = (CRCKMotionSettings, CRCK_OT_add_rig, CRCK_OT_key_pose, CRCK_OT_toggle_pin, CRCK_OT_bake,
           autopose.CRCK_OT_autopose, CRCK_PT_panel)


def register():
    for c in classes:
        bpy.utils.register_class(c)
    bpy.types.Scene.crck_motion = bpy.props.PointerProperty(type=CRCKMotionSettings)


def unregister():
    if autopose._running is not None:
        autopose._running.stop = True
    del bpy.types.Scene.crck_motion
    for c in reversed(classes):
        bpy.utils.unregister_class(c)
