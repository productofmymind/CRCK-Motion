from pathlib import Path

import bpy

from . import autopose, gta_clip, presets
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
    preset_category: bpy.props.EnumProperty(name="Category", items=presets.CATEGORIES)
    preset: bpy.props.EnumProperty(name="Preset", items=presets.preset_items, update=presets.preview,
                                   description="Click to preview on the character, then Apply to key it")
    preset_blend: bpy.props.FloatProperty(
        name="Blend", default=1.0, min=0, max=1, subtype="FACTOR", update=presets.redo_preview,
        description="How much of the preset to mix into the current pose")
    preset_mirror: bpy.props.BoolProperty(
        name="Mirror", update=presets.redo_preview, description="Swap left and right")
    preset_region: bpy.props.EnumProperty(name="Apply To", update=presets.redo_preview, items=[
        ("ALL", "Whole Body", "Body, arms, legs and fingers"),
        ("BODY", "Body Only", "Everything except the fingers"),
        ("HANDS", "Fingers Only", "Only the finger bones: a hand shape"),
    ])


def find_game_skel(rig):
    """The armature whose bones copy this control rig (the one Sollumz exports)."""
    for ob in bpy.data.objects:
        if ob.type == "ARMATURE" and ob is not rig:
            for pb in ob.pose.bones:
                if any(getattr(c, "target", None) is rig for c in pb.constraints):
                    return ob
    return None


class CRCKMotionPreferences(bpy.types.AddonPreferences):
    bl_idname = __package__

    body_path: bpy.props.StringProperty(
        name="Body File", subtype="FILE_PATH",
        description="A .blend with the ped mesh bound to its skeleton (e.g. Freemode_M_IK_Fixed.blend). "
                    "Add Freemode Rig imports the body from it")
    codewalker_dir: bpy.props.StringProperty(
        name="CodeWalker Folder", subtype="DIR_PATH",
        description="Your CodeWalker install (the folder with CodeWalker.Core.dll). Import GTA Clip uses it to "
                    "read binary .ycd files; .ycd.xml files don't need it")

    def draw(self, context):
        self.layout.prop(self, "body_path")
        self.layout.prop(self, "codewalker_dir")


def prefs():
    addon = bpy.context.preferences.addons.get(__package__)
    return addon.preferences if addon else None


def body_path():
    p = prefs()
    return bpy.path.abspath(p.body_path) if p and p.body_path else ""


def has_body(skel):
    return any(m.type == "ARMATURE" and m.object is skel for ob in bpy.data.objects
               if ob.type == "MESH" for m in ob.modifiers)


def import_body(context, path, rig):
    """Append the meshes skinned in `path` and bind them to this rig's game skeleton."""
    skel = find_game_skel(rig)
    with bpy.data.libraries.load(path, link=False) as (src, dst):
        dst.objects = list(src.objects)
    bodies = [ob for ob in dst.objects if ob and ob.type == "MESH"
              and any(m.type == "ARMATURE" and m.object for m in ob.modifiers)]
    keep = set(bodies) | {ob.parent for ob in bodies if ob.parent and ob.parent.type == "EMPTY"}
    coll = rig.users_collection[0]
    for ob in dst.objects:
        if ob is None:
            continue
        if ob not in keep:
            bpy.data.objects.remove(ob)
            continue
        coll.objects.link(ob)
        if ob.parent is None or ob.parent not in keep:  # top of the body: hang it under the game skeleton
            # exactly as it hung under its own skeleton, local transform kept: a Sollumz ped body carries a 180°
            # turn there that its mesh data undoes. (matrix_world of a just-appended object isn't evaluated yet,
            # so re-deriving it from that dropped the turn and put every vertex opposite its bone.)
            ob.parent = skel
        for m in ob.modifiers:
            if m.type == "ARMATURE":
                m.object = skel
    return bodies


class CRCK_OT_import_body(bpy.types.Operator):
    """Import the ped body from a .blend and bind it to the game skeleton, so you see it deform"""
    bl_idname = "crck_motion.import_body"
    bl_label = "Import Body"
    bl_options = {"REGISTER", "UNDO"}

    filepath: bpy.props.StringProperty(subtype="FILE_PATH")
    filter_glob: bpy.props.StringProperty(default="*.blend", options={"HIDDEN"})

    @classmethod
    def poll(cls, context):
        return is_ped_rig(context.active_object) and find_game_skel(context.active_object) is not None

    def invoke(self, context, event):
        self.filepath = body_path()
        context.window_manager.fileselect_add(self)
        return {"RUNNING_MODAL"}

    def execute(self, context):
        path = bpy.path.abspath(self.filepath)
        if not Path(path).is_file():
            self.report({"ERROR"}, "Pick a .blend file")
            return {"CANCELLED"}
        mode = context.mode
        if mode != "OBJECT":
            bpy.ops.object.mode_set(mode="OBJECT")
        bodies = import_body(context, path, context.active_object)
        if mode == "POSE":
            bpy.ops.object.mode_set(mode="POSE")
        if not bodies:
            self.report({"ERROR"}, "No mesh with an Armature modifier in that file")
            return {"CANCELLED"}
        if prefs():
            prefs().body_path = path
        self.report({"INFO"}, f"Imported {', '.join(b.name for b in bodies)}")
        return {"FINISHED"}


class CRCK_OT_add_rig(bpy.types.Operator):
    """Append the Freemode male control rig and game skeleton, plus the body from your Body File"""
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
        body = body_path()
        if body and Path(body).is_file():
            import_body(context, body, rig)
        else:
            self.report({"WARNING"}, "No body: use Import Body to pick your ped .blend")
        context.view_layer.objects.active = rig
        rig.select_set(True)
        return {"FINISHED"}


def add_elbow_poles(rig):
    """Give each arm IK a pole bone behind the elbow, at the pole angle that leaves the current pose unchanged."""
    import math
    from mathutils import Vector

    arms = {s: (f"SKEL_{s}_UpperArm", f"SKEL_{s}_Forearm", f"POLE_{s}_FA") for s in "LR"}
    arms = {s: a for s, a in arms.items() if a[2] not in rig.data.bones}
    if not arms:
        return []
    vl = bpy.context.view_layer

    def mats():
        vl.update()
        return {n: rig.pose.bones[n].matrix.to_quaternion() for a in arms.values() for n in a[:2]}

    before = mats()
    bpy.ops.object.mode_set(mode="EDIT")
    eb = rig.data.edit_bones
    for ua, fa, pole in arms.values():
        b = eb.new(pole)
        b.head = eb[fa].head + Vector((0, -0.45, 0))  # elbows bend backwards; the ped faces +Y
        b.tail = b.head + Vector((0, -0.1, 0))
        b.use_deform = False
    bpy.ops.object.mode_set(mode="POSE")
    looks = rig.pose.bones.get("LEG_L_POLE")
    for ua, fa, pole in arms.values():
        if looks:  # draw it like the knee poles
            rig.pose.bones[pole].custom_shape = looks.custom_shape
            rig.pose.bones[pole].custom_shape_scale_xyz = looks.custom_shape_scale_xyz
            for c in rig.data.bones["LEG_L_POLE"].collections:
                c.assign(rig.data.bones[pole])
        ik = next(c for c in rig.pose.bones[fa].constraints if c.type == "IK")
        ik.pole_target, ik.pole_subtarget = rig, pole
        best = min(range(-180, 180), key=lambda deg: (
            setattr(ik, "pole_angle", math.radians(deg)),
            sum(q.rotation_difference(before[n]).angle for n, q in mats().items() if n in (ua, fa)))[1])
        ik.pole_angle = math.radians(best)
    return [a[2] for a in arms.values()]


class CRCK_OT_add_elbow_poles(bpy.types.Operator):
    """Add pole bones behind the elbows so you can point them. The game skeleton is not touched"""
    bl_idname = "crck_motion.add_elbow_poles"
    bl_label = "Add Elbow Poles"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        return is_ped_rig(context.active_object) and context.mode in {"POSE", "OBJECT"}

    def execute(self, context):
        mode = context.mode
        added = add_elbow_poles(context.active_object)
        if mode == "OBJECT":
            bpy.ops.object.mode_set(mode="OBJECT")
        self.report({"INFO"}, f"Added {', '.join(added)}" if added else "Elbow poles already there")
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

        skel = find_game_skel(rig)
        if skel and not has_body(skel):
            layout.operator(CRCK_OT_import_body.bl_idname, icon="OUTLINER_OB_MESH")
        if "POLE_L_FA" not in rig.data.bones or "POLE_R_FA" not in rig.data.bones:
            layout.operator(CRCK_OT_add_elbow_poles.bl_idname, icon="CON_KINEMATIC")
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
        box.operator(gta_clip.CRCK_OT_thin_keys.bl_idname, icon="IPO_BEZIER")

        layout.operator(gta_clip.CRCK_OT_import_clip.bl_idname, icon="IMPORT")
        layout.operator(CRCK_OT_bake.bl_idname, icon="EXPORT")
        layout.operator(gta_clip.CRCK_OT_export_clip.bl_idname, icon="EXPORT")


classes = (CRCKMotionSettings, CRCKMotionPreferences, CRCK_OT_import_body, CRCK_OT_add_elbow_poles,
           CRCK_OT_add_rig, CRCK_OT_key_pose,
           CRCK_OT_toggle_pin, CRCK_OT_bake, autopose.CRCK_OT_autopose, gta_clip.CRCK_OT_import_clip, gta_clip.CRCK_OT_thin_keys, gta_clip.CRCK_OT_export_clip, CRCK_PT_panel, *presets.classes)


def register():
    for c in classes:
        bpy.utils.register_class(c)
    bpy.types.Scene.crck_motion = bpy.props.PointerProperty(type=CRCKMotionSettings)


def unregister():
    if autopose._running is not None:
        autopose._running.stop = True
    presets.unregister_previews()
    del bpy.types.Scene.crck_motion
    for c in reversed(classes):
        bpy.utils.unregister_class(c)
