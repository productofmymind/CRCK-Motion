"""Pose and animation presets: plain control-rig keys, so Autopose, Reach and the bake keep working on them."""

import json
import math
import re
from pathlib import Path

import bpy
import gpu
from mathutils import Matrix, Quaternion, Vector

from .autopose import keyed, rotation_owner_rest

CATEGORIES = [
    ("IDLE", "Idle & Standing", ""), ("SIT", "Sitting", ""), ("LEAN", "Leaning", ""),
    ("SOCIAL", "Social", ""), ("HANDS", "Hands & Gestures", ""), ("VEHICLE", "Vehicle", ""),
    ("POLICE", "Police & EMS", ""), ("CUSTOM", "Custom", ""),
]
MIRROR = Matrix.Diagonal((-1, 1, 1, 1))  # the rest pose is symmetric about armature X = 0
THUMB = 256


def user_dir():
    try:
        return Path(bpy.utils.extension_path_user(__package__, path="presets", create=True))
    except Exception:  # running from source, not installed as an extension
        p = Path(bpy.utils.user_resource("CONFIG", path="crck_motion_presets", create=True))
        return p


def finger_bones(rig):
    return [pb.name for pb in rig.pose.bones
            if pb.name.startswith("SKEL_") and "Finger" in pb.name and not pb.constraints]


# plain bones a pose can move that Autopose doesn't: real GTA clips turn these
EXTRA_BONES = ["SKEL_Pelvis", "SKEL_Spine_Root", "SKEL_L_Toe0", "SKEL_R_Toe0"]


def preset_bones(rig):
    extra = [n for n in EXTRA_BONES if n in rig.pose.bones and not rig.pose.bones[n].constraints]
    return keyed(rig) + finger_bones(rig) + extra


def mirror_name(n):
    return re.sub(r"_([LR])_", lambda m: "_R_" if m[1] == "L" else "_L_", n)


# --- where the character stands -------------------------------------------------------------
def placement(rig):
    """Where the character stands: the floor between the feet, facing the way the feet face (yaw only),
    in armature space. Feet, not hips, so a pose that shifts the hips keeps that shift."""
    fwd, mid = Vector(), Vector()
    for n in ("LEG_L_IK", "LEG_R_IK"):
        pb = rig.pose.bones[n]
        delta = pb.matrix @ rig.data.bones[n].matrix_local.inverted()
        fwd += delta.to_3x3() @ Vector((0, 1, 0))  # the ped faces +Y at rest
        mid += pb.head / 2
    yaw = math.atan2(-fwd.x, fwd.y)
    return Matrix.Translation((mid.x, mid.y, 0)) @ Matrix.Rotation(yaw, 4, "Z")


# --- capture --------------------------------------------------------------------------------
def capture(rig, bones):
    pose = {}
    for n in bones:
        b = rig.pose.bones[n].matrix_basis
        pose[n] = [*b.to_translation(), *b.to_quaternion()]
    return pose


def make_preset(rig, name, category, frames=None, scene=None):
    """frames: None for the current pose, else frame numbers to sample (keys of an animation)."""
    bones = preset_bones(rig)
    data = {"version": 1, "name": name, "category": category,
            "place": [list(r) for r in placement(rig)], "frames": {}}
    look = rig.get("crck_look")
    if frames is None:
        data["frames"]["0"] = {"bones": capture(rig, bones), "look": list(look) if look else None}
        return data
    keep = scene.frame_current
    f0 = frames[0]
    for i, f in enumerate(frames):
        scene.frame_set(f)
        if i == 0:
            data["place"] = [list(r) for r in placement(rig)]
        look = rig.get("crck_look")
        data["frames"][str(f - f0)] = {"bones": capture(rig, bones), "look": list(look) if look else None}
    scene.frame_set(keep)
    return data


def animation_frames(rig, scene):
    ad = rig.animation_data
    if not ad or not ad.action:
        return []
    from .autopose import fcurves
    fr = {round(kp.co.x) for fc in fcurves(rig) for kp in fc.keyframe_points}
    return sorted(f for f in fr if scene.frame_start <= f <= scene.frame_end)


# --- apply ----------------------------------------------------------------------------------
def pose_bases(rig, pose, place, here, mirror=False, region="ALL"):
    """Target matrix_basis per bone for one stored pose. `place`: where the character stood when it was
    saved, `here`: where it stands now (both from placement())."""
    bones = rig.data.bones
    S = MIRROR if mirror else Matrix.Identity(4)
    to_here = here @ S @ place.inverted()
    fingers = set(finger_bones(rig))
    owner = rotation_owner_rest(rig)
    turn3 = to_here.to_3x3()
    S3 = S.to_3x3()
    out = {}
    for n, v in pose["bones"].items():
        dst = mirror_name(n) if mirror else n
        if dst not in bones or n not in bones:
            continue
        if (region == "BODY" and dst in fingers) or (region == "HANDS" and dst not in fingers):
            continue
        t, q = Vector(v[:3]), Quaternion(v[3:]).to_matrix()
        root = bones[dst].parent is None  # moves in armature space, so it's re-placed; the rest only mirror
        # rotation in rest-pose axes; an IK control's in the axes of the bone that copies it (see rotation_owner_rest)
        Rs = owner.get(n, bones[n].matrix_local.to_3x3())
        Rd = owner.get(dst, bones[dst].matrix_local.to_3x3())
        D = Rs @ q @ Rs.inverted()
        D = (turn3 @ D @ S3) if root else (S3 @ D @ S3)
        rot = Rd.inverted() @ D @ Rd
        # translation of the head, in armature space
        src3, dst3 = bones[n].matrix_local.to_3x3(), bones[dst].matrix_local.to_3x3()
        if root:
            head = to_here @ (bones[n].head_local + src3 @ t)
            loc = dst3.inverted() @ (head - bones[dst].head_local)
        else:
            loc = dst3.inverted() @ S3 @ src3 @ t
        out[dst] = Matrix.LocRotScale(loc, rot.to_quaternion(), None)
    look = to_here @ Vector(pose["look"]) if pose.get("look") and region != "HANDS" else None
    return out, look


def blend_into(rig, bases, weight):
    for n, B in bases.items():
        pb = rig.pose.bones[n]
        if weight >= 1:
            pb.matrix_basis = B
            continue
        c = pb.matrix_basis
        q0, q1 = c.to_quaternion(), B.to_quaternion()
        if q0.dot(q1) < 0:
            q1.negate()
        pb.matrix_basis = Matrix.LocRotScale(c.to_translation().lerp(B.to_translation(), weight),
                                             q0.slerp(q1, weight), None)


def apply_preset(rig, data, scene, weight=1.0, mirror=False, region="ALL", key=True, here=None):
    """Pose the rig, and key it. An animated preset is keyed from the current frame on, every frame
    placed against where the character stood before the first one was written."""
    start = scene.frame_current
    here = here or placement(rig)
    place = Matrix(data["place"])
    frames = sorted(data["frames"], key=int)
    written = set()
    for k in frames:
        if len(frames) > 1 and key:
            scene.frame_set(start + int(k))
        bases, look = pose_bases(rig, data["frames"][k], place, here, mirror, region)
        blend_into(rig, bases, weight)
        if look is not None and weight >= 0.5:
            rig["crck_look"] = list(look)
        written |= set(bases)
        if key:
            key_bones(rig, bases, start + int(k))
        else:
            break  # a preview shows the first pose only
    if len(frames) > 1 and key:
        scene.frame_set(start)
    return written


def key_bones(rig, names, frame):
    for n in names:
        pb = rig.pose.bones[n]
        pb.keyframe_insert("location", frame=frame, group=n)
        pb.keyframe_insert("rotation_quaternion", frame=frame, group=n)


# --- files ----------------------------------------------------------------------------------
def slug(name):
    return re.sub(r"[^a-z0-9]+", "_", name.lower()).strip("_") or "preset"


def save_file(data, folder=None):
    folder = Path(folder or user_dir()) / data["category"]
    folder.mkdir(parents=True, exist_ok=True)
    path = folder / f"{slug(data['name'])}.json"
    path.write_text(json.dumps(data, indent=1))
    return path


def scan():
    """[(id, path, data)] for every preset in the user's folder."""
    out = []
    for tag, root in (("u", user_dir()),):
        for p in sorted(Path(root).glob("*/*.json")):
            try:
                out.append((f"{tag}:{p.parent.name}/{p.stem}", p, json.loads(p.read_text())))
            except (OSError, ValueError):
                continue
    return out


def remove_file(path):
    path = Path(path)
    for p in (path, path.with_suffix(".png")):
        if p.exists():
            p.unlink()


# --- thumbnails -----------------------------------------------------------------------------
def render_thumbnail(context, rig, path, size=THUMB):
    """Draw the character from the front three-quarter into a PNG next to the preset."""
    import numpy as np
    area = context.area if context.area and context.area.type == "VIEW_3D" else next(
        (a for a in context.screen.areas if a.type == "VIEW_3D"), None)
    if area is None:
        return False
    region = next(r for r in area.regions if r.type == "WINDOW")
    space = area.spaces.active
    W, pbs = rig.matrix_world, rig.pose.bones
    top = (W @ pbs["SKEL_Head"].head).z + 0.15
    bottom = min((W @ pbs[n].head).z for n in ("LEG_L_IK", "LEG_R_IK")) - 0.12
    P = W @ placement(rig)
    mid = W @ pbs["SKEL_ROOT"].head
    center = Vector((mid.x, mid.y, (top + bottom) / 2))
    fov = math.radians(30)
    dist = (top - bottom) / 2 / math.tan(fov / 2) * 1.12
    fwd, right = P.to_3x3() @ Vector((0, 1, 0)), P.to_3x3() @ Vector((1, 0, 0))
    eye = center + (fwd + 0.55 * right).normalized() * dist + Vector((0, 0, 0.12 * dist))
    z = (eye - center).normalized()
    x = Vector((0, 0, 1)).cross(z).normalized()
    y = z.cross(x)
    cam = Matrix((x, y, z)).transposed().to_4x4()
    cam.translation = eye
    f, near, far = 1 / math.tan(fov / 2), 0.05, 100.0
    proj = Matrix(((f, 0, 0, 0), (0, f, 0, 0),
                   (0, 0, (far + near) / (near - far), 2 * far * near / (near - far)), (0, 0, -1, 0)))
    overlays = space.overlay.show_overlays
    has_mesh = any(o.type == "MESH" and o.visible_get() and any(m.type == "ARMATURE" for m in o.modifiers)
                   for o in context.view_layer.objects)
    space.overlay.show_overlays = not has_mesh  # with a body the bones are clutter; without, they're the pose
    try:
        off = gpu.types.GPUOffScreen(size, size)
        off.draw_view3d(context.scene, context.view_layer, space, region, cam.inverted(), proj,
                        do_color_management=True)
        with off.bind():
            buf = gpu.state.active_framebuffer_get().read_color(0, 0, size, size, 4, 0, "UBYTE")
        off.free()
    finally:
        space.overlay.show_overlays = overlays
    img = bpy.data.images.new("crck_thumb", size, size, alpha=True)
    buf.dimensions = size * size * 4
    img.pixels.foreach_set(np.asarray(buf, np.float32) / 255)
    img.filepath_raw = str(Path(path).with_suffix(".png"))
    img.file_format = "PNG"
    img.save()
    bpy.data.images.remove(img)
    return True


# --- browser state --------------------------------------------------------------------------
_previews = None
_items = []      # enum items must outlive the callback, or Blender shows garbage
_index = {}      # id -> (path, data, icon_id)
_backup = {}     # what a preview replaced: rig name, stand point, bases, look-at


def refresh():
    global _previews
    import bpy.utils.previews
    if _previews is None:
        _previews = bpy.utils.previews.new()
    _index.clear()
    for pid, path, data in scan():
        icon = 0
        png = path.with_suffix(".png")
        if png.exists():
            key = f"{pid}:{png.stat().st_mtime_ns}"
            if key not in _previews:
                _previews.load(key, str(png), "IMAGE")
            icon = _previews[key].icon_id
        _index[pid] = (path, data, icon)


def preset_items(self, context):
    if not _index:
        refresh()
    _items.clear()
    for pid, (path, data, icon) in _index.items():
        if data.get("category") != self.preset_category:
            continue
        n = len(data["frames"])
        desc = data.get("source", "Your preset") + (f", animated: {n} keys" if n > 1 else "")
        _items.append((pid, data["name"], desc, icon, len(_items)))
    if not _items:
        _items.append(("NONE", "No presets yet", "Save a pose into this category", 0, 0))
    return _items


def entry(pid):
    if not _index:
        refresh()
    return _index.get(pid)


def restore_backup(context=None):
    rig = bpy.data.objects.get(_backup.get("rig", ""))
    if rig is not None:
        for n, B in _backup["bases"].items():
            if n in rig.pose.bones:
                rig.pose.bones[n].matrix_basis = B
        if _backup["look"] is None:
            rig.pop("crck_look", None)
        else:
            rig["crck_look"] = _backup["look"]
    _backup.clear()


def preview(self, context):
    """Pose the rig with the chosen preset, unkeyed, so it can be looked at before Apply."""
    rig = _rig(context)
    e = entry(self.preset)
    if e is None or rig is None:
        return
    if _backup.get("rig") != rig.name:
        restore_backup()
        _backup.update(rig=rig.name, here=placement(rig),
                       look=list(rig["crck_look"]) if "crck_look" in rig else None,
                       bases={n: rig.pose.bones[n].matrix_basis.copy() for n in preset_bones(rig)})
    else:  # back to the pose before the previous preview, so previews don't stack
        for n, B in _backup["bases"].items():
            rig.pose.bones[n].matrix_basis = B
    apply_preset(rig, e[1], context.scene, self.preset_blend, self.preset_mirror, self.preset_region,
                 key=False, here=_backup["here"])


def redo_preview(self, context):
    if _backup:
        preview(self, context)


# --- operators ------------------------------------------------------------------------------
def _rig(context):
    from .autopose import is_ped_rig
    ob = context.active_object
    return ob if is_ped_rig(ob) else None


class CRCK_OT_preset_apply(bpy.types.Operator):
    """Pose the character with this preset and key it at the current frame (an animated preset is keyed from here on)"""
    bl_idname = "crck_motion.preset_apply"
    bl_label = "Apply"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        return _rig(context) is not None and entry(context.scene.crck_motion.preset) is not None

    def execute(self, context):
        s, rig = context.scene.crck_motion, _rig(context)
        here = _backup.get("here") if _backup.get("rig") == rig.name else None
        restore_backup()
        written = apply_preset(rig, entry(s.preset)[1], context.scene, s.preset_blend, s.preset_mirror,
                               s.preset_region, key=True, here=here)
        self.report({"INFO"}, f"Keyed {len(written)} bones")
        return {"FINISHED"}


class CRCK_OT_preset_cancel(bpy.types.Operator):
    """Put back the pose from before the preview"""
    bl_idname = "crck_motion.preset_cancel"
    bl_label = "Cancel Preview"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        return bool(_backup)

    def execute(self, context):
        restore_backup()
        return {"FINISHED"}


class CRCK_OT_preset_save(bpy.types.Operator):
    """Save the current pose, or the keyed animation in the scene's frame range, as your own preset"""
    bl_idname = "crck_motion.preset_save"
    bl_label = "Save Preset"

    name: bpy.props.StringProperty(name="Name", default="My Pose")
    category: bpy.props.EnumProperty(name="Category", items=CATEGORIES, default="CUSTOM")
    animated: bpy.props.BoolProperty(
        name="Animation", description="Save every keyed frame in the scene's frame range, not just this pose")

    @classmethod
    def poll(cls, context):
        return _rig(context) is not None

    def invoke(self, context, event):
        self.category = context.scene.crck_motion.preset_category
        return context.window_manager.invoke_props_dialog(self)

    def execute(self, context):
        rig, scene = _rig(context), context.scene
        restore_backup()
        frames = animation_frames(rig, scene) if self.animated else None
        if self.animated and len(frames) < 2:
            self.report({"ERROR"}, "Need at least two keyed frames in the frame range to save an animation")
            return {"CANCELLED"}
        path = save_file(make_preset(rig, self.name, self.category, frames, scene))
        render_thumbnail(context, rig, path)
        refresh()
        s = scene.crck_motion
        s.preset_category = self.category
        ids = [i[0] for i in preset_items(s, context)]
        pid = f"u:{self.category}/{path.stem}"
        if pid in ids:
            s["preset"] = ids.index(pid)  # select it without previewing (it's already the pose)
        self.report({"INFO"}, f"Saved {path.name}")
        return {"FINISHED"}


class CRCK_OT_preset_delete(bpy.types.Operator):
    """Delete this preset"""
    bl_idname = "crck_motion.preset_delete"
    bl_label = "Delete Preset"

    @classmethod
    def poll(cls, context):
        return context.scene.crck_motion.preset.startswith("u:")

    def invoke(self, context, event):
        return context.window_manager.invoke_confirm(self, event)

    def execute(self, context):
        restore_backup()
        e = entry(context.scene.crck_motion.preset)
        if e:
            remove_file(e[0])
        refresh()
        return {"FINISHED"}


class CRCK_OT_preset_refresh(bpy.types.Operator):
    """Re-read the preset folders"""
    bl_idname = "crck_motion.preset_refresh"
    bl_label = "Refresh Presets"

    def execute(self, context):
        refresh()
        return {"FINISHED"}


class CRCK_PT_presets(bpy.types.Panel):
    bl_label = "Presets"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "CRCK Motion"

    @classmethod
    def poll(cls, context):
        return _rig(context) is not None

    def draw(self, context):
        layout, s = self.layout, context.scene.crck_motion
        row = layout.row(align=True)
        row.prop(s, "preset_category", text="")
        row.operator(CRCK_OT_preset_refresh.bl_idname, text="", icon="FILE_REFRESH")
        layout.template_icon_view(s, "preset", show_labels=True, scale=7, scale_popup=5)
        e = entry(s.preset)
        if e:
            n = len(e[1]["frames"])
            layout.label(text=e[1]["name"] + (f"  (animated, {n} keys)" if n > 1 else ""))
        col = layout.column(align=True)
        col.prop(s, "preset_blend", slider=True)
        row = col.row(align=True)
        row.prop(s, "preset_mirror", toggle=True, icon="MOD_MIRROR")
        row.prop(s, "preset_region", text="")
        row = layout.row(align=True)
        row.scale_y = 1.3
        row.operator(CRCK_OT_preset_apply.bl_idname, icon="KEY_HLT")
        if _backup:
            row.operator(CRCK_OT_preset_cancel.bl_idname, text="", icon="LOOP_BACK")
        row = layout.row(align=True)
        row.operator(CRCK_OT_preset_save.bl_idname, icon="FILE_NEW")
        row.operator(CRCK_OT_preset_delete.bl_idname, text="", icon="TRASH")


classes = (CRCK_OT_preset_apply, CRCK_OT_preset_cancel, CRCK_OT_preset_save, CRCK_OT_preset_delete,
           CRCK_OT_preset_refresh, CRCK_PT_presets)


def unregister_previews():
    global _previews
    if _previews is not None:
        import bpy.utils.previews
        bpy.utils.previews.remove(_previews)
        _previews = None
    _index.clear()
    _backup.clear()
