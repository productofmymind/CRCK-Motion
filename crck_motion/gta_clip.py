"""Import GTA V clips (.ycd or .ycd.xml) as editable control-rig actions.

A binary .ycd is turned into .ycd.xml by a small helper (ycdtool.cs) that drives the user's own CodeWalker
install; Sollumz imports the XML and plays it on the game skeleton; each frame is then converted onto the control
rig: plain bones copy their turn, hands and feet get their IK targets, elbows and knees their poles.
"""

import importlib
import math
import os
import re
import shutil
import subprocess
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import bpy
import numpy as np
from mathutils import Matrix, Vector

from . import autopose as ap, presets as pr

HELPER_SRC = Path(__file__).parent / "ycdtool.cs"
NO_WINDOW = getattr(subprocess, "CREATE_NO_WINDOW", 0)
CSC = Path(os.environ.get("WINDIR", r"C:\Windows")) / "Microsoft.NET" / "Framework64" / "v4.0.30319" / "csc.exe"
LIMB_ENDS = [("Hand", "IK_{}_FA", "UpperArm", "Forearm", "POLE_{}_FA"),
             ("Foot", "LEG_{}_IK", "Thigh", "Calf", "LEG_{}_POLE")]


# --- .ycd -> .ycd.xml ---------------------------------------------------------------------------
def helper_exe():
    """Build the helper with the C# compiler that comes with Windows, once (again when its source changes)."""
    out = Path(bpy.utils.extension_path_user(__package__, path="bin", create=True)) / "ycdtool.exe"
    if out.exists() and out.stat().st_mtime >= HELPER_SRC.stat().st_mtime:
        return out
    if not CSC.exists():
        raise RuntimeError("Reading .ycd needs the .NET Framework C# compiler, which wasn't found. "
                           "Export the clip to .ycd.xml with CodeWalker and import that instead")
    r = subprocess.run([str(CSC), "-nologo", f"-out:{out}", "-r:System.Xml.dll", "-r:System.Core.dll", str(HELPER_SRC)],
                       capture_output=True, text=True, creationflags=NO_WINDOW)
    if r.returncode:
        raise RuntimeError("Couldn't build the .ycd helper: " + (r.stdout + r.stderr).strip()[-400:])
    return out


def ycd_to_xml(path, codewalker):
    if not codewalker or not (Path(codewalker) / "CodeWalker.Core.dll").exists():
        raise RuntimeError("Reading .ycd uses your CodeWalker install: set its folder in "
                           "Preferences > Add-ons > CRCK Motion > CodeWalker Folder (or import a .ycd.xml)")
    out = Path(tempfile.mkdtemp(prefix="crck_ycd_")) / (Path(path).stem + ".ycd.xml")
    r = subprocess.run([str(helper_exe()), str(codewalker), "to-xml", str(path), str(out)], capture_output=True, text=True, creationflags=NO_WINDOW)
    if r.returncode:
        raise RuntimeError(f"CodeWalker couldn't read {Path(path).name}: {r.stderr.strip()}")
    return out


def xml_to_ycd(xml, out, codewalker):
    if not codewalker or not (Path(codewalker) / "CodeWalker.Core.dll").exists():
        raise RuntimeError("Writing .ycd uses your CodeWalker install: set its folder in "
                           "Preferences > Add-ons > CRCK Motion > CodeWalker Folder (or export a .ycd.xml)")
    r = subprocess.run([str(helper_exe()), str(codewalker), "to-ycd", str(xml), str(out)], capture_output=True, text=True, creationflags=NO_WINDOW)
    if r.returncode:
        raise RuntimeError(f"CodeWalker couldn't write {Path(out).name}: {r.stderr.strip()}")


def clip_names(xml_path):
    clips = ET.parse(xml_path).getroot().find("Clips")
    return [c.findtext("Hash") for c in clips] if clips is not None else []


# --- Sollumz ------------------------------------------------------------------------------------
def sollumz():
    """The installed Sollumz package (an extension or a legacy add-on)."""
    for name in bpy.context.preferences.addons.keys():
        if name.split(".")[-1].lower() == "sollumz":
            return name
    raise RuntimeError("Importing GTA clips needs Sollumz (it reads the clip dictionary)")


def game_skel(rig):
    for ob in bpy.data.objects:
        if ob.type == "ARMATURE" and ob is not rig and any(
                getattr(c, "target", None) is rig for pb in ob.pose.bones for c in pb.constraints):
            return ob
    raise RuntimeError("No game skeleton copies this control rig")


# --- conversion -------------------------------------------------------------------------------
class Converter:
    """Writes one sampled game-skeleton pose onto the control rig."""

    def __init__(self, rig, skel):
        self.rig, self.skel = rig, skel
        self.owner = ap.rotation_owner_rest(rig)
        controls = set(ap.TARGET_BONES.values())
        # every plain control bone the game skeleton also has; arms and legs come from their IK below
        self.plain = [pb.name for pb in rig.pose.bones if not pb.constraints and pb.name in skel.pose.bones
                      and pb.name not in controls and not pb.name.startswith(("IK_", "PH_"))]
        ik = [name.format(s) for s in "LR" for _, tgt, _, _, pole in LIMB_ENDS for name in (tgt, pole)]
        self.keyed = self.plain + [n for n in ik if n in rig.pose.bones]

    def local(self, n, B):
        """The game skeleton copies each control bone's own turn re-expressed in its rest axes: undo that."""
        Rs = self.skel.data.bones[n].matrix_local.to_3x3()
        Rc = self.rig.data.bones[n].matrix_local.to_3x3()
        q = Rc.inverted() @ Rs @ B.to_3x3() @ Rs.inverted() @ Rc
        return Matrix.LocRotScale(Rc.inverted() @ Rs @ B.to_translation(), q.to_quaternion(), None)

    def write(self, ref, basis, mover=None):
        rig, rb, pbs = self.rig, self.rig.data.bones, self.rig.pose.bones
        for pb in pbs:
            pb.matrix_basis = Matrix()
        for n in self.plain:
            pbs[n].matrix_basis = self.local(n, basis[n])
        for s in "LR":
            for end, tgt, upper, lower, pole in LIMB_ENDS:
                end, tgt, pole = f"SKEL_{s}_{end}", tgt.format(s), pole.format(s)
                a, b, c = ref[f"SKEL_{s}_{upper}"].translation, ref[f"SKEL_{s}_{lower}"].translation, ref[end].translation
                pbs[tgt].location = rb[tgt].matrix_local.to_3x3().inverted() @ (c - rb[tgt].head_local)
                bend = b - (a + (c - a) * ((b - a).dot(c - a) / max((c - a).length_squared, 1e-9)))
                if bend.length > 1e-4 and pole in pbs:  # the pole sits out along the way the joint bends
                    pbs[pole].location = rb[pole].matrix_local.to_3x3().inverted() @ (b + bend.normalized() * 0.5 - rb[pole].head_local)
        bpy.context.view_layer.update()
        for _ in range(2):  # the rig's pole angle isn't exactly "the joint bends toward the pole": turn each pole
            for s in "LR":  # about the root-to-end line by however far the knee or elbow missed
                for end, _, upper, lower, pole in LIMB_ENDS:
                    pole = pole.format(s)
                    if pole not in pbs:
                        continue
                    a, c = ref[f"SKEL_{s}_{upper}"].translation, ref[f"SKEL_{s}_{end}"].translation
                    axis = (c - a).normalized()
                    flat = lambda p: (p - a) - axis * (p - a).dot(axis)
                    want, got = flat(ref[f"SKEL_{s}_{lower}"].translation), flat(pbs[f"SKEL_{s}_{lower}"].head)
                    if want.length < 1e-3 or got.length < 1e-3:
                        continue  # straight limb: nothing to aim
                    angle = got.angle(want) * (1 if got.cross(want).dot(axis) > 0 else -1)
                    p = rb[pole].head_local + rb[pole].matrix_local.to_3x3() @ pbs[pole].location
                    p = a + Matrix.Rotation(angle, 3, axis) @ (p - a)
                    pbs[pole].location = rb[pole].matrix_local.to_3x3().inverted() @ (p - rb[pole].head_local)
            bpy.context.view_layer.update()
        for s in "LR":  # hand and foot turn: what the clip gives them, relative to the limb as the IK placed it
            for end, tgt, _, lower, _ in LIMB_ENDS:
                end, tgt, lower = f"SKEL_{s}_{end}", tgt.format(s), f"SKEL_{s}_{lower}"
                want = pbs[lower].matrix @ (rb[lower].matrix_local.inverted() @ rb[end].matrix_local) @ self.local(end, basis[end])
                E = self.owner[tgt]
                pbs[tgt].rotation_quaternion = (E.inverted() @ want.to_3x3() @ rb[end].matrix_local.to_3x3().inverted() @ E).to_quaternion()
        if mover is not None:  # root motion: move the whole character, as Autopose's floor ring does
            R = mover.to_3x3().to_4x4()
            for n in [b.name for b in rb if b.parent is None and b.name in self.keyed]:
                M = ap.turn_control(rig, n, rb[n].matrix_local @ pbs[n].matrix_basis, R, Vector())
                M.translation += mover.translation
                pbs[n].matrix_basis = rb[n].matrix_local.inverted() @ M


# --- key thinning -------------------------------------------------------------------------------
def action_fcurves(action):
    return [fc for layer in action.layers for strip in layer.strips for bag in strip.channelbags for fc in bag.fcurves]


def thin(action, tolerance=1.0):
    """Keep the fewest whole-pose keys (one frame set shared by every curve, so it stays easy to pose) that
    redraw the motion within `tolerance`: 1 = 5 mm of movement, about 1 degree of turn. Returns keys kept."""
    fcs = [fc for fc in action_fcurves(action) if fc.keyframe_points]
    if not fcs or tolerance <= 0:
        return 0
    f0 = round(min(fc.keyframe_points[0].co.x for fc in fcs))
    f1 = round(max(fc.keyframe_points[-1].co.x for fc in fcs))
    xs = list(range(f0, f1 + 1))
    data = np.array([[fc.evaluate(f) for f in xs] for fc in fcs])
    # a quaternion and its negative are the same turn; sampled poses can flip between them, which a curve
    # between two sparse keys would swing through. Keep each bone's quaternion on one side.
    quats = {}
    for i, fc in enumerate(fcs):
        if fc.data_path.endswith("rotation_quaternion"):
            quats.setdefault(fc.data_path, [None] * 4)[fc.array_index] = i
    for idx in quats.values():
        if None in idx:
            continue
        q = data[idx]
        for f in range(1, len(xs)):
            if np.dot(q[:, f], q[:, f - 1]) < 0:
                q[:, f] *= -1
        data[idx] = q
    scale = np.array([0.005 if fc.data_path.endswith("location") else 0.01 for fc in fcs])[:, None] * tolerance
    keep = {f0, f1}
    while True:
        ks = sorted(keep)
        for fc, row in zip(fcs, data):
            pts = fc.keyframe_points
            pts.clear()
            pts.add(len(ks))
            pts.foreach_set("co", [v for f in ks for v in (f, row[f - f0])])
            for p in pts:
                p.interpolation = "BEZIER"
                p.handle_left_type = p.handle_right_type = "AUTO_CLAMPED"
            fc.update()
        err = (np.abs(np.array([[fc.evaluate(f) for f in xs] for fc in fcs]) - data) / scale).max(axis=0)
        added = False
        for a, b in zip(ks, ks[1:]):
            seg = err[a - f0 + 1:b - f0]
            if seg.size and seg.max() > 1:
                keep.add(a + 1 + int(seg.argmax()))
                added = True
        if not added:
            return len(ks)


def import_clips(context, rig, path, codewalker="", root_motion=False, tolerance=1.0, names=None):
    """Import the clips in a .ycd / .ycd.xml (all, or those in `names`), each as its own control-rig action.
    Returns the actions."""
    pkg = sollumz()
    ycdimport = importlib.import_module(pkg + ".ycd.ycdimport")
    SollumType = importlib.import_module(pkg + ".sollumz_properties").SollumType
    path = Path(path)
    xml = path if path.name.lower().endswith(".xml") else ycd_to_xml(path, codewalker)
    dict_name = path.name.split(".")[0]
    scene, vl = context.scene, context.view_layer
    skel = game_skel(rig)
    cons = [c for pb in skel.pose.bones for c in pb.constraints]
    keep = (scene.frame_current, scene.frame_start, scene.frame_end, skel.delta_location.copy(),
            skel.delta_rotation_quaternion.copy(), rig.animation_data.action if rig.animation_data else None)
    mode = context.mode
    # the body doesn't need to deform while bones are sampled, and deforming it each frame is most of the cost
    deform = [m for ob in bpy.data.objects if ob.type == "MESH" for m in ob.modifiers
              if m.type == "ARMATURE" and m.object in (skel, rig) and m.show_viewport]
    wm = context.window_manager
    cd = ycdimport.import_ycd(str(xml))
    made = []
    try:
        for m in deform:
            m.show_viewport = False
        anims = [o for o in cd.children_recursive if o.sollum_type == SollumType.ANIMATION]
        for a in anims:
            a.animation_properties.target_id = skel.data
        conv = Converter(rig, skel)
        clips = [o for o in cd.children_recursive if o.sollum_type == SollumType.CLIP
                 and (names is None or o.clip_properties.hash in names)]
        wm.progress_begin(0, len(clips))
        for ci, clip in enumerate(clips):
            wm.progress_update(ci)
            name = clip.clip_properties.hash
            # play the clip on the game skeleton, with its copy of the control rig switched off
            for c in cons:
                c.mute = True
            if context.mode != "OBJECT":
                bpy.ops.object.mode_set(mode="OBJECT")
            with context.temp_override(selected_objects=[clip], active_object=clip):
                bpy.ops.sollumz.anim_apply_nla()
            # at the clip's own speed: a clip's rate is how fast the game plays it, not part of the motion
            # (some modded clips carry a rate like 1/24, which would stretch a 4 s clip to 90 s)
            strips = [st for t in skel.animation_data.nla_tracks for st in t.strips]
            for st in strips:
                st.scale = 1.0
            f0 = math.floor(min(st.frame_start for st in strips))
            f1 = math.floor(max(st.frame_end for st in strips) + 1e-4)  # the last key sits on frame_end
            frames = []
            for f in range(f0, f1 + 1):
                scene.frame_set(f)
                vl.update()
                mover = Matrix.LocRotScale(skel.delta_location, skel.delta_rotation_quaternion, None)
                frames.append(({pb.name: pb.matrix.copy() for pb in skel.pose.bones},
                               {pb.name: pb.matrix_basis.copy() for pb in skel.pose.bones}, mover))
            for t in list(skel.animation_data.nla_tracks):
                skel.animation_data.nla_tracks.remove(t)
            for c in cons:
                c.mute = False
            skel.delta_location, skel.delta_rotation_quaternion = keep[3], keep[4]
            # write it onto the control rig, a key per frame
            rig.animation_data_create()
            act = bpy.data.actions.new(f"{dict_name} {name}")
            act.use_fake_user = True
            rig.animation_data.action = act
            for i, (ref, basis, mover) in enumerate(frames):
                f = f0 + i
                scene.frame_set(f)
                conv.write(ref, basis, mover if root_motion else None)
                pr.key_bones(rig, conv.keyed, f)
            act.use_frame_range = True
            act.frame_range = (f0, f1)
            act["crck_keys_before_thinning"] = len(frames)
            if tolerance > 0:
                thin(act, tolerance)
            made.append(act)
    finally:
        wm.progress_end()
        for m in deform:
            m.show_viewport = True
        for o in [cd, *cd.children_recursive]:
            act = getattr(getattr(o, "animation_properties", None), "action", None)
            bpy.data.objects.remove(o)
            if act is not None and act.users == 0:
                bpy.data.actions.remove(act)
        for c in cons:
            c.mute = False
        skel.delta_location, skel.delta_rotation_quaternion = keep[3], keep[4]
        context.view_layer.objects.active = rig
        rig.select_set(True)
        if mode == "POSE" and context.mode != "POSE":
            bpy.ops.object.mode_set(mode="POSE")
    rig.animation_data.action = made[0] if made else keep[5]
    if made:
        scene.frame_start, scene.frame_end = (int(v) for v in made[0].frame_range)
    scene.frame_set(scene.frame_start if made else keep[0])
    return made


# --- export ------------------------------------------------------------------------------------
def tidy_baked(action):
    """Drop what a game clip doesn't carry: scale, and bones the bake left where they rest."""
    for bag in [bag for layer in action.layers for strip in layer.strips for bag in strip.channelbags]:
        by_path = {}
        for fc in bag.fcurves:
            by_path.setdefault(fc.data_path, []).append(fc)
        for path, fcs in by_path.items():
            vals = [[k.co.y for k in fc.keyframe_points] for fc in fcs]
            still = all(max(v) - min(v) < 1e-5 for v in vals if v)
            rest = (path.endswith("location") and all(abs(v[0]) < 1e-5 for v in vals if v)) or (
                path.endswith("rotation_quaternion") and all(abs(v[0] - (fc.array_index == 0)) < 1e-5 for fc, v in zip(fcs, vals) if v))
            if path.endswith("scale") or (still and rest and '"SKEL_ROOT"' not in path):
                for fc in fcs:
                    bag.fcurves.remove(fc)


def export_clip(context, rig, filepath, name, codewalker=""):
    """Bake the scene's frame range onto the game skeleton and write it as a one-clip dictionary."""
    pkg = sollumz()
    ycdimport = importlib.import_module(pkg + ".ycd.ycdimport")
    ycdexport = importlib.import_module(pkg + ".ycd.ycdexport")
    SollumType = importlib.import_module(pkg + ".sollumz_properties").SollumType
    skel = game_skel(rig)
    if bpy.ops.crck_motion.bake() != {"FINISHED"}:
        raise RuntimeError("Baking to the game skeleton failed")
    act = skel.animation_data.action
    tidy_baked(act)
    filepath = Path(filepath)
    as_xml = filepath.name.lower().endswith(".xml")
    dict_name = filepath.name.split(".")[0]
    cd, clips_obj, anims_obj = ycdimport.create_clip_dictionary_template(dict_name)
    made = [cd, clips_obj, anims_obj]
    try:
        anim = ycdimport.create_anim_obj(SollumType.ANIMATION)
        made.append(anim)
        anim.parent = anims_obj
        props = anim.animation_properties
        props.hash = name
        props.target_id = skel.data  # target first: setting it after the action would re-map the action's bones
        props.action = act
        clip = ycdimport.create_anim_obj(SollumType.CLIP)
        made.append(clip)
        clip.parent = clips_obj
        cp = clip.clip_properties
        cp.hash, cp.name = name, f"{name}.clip"
        frames = act.frame_range[1] - act.frame_range[0]
        cp.duration = frames / (context.scene.render.fps / context.scene.render.fps_base)
        link = cp.animations.add()
        link.animation, link.start_frame, link.end_frame = anim, 0, frames
        xml = filepath if as_xml else Path(tempfile.mkdtemp(prefix="crck_ycd_")) / f"{dict_name}.ycd.xml"
        if not ycdexport.export_ycd(cd, str(xml)):
            raise RuntimeError("Sollumz couldn't export the clip (see the Info editor)")
        if not as_xml:
            xml_to_ycd(xml, filepath, codewalker)
    finally:
        for o in made:
            bpy.data.objects.remove(o)
    return filepath


class CRCK_OT_export_clip(bpy.types.Operator):
    """Bake the scene's frame range onto the game skeleton and save it as a GTA V clip dictionary (.ycd or .ycd.xml)"""
    bl_idname = "crck_motion.export_clip"
    bl_label = "Export GTA Clip"

    filepath: bpy.props.StringProperty(subtype="FILE_PATH")
    filter_glob: bpy.props.StringProperty(default="*.ycd;*.ycd.xml", options={"HIDDEN"})
    clip_name: bpy.props.StringProperty(
        name="Clip Name", description="The clip's name inside the dictionary: the name you play it by in the game")

    @classmethod
    def poll(cls, context):
        return ap.is_ped_rig(context.active_object)

    def invoke(self, context, event):
        ad = context.active_object.animation_data
        act = ad.action.name if ad and ad.action else "clip"
        base = act.split(" ")[0] if " " in act else "my_anims"
        if not self.filepath:
            self.filepath = bpy.path.abspath("//") + base + ".ycd"
        if not self.clip_name:
            self.clip_name = act.split(" ")[-1].replace(".", "_")
        context.window_manager.fileselect_add(self)
        return {"RUNNING_MODAL"}

    def execute(self, context):
        from . import prefs
        p = prefs()
        cw = bpy.path.abspath(p.codewalker_dir) if p and p.codewalker_dir else ""
        if not self.filepath.lower().endswith((".ycd", ".xml")):
            self.filepath += ".ycd"
        try:
            out = export_clip(context, context.active_object, self.filepath, self.clip_name or "clip", cw)
        except RuntimeError as e:
            self.report({"ERROR"}, str(e))
            return {"CANCELLED"}
        self.report({"INFO"}, f"Saved clip '{self.clip_name}' to {out}")
        return {"FINISHED"}


class CRCK_OT_thin_keys(bpy.types.Operator):
    """Thin the active action down to the fewest whole-pose keys that stay within the tolerance of the motion"""
    bl_idname = "crck_motion.thin_keys"
    bl_label = "Thin Keys"
    bl_options = {"REGISTER", "UNDO"}

    tolerance: bpy.props.FloatProperty(
        name="Tolerance", default=1.0, min=0.05, max=10,
        description="How close to the current motion: 1 = 5 mm, about 1 degree")

    @classmethod
    def poll(cls, context):
        ob = context.active_object
        return ap.is_ped_rig(ob) and ob.animation_data and ob.animation_data.action

    def execute(self, context):
        act = context.active_object.animation_data.action
        before = len({round(k.co.x) for fc in action_fcurves(act) for k in fc.keyframe_points})
        kept = thin(act, self.tolerance)
        self.report({"INFO"}, f"{act.name}: {before} keyed frames -> {kept}")
        return {"FINISHED"}


class CRCK_OT_import_clip(bpy.types.Operator):
    """Import a GTA V clip dictionary (.ycd or .ycd.xml) onto the control rig as editable keys, one action per clip"""
    bl_idname = "crck_motion.import_clip"
    bl_label = "Import GTA Clip"
    bl_options = {"REGISTER", "UNDO"}

    filepath: bpy.props.StringProperty(subtype="FILE_PATH")
    filter_glob: bpy.props.StringProperty(default="*.ycd;*.ycd.xml", options={"HIDDEN"})
    tolerance: bpy.props.FloatProperty(
        name="Key Tolerance", default=1.0, min=0, max=10,
        description="Thin the keys down to the fewest whole-pose keys that stay this close to the clip "
                    "(1 = 5 mm, about 1 degree). 0 keeps a key on every frame")
    root_motion: bpy.props.BoolProperty(
        name="Root Motion", default=False,
        description="Keep the clip's root motion: the character moves where the game would put it. "
                    "Off: it plays where the character stands")
    clips: bpy.props.StringProperty(
        options={"HIDDEN", "SKIP_SAVE"},
        description="Which clips, comma separated; '*' for all. Empty: ask when the file has more than one")

    @classmethod
    def poll(cls, context):
        return ap.is_ped_rig(context.active_object)

    def invoke(self, context, event):
        context.window_manager.fileselect_add(self)
        return {"RUNNING_MODAL"}

    def execute(self, context):
        from . import prefs
        p = prefs()
        cw = bpy.path.abspath(p.codewalker_dir) if p and p.codewalker_dir else ""
        try:
            path = Path(self.filepath)
            xml = path if path.name.lower().endswith(".xml") else ycd_to_xml(path, cw)
            names = None if self.clips in ("", "*") else {n.strip() for n in self.clips.split(",")}
            if not self.clips and len(clip_names(xml)) > 1:  # several clips: let the user pick
                return bpy.ops.crck_motion.import_clip_pick("INVOKE_DEFAULT", filepath=str(xml),
                                                            root_motion=self.root_motion, tolerance=self.tolerance)
            made = import_clips(context, context.active_object, xml, cw, self.root_motion, self.tolerance, names)
        except RuntimeError as e:
            self.report({"ERROR"}, str(e))
            return {"CANCELLED"}
        if not made:
            self.report({"WARNING"}, "No clips in that file")
            return {"CANCELLED"}
        self.report({"INFO"}, f"Imported {len(made)} clip(s): {', '.join(a.name for a in made)}. "
                              "Switch between them in the Action editor")
        return {"FINISHED"}


# --- browsing the game's own clips ----------------------------------------------------------------
_items = []  # enum items must outlive the callback or Blender shows garbage


def explorer_cli(configured=""):
    """CRCK Explorer's command line: the preference, else one on PATH."""
    cli = Path(bpy.path.abspath(configured)) if configured else None
    if cli and cli.is_file():
        return cli
    found = shutil.which("crck-explorer-cli")
    if found:
        return Path(found)
    raise RuntimeError("Browsing the game's clips uses CRCK Explorer: set the path to crck-explorer-cli.exe in "
                       "Preferences > Add-ons > CRCK Motion > CRCK Explorer")


def parse_index(text):
    """[(dictionary name, archive path)] from `game search` output: the in-game animations only (cutscenes
    are built for their own scene skeletons), one per name. A patch or DLC archive listed later overrides an
    earlier one, as it does in the game, so the last listing wins."""
    best = {}
    for line in text.splitlines():
        m = re.match(r"^(.+\.ycd)\s+\d+\s*$", line)
        if m and "/anim/ingame/" in m[1]:
            best[Path(m[1]).stem] = m[1]
    return sorted(best.items(), key=lambda kv: kv[0].lower())


def cache_dir(name):
    return Path(bpy.utils.extension_path_user(__package__, path=name, create=True))


def game_index(cli, refresh=False):
    """The game's clip dictionaries, indexed once and remembered (a game update means refreshing)."""
    cache = cache_dir("clip_index") / "index.txt"
    if refresh or not cache.exists():
        r = subprocess.run([str(cli), "game", "search", "*.ycd", "--kind", "files", "--limit", "500000"],
                           capture_output=True, text=True, creationflags=NO_WINDOW)
        if r.returncode or "/anim/ingame/" not in r.stdout:
            raise RuntimeError("CRCK Explorer couldn't list the game's clips: "
                               + ((r.stderr or r.stdout).strip() or "no game found")[-300:])
        cache.write_text(r.stdout, encoding="utf-8")
    return parse_index(cache.read_text(encoding="utf-8"))


def extract_dictionary(cli, archive_path):
    out = cache_dir("clips") / Path(archive_path).name
    if not out.exists():
        r = subprocess.run([str(cli), "game", "get", archive_path, str(out)], capture_output=True, text=True,
                           creationflags=NO_WINDOW)
        if r.returncode or not out.exists():
            raise RuntimeError(f"CRCK Explorer couldn't extract {Path(archive_path).name}: {(r.stderr or r.stdout).strip()[-300:]}")
    return out


def fill_items(index):
    _items[:] = [(path, name, Path(path).parent.name if "/dlcpacks/" not in path else
                  f"{path.split('dlcpacks/')[1].split('/')[0]} DLC / {Path(path).parent.name}", "", i)
                 for i, (name, path) in enumerate(index)]


def _dictionary_items(self, context):
    return _items


class CRCK_OT_browse_clips(bpy.types.Operator):
    """Search every in-game animation dictionary in your GTA V files and import one onto the control rig"""
    bl_idname = "crck_motion.browse_clips"
    bl_label = "Browse Game Clips"
    bl_options = {"REGISTER", "UNDO"}
    bl_property = "dictionary"  # what the search popup lists

    dictionary: bpy.props.EnumProperty(name="Dictionary", items=_dictionary_items)
    refresh: bpy.props.BoolProperty(
        name="Refresh", default=False, options={"HIDDEN", "SKIP_SAVE"},
        description="List the game's clips again (after a game update)")

    @classmethod
    def poll(cls, context):
        return ap.is_ped_rig(context.active_object)

    def invoke(self, context, event):
        from . import prefs
        p = prefs()
        try:
            index = game_index(explorer_cli(p.explorer_cli if p else ""), self.refresh)
        except RuntimeError as e:
            self.report({"ERROR"}, str(e))
            return {"CANCELLED"}
        fill_items(index)
        context.window_manager.invoke_search_popup(self)
        return {"RUNNING_MODAL"}

    def execute(self, context):
        from . import prefs
        p = prefs()
        if not self.dictionary:
            return {"CANCELLED"}
        try:
            file = extract_dictionary(explorer_cli(p.explorer_cli if p else ""), self.dictionary)
        except RuntimeError as e:
            self.report({"ERROR"}, str(e))
            return {"CANCELLED"}
        return bpy.ops.crck_motion.import_clip("EXEC_DEFAULT", filepath=str(file))


class CRCKClipChoice(bpy.types.PropertyGroup):
    use: bpy.props.BoolProperty(default=True)


class CRCK_OT_import_clip_pick(bpy.types.Operator):
    """Choose which clips of the dictionary to import"""
    bl_idname = "crck_motion.import_clip_pick"
    bl_label = "Import GTA Clips"
    bl_options = {"REGISTER", "UNDO", "INTERNAL"}

    filepath: bpy.props.StringProperty(options={"HIDDEN"})
    tolerance: bpy.props.FloatProperty(options={"HIDDEN"})
    root_motion: bpy.props.BoolProperty(options={"HIDDEN"})
    choices: bpy.props.CollectionProperty(type=CRCKClipChoice)

    def invoke(self, context, event):
        self.choices.clear()
        for n in clip_names(self.filepath):
            self.choices.add().name = n
        return context.window_manager.invoke_props_dialog(self, width=360)

    def draw(self, context):
        col = self.layout.column(align=True)
        col.label(text=f"{len(self.choices)} clips in {Path(self.filepath).name.split('.')[0]}:")
        for c in self.choices:
            col.prop(c, "use", text=c.name)

    def execute(self, context):
        names = [c.name for c in self.choices if c.use]
        if not names:
            return {"CANCELLED"}
        return bpy.ops.crck_motion.import_clip("EXEC_DEFAULT", filepath=self.filepath, clips=",".join(names),
                                               root_motion=self.root_motion, tolerance=self.tolerance)
