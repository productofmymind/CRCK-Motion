"""Import GTA V clips (.ycd or .ycd.xml) as editable control-rig actions.

A binary .ycd is turned into .ycd.xml by a small helper (ycdtool.cs) that drives the user's own CodeWalker
install; Sollumz imports the XML and plays it on the game skeleton; each frame is then converted onto the control
rig: plain bones copy their turn, hands and feet get their IK targets, elbows and knees their poles.
"""

import importlib
import math
import os
import subprocess
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import bpy
from mathutils import Matrix, Vector

from . import autopose as ap, presets as pr

HELPER_SRC = Path(__file__).parent / "ycdtool.cs"
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
                       capture_output=True, text=True)
    if r.returncode:
        raise RuntimeError("Couldn't build the .ycd helper: " + (r.stdout + r.stderr).strip()[-400:])
    return out


def ycd_to_xml(path, codewalker):
    if not codewalker or not (Path(codewalker) / "CodeWalker.Core.dll").exists():
        raise RuntimeError("Reading .ycd uses your CodeWalker install: set its folder in "
                           "Preferences > Add-ons > CRCK Motion > CodeWalker Folder (or import a .ycd.xml)")
    out = Path(tempfile.mkdtemp(prefix="crck_ycd_")) / (Path(path).stem + ".ycd.xml")
    r = subprocess.run([str(helper_exe()), str(codewalker), "to-xml", str(path), str(out)], capture_output=True, text=True)
    if r.returncode:
        raise RuntimeError(f"CodeWalker couldn't read {Path(path).name}: {r.stderr.strip()}")
    return out


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


def import_clips(context, rig, path, codewalker="", root_motion=False):
    """Import every clip in a .ycd / .ycd.xml as its own control-rig action. Returns the actions."""
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
        clips = [o for o in cd.children_recursive if o.sollum_type == SollumType.CLIP]
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
            f1 = math.ceil(max(st.frame_end for st in strips)) - 1
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


class CRCK_OT_import_clip(bpy.types.Operator):
    """Import a GTA V clip dictionary (.ycd or .ycd.xml) onto the control rig as editable keys, one action per clip"""
    bl_idname = "crck_motion.import_clip"
    bl_label = "Import GTA Clip"
    bl_options = {"REGISTER", "UNDO"}

    filepath: bpy.props.StringProperty(subtype="FILE_PATH")
    filter_glob: bpy.props.StringProperty(default="*.ycd;*.ycd.xml", options={"HIDDEN"})
    root_motion: bpy.props.BoolProperty(
        name="Root Motion", default=False,
        description="Keep the clip's root motion: the character moves where the game would put it. "
                    "Off: it plays where the character stands")

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
            made = import_clips(context, context.active_object, self.filepath, cw, self.root_motion)
        except RuntimeError as e:
            self.report({"ERROR"}, str(e))
            return {"CANCELLED"}
        if not made:
            self.report({"WARNING"}, "No clips in that file")
            return {"CANCELLED"}
        self.report({"INFO"}, f"Imported {len(made)} clip(s): {', '.join(a.name for a in made)}. "
                              "Switch between them in the Action editor")
        return {"FINISHED"}
