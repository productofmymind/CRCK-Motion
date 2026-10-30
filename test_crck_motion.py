# Run: blender -b crck_motion/assets/freemode_m.blend --factory-startup --python test_crck_motion.py
import os, sys, time, bpy
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import crck_motion
from crck_motion import autopose as ap
from mathutils import Quaternion, Vector
crck_motion.register()
C = bpy.context; vl = C.view_layer
rig = bpy.data.objects["control_rig"]; vl.objects.active = rig; rig.select_set(True)
bpy.ops.object.mode_set(mode="POSE")
pb = rig.pose.bones
ok = True
def check(name, cond, info=""):
    global ok; ok &= bool(cond); print(("PASS " if cond else "FAIL ") + name, info)

# 0 Autopose survives the rig being replaced, as undo and file reloads do
class _Op: rig_name = "control_rig"
old = rig
bpy.ops.wm.revert_mainfile()
try:
    old.name; stale = False
except ReferenceError:
    stale = True
found = ap.CRCK_OT_autopose.rig.fget(_Op())
check("rig found again after it's replaced", stale and ap.is_ped_rig(found), f"old ref stale={stale}")
C = bpy.context; vl = C.view_layer
rig = bpy.data.objects["control_rig"]; vl.objects.active = rig; rig.select_set(True)
bpy.ops.object.mode_set(mode="POSE")
pb = rig.pose.bones

# 1 FK matches Blender
d = ap.Drag(rig, "hand_l", "REACH", 0.7)
err = max((d.fk0[n].translation - pb[n].head).length for n in ap.TORSO)
check("fk matches blender", err < 1e-4, f"{err:.2e}")

# 2 hand beyond reach: body follows, hand reaches, feet stay
foot0 = pb["SKEL_L_Foot"].head.copy(); hand0 = pb["IK_L_FA"].head.copy()
target = hand0 + Vector((-0.35, 0.25, 0.1))
t = time.perf_counter(); d.move(target); dt = time.perf_counter() - t
vl.update()
reach_err = (pb["SKEL_L_Hand"].head - target).length
check("hand reaches as far as it can", reach_err < 0.12, f"err={reach_err:.3f} solve={dt*1000:.1f}ms x={d.x.round(3)}")
check("foot stays planted", (pb["SKEL_L_Foot"].head - foot0).length < 1e-3, f"{(pb['SKEL_L_Foot'].head - foot0).length:.4f} R {(pb['SKEL_R_Foot'].head - pb['LEG_R_IK'].head).length:.4f}")
check("torso moved", abs(d.x).max() > 0.01)
d.cancel(); vl.update()
check("cancel restores", (pb["SKEL_L_Hand"].head - hand0).length < 1e-4)

# body_follows 0: torso untouched
d = ap.Drag(rig, "hand_l", "REACH", 0.0); d.move(target); vl.update()
check("bf=0 leaves torso", all((pb[n].matrix_basis.to_translation()-d.basis0[n].to_translation()).length<1e-6 and pb[n].matrix_basis.to_quaternion().rotation_difference(d.basis0[n].to_quaternion()).angle<1e-5 for n in ap.TORSO))
d.cancel(); vl.update()

# 3 hips down: knees bend, feet stay
hip0 = pb["SKEL_ROOT"].head.copy()
d = ap.Drag(rig, "hips", "BODY", 0.5); d.move(hip0 + Vector((0, 0, -0.25))); vl.update()
check("hips lowered", abs(pb["SKEL_ROOT"].head.z - (hip0.z - 0.25)) < 0.01, f"{pb['SKEL_ROOT'].head.z - hip0.z:.3f}")
check("foot stays when crouching", (pb["SKEL_L_Foot"].head - foot0).length < 1e-3)
check("carried hand follows chest", (pb["IK_L_FA"].head - hand0).length > 0.1)
d.move(hip0 + Vector((0, 0, -0.1))); vl.update()
check("crouch then rise partway", abs(pb["SKEL_ROOT"].head.z - (hip0.z - 0.1)) < 0.01, f"{pb['SKEL_ROOT'].head.z - hip0.z:.3f}")
# hips up beyond leg length: legs block it
d.move(hip0 + Vector((0, 0, 0.3))); vl.update()
rise = pb["SKEL_ROOT"].head.z - hip0.z
legd = (pb["SKEL_L_Foot"].head - pb["SKEL_L_Thigh"].head).length; L = rig.data.bones["SKEL_L_Thigh"].length + rig.data.bones["SKEL_L_Calf"].length
check("hips stop when legs straight", rise < 0.01 and legd <= L + 1e-3 and (pb["SKEL_L_Foot"].head - foot0).length < 1e-3, f"rise={rise:.3f} legd={legd:.3f} L={L:.3f}")
d.cancel(); vl.update()

# plain hand drag never touches the body, even far past reach (like the rig's own IK)
torso0 = {n: pb[n].matrix.copy() for n in ap.TORSO}; lh = pb["SKEL_R_Hand"].head.copy()
d = ap.Drag(rig, "hand_l", "BODY", 0.5); d.move(d.handle0 + Vector((-0.3, 0.27, 0))); vl.update()
moved = max((pb[n].matrix.to_translation() - torso0[n].to_translation()).length for n in ap.TORSO)
check("plain hand drag leaves body and other hand", moved < 1e-5 and (pb["SKEL_R_Hand"].head - lh).length < 1e-5, f"{moved:.5f}")
d.cancel(); vl.update()

# 3b chest, knee, turning and the root ring
chest0 = pb["SKEL_Spine3"].head.copy()
d = ap.Drag(rig, "chest", "BODY", 0.5); d.move(chest0 + Vector((0, 0.15, -0.05))); vl.update()
check("chest leans forward", pb["SKEL_Spine3"].head.y - chest0.y > 0.08, f"{pb['SKEL_Spine3'].head.y - chest0.y:.3f}")
check("chest keeps feet", (pb["SKEL_L_Foot"].head - foot0).length < 1e-3)
d.cancel(); vl.update()

crouch = ap.Drag(rig, "hips", "BODY", 0.5); crouch.move(hip0 + Vector((0, 0, -0.2))); vl.update()
knee0 = pb["SKEL_L_Calf"].head.copy()
d = ap.Drag(rig, "knee_l", "BODY", 0.5); d.move(knee0 + Vector((-0.2, 0, 0))); vl.update()
check("knee points out (crouched)", pb["SKEL_L_Calf"].head.x - knee0.x < -0.05, f"{pb['SKEL_L_Calf'].head.x - knee0.x:.3f}")
check("knee keeps foot", (pb["SKEL_L_Foot"].head - foot0).length < 1e-3)
d.cancel(); crouch.cancel(); vl.update()

import math
q0 = pb["SKEL_L_Hand"].matrix.to_quaternion()
d = ap.Drag(rig, "hand_l", "BODY", 0.5); d.turn((1, 0, 0), math.radians(40)); vl.update()
ang = math.degrees(pb["SKEL_L_Hand"].matrix.to_quaternion().rotation_difference(q0).angle)
check("hand turns 40 deg", abs(ang - 40) < 1, f"{ang:.1f}")
check("hand stays put while turning", (pb["SKEL_L_Hand"].head - pb["IK_L_FA"].head).length < 0.01)
d.cancel(); vl.update()
for key, bone, axis in (("hand_l", "SKEL_L_Hand", (1, 0, 0)), ("foot_l", "SKEL_L_Foot", (0, 0, 1))):
    q0 = pb[bone].matrix.to_quaternion()
    d = ap.Drag(rig, key, "BODY", 0.5); d.turn(axis, math.radians(30)); vl.update()
    dq = pb[bone].matrix.to_quaternion() @ q0.inverted()
    check(f"{key} turns about the axis asked for", Vector(dq.axis).dot(Vector(axis)) > 0.99 and abs(math.degrees(dq.angle) - 30) < 1,
          f"axis {tuple(round(v, 2) for v in dq.axis)} {math.degrees(dq.angle):.1f}")
    d.cancel(); vl.update()

d = ap.Drag(rig, "root", "BODY", 0.5); d.turn((0, 0, 1), math.radians(90)); vl.update()
f = pb["SKEL_L_Foot"].head; hp = pb["SKEL_ROOT"].head
check("root turns whole body", abs(math.degrees(math.atan2(f.y - hp.y, f.x - hp.x) - math.atan2(foot0.y - hip0.y, foot0.x - hip0.x)) % 360 - 90) < 2)
d.cancel(); vl.update()
d = ap.Drag(rig, "root", "BODY", 0.5); d.move(d.handle0 + Vector((0.5, 0.3, 0))); vl.update()
check("root slides everything", (pb["SKEL_L_Foot"].head - foot0 - Vector((0.5, 0.3, 0))).length < 1e-3
      and (pb["SKEL_ROOT"].head - hip0 - Vector((0.5, 0.3, 0))).length < 1e-3)
d.cancel(); vl.update()

# 3c the other hand stays, the body comes along, elbows and head
lh0, rh0 = pb["SKEL_L_Hand"].head.copy(), pb["IK_R_FA"].head.copy()
d = ap.Drag(rig, "hand_r", "REACH", 0.7); d.move(rh0 + Vector((0.3, 0.3, 0.25))); vl.update()
check("dragging right hand leaves left hand", (pb["SKEL_L_Hand"].head - lh0).length < 0.01, f"{(pb['SKEL_L_Hand'].head - lh0).length:.3f}")
check("left hand handle not carried", (pb["IK_L_FA"].head - lh0).length < 1e-4)
d.cancel(); vl.update()

# within reach only the arm moves, like dragging the rig's own IK target
bend = ap.Drag(rig, "hand_l", "LIMB", 0.5); bend.move(pb["IK_L_FA"].head + Vector((0, 0.15, 0.25))); vl.update()
spine_before = pb["SKEL_Spine3"].matrix.copy()
d = ap.Drag(rig, "hand_l", "REACH", 0.7); d.move(pb["IK_L_FA"].head + Vector((0.05, 0.05, 0))); vl.update()
still = (pb["SKEL_Spine3"].matrix.to_translation() - spine_before.to_translation()).length
check("reachable hand moves only the arm", still < 1e-4 and (pb["SKEL_L_Hand"].head - pb["IK_L_FA"].head).length < 0.005, f"chest moved {still:.5f}")
d.cancel(); bend.cancel(); vl.update()

# a small pull past a straight arm moves the shoulder, barely the body (the lurch you saw)
q = pb["SKEL_Spine3"].matrix.to_quaternion()
d = ap.Drag(rig, "hand_r", "REACH", 0.5); d.move(d.handle0 + Vector((0.06, 0, 0))); vl.update()
turn = math.degrees(q.rotation_difference(pb["SKEL_Spine3"].matrix.to_quaternion()).angle)
hips = (pb["SKEL_ROOT"].head - hip0).length
check("6 cm nudge doesn't lurch the body", turn < 4 and hips < 0.02 and (pb["SKEL_R_Hand"].head - pb["IK_R_FA"].head).length < 0.005,
      f"chest {turn:.1f}deg hips {hips:.3f}")
d.cancel(); vl.update()

check("bundled rig has elbow handles", "elbow_l" in ap.targets(rig) and "elbow_r" in ap.targets(rig))
crouch = ap.Drag(rig, "hand_l", "LIMB", 0.5); crouch.move(pb["IK_L_FA"].head + Vector((0, 0.15, 0.25))); vl.update()
el0 = pb["SKEL_L_Forearm"].head.copy()
d = ap.Drag(rig, "elbow_l", "BODY", 0.5); d.move(el0 + Vector((0, 0, -0.2))); vl.update()
check("elbow points down (bent arm)", pb["SKEL_L_Forearm"].head.z - el0.z < -0.03, f"{pb['SKEL_L_Forearm'].head.z - el0.z:.3f}")
check("elbow keeps the hand", (pb["SKEL_L_Hand"].head - pb["IK_L_FA"].head).length < 0.01)
d.cancel(); crouch.cancel(); vl.update()

head0 = pb["SKEL_Head"].head.copy()
d = ap.Drag(rig, "head", "BODY", 0.5); d.move(head0 + Vector((0.1, 0.1, -0.05))); vl.update()
check("head drag leans the body", (pb["SKEL_Head"].head - head0).length > 0.08, f"{(pb['SKEL_Head'].head - head0).length:.3f}")
check("head drag keeps feet", (pb["SKEL_L_Foot"].head - foot0).length < 1e-3)
d.cancel(); vl.update()

# 4 look-at
look = pb["SKEL_Head"].head + Vector((0.6, 0.6, 0.3))
d = ap.Drag(rig, "look", "BODY", 0.5); d.move(look); vl.update()
fwd = pb["SKEL_Head"].matrix.col[1].xyz.normalized(); want = (look - pb["SKEL_Head"].head).normalized()
check("head aims at look", fwd.angle(want) < 0.05, f"{fwd.angle(want):.3f}rad")
d.cancel(); vl.update(); check("look cancel clears", "crck_look" not in rig)

# 4b turning the whole character turns the feet once, not twice (they copy their IK target's local rotation)
import math
toe = lambda s: (pb[f"SKEL_{s}_Toe0"].head - pb[f"SKEL_{s}_Foot"].head).normalized()
t0 = toe("L")
d = ap.Drag(rig, "root", "BODY", 0.5); d.turn((0, 0, 1), math.radians(90)); vl.update()
ang = math.degrees(math.atan2(toe("L").y, toe("L").x) - math.atan2(t0.y, t0.x)) % 360
check("root turn turns the feet exactly 90", abs(ang - 90) < 1, f"{ang:.1f}")
d.cancel(); vl.update()

# 4c presets: round trip, placed where the character stands, mirror, blend, key, files
from crck_motion import presets as pr
import tempfile
from mathutils import Matrix
def heads():
    vl.update(); return {n: p.head.copy() for n, p in pb.items()}
def to_rest():
    for p in pb: p.matrix_basis = Matrix()
    rig.pop("crck_look", None); vl.update()
to_rest()
ap.Drag(rig, "hips", "BODY", 0.5).move(pb["SKEL_ROOT"].head + Vector((0.05, 0.1, -0.2))); vl.update()
ap.Drag(rig, "hand_l", "LIMB", 0.5).move(pb["IK_L_FA"].head + Vector((-0.05, 0.3, 0.35))); vl.update()
pb["SKEL_L_Finger11"].rotation_quaternion = (0.9, 0.43, 0, 0); A = heads()
data = pr.make_preset(rig, "Test", "CUSTOM")
to_rest(); pr.apply_preset(rig, data, C.scene, key=False); B = heads()
check("preset round trip", max((A[n] - B[n]).length for n in A) < 1e-4)
to_rest()
ap.Drag(rig, "root", "BODY", 0.5).move(pr.placement(rig).translation + Vector((1, 0.5, 0))); vl.update()
d = ap.Drag(rig, "root", "BODY", 0.5); d.turn((0, 0, 1), math.radians(90)); vl.update()
T = pr.placement(rig) @ Matrix(data["place"]).inverted()
pr.apply_preset(rig, data, C.scene, key=False); B = heads()
check("preset lands where the character stands", max(((T @ A[n]) - B[n]).length for n in A) < 1e-4)
to_rest(); pr.apply_preset(rig, data, C.scene, key=False, mirror=True); B = heads()
S = Matrix.Diagonal((-1, 1, 1, 1))
check("preset mirrors left to right", all(((S @ A[f"SKEL_L_{p}"]) - B[f"SKEL_R_{p}"]).length < 1e-3 for p in ("Hand", "Forearm", "Finger12")))
to_rest(); z0 = pb["SKEL_ROOT"].head.z; pr.apply_preset(rig, data, C.scene, key=False, weight=0.5); vl.update()
check("preset blend goes halfway", abs(pb["SKEL_ROOT"].head.z - (z0 + A["SKEL_ROOT"].z) / 2) < 0.01)
to_rest(); pr.apply_preset(rig, data, C.scene, key=False, region="HANDS"); vl.update()
check("fingers-only leaves the body", (pb["SKEL_ROOT"].head - heads()["SKEL_ROOT"]).length < 1e-6 and (pb["IK_L_FA"].head - A["IK_L_FA"]).length > 0.1
      and pb["SKEL_L_Finger11"].rotation_quaternion.w < 0.95)
to_rest(); C.scene.frame_set(4); written = pr.apply_preset(rig, data, C.scene)
check("apply keys the pose", "IK_L_FA" in written and any('"SKEL_L_Finger11"' in fc.data_path for fc in ap.fcurves(rig)))
anim = dict(data, frames={"0": data["frames"]["0"], "10": pr.make_preset(rig, "x", "CUSTOM")["frames"]["0"]})
to_rest(); C.scene.frame_set(30); pr.apply_preset(rig, anim, C.scene)
frames = {round(k.co.x) for fc in ap.fcurves(rig) for k in fc.keyframe_points}
check("animated preset keyed from the current frame", {30, 40} <= frames, sorted(frames))
rig.animation_data.action = None; to_rest(); C.scene.frame_set(0)
with tempfile.TemporaryDirectory() as tmp:
    path = pr.save_file(data, tmp)
    import json
    check("preset saves and loads", json.loads(path.read_text())["frames"]["0"]["bones"].keys() == data["frames"]["0"]["bones"].keys())
to_rest()

# 5 Reach propagation on existing keys
s = C.scene.crck_motion; s.reach = 6
for f in (0, 3, 6, 9, 20):
    C.scene.frame_set(f); ap.key_pose(rig, f)
C.scene.frame_set(6); C.scene.tool_settings.use_keyframe_insert_auto = True
d = ap.Drag(rig, "hand_l", "BODY", 0.5); d.move(hand0 + Vector((0, 0.3, 0.2))); d.commit(C.scene, s)
fcs = {(fc.data_path, fc.array_index): fc for fc in ap.fcurves(rig)}
fy = fcs[('pose.bones["IK_L_FA"].location', 1)]; 
vals = {f: fy.evaluate(f) for f in (0, 3, 6, 9, 20)}
base = vals[20]
r3, r6 = vals[3] - base, vals[6] - base
check("edited frame moved", abs(r6) > 0.1, f"{r6:.3f}")
check("frame 3 partial", 0.05 * abs(r6) < abs(r3) < abs(r6), f"{r3:.3f}")
check("frame 20 untouched", abs(vals[20] - fy.keyframe_points[-1].co.y) < 1e-6 and abs(vals[0]-base) < abs(r3))
check("pin saved", "hand_l" in ap.pinned(rig))

# 5b key thinning: a smooth move keyed every frame comes back with few keys and the same motion
from crck_motion import gta_clip
kept_action = rig.animation_data.action
rig.animation_data.action = bpy.data.actions.new("thin_test")
C.scene.tool_settings.use_keyframe_insert_auto = False
start = pb["IK_L_FA"].head.copy()
for f in range(0, 61):
    C.scene.frame_set(f)
    to_rest()
    pb["IK_L_FA"].location = rig.data.bones["IK_L_FA"].matrix_local.to_3x3().inverted() @ (
        start + Vector((0.15 * math.sin(f / 9), 0.2 * math.sin(f / 15), 0.1 * math.cos(f / 12))) - rig.data.bones["IK_L_FA"].head_local)
    pb["SKEL_Head"].rotation_quaternion = Quaternion((0, 0, 1), math.radians(f)) * (-1 if f % 2 else 1)  # sign flips every frame
    ap.key_pose(rig, f)
def track():
    out = []
    for f in range(0, 61):
        C.scene.frame_set(f); vl.update()
        out.append((pb["SKEL_L_Hand"].head.copy(), pb["SKEL_Head"].matrix.to_quaternion()))
    return out
dense = track()
kept = gta_clip.thin(rig.animation_data.action, 1.0)
thinned = track()
move = max((a[0] - b[0]).length for a, b in zip(dense, thinned))
turn = max(math.degrees(a[1].rotation_difference(b[1]).angle) for a, b in zip(dense, thinned))
check("thinning keeps few keys", kept < 20, f"61 -> {kept}")
check("thinned motion stays within 5 mm", move < 0.005, f"{move*1000:.2f} mm")
check("thinning ignores quaternion sign flips", turn < 1.5, f"{turn:.2f} deg")
bpy.data.actions.remove(rig.animation_data.action)
rig.animation_data.action = kept_action

# 5c game clip index: in-game animations only, one per name, a later archive overrides an earlier one
sample = chr(10).join([
    "x64c.rpf/anim/ingame/clip_amb@.rpf/amb@a@base.ycd  100",
    "x64c.rpf/anim/cutscene/cuts_x.rpf/cs_scene.ycd  200",
    "x64c.rpf/anim/ingame/clip_misc.rpf/b_clip.ycd  300",
    "update/x64/dlcpacks/patch1/dlc.rpf/x64/anim/ingame/clip_amb@.rpf/amb@a@base.ycd  150",
    "x64t.rpf/levels/gta5/props/prop_anim.ycd  50",
    "12 match(es)"])
idx = dict(gta_clip.parse_index(sample))
check("clip index keeps in-game dictionaries only", set(idx) == {"amb@a@base", "b_clip"}, sorted(idx))
check("a later archive overrides an earlier one", "dlcpacks/patch1" in idx["amb@a@base"], idx["amb@a@base"])

# 6 bake
C.scene.frame_start, C.scene.frame_end = 0, 20
with C.temp_override(active_object=rig):
    res = bpy.ops.crck_motion.bake()
skel = bpy.data.objects["skel"]
act = skel.animation_data.action if skel.animation_data else None
check("bake made action", res == {"FINISHED"} and act is not None, f"{res} {act and act.name}")
if act:
    C.scene.frame_set(6); vl.update()
    sk = skel.pose.bones["SKEL_L_Hand"].head; cr = pb["SKEL_L_Hand"].head
    check("baked matches control", (sk - cr).length < 1e-3, f"{(sk-cr).length:.2e}")

# 7 add rig into empty scene
bpy.ops.wm.read_factory_settings(use_empty=True)
crck_motion.register() if not hasattr(bpy.types.Scene, "crck_motion") else None
res = bpy.ops.crck_motion.add_rig()
new = bpy.context.view_layer.objects.active
check("add rig", res == {"FINISHED"} and ap.is_ped_rig(new) and crck_motion.find_game_skel(new) is not None, f"{new and new.name}")

# 8 body from a local .blend (not bundled): set CRCK_BODY, or it looks in Downloads
body = os.environ.get("CRCK_BODY", os.path.expanduser(r"~\Downloads\Freemode_M_IK_Fixed.blend"))
if os.path.isfile(body):
    skel = crck_motion.find_game_skel(new)
    bodies = crck_motion.import_body(bpy.context, body, new)
    check("body imported and bound", bodies and crck_motion.has_body(skel)
          and all(m.object is skel for b in bodies for m in b.modifiers if m.type == "ARMATURE"), [b.name for b in bodies])
    bpy.context.view_layer.update()
    rv = [mesh.matrix_world @ v.co for mesh in bodies for v in mesh.data.vertices for g in v.groups
          if mesh.vertex_groups[g.group].name == "SKEL_R_Hand" and g.weight > 0.5]
    check("body faces the way its skeleton does (right-hand vertices on the right)", rv and sum(v.x for v in rv) / len(rv) > 0.3,
          f"{sum(v.x for v in rv) / max(len(rv), 1):.2f}")
    import math as _m
    from mathutils import Matrix as _M
    top = bodies[0]
    while top.parent is not skel: top = top.parent
    top.matrix_world = _M.Rotation(_m.pi, 4, "Z") @ top.matrix_world; bpy.context.view_layer.update(); crck_motion._facing.clear()
    flagged = crck_motion.body_backwards(skel, bodies[0])
    with bpy.context.temp_override(active_object=new):
        bpy.ops.crck_motion.fix_body_facing()
    check("a backwards body is flagged and Fix Body Facing turns it back", flagged and not crck_motion.body_backwards(skel, bodies[0]))
    check("no extra armatures", sum(o.type == "ARMATURE" for o in bpy.data.objects if o.users_scene) == 2)
    mesh, vl = bodies[0], bpy.context.view_layer
    co = lambda: [v.co.copy() for v in mesh.evaluated_get(bpy.context.evaluated_depsgraph_get()).data.vertices]
    vl.objects.active = new; bpy.ops.object.mode_set(mode="POSE")
    before = co()
    d = ap.Drag(new, "hips", "BODY", 0.5); d.move(new.pose.bones["SKEL_ROOT"].head + Vector((0, 0, -0.25))); vl.update()
    moved = max((a - b).length for a, b in zip(before, co()))
    check("body deforms with the rig", moved > 0.1, f"max vertex move {moved:.3f}")
    # 9 elbow poles added to a rig that has none, without moving the pose
    bpy.ops.wm.open_mainfile(filepath=body)
    rig = bpy.data.objects["control_rig"]; vl = bpy.context.view_layer; vl.objects.active = rig
    bpy.ops.object.mode_set(mode="POSE"); vl.update()
    arm0 = {n: rig.pose.bones[n].matrix.copy() for n in ("SKEL_L_Forearm", "SKEL_R_Forearm", "SKEL_L_Hand", "SKEL_R_Hand")}
    added = crck_motion.add_elbow_poles(rig); vl.update()
    moved = max((rig.pose.bones[n].matrix.to_translation() - m.to_translation()).length +
                rig.pose.bones[n].matrix.to_quaternion().rotation_difference(m.to_quaternion()).angle for n, m in arm0.items())
    check("add elbow poles", sorted(added) == ["POLE_L_FA", "POLE_R_FA"] and "elbow_l" in ap.targets(rig), added)
    check("adding poles doesn't move the arms", moved < 1e-3, f"{moved:.2e}")
else:
    print("SKIP body test: no", body)
print("ALL OK" if ok else "SOME FAILED"); sys.exit(0 if ok else 1)
