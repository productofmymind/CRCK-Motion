# Run: blender -b crck_motion/assets/freemode_m.blend --factory-startup --python test_crck_motion.py
import os, sys, time, bpy
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import crck_motion
from crck_motion import autopose as ap
from mathutils import Vector
crck_motion.register()
C = bpy.context; vl = C.view_layer
rig = bpy.data.objects["control_rig"]; vl.objects.active = rig; rig.select_set(True)
bpy.ops.object.mode_set(mode="POSE")
pb = rig.pose.bones
ok = True
def check(name, cond, info=""):
    global ok; ok &= bool(cond); print(("PASS " if cond else "FAIL ") + name, info)

# 1 FK matches Blender
d = ap.Drag(rig, "hand_l", "BODY", 0.7)
err = max((d.fk0[n].translation - pb[n].head).length for n in ap.TORSO)
check("fk matches blender", err < 1e-4, f"{err:.2e}")

# 2 hand beyond reach: body follows, hand reaches, feet stay
foot0 = pb["SKEL_L_Foot"].head.copy(); hand0 = pb["IK_L_FA"].head.copy()
target = hand0 + Vector((-0.35, 0.25, 0.1))
t = time.perf_counter(); d.move(target); dt = time.perf_counter() - t
vl.update()
reach_err = (pb["SKEL_L_Hand"].head - target).length
check("hand reaches far target", reach_err < 0.02, f"err={reach_err:.3f} solve={dt*1000:.1f}ms x={d.x.round(3)}")
check("foot stays planted", (pb["SKEL_L_Foot"].head - foot0).length < 1e-3)
check("torso moved", abs(d.x).max() > 0.01)
d.cancel(); vl.update()
check("cancel restores", (pb["SKEL_L_Hand"].head - hand0).length < 1e-4)

# body_follows 0: torso untouched
d = ap.Drag(rig, "hand_l", "BODY", 0.0); d.move(target); vl.update()
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

# 4 look-at
look = pb["SKEL_Head"].head + Vector((0.6, 0.6, 0.3))
d = ap.Drag(rig, "look", "BODY", 0.5); d.move(look); vl.update()
fwd = pb["SKEL_Head"].matrix.col[1].xyz.normalized(); want = (look - pb["SKEL_Head"].head).normalized()
check("head aims at look", fwd.angle(want) < 0.05, f"{fwd.angle(want):.3f}rad")
d.cancel(); vl.update(); check("look cancel clears", "crck_look" not in rig)

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
print("ALL OK" if ok else "SOME FAILED"); sys.exit(0 if ok else 1)
