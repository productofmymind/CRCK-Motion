"""Autopose: drag or turn a handle, solve the torso so the pinned limbs can still reach, let the rig's IK do the limbs."""

import math
import re

import bpy
import gpu
import numpy as np
from bpy_extras import anim_utils, view3d_utils
from gpu_extras.batch import batch_for_shader
from mathutils import Matrix, Quaternion, Vector, geometry

TORSO = [
    "SKEL_ROOT", "SKEL_Pelvis", "SKEL_L_Thigh", "SKEL_R_Thigh",
    "SKEL_Spine_Root", "SKEL_Spine0", "SKEL_Spine1", "SKEL_Spine2", "SKEL_Spine3",
    "SKEL_L_Clavicle", "SKEL_R_Clavicle", "SKEL_L_UpperArm", "SKEL_R_UpperArm",
    "SKEL_Neck_1", "SKEL_Head",
]
SPINE = ["SKEL_Spine0", "SKEL_Spine1", "SKEL_Spine2", "SKEL_Spine3"]

# IK target handle -> the control bone it moves
TARGET_BONES = {
    "hand_l": "IK_L_FA", "hand_r": "IK_R_FA",
    "foot_l": "LEG_L_IK", "foot_r": "LEG_R_IK",
    "knee_l": "LEG_L_POLE", "knee_r": "LEG_R_POLE",
    "elbow_l": "POLE_L_FA", "elbow_r": "POLE_R_FA",  # optional: older rigs have no arm poles
}
OPTIONAL = {"elbow_l", "elbow_r"}
# where each handle is drawn: on the body, not on the control bone (a pole floats away from the limb)
DRAWN_AT = {"hips": "SKEL_ROOT", "chest": "SKEL_Spine3", "head": "SKEL_Head",
            "hand_l": "IK_L_FA", "hand_r": "IK_R_FA", "foot_l": "LEG_L_IK", "foot_r": "LEG_R_IK",
            "knee_l": "SKEL_L_Calf", "knee_r": "SKEL_R_Calf",
            "elbow_l": "SKEL_L_Forearm", "elbow_r": "SKEL_R_Forearm"}
LIMBS = {
    "hand_l": ("SKEL_L_UpperArm", "SKEL_L_Forearm"), "hand_r": ("SKEL_R_UpperArm", "SKEL_R_Forearm"),
    "foot_l": ("SKEL_L_Thigh", "SKEL_L_Calf"), "foot_r": ("SKEL_R_Thigh", "SKEL_R_Calf"),
}
OTHER_HAND = {"hand_l": "hand_r", "hand_r": "hand_l"}
# pole handle -> (limb root bone, end handle): dragging it points the knee or elbow
BENDS = {"knee_l": ("SKEL_L_Thigh", "foot_l"), "knee_r": ("SKEL_R_Thigh", "foot_r"),
         "elbow_l": ("SKEL_L_UpperArm", "hand_l"), "elbow_r": ("SKEL_R_UpperArm", "hand_r")}
# dragging these pulls a torso bone toward the mouse
GOAL_BONES = {"hips": "SKEL_ROOT", "chest": "SKEL_Spine3", "head": "SKEL_Head"}
# a target that isn't pinned rides along with this torso bone
CARRY = {"hand_l": "SKEL_Spine3", "hand_r": "SKEL_Spine3",
         "foot_l": "SKEL_Pelvis", "foot_r": "SKEL_Pelvis",
         "knee_l": "SKEL_Pelvis", "knee_r": "SKEL_Pelvis",
         "elbow_l": "SKEL_Spine3", "elbow_r": "SKEL_Spine3"}
TURNABLE = {"hand_l", "hand_r", "foot_l", "foot_r", "hips", "chest", "root"} | set(BENDS)
BODY_BONES = ["SKEL_ROOT", *SPINE, "SKEL_L_Clavicle", "SKEL_R_Clavicle", "SKEL_Neck_1", "SKEL_Head"]
REQUIRED = set(TORSO) | {TARGET_BONES[k] for k in TARGET_BONES if k not in OPTIONAL} | {
    "SKEL_L_Forearm", "SKEL_R_Forearm", "SKEL_L_Calf", "SKEL_R_Calf"}

# rig colour convention: yellow left, red right, purple lower body, green upper body
YELLOW, RED = (1, 0.85, 0.1, 1), (1, 0.25, 0.25, 1)
COLORS = {"hips": (0.75, 0.4, 1, 1), "chest": (0.35, 0.9, 0.45, 1), "head": (0.35, 0.9, 0.45, 1),
          "look": (0.35, 0.9, 0.45, 1), "root": (1, 0.6, 0.1, 1),
          "hand_l": YELLOW, "foot_l": YELLOW, "knee_l": YELLOW, "elbow_l": YELLOW,
          "hand_r": RED, "foot_r": RED, "knee_r": RED, "elbow_r": RED}
RINGS = {"knee_l", "knee_r", "elbow_l", "elbow_r", "look"}  # drawn as rings: they aim, they don't hold
FLOOR_BELOW_ANKLE = 0.09  # foot bone head sits this far above the sole
ROOT_RADIUS = 0.45

# solver parameter layout: root move(3) root turn(3) spine bend(3) clavicles(3+3), with stiffness per param.
# Cheapest first, like a person reaching: shoulder, then spine, the hips last.
# Root move is per metre, the rest per radian: 6 cm of hips must cost more than 20 degrees of shoulder.
PARAM_W = np.array([60, 60, 60, 10, 10, 10, 3, 3, 3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3], float)
EPS = 1e-4


def is_ped_rig(obj):
    return obj is not None and obj.type == "ARMATURE" and REQUIRED.issubset(obj.pose.bones.keys())


def targets(rig):
    """IK target handles this rig has."""
    return [k for k, b in TARGET_BONES.items() if b in rig.pose.bones]


def keyed(rig):
    """Everything a pose writes, so everything Key Pose keys."""
    return BODY_BONES + [TARGET_BONES[k] for k in targets(rig)]


def rotation_owner_rest(rig):
    """{control: axes the bone that copies its local rotation turns about}, e.g. IK_L_FA -> SKEL_L_Hand.
    That bone turns by the control's own rotation, but about its own rest axes (with the constraint's
    inverted axes flipped: the feet invert Y and Z), so a turn meant for the foot has to be expressed in
    those axes, not the control's."""
    out = {}
    for pb in rig.pose.bones:
        for c in pb.constraints:
            if c.type == "COPY_ROTATION" and c.target == rig and c.target_space == "LOCAL" and c.subtarget:
                flip = Matrix.Diagonal([-1 if inv else 1 for inv in (c.invert_x, c.invert_y, c.invert_z)])
                out[c.subtarget] = rig.data.bones[pb.name].matrix_local.to_3x3() @ flip
    return out


def turn_control(rig, bone, m, R, pivot):
    """Pose matrix m of `bone` turned by R about pivot, so whatever copies its rotation turns by R too."""
    own = rotation_owner_rest(rig).get(bone)
    if own is None:
        return _about(m, pivot, R)
    C = rig.data.bones[bone].matrix_local.to_3x3() @ own.inverted()
    out = (C @ R.to_3x3() @ C.inverted() @ m.to_3x3()).to_4x4()
    out.translation = (R @ (m.translation - pivot).to_4d()).to_3d() + pivot
    return out


def pinned(rig):
    return set(filter(None, rig.get("crck_pinned", "foot_l,foot_r").split(",")))


def set_pinned(rig, keys):
    rig["crck_pinned"] = ",".join(sorted(keys))


def look_point(rig):
    """Look-at target in armature space, and whether it is on."""
    if "crck_look" in rig:
        return Vector(rig["crck_look"]), True
    head = rig.pose.bones["SKEL_Head"]
    return head.head + head.matrix.col[1].xyz.normalized() * 0.8, False


def root_center(rig):
    pbs = rig.pose.bones
    h = pbs["SKEL_ROOT"].head
    return Vector((h.x, h.y, min(pbs["LEG_L_IK"].head.z, pbs["LEG_R_IK"].head.z) - FLOOR_BELOW_ANKLE))


def handle_positions(rig):
    pbs = rig.pose.bones
    have = set(targets(rig))
    out = {k: pbs[b].head.copy() for k, b in DRAWN_AT.items() if k in have or k not in TARGET_BONES}
    out["look"] = look_point(rig)[0]
    out["root"] = root_center(rig)
    return out


def _rotvec(v):
    v = Vector(v)
    return Quaternion(v.normalized(), v.length) if v.length > 1e-9 else Quaternion()


def _conj(q, Rw):
    """World-space turn q expressed as a local turn of a bone whose world rotation is Rw."""
    return (Rw.inverted() @ q @ Rw).to_matrix().to_4x4()


def _about(m, pivot, R):
    return Matrix.Translation(pivot) @ R @ Matrix.Translation(-pivot) @ m


class Drag:
    """One move or turn of one handle. Kept free of modal/UI code so it can be driven from a script."""

    def __init__(self, rig, key, mode="BODY", body_follows=0.5):
        self.rig, self.key, self.mode, self.bf = rig, key, mode, body_follows
        bones, pbs = rig.data.bones, rig.pose.bones
        self.rel = {}
        for n in TORSO:
            b = bones[n]
            self.rel[n] = b.parent.matrix_local.inverted() @ b.matrix_local if b.parent else b.matrix_local.copy()
        self.parent = {n: bones[n].parent.name if bones[n].parent else None for n in TORSO}
        self.basis0 = {n: pbs[n].matrix_basis.copy() for n in TORSO}
        self.keyed0 = {n: pbs[n].matrix_basis.copy() for n in keyed(rig)}
        self.targets = targets(rig)
        self.targets0 = {k: pbs[TARGET_BONES[k]].matrix.copy() for k in self.targets}
        self.fk0 = self.fk(self.basis0)
        self.R0 = {n: m.to_quaternion() for n, m in self.fk0.items()}
        self.look0, self.look_on = look_point(rig)
        self.handle0 = handle_positions(rig)[key]
        self.pins = pinned(rig)
        if key in LIMBS and mode in ("BODY", "REACH"):
            self.pins.add(key)  # a dragged hand or foot stays where you put it
        # while one hand is dragged the other stays where it is, softly: it gives way before the drag stalls
        self.hold = {OTHER_HAND[key]} - self.pins if key in OTHER_HAND else set()
        self.limbs = {}
        for k, (a, b) in LIMBS.items():
            L = bones[a].length + bones[b].length
            d0 = (self.targets0[k].translation - self.fk0[a].translation).length
            self.limbs[k] = (a, L, max(0.995 * L, d0))
        self.com0 = self.com(self.fk0, {k: m.translation for k, m in self.targets0.items()})
        self.x = np.zeros(15)

    @staticmethod
    def com(F, T):
        """Rough centre of mass: mostly hips and chest, a little of each hand."""
        return (0.45 * F["SKEL_ROOT"].translation + 0.35 * F["SKEL_Spine3"].translation
                + 0.1 * T["hand_l"] + 0.1 * T["hand_r"])

    # --- kinematics -------------------------------------------------------------------------
    def fk(self, bases):
        out = {}
        for n in TORSO:
            p = self.parent[n]
            out[n] = (out[p] @ self.rel[n] if p else self.rel[n]) @ bases[n]
        return out

    def bases(self, x):
        B = dict(self.basis0)
        b0 = self.basis0["SKEL_ROOT"]
        loc = b0.to_translation() + self.rel["SKEL_ROOT"].to_quaternion().inverted() @ Vector(x[0:3])
        B["SKEL_ROOT"] = Matrix.Translation(loc) @ b0.to_quaternion().to_matrix().to_4x4() @ _conj(_rotvec(x[3:6]), self.R0["SKEL_ROOT"])
        q = _rotvec(x[6:9] / len(SPINE))
        for n in SPINE:
            B[n] = self.basis0[n] @ _conj(q, self.R0[n])
        B["SKEL_L_Clavicle"] = self.basis0["SKEL_L_Clavicle"] @ _conj(_rotvec(x[9:12]), self.R0["SKEL_L_Clavicle"])
        B["SKEL_R_Clavicle"] = self.basis0["SKEL_R_Clavicle"] @ _conj(_rotvec(x[12:15]), self.R0["SKEL_R_Clavicle"])
        return B

    def aim(self, B, look):
        """Turn neck (40%) then head (rest) so the head's +Y faces the look target."""
        for n, share in (("SKEL_Neck_1", 0.4), ("SKEL_Head", 1.0)):
            F = self.fk(B)
            head = F["SKEL_Head"]
            fwd = head.col[1].xyz.normalized()
            to = (look - head.translation).normalized()
            q = fwd.rotation_difference(to)
            if q.angle > 1.4:  # ~80 degrees, past that a head looks broken
                q = Quaternion(q.axis, 1.4)
            q = Quaternion().slerp(q, share)
            B[n] = B[n] @ _conj(q, F[n].to_quaternion())
        return B

    # --- solve ------------------------------------------------------------------------------
    def residuals(self, x, targets, goals, balance):
        F = self.fk(self.bases(x))
        r = []
        for k in (self.pins | self.hold) & LIMBS.keys():
            root, L, cap = self.limbs[k]
            d = (targets[k] - F[root].translation).length
            # when not everything can be reached, the dragged limb falls short first, then a held hand;
            # a pin never gives
            w = 100 if k == self.key else 300 if k in self.hold else 10000
            r.append(w * max(0.0, d - cap))
        for bone, (p, w) in goals.items():  # a torso bone pulled toward p
            r.extend(w * (F[bone].translation - p))
        if balance:  # hips shift against a reach, so the weight stays over the feet
            c = self.com(F, targets) - self.com0
            r.extend((6 * self.bf * c.x, 6 * self.bf * c.y))
        stiff = (1 - self.bf) * 8 + 0.15
        r.extend(np.sqrt(stiff * PARAM_W) * x)
        return np.array(r, float)

    def solve(self, targets, goals, balance=False):
        x, mu = self.x.copy(), 1e-3
        f = lambda v: self.residuals(v, targets, goals, balance)
        r = f(x)
        for _ in range(15):
            J = np.empty((len(r), len(x)))
            for i in range(len(x)):
                xp = x.copy()
                xp[i] += EPS
                J[:, i] = (f(xp) - r) / EPS
            dx = np.linalg.solve(J.T @ J + mu * np.eye(len(x)), -J.T @ r)
            for step in (1, 0.5, 0.25, 0.125):  # a full step can overshoot into a pin's hinge
                r2 = f(x + step * dx)
                if r2 @ r2 < r @ r:
                    x, r, mu = x + step * dx, r2, mu * 0.5
                    break
            else:
                mu *= 4
            if np.abs(dx).max() < 1e-5:
                break
        self.x = x
        return x

    # --- write ------------------------------------------------------------------------------
    def _write(self, x, T, carried, look, look_on):
        B = self.bases(x)
        if look_on:
            B = self.aim(B, look)
            self.rig["crck_look"] = list(look)
        pbs = self.rig.pose.bones
        for n in BODY_BONES:
            pbs[n].matrix_basis = B[n]
        F = self.fk(B)
        for k in self.targets:
            m = T[k]
            if k in carried:
                c = CARRY[k]
                m = F[c] @ self.fk0[c].inverted() @ self.targets0[k]
            pbs[TARGET_BONES[k]].matrix = m

    def _start(self):
        T = {k: m.copy() for k, m in self.targets0.items()}
        carried = {k for k in self.targets if k not in self.pins | self.hold and k != self.key}
        return T, carried, self.look0.copy(), self.look_on

    def move(self, p):
        """p: new handle position in armature space."""
        key = self.key
        T, carried, look, look_on = self._start()
        pos = lambda: {k: m.translation for k, m in T.items()}
        x = np.zeros(15)

        if self.mode == "WHOLE" or key == "root":  # everything moves rigidly
            d = p - self.handle0
            x[0:3] = d
            for k in T:
                T[k] = Matrix.Translation(d) @ T[k]
            look += d
            carried = set()
        elif key == "look":
            look, look_on = p, True
        elif key in GOAL_BONES:
            if key == "hips" and (self.mode == "LIMB" or self.bf <= 0):
                x[0:3] = p - self.handle0
            else:
                x = self.solve(pos(), {GOAL_BONES[key]: (p, 300)}, balance=key != "hips")
        elif key in BENDS:
            # point the knee or elbow at p: the pole goes out past p, away from the root-to-end line
            root = self.fk0[BENDS[key][0]].translation
            end = T[BENDS[key][1]].translation
            axis = (end - root).normalized()
            out = (p - root) - axis * (p - root).dot(axis)
            if out.length > 1e-4:
                pole = p + out.normalized() * 0.5
                T[key] = Matrix.Translation(pole - T[key].translation) @ T[key]
        else:
            T[key] = Matrix.Translation(p - T[key].translation) @ T[key]
            if self.mode == "REACH" and self.bf > 0:  # Ctrl-drag: the body helps once the limb can't reach
                x = self.solve(pos(), {})
            if self.mode == "LIMB":
                carried = set()
        self._write(x, T, carried, look, look_on)

    def turn(self, axis, angle):
        """Turn the handle by `angle` radians about `axis` (armature space)."""
        key = self.key
        T, carried, look, look_on = self._start()
        x = np.zeros(15)
        axis = Vector(axis).normalized()
        R = Matrix.Rotation(angle, 4, axis)
        if key in LIMBS:
            T[key] = turn_control(self.rig, TARGET_BONES[key], T[key], R, T[key].translation)
        elif key in BENDS:
            # twist the limb about its root-to-end line: the knee or elbow swings round it and the foot or hand
            # turns by the same angle. Pressing X, Y, Z or V picks the axis, and which way along the limb it points
            # decides which way the mouse turns the limb.
            root_bone, end = BENDS[key]
            root = self.fk0[root_bone].translation
            along = (T[end].translation - root).normalized()
            R = Matrix.Rotation(angle if axis.dot(along) >= 0 else -angle, 4, along)
            joint = T[key].translation
            T[key] = Matrix.Translation(root + R.to_3x3() @ (joint - root) - joint) @ T[key]
            T[end] = turn_control(self.rig, TARGET_BONES[end], T[end], R, T[end].translation)
            carried.discard(end)  # a carried target would be put back where the body leaves it
        elif key == "hips":
            x[3:6] = axis * angle
        elif key == "chest":
            x[6:9] = axis * angle
        elif key == "root":  # the whole character about the vertical through the hips
            R = Matrix.Rotation(angle, 4, "Z")
            x[3:6] = (0, 0, angle)
            pivot = self.fk0["SKEL_ROOT"].translation
            for k in T:
                T[k] = turn_control(self.rig, TARGET_BONES[k], T[k], R, pivot)
            look = (R @ (look - pivot).to_4d()).to_3d() + pivot
            carried = set()
        self._write(x, T, carried, look, look_on)

    def cancel(self):
        for n, m in self.keyed0.items():
            self.rig.pose.bones[n].matrix_basis = m
        if self.look_on:
            self.rig["crck_look"] = list(self.look0)
        elif "crck_look" in self.rig:
            del self.rig["crck_look"]

    def commit(self, scene, settings):
        set_pinned(self.rig, self.pins)
        if scene.tool_settings.use_keyframe_insert_auto:
            key_pose(self.rig, scene.frame_current)
            propagate(self.rig, scene.frame_current, self.keyed0, settings)


# --- keys ---------------------------------------------------------------------------------
def key_pose(rig, frame):
    for n in keyed(rig):
        pb = rig.pose.bones[n]
        pb.keyframe_insert("location", frame=frame, group=n)
        pb.keyframe_insert("rotation_quaternion", frame=frame, group=n)


def fcurves(rig):
    ad = rig.animation_data
    if not ad or not ad.action:
        return []
    cb = anim_utils.action_get_channelbag_for_slot(ad.action, ad.action_slot)
    return list(cb.fcurves) if cb else []


def falloff(kind, u):
    if kind == "LINEAR":
        return 1 - u
    if kind == "SHARP":
        return (1 - u) ** 3
    if kind == "HOLD":
        return 1.0 if u < 0.6 else falloff("SMOOTH", (u - 0.6) / 0.4)
    return 1 - u * u * (3 - 2 * u)


def _set_key(fc, frame, value):
    for kp in fc.keyframe_points:
        if abs(kp.co.x - frame) < 1e-3:
            d = value - kp.co.y
            kp.co.y += d
            kp.handle_left.y += d
            kp.handle_right.y += d
            return
    fc.keyframe_points.insert(frame, value)


def propagate(rig, frame, before, settings):
    """Carry the edit made at `frame` into neighbouring keys, faded by the falloff (Reach)."""
    R, s = settings.reach, settings.intensity
    if R <= 0 or s <= 0:
        return
    curves = {}
    for fc in fcurves(rig):
        m = re.match(r'pose\.bones\["(.+)"\]\.(location|rotation_quaternion)$', fc.data_path)
        if m:
            curves[(m[1], m[2], fc.array_index)] = fc
    touched = set()
    for n, B0 in before.items():
        B1 = rig.pose.bones[n].matrix_basis
        dl = B1.to_translation() - B0.to_translation()
        ql = B0.to_quaternion().inverted() @ B1.to_quaternion()
        if ql.w < 0:
            ql.negate()
        if dl.length < 1e-6 and ql.angle < 1e-5:
            continue
        loc = [curves.get((n, "location", i)) for i in range(3)]
        rot = [curves.get((n, "rotation_quaternion", i)) for i in range(4)]
        frames = {round(kp.co.x) for fc in loc + rot if fc for kp in fc.keyframe_points}
        for f in frames:
            if not 0 < abs(f - frame) <= R:
                continue
            w = falloff(settings.falloff, abs(f - frame) / (R + 1)) * s
            if all(loc):
                for i, fc in enumerate(loc):
                    _set_key(fc, f, fc.evaluate(f) + dl[i] * w)
                touched.update(loc)
            if all(rot):
                q = Quaternion([fc.evaluate(f) for fc in rot]) @ Quaternion().slerp(ql, w)
                for i, fc in enumerate(rot):
                    _set_key(fc, f, q[i])
                touched.update(rot)
    for fc in touched:
        fc.update()


# --- viewport tool ------------------------------------------------------------------------
_running = None  # the live AutoposeOperator, read by the draw callback and the panel
_handle = None   # its viewport draw handler


def is_running():
    """The live Autopose operator, or None. Blender throws a running modal operator away without telling us
    when a file loads or when the modal raises, which leaves `_running` pointing at a dead object."""
    try:
        _running.stop  # touching a discarded operator raises
        return _running
    except (ReferenceError, AttributeError):
        return None


def cleanup():
    """Forget the operator and take its handles off the viewport. Safe to call at any time, any number of times."""
    global _running, _handle
    if _handle is not None:
        try:
            bpy.types.SpaceView3D.draw_handler_remove(_handle, "WINDOW")
        except (ValueError, RuntimeError):
            pass
        _handle = None
    _running = None


@bpy.app.handlers.persistent
def on_load(*_):
    cleanup()  # the modal operator doesn't survive loading a file


def _circle2d(c, r, n=20):
    return [(c.x + r * math.cos(2 * math.pi * i / n), c.y + r * math.sin(2 * math.pi * i / n)) for i in range(n)]


def _ring3d(center, r, n=48):
    return [center + Vector((r * math.cos(2 * math.pi * i / n), r * math.sin(2 * math.pi * i / n), 0)) for i in range(n)]


def _loop(pts):
    return [v for i in range(len(pts)) for v in (pts[i], pts[(i + 1) % len(pts)])]


def _project(region, rv3d, W, pts):
    out = [view3d_utils.location_3d_to_region_2d(region, rv3d, W @ p) for p in pts]
    return None if any(p is None for p in out) else out


def _draw():
    op = is_running()
    if op is None:  # dead operator: a draw callback can't remove itself, so ask for it a moment later
        if _handle is not None and not bpy.app.timers.is_registered(cleanup):
            bpy.app.timers.register(cleanup, first_interval=0.0)
        return
    rig = op.rig
    if rig is None:
        return
    region, rv3d = bpy.context.region, bpy.context.region_data
    W = rig.matrix_world
    pins = op.drag.pins if op.drag else pinned(rig)
    look_on = look_point(rig)[1]
    shader = gpu.shader.from_builtin("UNIFORM_COLOR")

    def draw(prim, pts, col):
        shader.uniform_float("color", col)
        batch_for_shader(shader, prim, {"pos": pts}).draw(shader)

    gpu.state.blend_set("ALPHA")
    gpu.state.line_width_set(2.0)
    pos = handle_positions(rig)

    ring = _project(region, rv3d, W, _ring3d(pos.pop("root"), ROOT_RADIUS))
    if ring:
        col = COLORS["root"] if op.hover == "root" else (*COLORS["root"][:3], 0.6)
        draw("LINES", _loop(ring), col)

    for key, p3 in pos.items():
        p = view3d_utils.location_3d_to_region_2d(region, rv3d, W @ p3)
        if p is None:
            continue
        col = list(COLORS[key])
        if key == "look" and not look_on:
            col[3] = 0.35
        hover = key == op.hover
        r = 11 if hover else 9
        disc = lambda rad: [v for c in [_circle2d(p, rad)] for i in range(len(c)) for v in (p, c[i], c[(i + 1) % len(c)])]
        if key == "hips":  # a square, like a centre of gravity control
            for rr, cc in ((r + 2, (0, 0, 0, 0.6)), (r, col)):
                sq = [(p.x - rr, p.y - rr), (p.x + rr, p.y - rr), (p.x + rr, p.y + rr), (p.x - rr, p.y + rr)]
                draw("TRIS", [sq[0], sq[1], sq[2], sq[0], sq[2], sq[3]], cc)
        elif key in RINGS or (key in LIMBS and key not in pins):  # aims, or rides with the body: a ring
            draw("LINES", _loop(_circle2d(p, r + 1)), (0, 0, 0, 0.6))
            draw("LINES", _loop(_circle2d(p, r)), col)
            draw("TRIS", disc(3), col)
        else:  # holds its place: a solid dot with a dark rim so it reads on any background
            draw("TRIS", disc(r + 2), (0, 0, 0, 0.6))
            draw("TRIS", disc(r), col)
        if hover:
            draw("LINES", _loop(_circle2d(p, r + 4)), (1, 1, 1, 0.95))
    gpu.state.line_width_set(1.0)
    gpu.state.blend_set("NONE")


class CRCK_OT_autopose(bpy.types.Operator):
    """Drag hips, chest, hands, feet, knees, the look-at or the ground ring; the body follows. Esc to stop"""
    bl_idname = "crck_motion.autopose"
    bl_label = "Autopose"

    @property
    def rig(self):
        """Looked up by name each time: undo and file reloads replace the object, so a stored one goes stale."""
        return bpy.data.objects.get(self.rig_name)

    @classmethod
    def poll(cls, context):
        return context.area and context.area.type == "VIEW_3D" and is_ped_rig(context.active_object)

    def invoke(self, context, event):
        global _running, _handle
        live = is_running()
        if live is not None:  # second click on the button stops it
            live.stop = True
            live.teardown()  # at once: don't wait for the next mouse move
            return {"FINISHED"}
        cleanup()  # whatever a dead operator left behind
        self.rig_name = context.active_object.name
        self.region = next(r for r in context.area.regions if r.type == "WINDOW")
        self.rv3d = context.space_data.region_3d
        self.area = context.area
        self.drag, self.hover, self.stop = None, None, False
        self.turning, self.click_confirms = False, False
        if context.mode != "POSE":
            bpy.ops.object.mode_set(mode="POSE")
        _handle = bpy.types.SpaceView3D.draw_handler_add(_draw, (), "WINDOW", "POST_PIXEL")
        _running = self
        context.window_manager.modal_handler_add(self)
        self.area.tag_redraw()
        return {"RUNNING_MODAL"}

    def teardown(self):
        """Switch Autopose off: undo a drag in progress, drop the handles. Idempotent."""
        if self.drag:
            try:
                self.drag.cancel()
            except ReferenceError:  # the rig was replaced under the drag; nothing left to restore
                pass
            self.drag = None
        cleanup()
        try:
            self.area.header_text_set(None)
            self.area.tag_redraw()
        except ReferenceError:  # the area is gone (workspace or file changed)
            pass

    def finish(self):
        self.teardown()
        return {"FINISHED"}

    # --- screen helpers -----------------------------------------------------------------------
    def mouse(self, event):
        return Vector((event.mouse_x - self.region.x, event.mouse_y - self.region.y))

    def in_region(self, m):
        return 0 <= m.x < self.region.width and 0 <= m.y < self.region.height

    def screen(self, p):
        return view3d_utils.location_3d_to_region_2d(self.region, self.rv3d, self.rig.matrix_world @ p)

    def pick(self, m):
        pos = handle_positions(self.rig)
        center = pos.pop("root")
        best, best_d = None, 15.0
        for key, p3 in pos.items():
            p = self.screen(p3)
            if p is not None and (p - m).length < best_d:
                best, best_d = key, (p - m).length
        if best:
            return best
        ring = _project(self.region, self.rv3d, self.rig.matrix_world, _ring3d(center, ROOT_RADIUS))
        if ring and any(geometry.intersect_point_line_segment(m, a, b)[1] < 8
                        for a, b in zip(ring, ring[1:] + ring[:1])):
            return "root"
        return None

    def to_armature(self, m):
        W = self.rig.matrix_world
        if self.drag.key == "root":  # slide along the floor
            o = view3d_utils.region_2d_to_origin_3d(self.region, self.rv3d, m)
            d = view3d_utils.region_2d_to_vector_3d(self.region, self.rv3d, m)
            floor = W @ self.drag.handle0
            hit = geometry.intersect_line_plane(o, o + d, floor, W.to_3x3() @ Vector((0, 0, 1)))
            world = hit if hit is not None else floor
        else:
            world = view3d_utils.region_2d_to_location_3d(self.region, self.rv3d, m, W @ self.drag.handle0)
        return W.inverted() @ world + self.grab_offset

    def view_axis(self):
        """Toward the viewer, in armature space: turning about it follows the mouse on screen."""
        to_viewer = self.rv3d.view_rotation @ Vector((0, 0, 1))
        return (self.rig.matrix_world.to_3x3().inverted() @ to_viewer).normalized()

    def screen_angle(self, m):
        """Where the mouse is around the handle, in radians, or None while it is too close to say."""
        c = self.screen(self.drag.handle0) or m
        v = m - c
        return math.atan2(v.y, v.x) if v.length >= 20 else None

    def turn_by_mouse(self, m):
        """Total turn so far: the mouse's angle around the handle, counted from where it first got clear of the
        handle (pressing R with the mouse on the dot gives no direction yet), and unwrapped so a full turn works."""
        a = self.screen_angle(m)
        if a is not None:
            if self.angle_prev is not None:
                d = a - self.angle_prev
                self.angle_total += (d + math.pi) % (2 * math.pi) - math.pi
            self.angle_prev = a
        return self.angle_total

    def header(self, context):
        s = context.scene.crck_motion
        if self.turning and self.drag and self.drag.key in BENDS:
            text = "Twist the limb  |  X/Y/Z or V: flip the direction   Click/Enter: confirm   Esc/RMB: cancel"
        elif self.turning:
            text = "Turn  |  X/Y/Z: axis   V: view axis   Click/Enter: confirm   Esc/RMB: cancel"
        else:
            text = "Move  |  Shift: whole body   Ctrl: body helps a hand/foot reach   Alt: only this limb   Wheel: Reach {}   Esc/RMB: cancel".format(s.reach)
        self.area.header_text_set(text)

    # --- drags ---------------------------------------------------------------------------------
    def begin(self, context, event, key, turning=False, click_confirms=False):
        s = context.scene.crck_motion
        mode = "WHOLE" if event.shift else "LIMB" if event.alt else "REACH" if event.ctrl else "BODY"
        self.drag = Drag(self.rig, key, mode, s.body_follows)
        self.turning, self.click_confirms = turning, click_confirms
        m = self.mouse(event)
        if turning:
            self.axis = self.view_axis()
            self.angle_prev, self.angle_total = None, 0.0
        else:
            self.grab_offset = Vector()
            self.grab_offset = self.drag.handle0 - self.to_armature(m)
        self.header(context)

    def update(self, m):
        if self.turning:
            self.drag.turn(self.axis, self.turn_by_mouse(m))
        else:
            self.drag.move(self.to_armature(m))

    def end(self, context, keep):
        if keep:
            self.drag.commit(context.scene, context.scene.crck_motion)
            bpy.ops.ed.undo_push(message="Autopose")
        else:
            self.drag.cancel()
        self.drag = None
        self.turning = False
        self.area.header_text_set(None)

    def modal(self, context, event):
        try:
            return self._modal(context, event)
        except Exception:  # whatever went wrong, don't leave a half-running tool behind
            import traceback
            traceback.print_exc()
            self.teardown()
            self.report({"ERROR"}, "Autopose stopped on an error (details in the console)")
            return {"CANCELLED"}

    def _modal(self, context, event):
        if self.stop or not is_ped_rig(self.rig) or context.mode != "POSE":
            return self.finish()
        self.area.tag_redraw()
        m = self.mouse(event)
        s = context.scene.crck_motion

        if self.drag:
            if event.type == "MOUSEMOVE":
                self.update(m)
            elif self.turning and event.type in {"X", "Y", "Z", "V"} and event.value == "PRESS":
                self.axis = self.view_axis() if event.type == "V" else Vector(
                    {"X": (1, 0, 0), "Y": (0, 1, 0), "Z": (0, 0, 1)}[event.type])
                self.update(m)
            elif event.type in {"WHEELUPMOUSE", "WHEELDOWNMOUSE"}:
                s.reach = max(0, s.reach + (1 if event.type == "WHEELUPMOUSE" else -1))
                self.header(context)
            elif event.type == "LEFTMOUSE" and event.value == ("PRESS" if self.click_confirms else "RELEASE"):
                self.end(context, True)
            elif event.type in {"RET", "NUMPAD_ENTER"} and event.value == "PRESS":
                self.end(context, True)
            elif event.type in {"RIGHTMOUSE", "ESC"} and event.value == "PRESS":
                self.end(context, False)
            return {"RUNNING_MODAL"}

        if not self.in_region(m):
            return {"PASS_THROUGH"}
        if event.type == "MOUSEMOVE":
            self.hover = self.pick(m)
        elif event.type == "LEFTMOUSE" and event.value == "PRESS" and self.hover:
            self.begin(context, event, self.hover)
            return {"RUNNING_MODAL"}
        elif event.type == "G" and event.value == "PRESS" and self.hover:
            self.begin(context, event, self.hover, click_confirms=True)
            return {"RUNNING_MODAL"}
        elif event.type == "R" and event.value == "PRESS" and self.hover in TURNABLE:
            self.begin(context, event, self.hover, turning=True, click_confirms=True)
            return {"RUNNING_MODAL"}
        elif event.type == "P" and event.value == "PRESS" and self.hover in LIMBS:
            toggle_pin(self.rig, self.hover)
            return {"RUNNING_MODAL"}
        elif event.type == "ESC" and event.value == "PRESS":
            return self.finish()
        return {"PASS_THROUGH"}


def toggle_pin(rig, key):
    if key == "look":
        if "crck_look" in rig:
            del rig["crck_look"]
        return
    pins = pinned(rig)
    pins ^= {key}
    set_pinned(rig, pins)
