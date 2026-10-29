"""Autopose: drag a handle, solve the torso so the pinned limbs can still reach, let the rig's IK do the limbs."""

import re

import bpy
import gpu
import numpy as np
from bpy_extras import anim_utils, view3d_utils
from gpu_extras.batch import batch_for_shader
from mathutils import Matrix, Quaternion, Vector

TORSO = [
    "SKEL_ROOT", "SKEL_Pelvis", "SKEL_L_Thigh", "SKEL_R_Thigh",
    "SKEL_Spine_Root", "SKEL_Spine0", "SKEL_Spine1", "SKEL_Spine2", "SKEL_Spine3",
    "SKEL_L_Clavicle", "SKEL_R_Clavicle", "SKEL_L_UpperArm", "SKEL_R_UpperArm",
    "SKEL_Neck_1", "SKEL_Head",
]
SPINE = ["SKEL_Spine0", "SKEL_Spine1", "SKEL_Spine2", "SKEL_Spine3"]

# handle -> control bone it moves
HANDLE_BONES = {
    "hips": "SKEL_ROOT",
    "hand_l": "IK_L_FA", "hand_r": "IK_R_FA",
    "foot_l": "LEG_L_IK", "foot_r": "LEG_R_IK",
    "knee_l": "LEG_L_POLE", "knee_r": "LEG_R_POLE",
}
TARGETS = [k for k in HANDLE_BONES if k != "hips"]
LIMBS = {
    "hand_l": ("SKEL_L_UpperArm", "SKEL_L_Forearm"), "hand_r": ("SKEL_R_UpperArm", "SKEL_R_Forearm"),
    "foot_l": ("SKEL_L_Thigh", "SKEL_L_Calf"), "foot_r": ("SKEL_R_Thigh", "SKEL_R_Calf"),
}
# a target that isn't pinned rides along with this torso bone
CARRY = {"hand_l": "SKEL_Spine3", "hand_r": "SKEL_Spine3",
         "foot_l": "SKEL_Pelvis", "foot_r": "SKEL_Pelvis",
         "knee_l": "SKEL_Pelvis", "knee_r": "SKEL_Pelvis"}
# everything a pose writes, so everything Key Pose keys
KEYED = ["SKEL_ROOT", *SPINE, "SKEL_L_Clavicle", "SKEL_R_Clavicle", "SKEL_Neck_1", "SKEL_Head",
         *(HANDLE_BONES[k] for k in TARGETS)]
REQUIRED = set(TORSO) | set(HANDLE_BONES.values()) | {"SKEL_L_Forearm", "SKEL_R_Forearm", "SKEL_L_Calf", "SKEL_R_Calf"}

# rig colour convention: yellow left, red right, purple lower body, green look
COLORS = {"hips": (0.7, 0.35, 1, 1), "hand_l": (1, 0.85, 0.1, 1), "hand_r": (1, 0.2, 0.2, 1),
          "foot_l": (1, 0.85, 0.1, 1), "foot_r": (1, 0.2, 0.2, 1),
          "knee_l": (1, 0.85, 0.1, 0.6), "knee_r": (1, 0.2, 0.2, 0.6), "look": (0.3, 1, 0.4, 1)}

# solver parameter layout: root move(3) root turn(3) spine bend(3) clavicles(3+3), with stiffness per param
PARAM_W = np.array([2, 2, 2, 3, 3, 3, 1, 1, 1, 6, 6, 6, 6, 6, 6], float)
EPS = 1e-4


def is_ped_rig(obj):
    return obj is not None and obj.type == "ARMATURE" and REQUIRED.issubset(obj.pose.bones.keys())


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


def handle_positions(rig):
    pbs = rig.pose.bones
    out = {k: pbs[b].head.copy() for k, b in HANDLE_BONES.items()}
    out["look"] = look_point(rig)[0]
    return out


def _rotvec(v):
    v = Vector(v)
    return Quaternion(v.normalized(), v.length) if v.length > 1e-9 else Quaternion()


def _conj(q, Rw):
    """World-space turn q expressed as a local turn of a bone whose world rotation is Rw."""
    return (Rw.inverted() @ q @ Rw).to_matrix().to_4x4()


class Drag:
    """One drag of one handle. Kept free of modal/UI code so it can be driven from a script."""

    def __init__(self, rig, key, mode="BODY", body_follows=0.5):
        self.rig, self.key, self.mode, self.bf = rig, key, mode, body_follows
        bones, pbs = rig.data.bones, rig.pose.bones
        self.rel = {}
        for n in TORSO:
            b = bones[n]
            self.rel[n] = b.parent.matrix_local.inverted() @ b.matrix_local if b.parent else b.matrix_local.copy()
        self.parent = {n: bones[n].parent.name if bones[n].parent else None for n in TORSO}
        self.basis0 = {n: pbs[n].matrix_basis.copy() for n in TORSO}
        self.keyed0 = {n: pbs[n].matrix_basis.copy() for n in KEYED}
        self.targets0 = {k: pbs[HANDLE_BONES[k]].matrix.copy() for k in TARGETS}
        self.fk0 = self.fk(self.basis0)
        self.R0 = {n: m.to_quaternion() for n, m in self.fk0.items()}
        self.look0, self.look_on = look_point(rig)
        self.handle0 = self.look0.copy() if key == "look" else handle_positions(rig)[key]
        self.pins = pinned(rig)
        if key in LIMBS and mode == "BODY":
            self.pins.add(key)  # a dragged hand or foot stays where you put it
        self.limbs = {}
        for k, (a, b) in LIMBS.items():
            L = bones[a].length + bones[b].length
            d0 = (self.targets0[k].translation - self.fk0[a].translation).length
            self.limbs[k] = (a, L, max(0.995 * L, d0))
        self.x = np.zeros(15)

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
    def residuals(self, x, targets, hips_target):
        F = self.fk(self.bases(x))
        r = []
        for k in self.pins & LIMBS.keys():
            root, L, cap = self.limbs[k]
            d = (targets[k] - F[root].translation).length
            r.append(1000 * max(0.0, d - cap))  # far above the hips pull, so a pin always wins
            if k.startswith("hand"):  # keep the arm a little bent if the body is allowed to follow
                r.append(2 * self.bf * max(0.0, d - 0.85 * L))
        if hips_target is not None:
            r.extend(30 * (F["SKEL_ROOT"].translation - hips_target))
        stiff = (1 - self.bf) * 8 + 0.15
        r.extend(np.sqrt(stiff * PARAM_W) * x)
        return np.array(r, float)

    def solve(self, targets, hips_target):
        x, mu = self.x.copy(), 1e-3
        f = lambda v: self.residuals(v, targets, hips_target)
        r = f(x)
        for _ in range(10):
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

    # --- drag -------------------------------------------------------------------------------
    def move(self, p):
        """p: new handle position in armature space."""
        rig, key = self.rig, self.key
        targets = {k: m.translation.copy() for k, m in self.targets0.items()}
        look, look_on = self.look0.copy(), self.look_on
        carried = {k for k in TARGETS if k not in self.pins and k != key}
        x, hips_target = np.zeros(15), None

        if self.mode == "WHOLE":  # everything moves rigidly
            d = p - self.handle0
            x[0:3] = d
            for k in targets:
                targets[k] += d
            look += d
            carried = set()
        elif key == "look":
            look, look_on = p, True
        elif key == "hips":
            hips_target = p
            if self.mode == "LIMB" or self.bf <= 0:
                x[0:3] = p - self.handle0
            else:
                x = self.solve(targets, hips_target)
        else:
            targets[key] = p
            if self.mode == "BODY" and self.bf > 0 and key in LIMBS:
                x = self.solve(targets, None)
            if self.mode == "LIMB":
                carried = set()

        B = self.bases(x)
        if look_on:
            B = self.aim(B, look)
            rig["crck_look"] = list(look)
        pbs = rig.pose.bones
        for n in KEYED:
            if n in B:
                pbs[n].matrix_basis = B[n]
        F = self.fk(B)
        for k in TARGETS:
            m = self.targets0[k]
            if k in carried:
                c = CARRY[k]
                m = F[c] @ self.fk0[c].inverted() @ m
            else:
                m = Matrix.Translation(targets[k] - m.translation) @ m
            pbs[HANDLE_BONES[k]].matrix = m

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
    for n in KEYED:
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


def _circle(c, r, n=20):
    import math
    return [(c.x + r * math.cos(2 * math.pi * i / n), c.y + r * math.sin(2 * math.pi * i / n)) for i in range(n)]


def _draw():
    op = _running
    if op is None or op.rig.name not in bpy.data.objects:
        return
    region, rv3d = bpy.context.region, bpy.context.region_data
    rig, W = op.rig, op.rig.matrix_world
    pins = op.drag.pins if op.drag else pinned(rig)
    look_on = look_point(rig)[1]
    shader = gpu.shader.from_builtin("UNIFORM_COLOR")
    gpu.state.blend_set("ALPHA")
    gpu.state.line_width_set(2.0)
    for key, pos in handle_positions(rig).items():
        p = view3d_utils.location_3d_to_region_2d(region, rv3d, W @ pos)
        if p is None:
            continue
        col = list(COLORS[key])
        if key == "look" and not look_on:
            col[3] = 0.35
        r = 11 if key == op.hover else 8
        ring = _circle(p, r)
        lines = [v for i in range(len(ring)) for v in (ring[i], ring[(i + 1) % len(ring)])]
        shader.uniform_float("color", col)
        batch_for_shader(shader, "LINES", {"pos": lines}).draw(shader)
        if key in pins or key == "hips":
            tris = [v for i in range(len(ring)) for v in (p, ring[i], ring[(i + 1) % len(ring)])]
            shader.uniform_float("color", (col[0], col[1], col[2], 0.45))
            batch_for_shader(shader, "TRIS", {"pos": tris}).draw(shader)
    gpu.state.line_width_set(1.0)
    gpu.state.blend_set("NONE")


class CRCK_OT_autopose(bpy.types.Operator):
    """Drag hips, hands, feet, knees or the look-at; the body follows. Esc to stop"""
    bl_idname = "crck_motion.autopose"
    bl_label = "Autopose"

    _handle = None

    @classmethod
    def poll(cls, context):
        return context.area and context.area.type == "VIEW_3D" and is_ped_rig(context.active_object)

    def invoke(self, context, event):
        global _running
        if _running is not None:  # second click on the button stops it
            _running.stop = True
            return {"FINISHED"}
        self.rig = context.active_object
        self.region = next(r for r in context.area.regions if r.type == "WINDOW")
        self.rv3d = context.space_data.region_3d
        self.area = context.area
        self.drag, self.hover, self.stop = None, None, False
        if context.mode != "POSE":
            bpy.ops.object.mode_set(mode="POSE")
        CRCK_OT_autopose._handle = bpy.types.SpaceView3D.draw_handler_add(_draw, (), "WINDOW", "POST_PIXEL")
        _running = self
        context.window_manager.modal_handler_add(self)
        self.area.tag_redraw()
        return {"RUNNING_MODAL"}

    def finish(self):
        global _running
        if self.drag:
            self.drag.cancel()
        bpy.types.SpaceView3D.draw_handler_remove(CRCK_OT_autopose._handle, "WINDOW")
        _running = None
        self.area.header_text_set(None)
        self.area.tag_redraw()
        return {"FINISHED"}

    def mouse(self, event):
        return Vector((event.mouse_x - self.region.x, event.mouse_y - self.region.y))

    def in_region(self, m):
        return 0 <= m.x < self.region.width and 0 <= m.y < self.region.height

    def pick(self, m):
        W, best, best_d = self.rig.matrix_world, None, 15.0
        for key, pos in handle_positions(self.rig).items():
            p = view3d_utils.location_3d_to_region_2d(self.region, self.rv3d, W @ pos)
            if p is not None and (p - m).length < best_d:
                best, best_d = key, (p - m).length
        return best

    def to_armature(self, m):
        W = self.rig.matrix_world
        world = view3d_utils.region_2d_to_location_3d(self.region, self.rv3d, m, W @ self.drag.handle0)
        return W.inverted() @ world + self.grab_offset

    def header(self, context):
        s = context.scene.crck_motion
        self.area.header_text_set(
            f"Autopose  |  Shift: whole body   Alt: only this limb   Wheel: Reach {s.reach}   Esc/RMB: cancel")

    def modal(self, context, event):
        if self.stop or self.rig.name not in bpy.data.objects or context.mode != "POSE":
            return self.finish()
        self.area.tag_redraw()
        m = self.mouse(event)
        s = context.scene.crck_motion

        if self.drag:
            if event.type == "MOUSEMOVE":
                self.drag.move(self.to_armature(m))
            elif event.type in {"WHEELUPMOUSE", "WHEELDOWNMOUSE"}:
                s.reach = max(0, s.reach + (1 if event.type == "WHEELUPMOUSE" else -1))
                self.header(context)
            elif event.type == "LEFTMOUSE" and event.value == "RELEASE":
                self.drag.commit(context.scene, s)
                self.drag = None
                self.area.header_text_set(None)
                bpy.ops.ed.undo_push(message="Autopose")
            elif event.type in {"RIGHTMOUSE", "ESC"} and event.value == "PRESS":
                self.drag.cancel()
                self.drag = None
                self.area.header_text_set(None)
            return {"RUNNING_MODAL"}

        if not self.in_region(m):
            return {"PASS_THROUGH"}
        if event.type == "MOUSEMOVE":
            self.hover = self.pick(m)
        elif event.type == "LEFTMOUSE" and event.value == "PRESS" and self.hover:
            mode = "WHOLE" if event.shift else "LIMB" if event.alt else "BODY"
            self.drag = Drag(self.rig, self.hover, mode, s.body_follows)
            W = self.rig.matrix_world
            hit = view3d_utils.region_2d_to_location_3d(self.region, self.rv3d, m, W @ self.drag.handle0)
            self.grab_offset = self.drag.handle0 - W.inverted() @ hit
            self.header(context)
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
