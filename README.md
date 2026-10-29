# CRCK Motion

**CRCK Motion** is a Blender add-on for posing and animating the GTA V Freemode ped by hand. Drag a hand, a foot or the hips, and the body follows: the torso leans, twists and drops so the limbs you've pinned can still reach, and the rig's own IK does the arms and legs. Every pose lands as ordinary keyframes on the control rig. When you're done, one click bakes it onto the game skeleton, ready to export with [Sollumz](https://github.com/Sollumz/Sollumz).

**Version 0.1.0** · Blender 4.4+ (tested on 5.2.3) · Windows, macOS and Linux · GPL-3.0-or-later

> First release. The solver, keying, bake and rig import are covered by an automated test. The viewport handles have not had much hands-on use yet, so please report anything odd in the Issues tab.

## What it does

| You want to... | CRCK Motion gives you |
|---|---|
| Start from a ready rig | **Add Freemode Rig**: the Freemode male control rig and its game skeleton (`skel`), bundled in the add-on. No ped mesh is included (it is Rockstar's): import your ped with Sollumz and bind it to `skel` to see it |
| Pose without rotating bone by bone | **Autopose**: drag handles on the hips, hands, feet, knees and a look-at target. The body follows |
| Keep a foot planted, or let a hand ride along | **Pinned or carried** per hand and foot (**P** over a handle) |
| Change one frame without breaking the motion | **Reach**: with Auto Keying on, an edit fades into the keys around it (Smooth, Hold, Linear or Sharp falloff) |
| Get the animation into the game | **Bake to Game Skeleton**: a visual bake onto `skel` with constraints kept, named `<action>_game`, then export with Sollumz as usual |

## Install

1. Download `crck_motion-<version>.zip` from the [Releases](../../releases) page, or build it yourself (see below).
2. In Blender, go to Edit > Preferences > Get Extensions > the arrow menu > **Install from Disk** and pick the zip.
3. Everything is in one tab of the 3D Viewport sidebar (press **N**): **CRCK Motion**.

CRCK Motion is pure Python. It uses only what ships with Blender (numpy included), so there's nothing else to install. Sollumz is only needed when you export.

## How to use it

1. **Add Freemode Rig**, or select a control rig built the same way (bone names `SKEL_*`, IK targets `IK_L_FA` / `IK_R_FA`, `LEG_L_IK` / `LEG_R_IK`, knee poles `LEG_L_POLE` / `LEG_R_POLE`).
2. Click **Autopose**. Handles appear: hips (purple), hands and feet (yellow is left, red is right), knees, and the look-at (green).

| In Autopose | What it does |
|---|---|
| Drag a handle | Moves that joint. The torso helps as much as **Body Follows** allows (0 means only the limb moves) |
| Shift-drag | Moves the whole character |
| Alt-drag | Moves only that limb |
| **P** over a hand or foot | Pinned (stays put in the world) or carried (rides with the chest or pelvis). Feet start pinned, and a hand or foot you drag becomes pinned |
| Drag the look-at | Aims the head (40% neck, 60% head). **Look-at Off** in the panel turns it off |
| Mouse wheel while dragging | Changes Reach |
| Esc or right-click while dragging | Cancels the drag |
| Esc otherwise | Leaves the tool |

Clicks away from the handles behave as normal in Blender. Each drag is one undo step.

3. Key the pose with **Key Pose**, or switch on Auto Keying so every drag is keyed and carried into the keys within **Reach** frames.
4. **Bake to Game Skeleton** bakes the scene's frame range onto `skel`. Then export with Sollumz.

## How it works

Each control-rig bone drives the matching `skel` bone through Copy Transforms. The game only ever sees `skel`, and CRCK Motion adds no bones to it.

When you drag, a small solver (damped least squares) finds how far the hips should move and turn, how much the spine should bend and how far the collarbones should shift, so that every pinned hand and foot stays within reach of its limb. A pin always wins over a hips drag. The IK constraints already on the rig then place the arms and legs. A solve takes about 5 to 10 ms.

## Known gaps

- The bundled rig's right-arm IK points at a pole bone, `POLE_R_FA`, that doesn't exist, and the left arm has no pole. Elbows bend wherever the IK solver puts them.
- Not built yet: mirror pose, lock in place, motion trail, smoothing brush and hand presets. Reach only moves keys that already exist.
- Only the Freemode male rig is bundled.

## Build and test

```bash
blender --command extension build --source-dir crck_motion --output-dir .
```

```bash
blender -b crck_motion/assets/freemode_m.blend --factory-startup --python test_crck_motion.py
```

The test checks that a hand reaches a far target while the feet stay planted, that a crouch keeps the feet down, that the hips stop once the legs are straight, that the head aims at the look-at, that Reach falloff works on neighbouring keys, that the bake matches the control rig, and that Add Freemode Rig works.

## Built with AI

Artificial intelligence was used to help develop this tool.

## Licence

GPL-3.0-or-later. See [LICENSE](LICENSE). GTA V and its assets belong to Rockstar Games. This is an unofficial fan tool, not affiliated with or endorsed by Rockstar Games or Take-Two Interactive. Use it for single-player and FiveM work, not GTA Online.
