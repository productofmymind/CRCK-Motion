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
| Start from a pose instead of a T-pose | **Presets**: a thumbnail browser for saved poses and animations. Preview, blend, mirror, apply, and save your own |
| Start from a real GTA V animation | **Browse Game Clips** (search your game's ~11,000 dictionaries) or **Import GTA Clip** (a `.ycd` or `.ycd.xml` file): editable keys on the control rig, one action per clip |
| Get the animation into the game | **Export GTA Clip**: bakes onto `skel` and saves a `.ycd` (or `.ycd.xml`) in one step. **Bake to Game Skeleton** does just the bake, for exporting with Sollumz yourself |
| Lighter, easier-to-edit keys | **Thin Keys**: the fewest whole-pose keys that keep the motion within a tolerance (imports are thinned this way too) |

## Install

1. Download `crck_motion-<version>.zip` from the [Releases](../../releases) page, or build it yourself (see below).
2. In Blender, go to Edit > Preferences > Get Extensions > the arrow menu > **Install from Disk** and pick the zip.
3. Everything is in one tab of the 3D Viewport sidebar (press **N**): **CRCK Motion**.

CRCK Motion is pure Python. It uses only what ships with Blender (numpy included), so there's nothing else to install. Sollumz is only needed when you export.

## How to use it

1. **Add Freemode Rig**, or select a control rig built the same way (bone names `SKEL_*`, IK targets `IK_L_FA` / `IK_R_FA`, `LEG_L_IK` / `LEG_R_IK`, knee poles `LEG_L_POLE` / `LEG_R_POLE`).
2. **Import Body**: pick a `.blend` that has the ped mesh skinned to its skeleton (for example your own `Freemode_M_IK_Fixed.blend`). The mesh is bound to the game skeleton so you see it deform. The file is remembered (Preferences > Add-ons > CRCK Motion > Body File), and **Add Freemode Rig** imports it every time after that. No Rockstar mesh ships with the add-on.
3. If the panel says **The body faces away from its skeleton** (scenes made with an older Add Freemode Rig), click **Fix Body Facing**: a backwards body deforms into long spikes.
4. Rig without elbow poles? Click **Add Elbow Poles** once. It adds a pole bone behind each elbow on the control rig, fitted so the pose doesn't move. The game skeleton is not touched. The bundled rig already has them.
5. Click **Autopose**. Dots sit on the body: hips (purple square), chest and head (green), hands, feet, elbows and knees (yellow is left, red is right), the look-at (green), and an orange ring on the floor. A solid dot holds its place in the world; a ring rides with the body or aims a joint.

| In Autopose | What it does |
|---|---|
| Drag a hand or foot | Moves only that arm or leg, like dragging the rig's own IK target. The body and the other limbs stay where they are |
| Ctrl-drag a hand or foot | The body helps once the limb runs out of reach: shoulder first, then the spine, the hips last. **Body Follows** sets how much |
| Drag the hips, chest or head | Moves the body there with the feet kept planted. The spine bends and the hips follow |
| Drag a knee or an elbow | Points it. It works when the limb is bent; a straight limb barely changes |
| Drag the floor ring | Slides the whole character along the floor |
| **G** over a handle | Moves it without holding the mouse button. Click or Enter to confirm |
| **R** over a hand, foot, hips, chest or floor ring | Turns it with the mouse. **X**, **Y** or **Z** turns about that axis, **V** goes back to the view axis. The floor ring always turns the character about the vertical |
| Shift-drag | Moves the whole character |
| Alt-drag | Moves only that limb |
| **P** over a hand or foot | Pinned (stays put in the world) or carried (rides with the chest or pelvis). Feet start pinned, and a hand or foot you drag becomes pinned |
| **R** over a knee or elbow | Twists the whole leg or arm about its hip-to-ankle (or shoulder-to-wrist) line: the knee swings round it and the foot or hand turns by the same angle. **X**, **Y**, **Z** or **V** flip the direction. (Dragging a knee or elbow instead only moves the joint) |
| Drag the look-at | Aims the head (40% neck, 60% head). **Look-at Off** in the panel turns it off |
| Mouse wheel while dragging | Changes Reach |
| Esc or right-click while dragging | Cancels the drag |
| Esc otherwise | Leaves the tool |

**R** turns by the mouse's angle around the handle, counted from where the mouse first moves clear of it, so you can press R with the mouse on the dot and then circle it. Autopose switches itself off, and tells you, if anything inside it fails or a file is loaded, so the button never gets stuck on.

Clicks away from the handles behave as normal in Blender. Each drag is one undo step.

6. Key the pose with **Key Pose**, or switch on Auto Keying so every drag is keyed and carried into the keys within **Reach** frames.
7. **Export GTA Clip** saves the scene's frame range as a `.ycd` for the game (or **Bake to Game Skeleton** and export with Sollumz yourself).

## Presets

The **Presets** panel sits under the main one in the CRCK Motion tab.

1. Pick a category: Idle & Standing, Sitting, Leaning, Social, Hands & Gestures, Vehicle, Police & EMS, or Custom.
2. Click a thumbnail. The character takes the pose straight away, unkeyed, so you can look at it. Change **Blend** (how much of the preset to mix into the current pose), **Mirror** (swap left and right) or **Apply To** (whole body, body only, or fingers only, for a hand shape), and the preview updates.
3. **Apply** keys the pose at the current frame. An animated preset is keyed from the current frame on. The back-arrow button puts back the pose from before the preview.
4. **Save Preset** saves the current pose, or every keyed frame in the scene's frame range (tick **Animation**), into a category of your choice, with a thumbnail. The trash button deletes one of your own presets.

A preset is ordinary keys on the control rig, so everything else keeps working on it: drag it with Autopose, carry edits into neighbouring keys with Reach, move it in the Dope Sheet, and bake it to the game skeleton as usual. It lands wherever the character is standing and facing: it's stored relative to the floor between the feet.

No presets ship with the add-on: it starts empty and fills with what you save. GTA V clips you convert are saved the same way, as your own presets, and are never part of the add-on.

Your own presets are saved in Blender's extension user folder (`extensions/.user/user_default/crck_motion/presets`) as one `.json` and one `.png` each, so they survive add-on updates and can be shared by copying the files.

## Import GTA Clip

**Import GTA Clip** (in the main panel) takes a clip dictionary, either the game's binary `.ycd` or a CodeWalker `.ycd.xml`, and turns every clip in it into an action on the control rig, ready to edit with Autopose, Reach, the Dope Sheet or the Graph Editor. **Key Tolerance** (default 1: within 5 mm, about 1 degree) thins the keys to the fewest whole poses that stay that close to the clip; a typical idle goes from 139 keys to 27. 0 keeps a key on every frame. Switch between the clips in the Action editor. **Save Preset** with **Animation** ticked keeps one as a preset.

- **Sollumz** has to be installed: it reads the clip dictionary.
- **Binary `.ycd`** is read through your own **CodeWalker** install: set its folder once in Preferences > Add-ons > CRCK Motion > CodeWalker Folder. CRCK Motion builds a small helper (`ycdtool.cs`, using the C# compiler that comes with Windows) that asks CodeWalker to turn the file into `.ycd.xml`. No CodeWalker code ships with the add-on. A `.ycd.xml` needs none of this.
- **Browse Game Clips** lists every in-game animation dictionary in your own GTA V files (about 11,000) in a searchable popup: type a few words (`cop idles`, `mime`, `sit chair`), pick one, and it is extracted and imported. The button next to it re-reads the game after an update. It uses **CRCK Explorer**: set the path to `crck-explorer-cli.exe` in Preferences > Add-ons > CRCK Motion > CRCK Explorer. When a name is in more than one archive (a patch or DLC copy), the one the game uses, the later archive, is listed. Cutscene dictionaries aren't listed: they are built for their own scene skeletons.
- A dictionary with several clips asks which to import (all ticked to start with).
- **Root Motion** (in the file browser's side panel): off, the clip plays where your character stands; on, the character moves where the game would put it (for a seated clip that is the seat's spot).
- How it converts: the game skeleton plays the clip, then each frame is written onto the control rig. Plain bones (spine, neck, head, fingers, pelvis, toes) copy their turn; hands and feet get their IK targets; elbows and knees get their poles, turned until the joint lands where the clip has it. Hands, feet, elbows, knees and head land within 0.01 cm of the clip.
- Known gaps: an arm's twist along its length can differ from the clip (the IK decides it), and facial tracks are skipped.

## Export GTA Clip

**Export GTA Clip** bakes the scene's frame range onto the game skeleton and saves it as a clip dictionary with one clip in it. Name the file (`.ycd`, or `.ycd.xml` for CodeWalker) and the clip (the name you play it by in the game). Scale, and bones that never leave their rest pose, are left out, as the game's own clips do. Writing binary `.ycd` uses your CodeWalker install, like importing it.

Round trip on a game idle: imported, thinned, exported to `.ycd` and imported again, it lands within 0.06 cm of the original clip typically, 0.84 cm at the worst point.

## How it works

Each control-rig bone drives the matching `skel` bone through Copy Transforms. The game only ever sees `skel`, and CRCK Motion adds no bones to it.

When you drag, a small solver (damped least squares) finds how far the hips should move and turn, how much the spine should bend and how far the collarbones should shift, so that every pinned hand and foot stays within reach of its limb. A pin always wins over a hips drag. The IK constraints already on the rig then place the arms and legs. A solve takes about 5 to 10 ms.

## Known gaps

- Binary `.ycd` import is Windows only (it uses CodeWalker and the .NET Framework compiler that comes with Windows). On macOS and Linux, export the clip to `.ycd.xml` with CodeWalker first.
- Not built yet: lock in place, motion trail and smoothing brush. Reach only moves keys that already exist.
- Only the Freemode male rig is bundled.

## Build and test

```bash
blender --command extension build --source-dir crck_motion --output-dir .
```

```bash
blender -b crck_motion/assets/freemode_m.blend --factory-startup --python test_crck_motion.py
```

The test checks that a hand reaches a far target while the feet stay planted, that a crouch keeps the feet down, that the hips stop once the legs are straight, that the head aims at the look-at, that Reach falloff works on neighbouring keys, that the bake matches the control rig, that presets round-trip, land where the character stands, mirror and blend, that turning the whole character turns the feet about the right axis, and that Add Freemode Rig imports the body facing the way its skeleton does.

## Built with AI

Artificial intelligence was used to help develop this tool.

## Licence

GPL-3.0-or-later. See [LICENSE](LICENSE). GTA V and its assets belong to Rockstar Games. This is an unofficial fan tool, not affiliated with or endorsed by Rockstar Games or Take-Two Interactive. Use it for single-player and FiveM work, not GTA Online.
