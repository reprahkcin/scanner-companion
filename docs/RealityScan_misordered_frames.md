Short answer: RealityScan treats your XMP pose data as per-image priors, not as a guaranteed “sequence definition.” If the cameras land in the right 3D positions but the images appear out of order, that is expected behavior and not a bug in respecting XMP. If the wrong image is at a given position, that means a mapping or metadata issue between files and XMP, not an ordering option.

Here are the concrete failure modes and how to lock it down.

1. Clarify what “mis-orders” means

There are two distinct cases:

A. Positions correct, list/preview order weird

3D path around the fly is correct.

Thumbnails / 1Ds list / swipe order on device are “scrambled” vs your file numbering.

Cause: RealityScan sorts and groups internally (by source, time, etc). There is no XMP field for “frame index,” so your sequential naming is not preserved as a semantic order. XMP has no way to force UI order.
RealityScan Help

Impact: Harmless. The solver uses poses per image; processing is unaffected.

B. Positions are respected but bound to the wrong images (actual problem)

Example: image that visually shows angle 0° ends up at 30° position in 3D.

This means the wrong XMP got attached to that file.

Everything below assumes you are in case B or unsure.

2. How XMP is matched (easy thing to break)

RealityScan/RealityCapture attach XMP to images by exact basename match:

fly_0001.jpg ↔ fly_0001.xmp

Same folder, same name (case-sensitive on some stacks), one XMP per image.
RealityScan Help

If you:

Renamed images after generating XMPs,

Generated XMPs from a sorted list that doesn’t match disk order,

Have leftover or duplicate XMPs,

Used non-padded names (1,2,10,11) and then scripted angles assuming numeric order,

then the pose/image pairing will be shifted or scrambled.

High-probability fix set:

Delete all existing XMPs.

Regenerate them from your pose data with an explicit 1:1 mapping:

Iterate images in the same order the files exist (or sorted alphabetically),

Write that camera’s pose into an XMP with the same basename.

Ensure no _common.xmp or stray XMPs live in that directory.

If your circle is wrong after that, the bug is in the generator, not RealityScan.

3. Pose locking: how far RealityScan will “respect” XMP

To make RealityScan treat your poses as hard constraints:

In each XMP:

Set xcr:PosePrior="locked" (or “Exact/Locked” depending on schema version).

Set xcr:Coordinates="absolute".
RealityScan Help
+1

In alignment settings:

Enable “Use camera priors” / equivalent so priors are honored.
RealityScan Help

When correctly set:

The camera center and orientation from XMP should not be altered during alignment (locked).

RealityScan still does not care about your intended “sequence”; it only cares about each pose.

There is no hidden flag to force “respect my XMP plus also keep my chosen index/order.” Order is not part of the contract.

4. Less obvious gotchas

Check these in your XMPs; any one can cause apparent mis-order:

Rig fields present by accident

If your XMPs contain xcr:Rig, xcr:RigInstance, xcr:RigPoseIndex (copied from sample exports), RealityScan may treat them as multi-camera rig data.

For a single rotating camera / turntable use-case, strip all rig-related tags.
RealityScan Help

Axis / handedness flip

If your generator uses a different axis convention (e.g. Y-up or left-handed), the cameras will go “the wrong way” around the circle or appear out of phase.

Symptom: content looks correct but sequence appears reversed or rotated by a constant offset.

Fix: apply the correct transform to your positions/rotations (e.g. flip sign on angle or one axis) and regenerate.

Mixed EXIF vs XMP

If EXIF GPS/pose exists and XMP is partial or inconsistent, RS can treat EXIF and XMP as competing priors.

Fix: either:

Strip pose-ish EXIF from these images, or

Ensure XMP is complete and uses PosePrior so it clearly wins.

5. Practical test to pinpoint the issue

Run this minimal check:

Take 4 images only: fly_0001..0004.jpg.

Give them absurdly distinct poses in 4 XMP files (e.g. square at 0°, 90°, 180°, 270°).

Import into RealityScan.

If:

Each image appears at the expected unique corner and content matches pose: mapping is correct; any “mis-order” you see is just UI sorting.

An image’s content is at the wrong corner: the XMP/image pairing is wrong (naming or generation bug).

All poses are rotated/flipped consistently: axis conversion bug.

Describe which of those three behaviors you see and I can give you the exact transform or script fix without guesswork.
