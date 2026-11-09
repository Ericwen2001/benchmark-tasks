# Reattach your detached wrist camera

## instruction
Reattach the loose camera to its mount on your left wrist. Release it once it is secured.

## context
Recovery of the robot’s own sensory configuration, rather than installation of an unrelated camera.

## setup
Fixed bimanual YAM station with wooden tabletop at z=0.7475 m; both metal arms begin in native READY. A rigid polymer camera housing (40x36x28 mm, 55 g), with front-facing lens panel, rests at (0.36,-0.19,0.7615)m, initial optical direction horizontal across the table. Two rigid polymer mounting rails on the left wrist are each 6x26x6 mm. The immutable original camera-to-gripper optical transform is position (0.0004,-0.069638,0.072762)m and quaternion XYZW (0.9756188502,-0.0028169996,0.0032269995,0.2194299663), OpenGL. A 66 mm,120 g apple rests at XY(0.30,0.28)m and a 64 mm,110 g orange at XY(0.40,0.26)m; natural skin appearance, rigid mesh simulation. Fruits are scene context, not manipulation goals. The detached camera feed remains live and follows its physical housing; top and right wrist feeds remain normal. Full mount geometry is specified in layout_prototype.json. The rails are joined to the wrist housing by two metal struts (9x9 mm cross-section; endpoints specified in wrist_adapter.py) and a 45x30x6 mm metal seating plate. These do not change the original optical target.

## scoring
Binary 1 after optical pose is within 5 mm and 5 deg of the original native mount, right TCP is over 8 cm away, live sensor matches physical housing-derived optical pose within 0.1 mm/0.001 rad and is nonblank, held for 2 seconds. Otherwise 0.

## fidelity
Live detached sensor follows the physical camera housing. Native YAM rigid-body contact, simplified keyed clip with pose-gated fixed joint at current pose; no teleport. Original optical calibration retained. Cables, elastic clip and electrical reconnection omitted. Apple/orange context meshes are rigid.

## scores
Dream-1 beta: 0.0; scripted demo: 1.0. Model trial: 1800 actions, one continuous session.