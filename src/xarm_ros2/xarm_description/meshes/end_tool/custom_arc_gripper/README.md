# xArm arc gripper jaws

The two STL meshes in this directory were exported from:

`/home/qluo/下载/零件2_弧线激进_内径65_M3.step`

The STEP assembly contains two separate solids.  They are used directly, not
mirrored, so the M3 mounting features retain their manufactured handedness.
The STEP and STL coordinates are millimetres; the URDF mesh scale is `0.001`.

`arc_gripper_left.stl` replaces the factory plate on `left_finger` and
`arc_gripper_right.stl` replaces the factory plate on `right_finger`.  The
original finger-plate meshes remain visible as grey RViz-only screw-hole
references, but have no collision geometry when `add_arc_gripper:=true`; the
white arc jaws are the only finger collision geometry. The default transforms
align the CAD M3 mounting faces symmetrically with the factory finger frames.
The source STEP places the two solids at different CAD Y origins, so the right
mesh requires `right_xyz.Y = 0.096583 m`; this is not a field-fit offset.
Both jaws' mounting faces then lie at +/-29.960 mm in their respective finger
frames.
Their original M3-hole centres are at the factory finger-plate positions
`x=0`, `y=0`, `z=25/37 mm` in the corresponding finger link frame.

No extension-plate collision envelope is included: the factory metal finger
plates have been removed and the printed jaws are fitted directly in their
mounting positions.

For a field-fit correction, the MoveIt real-robot launch accepts these optional
arguments in metres/radians:

`arc_gripper_left_xyz`, `arc_gripper_left_rpy`,
`arc_gripper_right_xyz`, and `arc_gripper_right_rpy`.
