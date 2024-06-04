

link1

|

joint1

|

link0 (base link)

|

joint0 (world joint)

|

world

## links & joints



## joint 

type: http://sdformat.org/spec?ver=1.11&elem=joint
* fixed
* continuous
* revolute: limit
* revolute2
* prismatic: limit
* ball
* universal: limit
* screw


parent:
	name or "world"

child:
	name

axis:
	[x y z],
	axis of rotation for revolute joints, the axis of translation for prismatic joints.

limit:
	uppper, lower

## geometry
type: 
* box: x y z
* sphere: r
* cylinder: r1 r2 h
* capsule: r h
* mesh: url translation


