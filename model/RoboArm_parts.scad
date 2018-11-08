use <RoboArm.scad>

printerpart=0;//default value for the constant printerpart, it can be changed through command line using -D
//https://en.wikibooks.org/wiki/OpenSCAD_User_Manual/Using_OpenSCAD_in_a_command_line_environment
//use compile.sh to generate the stl files in stl/ subdirectory
type=printerpart;  //assign the constant value to a variable to be able to test it, even if the constant hasn't been defined in command line

/////////////////////////////////////////////////////////////////////////
module overview_stl() {
	color( c = [1., 1., 1., 1.] ) foot(true);
	translate([0,0,28]) color( c = [1., 0., 0., 1.] )  import("stl/p2.stl"); //swing(true);
	translate([0,0,33]) color( c = [0., 1., 0., 1.] )  import("stl/p3.stl"); //arm(true, true);
	translate([49.5,0,33]) color( c = [0., 0., 1., 1.] ) import("stl/p9.stl"); // arm(true, false);
	translate([99,0,33])  color( c = [0., 1., 1., 1.] ){
		import("stl/p4.stl"); //armFront(true);
		translate([45,18,-2]) rotate([180,90,0]) import("stl/p5.stl"); // frontPlate();
		translate([81,-15.5,20]) rotate([90,180,0]) import("stl/p7.stl"); // gripper();
// gripperRack and gripperGear are commented because
// they slow down rendering considerably on my PC
 		translate([58,-22.5,18]) rotate([0,90,0]) import("stl/p8.stl"); //  gripperRack();
		translate([81,3.5,-3]) rotate([-90,180,0])  import("stl/p7.stl"); // gripper();
 		translate([58,12.5,-1]) rotate([0,-90,180]) import("stl/p8.stl"); //  gripperRack();
 		translate([61,-5,8.5]) rotate([0,-90,0]) import("stl/p6.stl"); //  gripperGear(4.5,2.5);
	}
}
/////////////////////////////////////////////////////////////////////////
if (type==0)
 {
    // shows the complete thing
    overview_stl();
 }
 //for generation of individual stl files
if (type==1)
    foot(true);
if (type==2)
    swing(true);
if (type==3)
    arm(true, true);
if (type==4)
    armFront(true);
if (type==5)
    frontPlate();
if (type==6) 
    gripperGear(4.5,2.5);
if (type==7)
    gripper();
if (type==8)
    gripperRack();
if (type==9)
    arm(true, false);