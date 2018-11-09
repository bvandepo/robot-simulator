/*Copyright (c) 2018, Bertrand Vandeportaele
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright
  notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright
  notice, this list of conditions and the following disclaimer in the
  documentation and/or other materials provided with the distribution.
* Neither the name of the University of California, Berkeley nor the
  names of its contributors may be used to endorse or promote products
  derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE REGENTS AND CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

use <RoboArm.scad>


printerpart=0;//default value for the constant printerpart, it can be changed through command line using -D
//https://en.wikibooks.org/wiki/OpenSCAD_User_Manual/Using_OpenSCAD_in_a_command_line_environment
//use compile.sh to generate the stl files in stl/ subdirectory
type=printerpart;  //assign the constant value to a variable to be able to test it, even if the constant hasn't been defined in command line


/////////////////////////////////////////////////////////////////////////
module overview_stl() {
	color( c = [1., 1., 1., 1.] ) import("stl/p1.stl"); // foot(true);
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
//utilise la fonction d'animation d'openscad pour déterminer la position des axes de rotations... Vue->animer, fait apparaitre Temps, FPS et Etapes sous l'appercu
//saisir Temps=0, FPS=20, Etapes=30 et vérifier si la pièce tourne bien autour de l'axe
angle= $t*360;
//décommenter la ligne suivante et assigner une valeur pour visualiser individuellement des animations pour chaque pièce
//laisser commenter pour régler type depuis le script compile.sh afin de pouvoir générer les stl des différentes parties
//type=6;
module align_axe_individual(){
/*	color( c = [1., 1., 1., 1.] ) import("stl/p1.stl"); // foot(true);
//////////////   
   rotate([0,0,angle]) translate([0,0,28]) {
        color( c = [1., 0., 0., 1.] ) import("stl/p2.stl"); //swing(true)
    }
//////////////
    rotate([0,angle,0]) translate([-0,0,33-28-13]) {
   	    color( c = [0., 1., 0., 1.] )  import("stl/p3.stl"); //arm(true, true);
    }
//////////////
    rotate([0,angle,0]) translate([0,0,-8]) { 
        color( c = [0., 0., 1., 1.] ) import("stl/p9.stl"); // arm(true, false);
    }
//////////////    
  */  
 //for generation of individual stl files with axis of rotation at origin
if (type==1)     foot(true);
if (type==2)     swing(true);
if (type==3)     rotate([0,angle,0]) translate([-0,0,33-28-13]) arm(true, true);        
if (type==4)     rotate([0,angle,0]) translate([-0,0,33-28-13]) armFront(true);
if (type==5)     frontPlate();
if (type==6)     rotate([angle,0,0]) rotate([0,-90,0]) gripperGear(4.5,2.5);
if (type==7)     gripper();
if (type==8)     gripperRack();
if (type==9)     rotate([0,angle,0]) translate([0,0,-8]) arm(true, false);

}
/////////////////////////////////////////////////////////////////////////

/*d1=0;
d2=0;
d3=0;
d4=0;
*/
d1=10;
d2=-23;
d3=21;
d4=10;
//angle de rotation de la pince
d5=-100+($t*120); //d5=-100; à d5=20;
//d5=-100+( (0.5+0.5*cos(2*3.14159*$t))*120); 
//translation pour chaque éléments de la pince
l5=d5/12;
module overview_stl_articulate() {
	color( c = [1., 1., 1., 1.] ) import("stl/p1.stl"); // foot(true);
    rotate([0,0,d1]) translate([0,0,28]) {
      color( c = [1., 0., 0., 1.] ) import("stl/p2.stl"); //swing(true);
      translate([0,0,13]) rotate([0,d2,0])  {
        color( c = [0., 1., 0., 1.] )  import("stl/p3.stl"); //arm(true, true);
	    translate([49.5,0,0])  rotate([0,d3,0]) {
            color( c = [0., 0., 1., 1.] ) import("stl/p9.stl"); // arm(true, false);
            translate([49.5,0,0]) rotate([0,d4,0]) {
              color( c = [0., 1., 1., 1.] ) import("stl/p4.stl"); //armFront(true);
              translate([0,0,-8]) {
                color( c = [1., 1., 0., 1.] ) { 
           		  translate([45,18,-2])  rotate([0,90,0]) rotate([180,0,0])  import("stl/p5.stl"); // frontPlate();
         		  translate([0,-l5,0]){
                    translate([81,-15.5,20]) rotate([0,180,0]) rotate([90,0,0]) import("stl/p7.stl"); // gripper();
         		    translate([58,-22.5,18]) rotate([0,90,0]) import("stl/p8.stl"); //  gripperRack();
                  }
                  translate([0,+l5,0]){
         		    translate([81,3.5,-3]) rotate([0,180,0]) rotate([-90,0,0]) import("stl/p7.stl"); // gripper();
         		    translate([58,12.5,-1]) rotate([0,0,180]) rotate([0,-90,0]) import("stl/p8.stl"); //  gripperRack();
                  }
                }
                color( c = [1., 0., 1., 1.] ) {
                  translate([61,-5,8.5])  
                  rotate([d5,0,0]) //rotate([50*$t,0,0])
         		    import("stl/p6.stl"); //  gripperGear(4.5,2.5);
                }
              }
            }
         }
       }             
    } 
}
/////////////////////////////////////////////////////////////////////////
if (type==0)
 {
   // shows the complete thing
   overview_stl_articulate(); 
 }
  align_axe_individual(); 
 