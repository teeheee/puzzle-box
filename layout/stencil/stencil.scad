
edge = "puzzlebox-Edge.Cuts.dxf";
oben = "puzzlebox-F.Paste.dxf";
unten = "puzzlebox-B.Paste.dxf";

$fn=10;
thickness = 0.12;

module stencil(Creamlayer)
{
     rotate([180,0,0])
    difference() { 
        translate([0,0,thickness])  
        {
            hull() 
            {
                linear_extrude(1.5)
                offset(0.8+0.5)
                import(edge);
            }
        }
        union()
        {
            hull() 
            {
                linear_extrude(1.5)
                offset(0.5)
                import(edge);
            }
            translate([0,0,2*thickness]) 
            minkowski()
            {
              linear_extrude(10)offset(-0.2)import(Creamlayer);
              cylinder(r=0.4,h=5);
            }
        }
    }
}

stencil(oben);
//mirror()
//stencil(unten);