package frc.robot.auto.modes;

import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.*;
import frc.robot.subsystems.ShooterAimingParameters;

import frc.lib.util.Path;
import frc.lib.util.Translation2d;
import frc.lib.util.Path.Waypoint;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * In this autonomous routine, the robot crosses the cheval de frise. There are
 * switchable modes in this routine- the driver can specify whether the robot
 * should return, if it should return on the right side low bar, and if it
 * should launch a single ball.
 */
public class ClimbMode extends AutoModeBase {
    



    

    public ClimbMode() {
     
    }

    @Override
    protected void routine() throws AutoModeEndedException {

     // runAction(new AlignWithBarAction(-.1)); //make 2 parallel 
     runAction(new SeriesAction(Arrays.asList(new SetLiftHeightAction(48)
     ,new ScanForBarAction(.25),
     
     new DriveStraightAction(-12.5, -9) //turret is the front so to climb drive backwards
     
     
     )));
     
     
     
     //runAction(new SetLiftHeightAction(48)); //~bar height 29.5 
    //  runAction(new ScanForBarAction(.25));
     // runAction(new DriveStraightAction(8, 5));
     // runAction(new SetLiftHeightAction(0));
     // runAction(new SetWinchHeightAction(10));


        
    }
}
