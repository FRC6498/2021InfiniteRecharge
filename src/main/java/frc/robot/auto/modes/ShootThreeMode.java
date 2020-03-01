package frc.robot.auto.modes;

import frc.robot.RobotState;
import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.*;
import frc.robot.subsystems.ShooterAimingParameters;

import frc.lib.util.Path;
import frc.lib.util.Translation2d;
import frc.lib.util.Path.Waypoint;

import java.util.ArrayList;
import java.util.List;



/**
 * In this autonomous routine, the robot crosses the cheval de frise. There are
 * switchable modes in this routine- the driver can specify whether the robot
 * should return, if it should return on the right side low bar, and if it
 * should launch a single ball.
 */
public class ShootThreeMode extends AutoModeBase {
   

    ShooterAimingParameters mHint;
   
    public ShootThreeMode(ShooterAimingParameters hint) {
        mHint = hint;
        RobotState.getInstance().setIntakeBalls(3);
    }

    @Override
    protected void routine() throws AutoModeEndedException {

       
        runAction(new StartAutoAimingAction());
        runAction(new PointTurretAction(mHint));
        runAction(new ShootWhenReadyAction());
        //runAction(new ShootWhenReadyAction());
        //runAction(new ShootWhenReadyAction());
       
    }
}
