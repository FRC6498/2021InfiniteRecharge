package frc.robot.auto.modes;

import java.util.Arrays;

import frc.lib.util.Rotation2d;
import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.DriveStraightAction;
import frc.robot.auto.actions.ParallelAction;
import frc.robot.auto.actions.PointTurretAction;
import frc.robot.auto.actions.SeriesAction;
import frc.robot.auto.actions.ClimbingSpecificActions.AlignWithBarAction;
import frc.robot.auto.actions.ClimbingSpecificActions.DriveOnBarAction;
import frc.robot.auto.actions.ClimbingSpecificActions.MoveLiftForTimeAction;
import frc.robot.auto.actions.ClimbingSpecificActions.ScanForBarAction;

import frc.robot.auto.actions.ClimbingSpecificActions.SetWinchHeightAction;
import frc.robot.auto.actions.ClimbingSpecificActions.WaitForConfirmationAction;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.ShooterAimingParameters;
import frc.robot.subsystems.Turret;

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
      Turret.getInstance().setDesiredAngle(Rotation2d.fromDegrees(45));

      runAction(new SeriesAction(Arrays.asList(
        new ParallelAction(Arrays.asList( new AlignWithBarAction(-.15), 
                                          new MoveLiftForTimeAction(.35,1))),
        new ParallelAction(Arrays.asList( new DriveStraightAction(-8, -4.5, Drive.getInstance().getGyroAngle().getDegrees()),
                                          new  ScanForBarAction(.2))),                               
        new DriveOnBarAction(-4.5, Drive.getInstance().getGyroAngle().getDegrees()), // 12.5 turret is the front so to climb drive backwards
        new MoveLiftForTimeAction(1, -1)
     
      )));

     
      


    }
}
