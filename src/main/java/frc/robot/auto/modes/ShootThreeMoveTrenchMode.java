package frc.robot.auto.modes;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import frc.lib.util.Path;
import frc.lib.util.Rotation2d;
import frc.lib.util.Translation2d;
import frc.lib.util.Path.Waypoint;
import frc.robot.RobotState;
import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.ChangeBallAmountAction;
import frc.robot.auto.actions.DeployIntakeAction;
import frc.robot.auto.actions.FollowPathAction;
import frc.robot.auto.actions.ParallelAction;
import frc.robot.auto.actions.PointTurretAction;
import frc.robot.auto.actions.RaiseIntakeAction;
import frc.robot.auto.actions.SeriesAction;
import frc.robot.auto.actions.ShootWhenReadyAction;
import frc.robot.auto.actions.StartAutoAimingAction;
import frc.robot.auto.actions.StopShootingAction;
import frc.robot.auto.actions.WaitAction;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.ShooterAimingParameters;



/**
 * In this autonomous routine, the robot crosses the cheval de frise. There are
 * switchable modes in this routine- the driver can specify whether the robot
 * should return, if it should return on the right side low bar, and if it
 * should launch a single ball.
 */
public class ShootThreeMoveTrenchMode extends AutoModeBase {

    ShooterAimingParameters mHint = new ShooterAimingParameters(12 * 10, Rotation2d.fromDegrees(0));

    double mHorizDistance = 0;

    public ShootThreeMoveTrenchMode(double horizontalDistance) {
        mHorizDistance = horizontalDistance;
        //RobotState.getInstance().setIntakeBalls(3);
    }

    @Override
    protected void routine() throws AutoModeEndedException {


        List<Waypoint> first_path = new ArrayList<>();
        first_path.add(new Waypoint(new Translation2d(0, 0), 60.0));
        first_path.add(new Waypoint(new Translation2d(-2*12,-mHorizDistance), 60.0));
      

      
      

        Drive.getInstance().setHighGear(false);

       runAction(new SeriesAction(Arrays.asList(
                 new ChangeBallAmountAction(1,2),
                 new ParallelAction(Arrays.asList(
                     new StartAutoAimingAction(), //done when belt has ball
                     new PointTurretAction(mHint))), 
                 new ShootWhenReadyAction(),
                 new ShootWhenReadyAction(),
                 new ShootWhenReadyAction(),
                 new WaitAction(1),
                 new StopShootingAction(),
                 new FollowPathAction(new Path(first_path), true)
                 )));
                   
    }
}
