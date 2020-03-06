package frc.robot.auto.modes;

import frc.robot.RobotState;
import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.*;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.ShooterAimingParameters;

import frc.lib.util.Path;
import frc.lib.util.Rotation2d;
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
public class GetTwoShootFiveMode extends AutoModeBase {

    ShooterAimingParameters mHint = new ShooterAimingParameters(12 * 10, Rotation2d.fromDegrees(0));

    double mHorizDistance = 0;

    public GetTwoShootFiveMode(double horizontalDistance) {
       mHorizDistance = horizontalDistance;
       //RobotState.getInstance().setIntakeBalls(3);
   }

   @Override
   protected void routine() throws AutoModeEndedException {


       List<Waypoint> first_path = new ArrayList<>();
       first_path.add(new Waypoint(new Translation2d(0, 0), 60.0));
       first_path.add(new Waypoint(new Translation2d(-2*12,-mHorizDistance), 60.0));
       first_path.add(new Waypoint(new Translation2d(-9*12, -mHorizDistance), 36.0));

       List<Waypoint> return_path = new ArrayList<>();
       return_path.add(new Waypoint(new Translation2d(-9*12, -mHorizDistance), 80.0));
       return_path.add(new Waypoint(new Translation2d(-5*12, -mHorizDistance+15), 80.0));
     

       Drive.getInstance().setHighGear(false);

      runAction(new SeriesAction(Arrays.asList(
                new ChangeBallAmountAction(1,2),
                new ParallelAction(Arrays.asList(
                    new StartAutoAimingAction(), //done when belt has ball
                    new PointTurretAction(mHint),
                    new FollowPathAction(new Path(first_path), true),
                    new DeployIntakeAction()
                    )), 
                new WaitAction(.5),
                new ParallelAction(Arrays.asList(
                    new RaiseIntakeAction(),
                    new ChangeBallAmountAction(2,0),
                    new FollowPathAction(new Path(return_path), false))),
                    new WaitAction(.5),
                new ShootWhenReadyAction(),
                new ShootWhenReadyAction(),
                new ShootWhenReadyAction(),
                new ShootWhenReadyAction(),
                new ShootWhenReadyAction(),
                new WaitAction(1),
                new StopShootingAction()
                
                )));
   }
}
