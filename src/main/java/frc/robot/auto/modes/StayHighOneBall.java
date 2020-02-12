package frc.robot.auto.modes;

import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.*;

import frc.lib.util.Path;
import frc.lib.util.Translation2d;
import frc.lib.util.Path.Waypoint;

import java.util.ArrayList;
import java.util.List;

/**
 * Go over the defenses in the starting configuration, then launch one ball (in
 * the robot at start)
 */
public class StayHighOneBall extends AutoModeBase {
   
    private final boolean mShouldDriveBack;
    private final double kDistanceToDrive = 200;

    public static final double DISTANCE_TO_DROP_ARM = 100;

    public StayHighOneBall(boolean shouldDriveBack) {
       
        mShouldDriveBack = shouldDriveBack;
    }

    @Override
    protected void routine() throws AutoModeEndedException {
       

        List<Waypoint> first_path = new ArrayList<>();
        first_path.add(new Waypoint(new Translation2d(0, 0), 48.0));
        first_path.add(new Waypoint(new Translation2d(kDistanceToDrive, 0), 48.0));

        List<Waypoint> return_path = new ArrayList<>();
        return_path.add(new Waypoint(new Translation2d(kDistanceToDrive, 0), 48.0));
        return_path.add(new Waypoint(new Translation2d(18, 0), 48.0));

        runAction(new WaitAction(1.0));
        runAction(new FollowPathAction(new Path(first_path), false));

        //runAction(new StartAutoAimingAction());
        //runAction(new PointTurretAction(mHint));
        //runAction(new ShootWhenReadyAction());

        if (mShouldDriveBack) {
            runAction(new FollowPathAction(new Path(return_path), true));
        }
    }
}
