package frc.robot.auto.modes;

import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.*;

import frc.lib.util.Path;
import frc.lib.util.Translation2d;
import frc.lib.util.Path.Waypoint;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Get's into portcullis/low bar position before driving through the defense.
 */
public class GetLowOneBallMode extends AutoModeBase {

  
    private final boolean mShouldDriveBack;
    private final boolean mComeBackRight;
    private final double kDistanceToDrive = 200;

    private static final double DISTANCE_TO_DROP_INTAKE = 12;
    private static final double DISTANCE_TO_POP_HOOD = 100;

    public GetLowOneBallMode(boolean shouldDriveBack, boolean comeBackRight) {
        
        mShouldDriveBack = shouldDriveBack;
        mComeBackRight = comeBackRight;
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        
        List<Waypoint> first_path = new ArrayList<>();
        first_path.add(new Waypoint(new Translation2d(0, 0), 48.0));
        first_path.add(new Waypoint(new Translation2d(DISTANCE_TO_DROP_INTAKE, 0), 48.0, "DropIntake"));
        first_path.add(new Waypoint(new Translation2d(DISTANCE_TO_POP_HOOD, 0), 48.0, "PopHood"));
        first_path.add(new Waypoint(new Translation2d(kDistanceToDrive, 0), 48.0));

        double y_distance = (mComeBackRight ? -53 : 53);
        List<Waypoint> return_path = new ArrayList<>();
        return_path.add(new Waypoint(new Translation2d(kDistanceToDrive, 0), 120.0));
        return_path.add(new Waypoint(new Translation2d(kDistanceToDrive, y_distance), 60.0));
        return_path.add(new Waypoint(new Translation2d(18, y_distance), 60.0));

        // Get low and wait a minimum of 1.5 seconds
       // runAction(new ParallelAction(Arrays.asList(new GetLowAction(), new WaitAction(1.0))));

        runAction(new FollowPathAction(new Path(first_path), false));//new ParallelAction(Arrays.asList(new FollowPathAction(new Path(first_path), false),
               // new SeriesAction(Arrays.asList(new WaitForPathMarkerAction("DropIntake"), new DeployIntakeAction(),
                //        new WaitForPathMarkerAction("PopHood"), new StartAutoAimingAction(),
               //         new PointTurretAction(mHint))))));

        //runAction(new ShootWhenReadyAction());
        //runAction(new SetArmModeAction(UtilityArm.WantedState.DRIVING));

        if (mShouldDriveBack) {
            runAction(new FollowPathAction(new Path(return_path), true));
        }
    }
}
