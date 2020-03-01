package frc.robot.auto.actions;

import frc.robot.StateMachines.Shooter;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.ShooterAimingParameters;
import frc.lib.util.DriveSignal;
import frc.lib.util.Rotation2d;

import java.util.List;

import edu.wpi.first.wpilibj.Timer;

/**
 * DriveUntilInRangeAction is an autonomous mode action that drives until the
 * shooter sees that a goal is in range. This is accomplished by looking for a
 * target with a supplied minimum distance and given a range of distances that
 * the robot will drive
 *
 * @see Shooter
 * @see Action
 * @see Drive
 */
public class DriveUntilInRangeAction implements Action {

    private double mMinDistanceAway, mMaxDistanceToDrive, mVelocity;
    private double startingDistance;
    private Drive mDrive = Drive.getInstance();
    private Shooter mShooter = Shooter.getInstance();

    public DriveUntilInRangeAction(double velocity, double minDistanceAway, double maxDistanceToDrive) {
        mMaxDistanceToDrive = maxDistanceToDrive;
        mMinDistanceAway = minDistanceAway;
        mVelocity = velocity;
    }

    @Override
    public boolean isFinished() {
        if (getCurrentDistance() - startingDistance >= mMaxDistanceToDrive) {
            return true;
        }

        //List<ShooterAimingParameters> params = mShooter.getCachedAimingParams();
        //System.out.println("#l: " + params.size());
        if (!mShooter.hasTarget()) {
            return false;
        }

       // System.out.println("R: " + params.get(0).getRange());
       //if (params.get(0).getRange() < mMinDistanceAway) {
        if (mShooter.getCurrentAimingParameters(Timer.getFPGATimestamp()).getRange() < mMinDistanceAway) {
            return true;
        }

        return false;
    }

    @Override
    public void update() {

    }

    @Override
    public void done() {
        System.out.println("Drive done, Setting drive to neutral");
        mDrive.setOpenLoop(DriveSignal.NEUTRAL);
    }

    @Override
    public void start() {
        startingDistance = getCurrentDistance();
        mDrive.setHighGear(false);
        mDrive.setVelocityHeadingSetpoint(mVelocity, Rotation2d.fromDegrees(0));
    }

    private double getCurrentDistance() {
        return (mDrive.getLeftDistanceInches() + mDrive.getRightDistanceInches()) / 2;
    }
}
