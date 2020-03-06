package frc.robot.auto.actions.ClimbingSpecificActions;

import frc.robot.ControlBoard;
import frc.robot.auto.actions.Action;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Lift;
import frc.lib.util.Rotation2d;

/**
 * DriveStraightAction drives the robot straight at a settable angle, distance,
 * and velocity. This action begins by setting the drive controller, and then
 * waits until the distance is reached.
 *
 * @see Action
 * @see Drive
 * @see Rotation2d
 */
public class DriveOnBarAction implements Action {

    private double startingDistance;
    private double mWantedDistance;
    private double mVelocity;
    private double mHeading;
    private Drive mDrive = Drive.getInstance();
    private Lift mLift = Lift.getInstance();
    private ControlBoard mControls = ControlBoard.getInstance();
    private double mScanSpeed = 0;

    public DriveOnBarAction(double distance, double velocity, double scanSpeed) {
        this(distance, velocity, 0, scanSpeed);
    }

    public DriveOnBarAction(double distance, double velocity, double heading, double scanSpeed) {
        mWantedDistance = distance;
        mVelocity = velocity;
        mHeading = heading;
        mScanSpeed = scanSpeed;
    }

    @Override
    public void start() {
        startingDistance = getCurrentDistance();
        mDrive.setHighGear(false);
        mDrive.setVelocityHeadingSetpoint(mVelocity, Rotation2d.fromDegrees(mHeading));
    }

    boolean driving = false;

    @Override
    public void update() {

     //   if(!mLift.seesBar()&&driving){
     //       mDrive.setVelocitySetpoint(0, 0);
     //       driving = false;
     //   }

        if(mLift.seesBar()&&!driving){
            mDrive.setVelocityHeadingSetpoint(mVelocity, Rotation2d.fromDegrees(mHeading));
            driving = true;
            mLift.setOpenLoop(0); //bar seen, stop scanning
        }

        //if(!driving){ //if its not driving/bar not seen - enable scanning

           if(mControls.scanUp()){
            mDrive.setVelocitySetpoint(0, 0);
                  driving = false;
                mLift.setOpenLoop(mScanSpeed);
           }else if(mControls.scanDown()){

                mDrive.setVelocitySetpoint(0, 0);
                   driving = false;
                mLift.setOpenLoop(-mScanSpeed);
           }

       // }
        



    }

    @Override
    public boolean isFinished() {
        
        return driveFinished();
    }


    boolean driveFinished(){
        boolean rv = false;
        if (mWantedDistance > 0) {
            rv = getCurrentDistance() - startingDistance >= mWantedDistance;
        } else {
            rv = getCurrentDistance() - startingDistance <= mWantedDistance;
        }
        if (rv) {
            mDrive.setVelocitySetpoint(0, 0);
        }
        return rv;
    }

    @Override
    public void done() {
        mDrive.setVelocitySetpoint(0, 0);
        mLift.setOpenLoop(0);
    }

    private double getCurrentDistance() {
        return (mDrive.getLeftDistanceInches() + mDrive.getRightDistanceInches()) / 2;
    }
}
