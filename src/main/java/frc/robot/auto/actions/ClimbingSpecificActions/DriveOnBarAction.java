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

   
    private double mVelocity;
    private double mHeading;
    private Drive mDrive = Drive.getInstance();
    private Lift mLift = Lift.getInstance();
    private ControlBoard mControls = ControlBoard.getInstance();
   


    public DriveOnBarAction(double velocity, double heading) {
      
        mVelocity = velocity;
        mHeading = heading;
        
    }

    @Override
    public void start() {
       
        mDrive.setHighGear(false);
        startDriving();
        driving = true;
    }

    boolean driving = false;

    @Override
    public void update() {
        

        if (!mLift.seesBar()){//doesnt see bar

            double speed=0;
            if(mControls.scanUp()) speed = .5;
            else if(mControls.scanDown()) speed=-.5;

            mLift.setOpenLoop(speed);

            if(driving){//switched from driving to no driving
                driving = false;
                stopDriving();
            } 


        }else{// sees bar
         
           if(!driving){ //switched from stop to driving

                mLift.setDesiredHeight(mLift.getHeight());
                startDriving();
                driving = true;
           }

        }

    }

    @Override
    public boolean isFinished() {
        
        return mLift.hookedOn();
    }


  

    @Override
    public void done() {
        mDrive.setVelocitySetpoint(0, 0);
        mLift.setOpenLoop(0);
    }

    void stopDriving(){
        mDrive.setVelocityHeadingSetpoint(0, Rotation2d.fromDegrees(mHeading));
    }

    void startDriving(){
        mDrive.setVelocityHeadingSetpoint(mVelocity, Rotation2d.fromDegrees(mHeading));
    }


   
}
