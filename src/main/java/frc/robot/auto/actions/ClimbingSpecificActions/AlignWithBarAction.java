package frc.robot.auto.actions.ClimbingSpecificActions;

import frc.robot.auto.actions.Action;
import frc.robot.subsystems.Drive;
import frc.lib.util.DriveSignal;

/**
 * AlignWithBarAction drives robot backwards using the sensors on the bottom of the chassis
 * to align with the bar on the ground.  This allows us to begin climbing parallel to the bar.
 *
 * @see Action
 * @see Drive
 */
public class AlignWithBarAction implements Action {

    //private double sensorTimeThreshold = .3;
   
    private Drive mDrive = Drive.getInstance();

    private double speed=0;


    public AlignWithBarAction(double percentOut) {
      speed=percentOut;
    }

    @Override
    public void start() {
        mDrive.setHighGear(false);
        mDrive.setOpenLoop(new DriveSignal(-speed, -speed));
    }

    @Override
    public void update() {
    }




    @Override
    public boolean isFinished() {

        double leftSpeed= -speed;
        double rightSpeed = -speed;

        boolean leftSeen = mDrive.getLeftBarSensor();
        boolean rightSeen = mDrive.getRightBarSensor();

   

        if(leftSeen)leftSpeed=-.15;

        if(rightSeen)rightSpeed=-.15;

        if(leftSeen&&rightSeen){
            mDrive.setOpenLoop(new DriveSignal(0, 0));
             return true;
        }else{
        
        mDrive.setOpenLoop(new DriveSignal(rightSpeed, leftSpeed));

        return false;
        }
    }

    @Override
    public void done() {
        mDrive.setOpenLoop(DriveSignal.BREAK);
    }

   
}
