package frc.robot.auto.actions;

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
       leftSeen=false;
       rightSeen=false;
        mDrive.setHighGear(false);
        mDrive.setOpenLoop(new DriveSignal(-speed, -speed));
    }

    @Override
    public void update() {
    }


    boolean leftSeen=false, rightSeen=false;

    @Override
    public boolean isFinished() {

        double leftSpeed= -speed;
        double rightSpeed = -speed;

        if(mDrive.getLeftBarSensor()) leftSeen=true;

        if(mDrive.getRightBarSensor()) rightSeen=true;


        if(leftSeen)leftSpeed=-leftSpeed;

        if(rightSeen)rightSpeed=-rightSpeed;

        if(leftSeen&&rightSeen) return true;
        
        mDrive.setOpenLoop(new DriveSignal(rightSpeed, leftSpeed));

        return false;
    }

    @Override
    public void done() {
        mDrive.setOpenLoop(DriveSignal.BREAK);
    }

   
}
