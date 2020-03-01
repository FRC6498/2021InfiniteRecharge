package frc.robot.auto.actions;

import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Lift;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.util.DriveSignal;

/**
 * AlignWithBarAction drives robot backwards using the sensors on the bottom of the chassis
 * to align with the bar on the ground.  This allows us to begin climbing parallel to the bar.
 *
 * @see Action
 * @see Drive
 */
public class ScanForBarAction implements Action {

    // private double sensorTimeThreshold = .3;

   Lift mLift = Lift.getInstance();

    private double scanSpeed = 0;
   

    public ScanForBarAction(double percentOut) {
      scanSpeed=percentOut;
  
    }

    @Override
    public void start() {
       
       mLift.setOpenLoop(scanSpeed);
    }

    @Override
    public void update() {


    }
double seenTime = 0;
    @Override
    public boolean isFinished() {

      double now = Timer.getFPGATimestamp();

      if(mLift.seesBar()) {
        if(seenTime==0) seenTime=now;


      }else{
        seenTime=0;
      }

        return seenTime!=0&&(now-seenTime)>.2;
    }

    @Override
    public void done() {
       mLift.setOpenLoop(0);
    }

   
}
