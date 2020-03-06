package frc.robot.auto.actions.ClimbingSpecificActions;

import frc.robot.auto.actions.Action;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Lift;



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

   
        return mLift.seesBar(); //required time taken care in lift
    }

    @Override
    public void done() {
       mLift.setOpenLoop(0);
    }

   
}
