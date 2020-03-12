package frc.robot.auto.actions.ClimbingSpecificActions;

import edu.wpi.first.wpilibj.Timer;
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
public class MoveLiftForTimeAction implements Action {

  // private double sensorTimeThreshold = .3;

  Lift mLift = Lift.getInstance();

  private double mTime, mSpeed;

  public MoveLiftForTimeAction(double time, double speed) {
      mTime = time;
      mSpeed = speed;
  
    }

    double startTime = 0;

    @Override
    public void start() {
       
       mLift.setOpenLoop(mSpeed);
       startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void update() {
    }

    @Override
    public boolean isFinished() {
        
     return Timer.getFPGATimestamp()-startTime >= mTime;
        
    }

    @Override
    public void done() {
       mLift.setOpenLoop(0);
    }

   
}
