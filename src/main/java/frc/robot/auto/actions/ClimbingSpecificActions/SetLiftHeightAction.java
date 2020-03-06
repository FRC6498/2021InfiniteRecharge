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
public class SetLiftHeightAction implements Action {

  // private double sensorTimeThreshold = .3;

  Lift mLift = Lift.getInstance();

  private double height = 0;

  public SetLiftHeightAction(double heightFromGround) {
      height=heightFromGround;
  
    }

    @Override
    public void start() {
       
       mLift.setDesiredHeight(height);
    }

    @Override
    public void update() {
    }

    @Override
    public boolean isFinished() {
        
      System.out.println("lift on target");
      return mLift.isOnTarget();
        
    }

    @Override
    public void done() {
       
    }

   
}
