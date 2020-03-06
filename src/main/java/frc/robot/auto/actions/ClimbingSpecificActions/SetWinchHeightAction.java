package frc.robot.auto.actions.ClimbingSpecificActions;

import frc.robot.auto.actions.Action;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Winch;


/**
 * AlignWithBarAction drives robot backwards using the sensors on the bottom of the chassis
 * to align with the bar on the ground.  This allows us to begin climbing parallel to the bar.
 *
 * @see Action
 * @see Drive
 */
public class SetWinchHeightAction implements Action {

  // private double sensorTimeThreshold = .3;

  Winch mWinch = Winch.getInstance();

  private double height = 0;

  public SetWinchHeightAction(double heightFromGround) {
      height=heightFromGround;
  
    }

    @Override
    public void start() {
       
       mWinch.setDesiredHeight(height);
    }

    @Override
    public void update() {
    }

    @Override
    public boolean isFinished() {
        return mWinch.isOnTarget();
    }

    @Override
    public void done() {
       
    }

   
}
