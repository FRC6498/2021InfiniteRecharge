package frc.robot.auto.actions.ClimbingSpecificActions;

import frc.robot.ControlBoard;
import frc.robot.auto.actions.Action;


/**
 * This action completes when the robot moves past a specified distance
 */
public class WaitForConfirmationAction implements Action {
    private ControlBoard mControls = ControlBoard.getInstance();
    

    public WaitForConfirmationAction() {
      
    }

    @Override
    public boolean isFinished() {
        return mControls.confirmButton();
    }

    @Override
    public void update() {
    }

    @Override
    public void done() {
    }

    @Override
    public void start() {
    }

   
}
