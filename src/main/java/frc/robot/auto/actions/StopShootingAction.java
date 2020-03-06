package frc.robot.auto.actions;

import frc.robot.StateMachines.Shooter;
import frc.robot.StateMachines.Shooter.WantedState;
import frc.robot.subsystems.ShooterAimingParameters;


/**
 * Action for aiming the turret at a specified target
 */
public class StopShootingAction implements Action {
  
    private boolean mIsDone;
    private final Shooter mShooter = Shooter.getInstance();

    public StopShootingAction() {
       
        mIsDone = false;
    }

    @Override
    public boolean isFinished() {
        return mIsDone;
    }

    @Override
    public void update() {
    }

    @Override
    public void done() {
    
    }

    @Override
    public void start() {
        mShooter.setWantedState(WantedState.WANT_IDLE);
        mIsDone = true;
    }
}
