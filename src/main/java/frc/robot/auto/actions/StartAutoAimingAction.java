package frc.robot.auto.actions;

import frc.robot.RobotState;
import frc.robot.StateMachines.Shooter;
import frc.robot.subsystems.FeederBelt;

/**
 * Action to begin aiming if the Shooter has a ball
 */
public class StartAutoAimingAction implements Action {

    private final Shooter mShooter = Shooter.getInstance();
    private boolean mIsDone = false;
    private final FeederBelt mBelt = FeederBelt.getInstance();

    @Override
    public void start() {
    }

    @Override
    public boolean isFinished() {
        return mIsDone;
    }

    @Override
    public void update() {
        if (mBelt.hasBall()) {
            mShooter.setWantedState(Shooter.WantedState.WANT_AUTO);
            mIsDone = true;
        }
    }

    @Override
    public void done() {
    }

}
