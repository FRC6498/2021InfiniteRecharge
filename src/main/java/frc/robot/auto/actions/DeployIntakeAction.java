package frc.robot.auto.actions;

import frc.robot.subsystems.Intake;


/**
 * DeployIntakeAction deploys the intake in autonomous mode. This Action is an
 * example of an action that only executes code once.
 *
 * @see Action
 * @see Superstructure
 */
public class DeployIntakeAction implements Action {

    private boolean mIsDone = false;
    private final Intake mIntake = Intake.getInstance();

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
        mIntake.setWantedState(Intake.WantedState.WANT_INTAKE_GROUND);
        mIsDone = true;
    }
}
