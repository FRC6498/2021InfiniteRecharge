package frc.robot.auto.actions;

import frc.robot.StateMachines.Shooter;
import frc.robot.subsystems.ShooterAimingParameters;


/**
 * Action for aiming the turret at a specified target
 */
public class PointTurretAction implements Action {
    private ShooterAimingParameters mHint;
    private boolean mIsDone;
    private final Shooter mShooter = Shooter.getInstance();

    public PointTurretAction(ShooterAimingParameters hint) {
        mHint = hint;
        mIsDone = false;
    }

    @Override
    public boolean isFinished() {
        return (mIsDone && mShooter.hasTarget());
    }

    @Override
    public void update() {
    }

    @Override
    public void done() {
        mShooter.clearTurretManualPositionSetpoint();
    }

    @Override
    public void start() {
        mShooter.setTurretManualPositionSetpoint(mHint);
        mIsDone = true;
    }
}
