package frc.robot.loops;

import frc.robot.StateMachines.Shooter;
import frc.robot.subsystems.Turret;

/**
 * Periodically checks if the pivoting turret hits either extreme (remember, the
 * turret cannot spin in a complete circle). There are bumper switches at both
 * extremes of the turret, and this checks if the bumper switches are pressed.
 * If so, the turret re-centers itself.
 */
public class TurretResetter implements Loop {
    Shooter mShooter = Shooter.getInstance();
    Turret mTurret = Shooter.getInstance().getTurret();

    @Override
    public void onStart() {
        // no-op
    }

    @Override
    public void onLoop() {
        if (mTurret.getForwardLimitSwitch()) {
            mShooter.resetTurretAtMax();
        } else if (mTurret.getReverseLimitSwitch()) {
            mShooter.resetTurretAtMin();
        }
    }

    @Override
    public void onStop() {
        // no-op
    }

}
