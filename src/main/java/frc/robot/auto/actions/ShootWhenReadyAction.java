package frc.robot.auto.actions;

import frc.robot.RobotState;
import frc.robot.StateMachines.Shooter;


/**
 * Shoots the ball when ready by setting the robot state machine's desired
 * state.
 * 
 * @return If the firing was successful
 */
public class ShootWhenReadyAction implements Action {

    private final Shooter mShooter = Shooter.getInstance();
    private final RobotState mRobotState = RobotState.getInstance();

    private int mNumBallsAtStart;

    @Override
    public boolean isFinished() {
        return mRobotState.getTotalBalls() < mNumBallsAtStart;
    }

    @Override
    public void start() {
        mNumBallsAtStart = mRobotState.getTotalBalls();
        //mShooter.setWantsToFireIfReady(Shooter.WantedFiringAmount.WANT_FIRE_ONE);
    }

    @Override
    public void update() {
        mShooter.setWantsToFireIfReady(Shooter.WantedFiringAmount.WANT_FIRE_ONE);
    }

    @Override
    public void done() {
       // mShooter.setWantedState(Superstructure.WantedState.WANT_TO_DEPLOY);
    }
}
