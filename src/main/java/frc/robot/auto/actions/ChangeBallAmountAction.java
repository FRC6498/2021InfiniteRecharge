package frc.robot.auto.actions;

import frc.robot.RobotState;

/**

 */
public class ChangeBallAmountAction implements Action {

    int mBeltBalls, mFeederBalls;
    boolean mIsDone = false;

    public ChangeBallAmountAction(int beltBalls, int feederBalls) {
        mBeltBalls = beltBalls;
        mFeederBalls = feederBalls;
        mIsDone=false;
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
       RobotState.getInstance().setIntakeBalls(mBeltBalls);
       RobotState.getInstance().setFeederBalls(mFeederBalls);
       mIsDone=true;
    }
}
