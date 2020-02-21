package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.loops.Loop;
import frc.robot.loops.Looper;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**

 */

 
public class FeederFlywheel extends Subsystem {
    CANSparkMax neo_;
    CANEncoder encoder_;
    CANPIDController pid_;

    FeederBelt mFeederBelt = FeederBelt.getInstance();

    private static FeederFlywheel instance_ = new FeederFlywheel();

    public static FeederFlywheel getInstance() {
        return instance_;
    }

    private FeederFlywheel() {
        neo_ = new CANSparkMax(Constants.kFeederFlywheelNeoId, MotorType.kBrushless);
        encoder_ = neo_.getEncoder();
       
        pid_ = neo_.getPIDController();
  
        pid_.setP(Constants.kFeederFlywheelKp);
        pid_.setI(Constants.kFeederFlywheelKi);
        pid_.setD(Constants.kFeederFlywheelKd);
        pid_.setFF(Constants.kFeederFlywheelKf);
        pid_.setIZone(Constants.kFeederFlywheelIZone);
        neo_.setClosedLoopRampRate(Constants.kFeederFlywheelRampRate);
        neo_.setOpenLoopRampRate(Constants.kFeederFlywheelRampRate);

        neo_.setIdleMode(IdleMode.kCoast);

    }

    
    
    enum SystemState {IDLE, FEED_CONTINUOS, FEED_ONE, SPINNING_UP};
    
    public enum WantedState {WANT_IDLE, WANT_FEED_CONTINUOUS, WANT_FEED_ONE};

    private WantedState mWantedState = WantedState.WANT_IDLE;

    private SystemState mSystemState = SystemState.IDLE;
    private boolean mStateChanged;
    
    Loop mLoop = new Loop() {
        private double mCurrentStateStartTime;

        @Override
        public void onStart() {
            synchronized (FeederFlywheel.this) {
                mWantedState = WantedState.WANT_IDLE;
               
                mCurrentStateStartTime = Timer.getFPGATimestamp();
                mSystemState = SystemState.IDLE;
                mStateChanged = true;
            }
        }
    
        @Override
        public void onLoop() {
            synchronized (FeederFlywheel.this) {


                 double now = Timer.getFPGATimestamp();
                SystemState newState;
                switch (mSystemState) {
               case IDLE:
                    newState = handleIdle();
                    break;
                case FEED_CONTINUOS:
                    newState = handleFeedContinuous();
                    break;
                case FEED_ONE:
                    newState = handleFeedOne(now);
                    break;
                case SPINNING_UP:
                    newState = handleSpinningUp();
                    break;
                default:
                    System.out.println("Unexpected feeder flywheel state: " + mSystemState);
                    newState = SystemState.IDLE;
                }

                if (newState != mSystemState) {
                    System.out.println("Feeder flywheel state " + mSystemState + " to " + newState);
                    mSystemState = newState;
                    mCurrentStateStartTime = now;
                    mStateChanged = true;
                } else {
                    mStateChanged = false;
                }
                
        }
     }
       

        @Override
        public void onStop() {
            synchronized (FeederFlywheel.this) {
               stop();
            }
        }
    };
    
    
    public synchronized void setWantedState(WantedState newState) {
        mWantedState = newState;
    }
    
    
    
    
    
    private synchronized SystemState handleIdle(){
        if(mStateChanged){
            stop();
        }

        switch(mWantedState){
            case WANT_FEED_CONTINUOUS:
            case WANT_FEED_ONE:
                return SystemState.SPINNING_UP;
            default:
                return SystemState.IDLE;
        }
    }

    private synchronized SystemState handleSpinningUp(){
        if(mStateChanged){
            setRpm(Constants.kFeederFlywheelShootRPM);
        }

        switch(mWantedState){
            case WANT_FEED_CONTINUOUS:
                if(isOnTarget())return SystemState.FEED_CONTINUOS;
                else return SystemState.SPINNING_UP;
            case WANT_FEED_ONE:
                if(isOnTarget())return SystemState.FEED_ONE;
                else return SystemState.SPINNING_UP;
            case WANT_IDLE:
                return SystemState.IDLE;
            default:
                return SystemState.SPINNING_UP;
        }
    }

    private synchronized SystemState handleFeedContinuous(){
        if(mStateChanged)mNeedsBall=true;

        if(RobotState.getInstance().getTotalBalls()<=0){
            mWantedState= WantedState.WANT_IDLE;
        }

        if(mWantedState!=WantedState.WANT_FEED_CONTINUOUS) mNeedsBall=false;

        switch(mWantedState){
            case WANT_FEED_CONTINUOUS:
            case WANT_FEED_ONE:
                return SystemState.SPINNING_UP;
            default:
                return SystemState.IDLE;
        }
    }

    private boolean mBeltStartedFeeding=false;
    private double mFeederStartTime = 0;
    private synchronized SystemState handleFeedOne(double now){

        if(mStateChanged){
            mNeedsBall=true;
            mFeederStartTime = 0;
            mBeltStartedFeeding=false;
            if(mFeederBelt.hasBall()) mBeltStartedFeeding=true;
            
        }

        if(!mBeltStartedFeeding&&mFeederBelt.hasBall())mBeltStartedFeeding=true;


        if(mBeltStartedFeeding&&mFeederBelt.needsBall()&&mFeederStartTime==0){
            mNeedsBall=false;
        mFeederStartTime = now;
        }

        if(mFeederStartTime!=0&&now-mFeederStartTime>=Constants.kFeederFlywheelActuationTime) {
            System.out.println("Fed One");
            
            if(mWantedState==WantedState.WANT_FEED_ONE) mWantedState=WantedState.WANT_IDLE;
        }

        if(mWantedState!=WantedState.WANT_FEED_ONE) mNeedsBall=false;

        switch(mWantedState){
            case WANT_FEED_CONTINUOUS:
                return SystemState.SPINNING_UP;
            case WANT_FEED_ONE:
                return SystemState.FEED_ONE;
            default:
                return SystemState.IDLE;
        }
    }
    
    
    private boolean mNeedsBall=false;

    public synchronized boolean needsBall(){
        return mNeedsBall;
    }
    
    public synchronized double getRpm() {
        return encoder_.getVelocity();
    }
    double rpmSetpoint=0;
    /**
     * Sets the RPM of the flywheel. The flywheel will then spin up to the set
     * speed within a preset tolerance.
     * 
     * @param Set
     *            flywheel RPM
     */
   public  synchronized void setRpm(double rpm) {
        pid_.setReference(rpm, ControlType.kVelocity);
        rpmSetpoint=rpm;
    }

    synchronized void setOpenLoop(double speed) {
        neo_.set(speed);
    }

    public synchronized double getSetpoint() {
        return rpmSetpoint;
    }

    /**
     * @return If the flywheel RPM is within the tolerance to the specified set
     *         point.
     */
    public synchronized boolean isOnTarget() {
        return (Math.abs(getRpm() - getSetpoint()) < Constants.kFeederFlywheelOnTargetTolerance);
    }

    @Override
    public synchronized void stop() {
        setOpenLoop(0);
    }

    @Override
    public void outputToSmartDashboard() {
        SmartDashboard.putNumber("Feeder_flywheel_rpm", getRpm());
        SmartDashboard.putNumber("Feeder_flywheel_setpoint", getSetpoint());
        SmartDashboard.putBoolean("Feeder_flywheel_on_target", isOnTarget());
       // SmartDashboard.putNumber("Feeder_flywheel_master_current", neo_.getOutputCurrent());
       
    }

    @Override
    public void zeroSensors() {
        // no-op
    }

    @Override
    public void registerEnabledLoops(Looper in) {
        in.register(mLoop);
    }


    double startTestTime=0;
    @Override
    public boolean test(double now) {
        if(startTestTime==0){
            startTestTime = now;
        }

        double timeElapsed = now-startTestTime;

        if(timeElapsed<3){
           
            setRpm(Constants.kFeederFlywheelShootRPM);
        }else if(!isOnTarget()){

        
        }else{
            stop();
            startTestTime=0;
            return true;
        }


        return false;
    }
}
