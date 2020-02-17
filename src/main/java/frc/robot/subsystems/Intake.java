package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;


import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.loops.Loop;
import frc.robot.loops.Looper;

/**
 
 */

 
public class Intake extends Subsystem {
   
 

    TalonSRX mTalon;
    Solenoid mSolenoid;
    
    
    
    private RobotState mRobotState = RobotState.getInstance();
    
    private static Intake instance_ = new Intake();

    

    

    public static Intake getInstance() {
        return instance_;
    }

    private Intake() {
        mTalon = new TalonSRX(Constants.kIntakeTalonId);

        mTalon.setNeutralMode(NeutralMode.Coast);

        mTalon.setInverted(true);

        mSolenoid = new Solenoid(Constants.intakeSolenoidId);
        mSolenoid.set(false);

    }

    
    
    enum SystemState {IDLE, INTAKE_GROUND, INTAKE_LOAD};
    
   public enum WantedState {WANT_IDLE, WANT_INTAKE_GROUND, WANT_INTAKE_LOAD};

    private WantedState mWantedState = WantedState.WANT_IDLE;

    private SystemState mSystemState = SystemState.IDLE;
    private boolean mStateChanged;
    
    Loop mLoop = new Loop() {
        private double mCurrentStateStartTime;

        @Override
        public void onStart() {
            synchronized (Intake.this) {
                mWantedState = WantedState.WANT_IDLE;
               
                mCurrentStateStartTime = Timer.getFPGATimestamp();
                mSystemState = SystemState.IDLE;
                mStateChanged = true;
            }
        }
    
        @Override
        public void onLoop() {
            synchronized (Intake.this) {


                 double now = Timer.getFPGATimestamp();
                SystemState newState;
                switch (mSystemState) {
               case IDLE:
                    newState = handleIdle();
                    break;
                case INTAKE_GROUND:
                    newState = handleIntakeGround(now, mCurrentStateStartTime);
                    break;
                case INTAKE_LOAD:
                    newState = handleIntakeLoad(now);
                default:
                    System.out.println("Unexpected Intake state: " + mSystemState);
                    newState = SystemState.IDLE;
                }

                if (newState != mSystemState) {
                    System.out.println("Intake state " + mSystemState + " to " + newState);
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
            synchronized (Intake.this) {
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
            case WANT_INTAKE_GROUND:
                return SystemState.INTAKE_GROUND;
            case WANT_INTAKE_LOAD:
                return SystemState.INTAKE_LOAD;
            default:
                return SystemState.IDLE;
        }
    }

    private double ballSeenStartTime=0;
    private double actuationStartTime=0;
    private synchronized SystemState handleIntakeGround(double now, double startTime){

        if(mStateChanged){
            mSolenoid.set(true);
           setOpenLoop(Constants.kIntakeGroundSpeed);
        }

        if(now-startTime>=Constants.kIntakeActuationTime){

            if(ballSeenStartTime==0&&getCurrent()>=Constants.kIntakeGroundCurrentThreshold ){
                ballSeenStartTime=now;
            //  System.out.println("Intake ball detected");
                mRobotState.setIntakeBalls(1);

            }else if(now-ballSeenStartTime>=Constants.kIntakeGroundTimeThreshold){
                ballSeenStartTime = 0;

            }
       
        }
            
        if(actuationStartTime!=0&&now-actuationStartTime>=Constants.kIntakeActuationTime){
            if(mWantedState==WantedState.WANT_INTAKE_GROUND) mWantedState=WantedState.WANT_IDLE;
            System.out.println("Intake reached max balls");
        }else if(actuationStartTime!=0){
        }else if(mRobotState.getTotalBalls()>=5){
            actuationStartTime=now;
        }

        switch(mWantedState){
            case WANT_INTAKE_GROUND:
                return SystemState.INTAKE_GROUND;
            case WANT_INTAKE_LOAD:
                return SystemState.INTAKE_LOAD;
            default:
                return SystemState.IDLE;
        }
    }

    private synchronized SystemState handleIntakeLoad(double now){

        switch(mWantedState){
            case WANT_INTAKE_GROUND:
                return SystemState.INTAKE_GROUND;
            case WANT_INTAKE_LOAD:
                return SystemState.INTAKE_LOAD;
            default:
                return SystemState.IDLE;
        }
    }


   public boolean getIntakeDown(){
       return mSolenoid.get();
   }
  
    synchronized void setOpenLoop(double speed) {
       mTalon.set(ControlMode.PercentOutput, speed);
    }

   synchronized double getCurrent(){
       return mTalon.getStatorCurrent();
   }
    

    @Override
    public synchronized void stop() {
        setOpenLoop(0);
        mSolenoid.set(false);
    }

    @Override
    public void outputToSmartDashboard() {
      
       SmartDashboard.putNumber("Intake Current", getCurrent());
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
            mSolenoid.set(true);
        }else if(timeElapsed<6){
            setOpenLoop(Constants.kIntakeGroundSpeed);
        }else if(timeElapsed<8){
           stop();
        }else{
            startTestTime=0;
            return true;
        }


        return false;
    }
}
