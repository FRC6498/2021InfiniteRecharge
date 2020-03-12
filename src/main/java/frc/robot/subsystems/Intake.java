package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.ControlBoard;
import frc.robot.RobotState;
import frc.robot.loops.Loop;
import frc.robot.loops.Looper;

/**
 
 */

 
public class Intake extends Subsystem {
   
 

    VictorSPX mVictor;
    DoubleSolenoid mSolenoid;
    
    
    
    private RobotState mRobotState = RobotState.getInstance();
    
    private static Intake instance_ = new Intake();

    

    

    public static Intake getInstance() {
        return instance_;
    }

    private Intake() {
        mVictor = new VictorSPX(Constants.kIntakeVictorId);

        mVictor.setNeutralMode(NeutralMode.Coast);

        mVictor.setInverted(true);

        mSolenoid = new DoubleSolenoid(Constants.intakeSolenoidOneId, Constants.intakeSolenoidTwoId);
        mSolenoid.set(Value.kReverse);

    }

    
    
    enum SystemState {IDLE, INTAKE_GROUND, INTAKE_LOAD, PLOW};
    
   public enum WantedState {WANT_IDLE, WANT_INTAKE_GROUND, WANT_INTAKE_LOAD, WANT_PLOW};

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
                    newState = handleIdle(now, mCurrentStateStartTime);
                    break;
                case INTAKE_GROUND:
                    newState = handleIntakeGround(now, mCurrentStateStartTime);
                    break;
                case INTAKE_LOAD:
                    newState = handleIntakeLoad(now);
                    break;
                case PLOW:
                    newState = handlePlow(now, mCurrentStateStartTime);
                    break;
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
    
    
    
   
    double idleWaitTime=.5;//2.5;
    boolean idleInitialized=false;
    double idleInitializedTime = 0;
    double idleStartTime =0;

    double pistonOnTime = .15;
    boolean pistonDone = false;

    boolean restartCycle = false;

    public synchronized void cycle(){
        restartCycle=true;
    }



    private synchronized SystemState handleIdle(double now, double startTime){
        if(mStateChanged||restartCycle){
            stop();
            idleInitialized=false;
            idleInitializedTime=0;
            pistonDone=false;
            idleStartTime = startTime;

            if(restartCycle) idleStartTime = now;

            restartCycle=false;
        }

        if(!idleInitialized&&now-idleStartTime>=idleWaitTime){
            idleInitialized=true;
            idleInitializedTime=now;
            mSolenoid.set(Value.kForward);
            System.out.println("Intake solenoid forward");
        }

        if(!pistonDone&&idleInitialized&&now-idleInitializedTime>=pistonOnTime){
            mSolenoid.set(Value.kOff);
            System.out.println("Intake solenoid off");
            pistonDone=true;
        }

        




        switch(mWantedState){
            case WANT_INTAKE_GROUND:
                return SystemState.INTAKE_GROUND;
            case WANT_INTAKE_LOAD:
                return SystemState.INTAKE_LOAD;
            case WANT_PLOW:
                return SystemState.PLOW;
            default:
                return SystemState.IDLE;
        }
    }

   
    private double actuationStartTime=0;
    private boolean actuatedDown = false;
  
    private synchronized SystemState handleIntakeGround(double now, double startTime){

        if(mStateChanged){
            actuatedDown = getIntakeDown();
            mSolenoid.set(Value.kForward);
            
            actuationStartTime=0;
        }
  

        if(!actuatedDown&&now-startTime>=Constants.kIntakeActuationTime){
            actuatedDown = true;
            setOpenLoop(Constants.kIntakeGroundSpeed);
        }
            
        WantedState newState = mWantedState;

        if(newState!=WantedState.WANT_INTAKE_GROUND){ //exiting intake ground
            setOpenLoop(0);

            if(actuatedDown){//if it wants to exit, but actuated down
                 newState = WantedState.WANT_INTAKE_GROUND;

                 if(actuatedDown&&actuationStartTime==0){//wants exit, actuated down, timer hasn't started
                    actuationStartTime = now;
                 }

                 //wants to exit, actuated down, and timer ready
                 if(actuationStartTime!=0&&now-actuationStartTime>=Constants.kIntakeActuationTime){
                    newState= mWantedState;
                }

            }
        }

       


        switch(newState){
            case WANT_INTAKE_GROUND:
                return SystemState.INTAKE_GROUND;
            case WANT_INTAKE_LOAD:
                return SystemState.INTAKE_LOAD;
            case WANT_PLOW:
                return SystemState.PLOW;
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
            case WANT_PLOW:
                return SystemState.PLOW;
            default:
                return SystemState.IDLE;
        }
    }

    private synchronized SystemState handlePlow(double now, double startTime){


        if(mStateChanged){
            actuatedDown = getIntakeDown();
            mSolenoid.set(Value.kForward);
            actuationStartTime=0;
        }




        if(!actuatedDown&&now-startTime>=Constants.kIntakeActuationTime){
            actuatedDown = true;
            setOpenLoop(-Constants.kIntakePlowSpeed);
        }

        if(mWantedState!=WantedState.WANT_PLOW) {
            setOpenLoop(0);
        }


        WantedState newState = mWantedState;

        if(newState!=WantedState.WANT_PLOW){ //exiting intake ground
            setOpenLoop(0);

            if(actuatedDown){//if it wants to exit, but actuated down
                 newState = WantedState.WANT_PLOW;

                 if(actuatedDown&&actuationStartTime==0){//wants exit, actuated down, timer hasn't started
                    actuationStartTime = now;
                 }

                 //wants to exit, actuated down, and timer ready
                 if(actuationStartTime!=0&&now-actuationStartTime>=Constants.kIntakeActuationTime){
                    newState= mWantedState;
                }

            }
        }

        switch(newState){
            case WANT_INTAKE_GROUND:
                return SystemState.INTAKE_GROUND;
            case WANT_INTAKE_LOAD:
                return SystemState.INTAKE_LOAD;
            case WANT_PLOW:
                return SystemState.PLOW;
            default:
                return SystemState.IDLE;
        }
    }


    public boolean getIntakeDown(){
       return mSolenoid.get()==Value.kForward;
   }
  
    synchronized void setOpenLoop(double speed) {
       mVictor.set(ControlMode.PercentOutput, speed);
    }

    

    @Override
    public synchronized void stop() {
        setOpenLoop(0);
        mSolenoid.set(Value.kReverse);
    }

    @Override
    public void outputToSmartDashboard() {
      
      // SmartDashboard.putNumber("Intake Current", getCurrent());
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
            mSolenoid.set(Value.kForward);
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
