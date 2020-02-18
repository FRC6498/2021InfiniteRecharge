package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.loops.Loop;
import frc.robot.loops.Looper;
import frc.robot.subsystems.Intake.WantedState;

/**
 
 */

 
public class BeltClamp extends Subsystem {
   
 




    VictorSPX mVictor;
    Solenoid mSolenoid;
    
    
    
    private RobotState mRobotState;
    private Intake mIntake;
    private Indexer mIndexer;

    
    private static BeltClamp instance_ = new BeltClamp();

    public static BeltClamp getInstance() {
        return instance_;
    }

    private BeltClamp() {
        mVictor = new VictorSPX(Constants.kBeltClampVictorId);

        mVictor.setNeutralMode(NeutralMode.Coast);

        mVictor.setInverted(false);

        mSolenoid = new Solenoid(Constants.beltClampSolenoidId);
        mSolenoid.set(false);

        mRobotState = RobotState.getInstance();
        mIntake = Intake.getInstance();
        mIndexer = Indexer.getInstance();

    }

    
    
    enum SystemState {IDLE, //open at rest
                    CONVEYING, //closed, moving
                    AGITATING}; //open, oscillating

    enum WantedState {WANT_IDLE, 
        WANT_CONVEY,
        WANT_AGITATING
        }; 

   
    private SystemState mSystemState = SystemState.IDLE;
    private WantedState mWantedState = WantedState.WANT_IDLE;
    private boolean mStateChanged;

    private double ballFromIntakeTime=0;
    
    Loop mLoop = new Loop() {
        private double mCurrentStateStartTime;
        

        @Override
        public void onStart() {
            synchronized (BeltClamp.this) {
               
                mCurrentStateStartTime = Timer.getFPGATimestamp();
                mSystemState = SystemState.IDLE;
                mWantedState = WantedState.WANT_IDLE;
                mStateChanged = true;
            }
        }
    
        @Override
        public void onLoop() {
            synchronized (BeltClamp.this) {


                 double now = Timer.getFPGATimestamp();
                SystemState newState=mSystemState;




                if(!Intake.getInstance().getIntakeDown()){ //not intaking

                    
                    if(Indexer.getInstance().getMoving()){ //indexer needs ball
                        if(mRobotState.getBeltBalls()>0){ //ball available to convey
                            mWantedState = WantedState.WANT_CONVEY;
                        }
                    }else{
                        mWantedState = WantedState.WANT_IDLE;
                    }

                   
                    ballFromIntakeTime=0;
                } else{ //intaking

                    if(RobotState.getInstance().getBeltBalls()>=2&&RobotState.getInstance().getFeederBalls()<2){ //belt is full right now, should probaly fill
                        
                            if(ballFromIntakeTime==0) ballFromIntakeTime=now;
                            else if(now-ballFromIntakeTime>=Constants.kBeltClampIntakeClampTime) mWantedState = WantedState.WANT_CONVEY;
                        
                    }else{
                        mWantedState = WantedState.WANT_AGITATING;
                        ballFromIntakeTime=0;
                    }

                   

                }
                
                
     



                switch (mSystemState) {
               case IDLE:
                    newState = handleIdle();
                    break;
                case CONVEYING:
                    newState = handleConveying(now, mCurrentStateStartTime);
                    break;
                case AGITATING:
                    newState = handleAgitating(now, mCurrentStateStartTime);
                    break;
                default:
                    System.out.println("Unexpected BeltClamp state: " + mSystemState);
                    newState = SystemState.IDLE;
                }

                if (newState != mSystemState) {
                    System.out.println("BeltClamp state " + mSystemState + " to " + newState);
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
            synchronized (BeltClamp.this) {
               stop();
            }
        }
    };
    
    
    
   
    
    
    private synchronized SystemState handleIdle(){
        if(mStateChanged){
            stop();
        }

        switch(mWantedState){
            case WANT_CONVEY:
                return SystemState.CONVEYING;
            case WANT_AGITATING:
                return SystemState.AGITATING;
            default:
                return SystemState.IDLE;
        }
        
    }

    private synchronized SystemState handleConveying(double now, double stateStartTime){

        if(mStateChanged){
            mSolenoid.set(true);
            setOpenLoop(Constants.kBeltClampConveySpeed);
        }

        boolean timedOut=false;

       // if(now-stateStartTime>=Constants.kBeltClampConveyTimeout) { //timed out, start agitating
      //      timedOut=true;
      //  }


        switch(mWantedState){
            case WANT_AGITATING:
                return SystemState.AGITATING;
            case WANT_CONVEY:
                if(timedOut) return SystemState.AGITATING;
                else return SystemState.CONVEYING;
            default:
                return SystemState.IDLE;
        }


    }

    private double agitationStartTime = 0;
    private int agitationDirection = 1;
    private synchronized SystemState handleAgitating(double now, double stateStartTime){

        if(mStateChanged){
            agitationStartTime = now;
            agitationDirection=1;
            setOpenLoop(Constants.kBeltClampAgitationSpeed);
            mSolenoid.set(false);
        }

        if(now-agitationStartTime>Constants.kBeltClampAgitationPeriod){
            agitationStartTime = now;
            agitationDirection*=-1;
            setOpenLoop(Constants.kBeltClampAgitationSpeed*agitationDirection);
        }

        boolean timerDone=false;
        
        if(now-stateStartTime>=Constants.kBeltClampConveyTimeout) { //timed out, start agitating
           timerDone=true;
        }

        switch(mWantedState){
            case WANT_CONVEY:
                if(timerDone) return SystemState.CONVEYING;
            case WANT_IDLE:
                return SystemState.IDLE;
            default:
                return SystemState.AGITATING;
        }

    }

   
    synchronized void setOpenLoop(double speed) {
       mVictor.set(ControlMode.PercentOutput, speed);
    }

   
    public boolean getConveying(){
        return mSystemState==SystemState.CONVEYING;
    }

    @Override
    public synchronized void stop() {
        setOpenLoop(0);
        mSolenoid.set(false);
    }

    @Override
    public void outputToSmartDashboard() {
      
     SmartDashboard.putString("BeltClampState", mSystemState.toString());
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
            setOpenLoop(Constants.kBeltClampConveySpeed);
        }else if(timeElapsed<8){
           stop();
        }else{
            startTestTime=0;
            return true;
        }


        return false;
    }
}
