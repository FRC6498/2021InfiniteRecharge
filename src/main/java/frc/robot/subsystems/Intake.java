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
                    newState = handlePlow();
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
    
    
    
   
    double idleWaitTime=2.5;
    boolean idleInitialized=false;
    double idleInitializedTime = 0;

    double pistonOnTime = .2;
    boolean pistonDone = false;


    private synchronized SystemState handleIdle(double now, double startTime){
        if(mStateChanged){
            stop();
            idleInitialized=false;
            idleInitializedTime=0;
            pistonDone=false;
        }

        if(!idleInitialized&&now-startTime>=idleWaitTime){
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

    private double ballSeenStartTime=0;
    private double actuationStartTime=0;
    private double rumbleTime=.3;
    private double rumbleStartTime=0;
    private synchronized SystemState handleIntakeGround(double now, double startTime){

        if(mStateChanged){
            mSolenoid.set(Value.kForward);
           setOpenLoop(Constants.kIntakeGroundSpeed);
        }
        double currentThreshold = Constants.kIntakeGroundCurrentThreshold;
        if(Constants.kIntakeVelocityCompensation){
            double avgVelocity = Math.abs( (Drive.getInstance().getLeftVelocityInchesPerSec()
            +Constants.kIntakeVelocityRateOfChange*Drive.getInstance().getRightVelocityInchesPerSec())/2);
            setOpenLoop(Constants.kIntakeGroundSpeed+Constants.kIntakeVelocityRateOfChange+avgVelocity);

            currentThreshold = currentThreshold+Constants.kIntakeCurrentRateOfChange*avgVelocity;
        }

        //if(now-startTime>=Constants.kIntakeActuationTime){
               
           
          //  if(ballSeenStartTime==0&&getCurrent()>=currentThreshold ){
         //       ballSeenStartTime=now;
         //   //  System.out.println("Intake ball detected");
          //      mRobotState.setIntakeBalls(1);
         //       ControlBoard.getInstance().setRumble(.75);
        //        rumbleStartTime=now; //rumble on when ball seen

        //    }else if(now-ballSeenStartTime>=Constants.kIntakeGroundTimeThreshold){
        //        ballSeenStartTime = 0;

        //    }
       
      //  }

        if(rumbleStartTime!=0&&now-rumbleStartTime>=rumbleTime){
            ControlBoard.getInstance().setRumble(0);
            rumbleStartTime=0;
        }
            
        if(actuationStartTime!=0&&now-actuationStartTime>=Constants.kIntakeActuationTime){
            if(mWantedState==WantedState.WANT_INTAKE_GROUND) mWantedState=WantedState.WANT_IDLE;
            System.out.println("Intake reached max balls");
        }else if(actuationStartTime!=0){ 
        }else if(mRobotState.getTotalBalls()>=5){
            actuationStartTime=now;
        }



        if(mWantedState!=WantedState.WANT_INTAKE_GROUND) {
            ControlBoard.getInstance().rumbleOff();
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

    private synchronized SystemState handlePlow(){


        if(mStateChanged){
            mSolenoid.set(Value.kForward);
           setOpenLoop(-Constants.kIntakePlowSpeed);
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


   public boolean getIntakeDown(){
       return mSolenoid.get()==Value.kForward;
   }
  
    synchronized void setOpenLoop(double speed) {
       mVictor.set(ControlMode.PercentOutput, speed);
    }

   //synchronized double getCurrent(){
    //   return mVictor.
  // }
    

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
