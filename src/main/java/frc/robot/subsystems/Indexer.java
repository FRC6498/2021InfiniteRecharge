package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.loops.Loop;
import frc.robot.loops.Looper;

/**
 
 */

 
public class Indexer extends Subsystem {

    TalonFX mFalcon;
    DigitalInput mSensor;

    private RobotState mRobotState = RobotState.getInstance();
    private BeltClamp mBeltClamp = BeltClamp.getInstance();

    private static Indexer instance_ = new Indexer();

    public static Indexer getInstance() {
        return instance_;
    }


   

    

    Loop mLoop = new Loop() {

        @Override
        public void onStart() {
            synchronized (Indexer.this) {
               
            }
        }
    
        @Override
        public void onLoop() {
            synchronized (Indexer.this) {


                if(hasBall()&&FeederBelt.getInstance().needsBall()){
                    set(Constants.kIndexandBeltSpeed);
                }else if(needsBall()/*&&mBeltClamp.getConveying()*/){
                    set(Constants.kIndexOnlySpeed);
                }else{
                    set(0);
                }


                ballUpdate();
                
        }
     }
       

        @Override
        public void onStop() {
            synchronized (Indexer.this) {
               stop();
            }
        }
    };

    private Indexer() {
        mFalcon = new TalonFX(Constants.kIndexFalconId);
        mFalcon.setNeutralMode(NeutralMode.Brake);

        mFalcon.configOpenloopRamp(Constants.kIndexRampRate);

        mFalcon.setInverted(TalonFXInvertType.Clockwise);

        mSensor = new DigitalInput(Constants.kIndexSensorPort);
    }

   

    private boolean isMoving=false;

    public synchronized boolean getMoving(){
        return isMoving;
    }
    
   
    private synchronized void set(double power) {
       mFalcon.set(ControlMode.PercentOutput, power);

       if(power>0) isMoving=true;
       else isMoving=false;

    }



    private boolean mHasBall=false;

    public synchronized boolean getRawSensor(){
        return mSensor.get();
    }

    public synchronized boolean hasBall(){
        return mHasBall;
    }

    public  synchronized boolean needsBall(){
        return !getRawSensor();
    }

    boolean mStartedLoading=false;
  //  private double startSeenTime=0;
    private  void ballUpdate(){

       /* if(needsBall()){
            if(startSeenTime==0&&getRawSensor()) startSeenTime=Timer.getFPGATimestamp();
            if(startSeenTime!=0&&getRawSensor()&&start) 
        }

        if(startSeenTime!=0&&getRawSensor()){*/
           mHasBall = getRawSensor();

           if(!mStartedLoading&&!mHasBall) mStartedLoading=true;

           if(mStartedLoading && mHasBall){     //this logic adds a ball to feeder and subtracts from belts when the sensor switches from false to true
                mStartedLoading=false;
                mRobotState.setFeederBalls(1);
                mRobotState.setIntakeBalls(-1);
               System.out.println("Add ball indexer");
           }

    }

  

    @Override
    public void outputToSmartDashboard() {
        SmartDashboard.putBoolean("Index Ball", hasBall());
    }

    @Override
    public synchronized void stop() {
        set(0);
    }

    @Override
    public synchronized void zeroSensors() {
     
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
           
            set(Constants.kIndexOnlySpeed);
        }else if(timeElapsed<4){
           stop();
        }else{
            startTestTime=0;
            return true;
        }


        return false;
    }
}
