package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.loops.Loop;
import frc.robot.loops.Looper;

/**
 
 */

public class FeederBelt extends Subsystem {

    VictorSPX mVictor;
    DigitalInput mSensor;

    Indexer mIndexer = Indexer.getInstance();
    private RobotState mRobotState = RobotState.getInstance();

    private static FeederBelt instance_ = new FeederBelt();

    public static FeederBelt getInstance() {
        return instance_;
    }

    Loop mLoop = new Loop() {

        @Override
        public void onStart() {
            synchronized (FeederBelt.this) {

            }
        }

        @Override
        public void onLoop() {
            synchronized (FeederBelt.this) {

                if(needsBall()&&(mIndexer.hasBall()||mIndexer.getMoving())){
                    set(Constants.kFeederBeltLoadingSpeed);
                }else if(needsBall()){//needs ball but no ball ready
                    set(0);
                }else if(FeederFlywheel.getInstance().needsBall()){//has ball and feeder fly needs one
                    set(Constants.kFeederBeltDeployingSpeed);
                }else{
                    set(0);
                }

                ballUpdate();

            }
        }

        @Override
        public void onStop() {
            synchronized (FeederBelt.this) {
                stop();
            }
        }
    };

    private FeederBelt() {
        mVictor = new VictorSPX(Constants.kFeederBeltVictorId);
        mVictor.setNeutralMode(NeutralMode.Brake);

        mVictor.configOpenloopRamp(Constants.kFeederBeltRampRate);

        mVictor.setInverted(true);

        mSensor = new DigitalInput(Constants.kFeederBeltSensorPort);
    }

   

  
   
    private synchronized void set(double power) {
       mVictor.set(ControlMode.PercentOutput, power);
    }



    private boolean mHasBall=false;

    public synchronized boolean getRawSensor(){
        return mSensor.get();
    }

    public synchronized boolean hasBall(){
        return getRawSensor();
    }

    public synchronized boolean needsBall(){
        return !getRawSensor();
    }

    private boolean mStartedFeeding=false;

  //  private double startSeenTime=0;
    private synchronized void ballUpdate(){

       /* if(needsBall()){
            if(startSeenTime==0&&getRawSensor()) startSeenTime=Timer.getFPGATimestamp();
            if(startSeenTime!=0&&getRawSensor()&&start) 
        }

        if(startSeenTime!=0&&getRawSensor()){*/
           mHasBall = getRawSensor();


           if(!mStartedFeeding&&mHasBall) mStartedFeeding=true;

           if(mStartedFeeding && !mHasBall){  //this logic subtracts ball when sensor goes from true to false
               mStartedFeeding=false;
               mRobotState.setFeederBalls(-1); 
               System.out.println("Subtract ball feeder belt");
           }

    }

    

  

    @Override
    public void outputToSmartDashboard() {
        SmartDashboard.putBoolean("Feeder Belt Ball", hasBall());
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
           
            set(Constants.kFeederBeltLoadingSpeed);
        }else if(timeElapsed<4){
           stop();
        }else{
            startTestTime=0;
            return true;
        }


        return false;
    }
}
