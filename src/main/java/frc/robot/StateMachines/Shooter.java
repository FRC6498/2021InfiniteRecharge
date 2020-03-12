package frc.robot.StateMachines;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.util.InterpolatingDouble;
import frc.lib.util.ReflectingCSVWriter;
import frc.lib.util.Rotation2d;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.ShooterTuning.FlywheelRPMs;
import frc.robot.ShooterTuning.HoodAngles;
import frc.robot.Vision.TurretCam;
import frc.robot.Vision.TurretCam.CameraMode;
import frc.robot.Vision.TurretCam.LightMode;
import frc.robot.loops.Loop;
import frc.robot.loops.Looper;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.FeederFlywheel;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.ShooterAimingParameters;
import frc.robot.subsystems.Subsystem;
import frc.robot.subsystems.Turret;

//import static frc.robot.subsystems.Superstructure.WantedIntakeState.WANT_TO_RUN_INTAKE;

/**
 * The overarching superclass containing all components of the superstructure:
 * the computer vision, flywheel, hood, hood roller, and intake. This
 * coordinates the entire firing sequence, from when the ball is intaked to when
 * it is fired.
 * 
 * @see Flywheel
 * @see Hood
 * @see Intake
 * @see VisionServer
 */

public class Shooter extends Subsystem {

    static Shooter mInstance = new Shooter();

    

    public static Shooter getInstance() {
        return mInstance;
    }

    public static class HoodTuningOutput {
        public double timestamp;
        public double range;
        public double angle;
        public double rpm;
    }

    /**
     * Drives actual state, all outputs should be dictated by this state
     */
    private enum SystemState {
        IDLE, //move to center and retract hood
        AUTO_AIM, //automatically aims when in range
        UNJAMMING, //ifits jammed, get the balls out
        DUMP_TO_TRENCH,
        OPEN_LOOP
    }

    /**
     * Drives state changes. Outputs should not be decided based on this enum.
     */
    public enum WantedState {
        WANT_AUTO,
        WANT_OPEN_LOOP,
        WANT_UNJAM,
        WANT_DUMP_TO_TRENCH,
        WANT_IDLE
    }

     /**
     * Orthogonal to the shooter state is the state of the firing mechanism.
     * Outputs should not be decided based on this enum.
     */
    public enum WantedFiringState {
        WANT_TO_HOLD_FIRE, // The user does not wish to fire
        WANT_TO_FIRE_NOW, // The user wants to fire now, regardless of readiness
        WANT_TO_FIRE_IF_READY // The user wants to fire as soon as we achieve
                                // readiness
    }

    public enum WantedFiringAmount{
        WANT_FIRE_ONE,
        WANT_FIRE_CONTINUOUS
    }

    private boolean automaticShooting = false;


    private WantedState mWantedState = WantedState.WANT_IDLE;
    private WantedFiringState mWantedFiringState = WantedFiringState.WANT_TO_HOLD_FIRE;
    private WantedFiringAmount mWantedFiringAmount = WantedFiringAmount.WANT_FIRE_ONE;
   
    private double mCurrentRangeForLogging;
    private double mCurrentAngleForLogging;
    
    private boolean mTuningMode = false;

    private HoodTuningOutput mHoodTuningOutput = new HoodTuningOutput();
    private final ReflectingCSVWriter<HoodTuningOutput> mCSVWriter;

    private double mTurretManualScanOutput = 0;
    private ShooterAimingParameters mTurretManualSetpoint = null;
    private double mHoodManualScanOutput = 0;
    private double mHoodAdjustment = 0.0;
    
    int mConsecutiveCyclesOnTarget = 0;
    int mNumShotsFired = 0;

  //  private List<ShooterAimingParameters> mCachedAimingParams = new ArrayList<>();

    Turret mTurret = Turret.getInstance();
    Flywheel mFlywheel = Flywheel.getInstance();
    Hood mHood = Hood.getInstance();
   
    
    RobotState mRobotState = RobotState.getInstance();
    Drive mDrive = Drive.getInstance();
    
    private SystemState mSystemState = SystemState.IDLE;
    private boolean mStateChanged;

    //NetworkTable mShooterTable = NetworkTable.getTable("shooter");

    Loop mLoop = new Loop() {

      
        // Every time we transition states, we update the current state start
        // time and the state changed boolean (for one cycle)
        private double mCurrentStateStartTime;
        

        @Override
        public void onStart() {
            synchronized (Shooter.this) {
               
                
                mWantedState = WantedState.WANT_IDLE;
                mWantedFiringState = WantedFiringState.WANT_TO_HOLD_FIRE;
                mCurrentStateStartTime = Timer.getFPGATimestamp();
                mSystemState = SystemState.IDLE;
                mStateChanged = true;
                mTurretManualSetpoint = null;
                System.out.println("Shooter loop start");
            }
        }

        @Override
        public void onLoop() {
            synchronized (Shooter.this) {
                
                double now = Timer.getFPGATimestamp();
                SystemState newState;
                switch (mSystemState) {
               case IDLE:
                    newState = handleIdle();
                    break;
                case AUTO_AIM:
                    newState = handleAutoAim(now);
                    break;
                case UNJAMMING:
                    newState = handleUnjamming();
                    break;
                case OPEN_LOOP:
                    newState = handleOpenLoop();
                    break;
                case DUMP_TO_TRENCH:
                    newState = handleDumpToTrench();
                    break;
                default:
                    System.out.println("Unexpected shooter state: " + mSystemState);
                    newState = SystemState.IDLE;
                }

                if (newState != mSystemState) {
                    System.out.println("Shooter state " + mSystemState + " to " + newState);
                    mSystemState = newState;
                    mCurrentStateStartTime = now;
                    mStateChanged = true;
                } else {
                    mStateChanged = false;
                }


                


              if((mWantedFiringState==WantedFiringState.WANT_TO_FIRE_IF_READY&&readyToFire(now)
                                        ||mWantedFiringState==WantedFiringState.WANT_TO_FIRE_NOW)
                                        &&RobotState.getInstance().getFeederBalls()>0){
                  if(mWantedFiringAmount==WantedFiringAmount.WANT_FIRE_ONE){
                      mWantedFiringState=WantedFiringState.WANT_TO_HOLD_FIRE;
                      FeederFlywheel.getInstance().setWantedState(FeederFlywheel.WantedState.WANT_FEED_ONE);
                  }else{
                    FeederFlywheel.getInstance().setWantedState(FeederFlywheel.WantedState.WANT_FEED_CONTINUOUS);
                  }
              }else{
                  mWantedFiringState=WantedFiringState.WANT_TO_HOLD_FIRE;
              }
               
                //mHoodRoller.getLoop().onLoop();
            }
        }

        @Override
        public void onStop() {
            synchronized (Shooter.this) {
              stop();
             if(mTuningMode) mCSVWriter.flush();
               
            }
        }
    };

    private Shooter() {

        mCSVWriter = new ReflectingCSVWriter<HoodTuningOutput>("/home/lvuser/HOOD-LOGS.csv",
        HoodTuningOutput.class);

        System.out.println("Shooter initalized");

    }

    public synchronized void addHoodCSV(){
        mHoodTuningOutput.timestamp=Timer.getFPGATimestamp();
        mHoodTuningOutput.range = mCurrentRangeForLogging;
        mHoodTuningOutput.rpm = mFlywheel.getRpm();
        mHoodTuningOutput.angle = mHood.getAngle();
        mCSVWriter.add(mHoodTuningOutput);
    }

  

    public synchronized void setWantedState(WantedState newState) {
        mWantedState = newState;
    }
    public synchronized void setWantsToFireNow(WantedFiringAmount amount) {
        mWantedFiringState = WantedFiringState.WANT_TO_FIRE_NOW;
        mWantedFiringAmount = amount;
    }

    public synchronized void setWantsToFireIfReady(WantedFiringAmount amount) {
        mWantedFiringState = WantedFiringState.WANT_TO_FIRE_IF_READY;
        mWantedFiringAmount = amount;
    }

    public synchronized void setWantsToHoldFire() {
        mWantedFiringState = WantedFiringState.WANT_TO_HOLD_FIRE;
    }

    public synchronized void setAutoShoot(boolean on){
        automaticShooting = on;
    }

    @Override
    public void outputToSmartDashboard() {
    
        
        SmartDashboard.putNumber("current_range", mCurrentRangeForLogging);
        SmartDashboard.putNumber("current_angle", mCurrentAngleForLogging);
        SmartDashboard.putString("shooter_state", "" + mSystemState);
        SmartDashboard.putBoolean("shooter_ready_to_fire",readyToFire(Timer.getFPGATimestamp()));
        SmartDashboard.putNumber("Hood adjustment", mHoodAdjustment);
    }

    @Override
    public synchronized void stop() {
      
    }

    @Override
    public synchronized void zeroSensors() {
        
      //  mCachedAimingParams.clear();
       
    }

    public synchronized void resetTurretAtMax() {
        mTurret.reset(Rotation2d.fromDegrees(Constants.kHardMaxTurretAngle));
        System.out.println("reset at max");
    }

    public synchronized void resetTurretAtMin() {
        mTurret.reset(Rotation2d.fromDegrees(Constants.kHardMinTurretAngle));
        System.out.println("reset at min");
    }

    public synchronized void zeroTurret() {
        mTurret.reset(new Rotation2d());
    }

    /**
     * Sets the manual turret output ONLY when the turret is auto-aiming without
     * a visible target
     */
    public synchronized void setTurretManualScanOutput(double output) {
        mTurretManualScanOutput = output;
    }

    public synchronized void setHoodManualScanOutput(double output) {
        mHoodManualScanOutput = output;
    }

    public synchronized void setTurretManualPositionSetpoint(ShooterAimingParameters parameters) {
        mTurretManualSetpoint = parameters;
    }

    public synchronized void clearTurretManualPositionSetpoint() {
        mTurretManualSetpoint = null;
    }

   

    
//whether it is a new or old ball
    public synchronized void setHoodAdjustment(double adjustment) {
        mHoodAdjustment = adjustment;
    }

   
    


    public synchronized void setTuningMode(boolean tuning_on) {
        mTuningMode = tuning_on;
    }

    private synchronized SystemState handleIdle(){
        if(mStateChanged){
            mFlywheel.stop();
            mTurret.setDesiredAngle(new Rotation2d());
       
            mHood.setDesiredAngle(Rotation2d.fromDegrees(Constants.kHoodNeutralAngle));
        }

        switch(mWantedState){
            case WANT_AUTO:
                return SystemState.AUTO_AIM;
            case WANT_UNJAM:
                return SystemState.UNJAMMING;            
            case WANT_OPEN_LOOP:
                return SystemState.OPEN_LOOP;    
            case WANT_DUMP_TO_TRENCH:
                return SystemState.DUMP_TO_TRENCH;
            default:
                return SystemState.IDLE;
        }
    }

    private synchronized SystemState handleUnjamming(){


        switch(mWantedState){
            case WANT_AUTO:
                return SystemState.AUTO_AIM;
            case WANT_UNJAM:
                return SystemState.UNJAMMING;       
            case WANT_OPEN_LOOP:
                return SystemState.OPEN_LOOP;       
            case WANT_DUMP_TO_TRENCH:
                return SystemState.DUMP_TO_TRENCH;
            default:
                return SystemState.IDLE;
        }
    }


    private synchronized SystemState handleDumpToTrench(){

        if(mStateChanged){
            mFlywheel.setRpm(Constants.kDumpToTrenchSpeed);
            mHood.setDesiredAngle(Rotation2d.fromDegrees(Constants.kDumpToTrenchPitch));
            mTurret.setDesiredAngle(Rotation2d.fromDegrees(Constants.kDumpToTrenchYaw));

            setWantsToFireIfReady(WantedFiringAmount.WANT_FIRE_CONTINUOUS);

          
        }


        
       if(mTurretManualScanOutput!=0) mTurret.setOpenLoop(mTurretManualScanOutput);
                   
       if(mHoodManualScanOutput!=0)mHood.setOpenLoop(mHoodManualScanOutput);

       setWantsToFireIfReady(WantedFiringAmount.WANT_FIRE_CONTINUOUS);

        if(mWantedState!=WantedState.WANT_DUMP_TO_TRENCH){
            setWantsToHoldFire();
        }

        switch(mWantedState){
            case WANT_AUTO:
                return SystemState.AUTO_AIM;
            case WANT_UNJAM:
                return SystemState.UNJAMMING;       
            case WANT_OPEN_LOOP:
                return SystemState.OPEN_LOOP;       
            case WANT_DUMP_TO_TRENCH:
                return SystemState.DUMP_TO_TRENCH;
            default:
                return SystemState.IDLE;
        }
    }


    private synchronized SystemState handleOpenLoop(){
        if(mStateChanged){
            mFlywheel.setRpm(Constants.kFlywheelGoodBallRpmSetpoint);
        }
    
       
                // Manual search
                if (mTurretManualSetpoint != null) {
                    mTurret.setDesiredAngle(mTurretManualSetpoint.getTurretAngle());
                   mHood.setDesiredAngle(Rotation2d.fromDegrees(getHoodAngleForRange(mTurretManualSetpoint.range)));
                } else {
                    mTurret.setOpenLoop(mTurretManualScanOutput);
                   
                    mHood.setOpenLoop(mHoodManualScanOutput);
                    
                }
               
                

        switch(mWantedState){
            case WANT_AUTO:
                return SystemState.AUTO_AIM;
            case WANT_UNJAM:
                return SystemState.UNJAMMING;       
            case WANT_OPEN_LOOP:
                return SystemState.OPEN_LOOP;     
            case WANT_DUMP_TO_TRENCH:
                return SystemState.DUMP_TO_TRENCH;
            default:
                return SystemState.IDLE;
        }
    }

    boolean has_target=false;

  private synchronized SystemState handleAutoAim(double now){

    if(mStateChanged){
        TurretCam.setCameraMode(CameraMode.eVision);
        TurretCam.setLedMode(LightMode.eOn);
    }

    ShooterAimingParameters aimingParameters = getCurrentAimingParameters(now);
      /*  if (aimingParameters==null){// && (allow_changing_tracks || mTurretManualSetpoint != null)) {
            // Manual search
            if (mTurretManualSetpoint != null) {
                mTurret.setDesiredAngle(mTurretManualSetpoint.getTurretAngle());
               mHood.setDesiredAngle(Rotation2d.fromDegrees(getHoodAngleForRange(mTurretManualSetpoint.range)));
            } else {
                mTurret.setOpenLoop(mTurretManualScanOutput);
                if (!mTuningMode) {
                    mHood.setDesiredAngle(Rotation2d.fromDegrees(Constants.kHoodNeutralAngle));
                } else {
                   mHood.setOpenLoop(mHoodManualScanOutput);
                }
            }*/
        
            // Pick the target to aim at
           has_target = false;
            //for (ShooterAimingParameters param : aimingParameters) {
                double turret_angle_degrees=0;
               if(aimingParameters!=null){
                     turret_angle_degrees = aimingParameters.getTurretAngle().getDegrees();
                if (turret_angle_degrees >= Constants.kSoftMinTurretAngle
                        && turret_angle_degrees <= Constants.kSoftMaxTurretAngle
                        && aimingParameters.getRange() >= Constants.kAutoAimMinRange
                        && aimingParameters.getRange() <= Constants.kAutoAimMaxRange
                        /*&& (allow_changing_tracks || mCurrentTrackId == aimingParameters.getTrackid())*/) {
                    // This target works
                    mFlywheel.setRpm(getShootingSetpointRpm(aimingParameters.getRange()));
                    if (!mTuningMode) {
                        double angle_degrees = getHoodAngleForRange(aimingParameters.getRange()) + mHoodAdjustment;
                        angle_degrees = Math.max(angle_degrees, Constants.kMinHoodAngle);
                        angle_degrees = Math.min(angle_degrees, Constants.kMaxHoodAngle);
                        mHood.setDesiredAngle(Rotation2d.fromDegrees(angle_degrees));
                    } else {
                        mHood.setOpenLoop(mHoodManualScanOutput);
                    }
                    mTurret.setDesiredAngle(aimingParameters.getTurretAngle());
                    mCurrentAngleForLogging = aimingParameters.getTurretAngle().getDegrees();
                    mCurrentRangeForLogging = aimingParameters.getRange();
                   // mCurrentTrackId = aimingParameters.getTrackid();
                    has_target = true;

                    if(automaticShooting){
                        setWantsToFireIfReady(WantedFiringAmount.WANT_FIRE_ONE);
                    }
                }
                    //break;
                //}
                
              
            
          //  if (!has_target) {
           //     mCurrentTrackId = -1;
           // }
        }


        if( mTurretManualSetpoint!=null&&has_target==false){//mTurretManualScanOutput!=0){

            mTurret.setDesiredAngle(mTurretManualSetpoint.getTurretAngle());
            mHood.setDesiredAngle(Rotation2d.fromDegrees(mTurretManualSetpoint.getRange()));
            mFlywheel.setRpm(getShootingSetpointRpm(mTurretManualSetpoint.getRange()));
         }



        if(mWantedState!=WantedState.WANT_AUTO){
            TurretCam.setCameraMode(CameraMode.eDriver);
           TurretCam.setLedMode(LightMode.eOff);
        }

        switch(mWantedState){
            case WANT_AUTO:
                return SystemState.AUTO_AIM;
            case WANT_UNJAM:
                return SystemState.UNJAMMING;            
            case WANT_OPEN_LOOP:
                return SystemState.OPEN_LOOP;
            case WANT_DUMP_TO_TRENCH:
                return SystemState.DUMP_TO_TRENCH;
            default:
                return SystemState.IDLE;
        }

    }
   

  
   

    private double getHoodAngleForRange(double range) {
        InterpolatingDouble result = HoodAngles.kHoodAutoAimMap.getInterpolated(new InterpolatingDouble(range));
        if (result != null) {
            return result.value;
        } else {
            return Constants.kHoodNeutralAngle;
        }
    }

    public ShooterAimingParameters getCurrentAimingParameters(double now) {
        ShooterAimingParameters param = mRobotState.getAimingParameters(now);
        //if(param!=null)mCachedAimingParams.add(param);
        return param;

    }

    //public List<ShooterAimingParameters> getCachedAimingParams() {
   //     return mCachedAimingParams;
   // }

    

    public synchronized boolean hasTarget() {
        return has_target;
    }

    private double getShootingSetpointRpm(double range) {
        return FlywheelRPMs.kFlywheelAutoAimMap.getInterpolated(new InterpolatingDouble(range)).value;
    }

    private boolean readyToFire(double now) {
        boolean is_stopped = Math.abs(mDrive.getLeftVelocityInchesPerSec()) < Constants.kAutoShootMaxDriveSpeed
                && Math.abs(mDrive.getRightVelocityInchesPerSec()) < Constants.kAutoShootMaxDriveSpeed;
        if (mSystemState == SystemState.AUTO_AIM) {
            if ((mTuningMode || mHood.isOnTarget()) && mFlywheel.isOnTarget() && mTurret.isOnTarget()
                    && hasTarget()) {
                mConsecutiveCyclesOnTarget++;
            } else {
                mConsecutiveCyclesOnTarget = 0;
            }
        } else if (mSystemState == SystemState.OPEN_LOOP||mSystemState==SystemState.DUMP_TO_TRENCH) {
            if (mFlywheel.isOnTarget()) {
                mConsecutiveCyclesOnTarget++;
            } else {
                mConsecutiveCyclesOnTarget = 0;
            }
        } else {
            mConsecutiveCyclesOnTarget = 0;
        }
        return mConsecutiveCyclesOnTarget > Constants.kAutoAimMinConsecutiveCyclesOnTarget && is_stopped;
    }

   

    @Override
    public void registerEnabledLoops(Looper in) {
        in.register(mLoop);
    }


    double startTestTime=0;
    @Override
    public boolean test(double now) {
        

        return true;
    }


    @Override
    public void writeToLog() {
        mCSVWriter.write();
    }
}
