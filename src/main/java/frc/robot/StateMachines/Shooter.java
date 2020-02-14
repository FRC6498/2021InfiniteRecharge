package frc.robot.StateMachines;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.util.InterpolatingDouble;
import frc.lib.util.Rotation2d;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.Vision.TurretCam;
import frc.robot.Vision.TurretCam.CameraMode;
import frc.robot.Vision.TurretCam.LightMode;
import frc.robot.loops.Loop;
import frc.robot.loops.Looper;
import frc.robot.subsystems.Drive;

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

    /**
     * Drives actual state, all outputs should be dictated by this state
     */
    private enum SystemState {
        IDLE, //move to center and retract hood
        AUTO_AIM, //automatically aims when in range
        UNJAMMING, //ifits jammed, get the balls out
        //OPEN_LOOP
    }

    /**
     * Drives state changes. Outputs should not be decided based on this enum.
     */
    public enum WantedState {
        WANT_AUTO,
        //WANT_OPEN,
        WANT_UNJAM,
        WANT_IDLE
    }


    private WantedState mWantedState = WantedState.WANT_IDLE;
   
    private double mCurrentRangeForLogging;
    private double mCurrentAngleForLogging;
    
    private boolean mTuningMode = false;

    private double mTurretManualScanOutput = 0;
    private ShooterAimingParameters mTurretManualSetpoint = null;
    private double mHoodManualScanOutput = 0;
    private double mHoodAdjustment = 0.0;
    
    int mConsecutiveCyclesOnTarget = 0;
    int mNumShotsFired = 0;

    private List<ShooterAimingParameters> mCachedAimingParams = new ArrayList<>();

    Turret mTurret = Turret.getInstance();
   // Flywheel mFlywheel = Flywheel.getInstance();
   // Hood mHood = Hood.getInstance();
   
    
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
               
                mCurrentStateStartTime = Timer.getFPGATimestamp();
                mSystemState = SystemState.IDLE;
                mStateChanged = true;
                mTurretManualSetpoint = null;
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
               
                //mHoodRoller.getLoop().onLoop();
            }
        }

        @Override
        public void onStop() {
            synchronized (Shooter.this) {
              stop();
               
            }
        }
    };

    private Shooter() {
    }

    

    public synchronized void setWantedState(WantedState newState) {
        mWantedState = newState;
    }

    @Override
    public void outputToSmartDashboard() {
    
        
        SmartDashboard.putNumber("current_range", mCurrentRangeForLogging);
        SmartDashboard.putNumber("current_angle", mCurrentAngleForLogging);
        SmartDashboard.putString("shooter_state", "" + mSystemState);
    }

    @Override
    public synchronized void stop() {
      
    }

    @Override
    public synchronized void zeroSensors() {
        
        mCachedAimingParams.clear();
       
    }

    public synchronized void resetTurretAtMax() {
        mTurret.reset(Rotation2d.fromDegrees(Constants.kHardMaxTurretAngle));
    }

    public synchronized void resetTurretAtMin() {
        mTurret.reset(Rotation2d.fromDegrees(Constants.kHardMinTurretAngle));
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
            //mFlywheel.stop();
            mTurret.setDesiredAngle(new Rotation2d());
       
            //mHood.setDesiredAngle(Rotation2d.fromDegrees(Constants.kBatterHoodAngle));
        }

        switch(mWantedState){
            case WANT_AUTO:
                return SystemState.AUTO_AIM;
            case WANT_UNJAM:
                return SystemState.UNJAMMING;            
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
        if (aimingParameters==null){// && (allow_changing_tracks || mTurretManualSetpoint != null)) {
            // Manual search
            if (mTurretManualSetpoint != null) {
                mTurret.setDesiredAngle(mTurretManualSetpoint.getTurretAngle());
               // mHood.setDesiredAngle(Rotation2d.fromDegrees(getHoodAngleForRange(mTurretManualSetpoint.range)));
            } else {
                mTurret.setOpenLoop(mTurretManualScanOutput);
                if (!mTuningMode) {
                  //  mHood.setDesiredAngle(Rotation2d.fromDegrees(Constants.kHoodNeutralAngle));
                } else {
                   // mHood.setOpenLoop(mHoodManualScanOutput);
                }
            }
            //mFlywheel.setRpm(Constants.kFlywheelGoodBallRpmSetpoint);
            has_target = false;
        } else {
            // Pick the target to aim at
           has_target = false;
            //for (ShooterAimingParameters param : aimingParameters) {
                double turret_angle_degrees = aimingParameters.getTurretAngle().getDegrees();
                if (turret_angle_degrees >= Constants.kSoftMinTurretAngle
                        && turret_angle_degrees <= Constants.kSoftMaxTurretAngle
                        && aimingParameters.getRange() >= Constants.kAutoAimMinRange
                        && aimingParameters.getRange() <= Constants.kAutoAimMaxRange
                        /*&& (allow_changing_tracks || mCurrentTrackId == aimingParameters.getTrackid())*/) {
                    // This target works
                   // mFlywheel.setRpm(getShootingSetpointRpm(aimingParameters.getRange()));
                    if (!mTuningMode) {
                        double angle_degrees = getHoodAngleForRange(aimingParameters.getRange()) + mHoodAdjustment;
                        angle_degrees = Math.max(angle_degrees, Constants.kMinHoodAngle);
                        angle_degrees = Math.min(angle_degrees, Constants.kMaxHoodAngle);
                       // mHood.setDesiredAngle(Rotation2d.fromDegrees(angle_degrees));
                    } else {
                        //mHood.setOpenLoop(mHoodManualScanOutput);
                    }
                    mTurret.setDesiredAngle(aimingParameters.getTurretAngle());
                    mCurrentAngleForLogging = aimingParameters.getTurretAngle().getDegrees();
                    mCurrentRangeForLogging = aimingParameters.getRange();
                   // mCurrentTrackId = aimingParameters.getTrackid();
                    has_target = true;
                    //break;
                //}
            }
          //  if (!has_target) {
           //     mCurrentTrackId = -1;
           // }
        }

        switch(mWantedState){
            case WANT_AUTO:
                return SystemState.AUTO_AIM;
            case WANT_UNJAM:
                return SystemState.UNJAMMING;            
            default:
                return SystemState.IDLE;
        }

    }
   

  
   

    private double getHoodAngleForRange(double range) {
        InterpolatingDouble result = Constants.kHoodAutoAimMap.getInterpolated(new InterpolatingDouble(range));
        if (result != null) {
            return result.value;
        } else {
            return Constants.kHoodNeutralAngle;
        }
    }

    private ShooterAimingParameters getCurrentAimingParameters(double now) {
        ShooterAimingParameters param = mRobotState.getAimingParameters(now);
        if(param!=null)mCachedAimingParams.add(param);
        return param;

    }

    public List<ShooterAimingParameters> getCachedAimingParams() {
        return mCachedAimingParams;
    }

    

    public synchronized boolean hasTarget() {
        return has_target;
    }

    private double getShootingSetpointRpm(double range) {
        return Constants.kFlywheelAutoAimMap.getInterpolated(new InterpolatingDouble(range)).value;
    }

    private boolean readyToFire(SystemState state, double now) {
        boolean is_stopped = Math.abs(mDrive.getLeftVelocityInchesPerSec()) < Constants.kAutoShootMaxDriveSpeed
                && Math.abs(mDrive.getRightVelocityInchesPerSec()) < Constants.kAutoShootMaxDriveSpeed;
        if (state == SystemState.AUTO_AIM) {
            if (/*(mTuningMode || mHood.isOnTarget()) && mFlywheel.isOnTarget() && */mTurret.isOnTarget()
                    && hasTarget()) {
                mConsecutiveCyclesOnTarget++;
            } else {
                mConsecutiveCyclesOnTarget = 0;
            }
        } /*else if (state == SystemState.OPEN_LOOP) {
            if ((mTuningMode || mHood.isOnTarget()) && mFlywheel.isOnTarget() && mTurret.isOnTarget()) {
                mConsecutiveCyclesOnTarget++;
            } else {
                mConsecutiveCyclesOnTarget = 0;
            }
        } */else {
            mConsecutiveCyclesOnTarget = 0;
        }
        return mConsecutiveCyclesOnTarget > Constants.kAutoAimMinConsecutiveCyclesOnTarget && is_stopped;
    }

   

    @Override
    public void registerEnabledLoops(Looper in) {
        in.register(mLoop);
    }
}
