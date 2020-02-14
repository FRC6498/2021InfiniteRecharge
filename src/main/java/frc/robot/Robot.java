package frc.robot;

import frc.robot.StateMachines.Shooter;
import frc.robot.auto.AutoModeExecuter;
import frc.robot.loops.GyroCalibrator;
import frc.robot.loops.Looper;
import frc.robot.loops.RobotStateEstimator;
import frc.robot.loops.TurretResetter;
import frc.robot.loops.VisionProcessor;
import frc.robot.subsystems.Drive;

import frc.robot.subsystems.Turret;
import frc.lib.util.*;

import java.util.Arrays;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The main robot class, which instantiates all robot parts and helper classes.
 * Some classes are already instantiated upon robot startup; for those classes,
 * the robot gets the instance as opposed to creating a new object (after all,
 * one can't have two drivetrains)
 * 
 * After initializing all robot parts, the code sets up the autonomous and
 * teleoperated cycles and also code that runs periodically inside both
 * routines.
 * 
 * This is the nexus/converging point of the robot code and the best place to
 * start exploring.
 */
public class Robot extends TimedRobot {
    // Subsystems
    Drive mDrive = Drive.getInstance();
    
    Compressor c;

    AutoModeExecuter mAutoModeExecuter = null;

   Shooter mShooter = Shooter.getInstance();

    // Other parts of the robot
    CheesyDriveHelper mCheesyDriveHelper = new CheesyDriveHelper();
    ControlBoard mControls = ControlBoard.getInstance();

    RobotState mRobotState = RobotState.getInstance();


       // Create subsystem manager
       private final SubsystemManager mSubsystemManager = new SubsystemManager(Arrays.asList(Drive.getInstance(), /*Flywheel.getInstance(), 
       Hood.getInstance(), */Turret.getInstance(), mShooter));
   
    // Enabled looper is called at 100Hz whenever the robot is enabled
    Looper mEnabledLooper = new Looper();
    // Disabled looper is called at 100Hz whenever the robot is disabled
    //Looper mDisabledLooper = new Looper();

    SmartDashboardInteractions mSmartDashboardInteractions = new SmartDashboardInteractions();

    boolean mLogToSmartdashboard = true;
    boolean mHoodTuningMode = false;

   

    public Robot() {
        CrashTracker.logRobotConstruction();
    }

    public void zeroAllSensors() {
        mSubsystemManager.zeroSensors();
     
    }

    

   

  

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit() {
        try {
            CrashTracker.logRobotInit();
           
           

           c=new Compressor();

            // Configure loopers

            mSubsystemManager.registerEnabledLoops(mEnabledLooper);
           
            mEnabledLooper.register(RobotStateEstimator.getInstance());
            mEnabledLooper.register(new TurretResetter());
            mEnabledLooper.register( VisionProcessor.getInstance());
          
           // mEnabledLooper.register(mDrive.getLoop());
           
            //mDisabledLooper.register(new GyroCalibrator());

            mSmartDashboardInteractions.initWithDefaults();

           
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }

         // Reset all state
         zeroAllSensors();
    }

    @Override
    public void disabledInit() {
        try {
            CrashTracker.logDisabledInit();
            if (mAutoModeExecuter != null) {
                mAutoModeExecuter.stop();
            }
            mAutoModeExecuter = null;

            // Configure loopers
            //mEnabledLooper.stop();
            //mDisabledLooper.start();

            mEnabledLooper.stop();

            // Call stop on all our Subsystems.
            mSubsystemManager.stop();

            c.stop();

            mDrive.setOpenLoop(DriveSignal.NEUTRAL);
            mDrive.setBrakeMode(true);
            // Stop all actuators
           
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void autonomousInit() {
        try {
            CrashTracker.logAutoInit();
            if (mAutoModeExecuter != null) {
                mAutoModeExecuter.stop();
            }
            mAutoModeExecuter = null;

            // Reset all sensors
            zeroAllSensors();

            // Shift to high
            mDrive.setHighGear(true);
            mDrive.setBrakeMode(true);
            
            mShooter.setHoodAdjustment(mSmartDashboardInteractions.areAutoBallsWorn()
            ? Constants.kOldBallHoodAdjustment : Constants.kNewBallHoodAdjustment);
            c.start();
           
            mEnabledLooper.start();

            mAutoModeExecuter = new AutoModeExecuter();
            mAutoModeExecuter.setAutoMode(mSmartDashboardInteractions.getSelectedAutonMode());
            mAutoModeExecuter.start();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void teleopInit() {
        try {
            CrashTracker.logTeleopInit();
            if (mAutoModeExecuter != null) {
                mAutoModeExecuter.stop();
            }
            mAutoModeExecuter = null;

            // Reset drive
            mDrive.resetEncoders();

           c.start();

            // Configure loopers
            //mDisabledLooper.stop();
            mEnabledLooper.start();
            mDrive.setOpenLoop(DriveSignal.NEUTRAL);
            mDrive.setBrakeMode(false);

            
           
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    /**
     * Keep kicking the CAN driver even though we are disabled... See:
     * https://www.ctr-electronics.com/Talon%20SRX%20Software%20Reference%
     * 20Manual.pdf page 130 NOTE: Not sure if this is still required with the
     * latest firmware, but no harm in leaving it in.
     */
    public void disabledPeriodic() {
        try {
         

            //mDrive.resetEncoders();

            //outputAllToSmartDashboard();

            zeroAllSensors();
        
            mHoodTuningMode = mSmartDashboardInteractions.isInHoodTuningMode();
            mLogToSmartdashboard = mSmartDashboardInteractions.shouldLogToSmartDashboard();
            mRobotState.reset(Timer.getFPGATimestamp(), new RigidTransform2d(), Turret.getInstance().getAngle());
           
            //mLogToSmartdashboard = mSmartDashboardInteractions.shouldLogToSmartDashboard();
            //mRobotState.reset(Timer.getFPGATimestamp(), new RigidTransform2d());

            allPeriodic();

            System.gc();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    /**
     * To control the robot, the code sets the desired state of the robot (a
     * state machine). The robot will constantly compare the desired and actual
     * state and act to bring the two closer. State machines help ensure that no
     * matter what buttons the driver presses, the robot behaves in a safe and
     * consistent manner.
     */
    @Override
    public void teleopPeriodic() {
        try {

            double throttle = mControls.getThrottle();
            double turn = mControls.getTurn();
            if (mControls.getTractionControl()) {
                Rotation2d heading_setpoint = mDrive.getGyroAngle();
                if (mDrive.getControlState() == Drive.DriveControlState.VELOCITY_HEADING_CONTROL) {
                    heading_setpoint = mDrive.getVelocityHeadingSetpoint().getHeading();
                }
                mDrive.setVelocityHeadingSetpoint(
                        mCheesyDriveHelper.handleDeadband(throttle, CheesyDriveHelper.kThrottleDeadband)
                                * Constants.kDriveLowGearMaxSpeedInchesPerSec,
                        heading_setpoint);
            } /*else if(mControls.getAlignWithLoadStation()){
                double angle = mDrive.getGyroAngle().getDegrees()-CameraVision.getTx();
                Rotation2d heading_setpoint = new Rotation2d().fromDegrees(angle);
                //if (mDrive.getControlState() == Drive.DriveControlState.VELOCITY_HEADING_CONTROL) {
                //    heading_setpoint = mDrive.getVelocityHeadingSetpoint().getHeading();
                //}
                mDrive.setVelocityHeadingSetpoint(
                        mCheesyDriveHelper.handleDeadband(throttle, CheesyDriveHelper.kThrottleDeadband)
                                * Constants.kDriveLowGearMaxSpeedInchesPerSec,
                        heading_setpoint);
            }*/else {
                mDrive.setBrakeMode(false);
                mDrive.setHighGear(!mControls.getLowGear());
                mDrive.setOpenLoop(mCheesyDriveHelper.cheesyDrive(throttle, turn, mControls.getQuickTurn()));
            }
            
            

            if (mControls.getAutoAimNewBalls()) {
                mShooter.setHoodAdjustment(Constants.kNewBallHoodAdjustment);
                mShooter.setWantedState(Shooter.WantedState.WANT_AUTO);
              
            } else if (mControls.getAutoAimOldBalls()) {
                mShooter.setHoodAdjustment(Constants.kOldBallHoodAdjustment);
                mShooter.setWantedState(Shooter.WantedState.WANT_AUTO);
              
            }
            
            mShooter.setTurretManualScanOutput(mControls.getTurretManual() * .12);

            if (mHoodTuningMode) {
                mShooter.setTuningMode(true);
                if (mControls.getHoodTuningPositiveButton()) {
                    mShooter.setHoodManualScanOutput(0.05);
                } else if (mControls.getHoodTuningNegativeButton()) {
                    mShooter.setHoodManualScanOutput(-0.05);
                } else {
                    mShooter.setHoodManualScanOutput(0.0);
                }
            } else {
                mShooter.setTuningMode(false);
            }

           allPeriodic();
     
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void autonomousPeriodic() {
        try {
            allPeriodic();

        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    
  
    @Override
    public void testPeriodic() {
      
    }

  /**
     * Helper function that is called in all periodic functions
     */
    public void allPeriodic() {
        
        mSubsystemManager.outputToSmartDashboard();
        mSubsystemManager.writeToLog();
        mEnabledLooper.outputToSmartDashboard();
       
        
    }
   
}
