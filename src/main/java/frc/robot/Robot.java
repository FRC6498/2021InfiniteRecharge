package frc.robot;

import frc.robot.auto.AutoModeExecuter;
import frc.robot.loops.GyroCalibrator;
import frc.robot.loops.Looper;
import frc.robot.loops.RobotStateEstimator;

import frc.robot.subsystems.Drive;

import frc.lib.util.*;

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

   

    // Other parts of the robot
    CheesyDriveHelper mCheesyDriveHelper = new CheesyDriveHelper();
    ControlBoard mControls = ControlBoard.getInstance();

    RobotState mRobotState = RobotState.getInstance();

    // Enabled looper is called at 100Hz whenever the robot is enabled
    Looper mEnabledLooper = new Looper();
    // Disabled looper is called at 100Hz whenever the robot is disabled
    Looper mDisabledLooper = new Looper();

    SmartDashboardInteractions mSmartDashboardInteractions = new SmartDashboardInteractions();

    boolean mLogToSmartdashboard = true;
    boolean mHoodTuningMode = false;

    boolean mGetDown = false;

    public Robot() {
        CrashTracker.logRobotConstruction();
    }

    public void stopAll() {
        mDrive.stop();
     
    }

    public void outputAllToSmartDashboard() {
        if (mLogToSmartdashboard) {
            mDrive.outputToSmartDashboard();
        
            mRobotState.outputToSmartDashboard();
        
            mEnabledLooper.outputToSmartDashboard();
        }
      
    }

    public void zeroAllSensors() {
        mDrive.zeroSensors();
       
        mRobotState.reset(Timer.getFPGATimestamp(), new RigidTransform2d());
    }

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit() {
        try {
            CrashTracker.logRobotInit();
           
            // Reset all state
            zeroAllSensors();

           c=new Compressor();

            // Configure loopers

            
           
            mEnabledLooper.register(RobotStateEstimator.getInstance());
          
            mEnabledLooper.register(mDrive.getLoop());
           
            mDisabledLooper.register(new GyroCalibrator());

            mSmartDashboardInteractions.initWithDefaults();

           
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
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
            mEnabledLooper.stop();
            mDisabledLooper.start();

            c.stop();

            mDrive.setOpenLoop(DriveSignal.NEUTRAL);
            mDrive.setBrakeMode(true);
            // Stop all actuators
            stopAll();
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
            
          
            c.start();
           
            // Configure loopers
            mDisabledLooper.stop();
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
            mDisabledLooper.stop();
            mEnabledLooper.start();
            mDrive.setOpenLoop(DriveSignal.NEUTRAL);
            mDrive.setBrakeMode(false);

            mGetDown = false;
           
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
            stopAll();

            mDrive.resetEncoders();

            outputAllToSmartDashboard();

           
            mLogToSmartdashboard = mSmartDashboardInteractions.shouldLogToSmartDashboard();
            mRobotState.reset(Timer.getFPGATimestamp(), new RigidTransform2d());

            

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
            } else if(mControls.getAlignWithLoadStation()){
                double angle = mDrive.getGyroAngle().getDegrees()-CameraVision.getTx();
                Rotation2d heading_setpoint = new Rotation2d().fromDegrees(angle);
                //if (mDrive.getControlState() == Drive.DriveControlState.VELOCITY_HEADING_CONTROL) {
                //    heading_setpoint = mDrive.getVelocityHeadingSetpoint().getHeading();
                //}
                mDrive.setVelocityHeadingSetpoint(
                        mCheesyDriveHelper.handleDeadband(throttle, CheesyDriveHelper.kThrottleDeadband)
                                * Constants.kDriveLowGearMaxSpeedInchesPerSec,
                        heading_setpoint);
            } else{
                mDrive.setBrakeMode(false);
                //mDrive.setHighGear(!mControls.getLowGear());
                mDrive.setOpenLoop(mCheesyDriveHelper.cheesyDrive(throttle, turn, mControls.getQuickTurn()));
            }
            
            if(mControls.getHighGear()) mDrive.setHighGear(true);
            else if(mControls.getLowGear()) mDrive.setHighGear(false);
            
           

            outputAllToSmartDashboard();
     
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void autonomousPeriodic() {
        try {
            outputAllToSmartDashboard();

        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    
  
    @Override
    public void testPeriodic() {
      
    }


   
}
