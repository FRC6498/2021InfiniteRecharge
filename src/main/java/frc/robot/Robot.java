package frc.robot;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.util.CrashTracker;
import frc.lib.util.DriveSignal;
import frc.lib.util.RigidTransform2d;
import frc.lib.util.Rotation2d;
import frc.robot.StateMachines.Shooter;
import frc.robot.Vision.TurretCam;
import frc.robot.Vision.TurretCam.LightMode;
import frc.robot.auto.AutoModeExecuter;
import frc.robot.auto.modes.ClimbMode;
import frc.robot.loops.Looper;
import frc.robot.loops.RobotStateEstimator;
import frc.robot.loops.VisionProcessor;
import frc.robot.subsystems.*;


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
   Intake mIntake = Intake.getInstance();
   Flywheel mFlywheel = Flywheel.getInstance();
   FeederFlywheel mFeederFlywheel = FeederFlywheel.getInstance();
   Lift mLift = Lift.getInstance();
   Winch mWinch = Winch.getInstance();
   Leveller mLeveller = Leveller.getInstance();


    // Other parts of the robot
    CheesyDriveHelper mCheesyDriveHelper = new CheesyDriveHelper();
    ControlBoard mControls = ControlBoard.getInstance();

    RobotState mRobotState = RobotState.getInstance();

        List<Subsystem> subsystemsList = Arrays.asList(Drive.getInstance(), mFlywheel, 
        Hood.getInstance(), Turret.getInstance(), mShooter, mIntake, BeltClamp.getInstance(), 
        Indexer.getInstance(), FeederBelt.getInstance(), mFeederFlywheel, mLift, mWinch, mLeveller);
       // Create subsystem manager
       private final SubsystemManager mSubsystemManager = new SubsystemManager(subsystemsList);
   
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

           // TurretCam.setLedMode(LightMode.eOff);

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

            mAutoModeExecuter = new AutoModeExecuter();
            mAutoModeExecuter.setAutoMode(new ClimbMode());
            

            // Reset drive
            mDrive.resetEncoders();

           
           c.start();

           zeroAllSensors();

            // Configure loopers
            //mDisabledLooper.stop();
            mEnabledLooper.start();
            mDrive.setOpenLoop(DriveSignal.NEUTRAL);
            mDrive.setBrakeMode(false);

            mShooter.setHoodAdjustment(mSmartDashboardInteractions.areAutoBallsWorn()
            ? Constants.kOldBallHoodAdjustment : Constants.kNewBallHoodAdjustment);



            if (mHoodTuningMode) {
                mShooter.setTuningMode(true);
              /*  if (mControls.getHoodTuningPositiveButton()) {
                    mShooter.setHoodManualScanOutput(0.4);
                } else if (mControls.getHoodTuningNegativeButton()) {
                    mShooter.setHoodManualScanOutput(-0.4);
                } else {
                    mShooter.setHoodManualScanOutput(0.0);
                }*/
            } else {
                mShooter.setTuningMode(false);
            }

            
           
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

            //zeroAllSensors();
        
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
boolean climbing=false;
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
            if(!climbing){
            double throttle = mControls.getThrottle();
            double turn = mCheesyDriveHelper.handleDeadband(mControls.getTurn(), .05) ;
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
               // mDrive.setBrakeMode(false);
           /*     double offset=0;
                if(Math.abs(turn)>.05) offset=-turn*1;
                Rotation2d  heading_setpoint;
              //  Rotation2d heading_setpoint = Rotation2d.fromDegrees(mDrive.getGyroAngle().getDegrees()+offset);
                if (mDrive.getControlState() == Drive.DriveControlState.VELOCITY_HEADING_CONTROL) {
                   heading_setpoint = mDrive.getVelocityHeadingSetpoint().getHeading().rotateBy(Rotation2d.fromDegrees(offset));
                }else{
                    heading_setpoint=mDrive.getGyroAngle();
                }
                mDrive.setVelocityHeadingSetpoint(
                        mCheesyDriveHelper.handleDeadband(throttle, CheesyDriveHelper.kThrottleDeadband)
                                * Constants.kDriveLowGearMaxSpeedInchesPerSec,
                        heading_setpoint);
                

                */
                //mDrive.setHighGear(!mControls.getLowGear());
                mDrive.setOpenLoop(mControls.getDriveSignal());//mCheesyDriveHelper.cheesyDrive(throttle, turn, mControls.getQuickTurn()));
            }

            if(mControls.getLowGear()) mDrive.setHighGear(false);
            else if(mControls.getHighGear()) mDrive.setHighGear(true);
            

            if(mControls.getIntake()) mIntake.setWantedState(Intake.WantedState.WANT_INTAKE_GROUND);
            else if(mControls.getStopIntake()) mIntake.setWantedState(Intake.WantedState.WANT_IDLE);
            else if(mControls.getPlow()) mIntake.setWantedState(Intake.WantedState.WANT_PLOW);
            

            if (mControls.getAutoAim()) {
                mShooter.setWantedState(Shooter.WantedState.WANT_AUTO);
            }else if(mControls.getShooterOpenLoop()){
                mShooter.setWantedState(Shooter.WantedState.WANT_OPEN_LOOP);
            } else if(mControls.getStopShooter()){
                mShooter.setWantedState(Shooter.WantedState.WANT_IDLE);
            }else if(mControls.getShooterDumpToTrench()){
                mShooter.setWantedState(Shooter.WantedState.WANT_DUMP_TO_TRENCH);
            }

            if(mControls.getShooterFireOneWhenReady()){
                mShooter.setWantsToFireIfReady(Shooter.WantedFiringAmount.WANT_FIRE_ONE);
            }else if(mControls.getShooterFireOneNow()){
                mShooter.setWantsToFireNow(Shooter.WantedFiringAmount.WANT_FIRE_ONE);
            }

            if(mControls.getAutoShootOn()) mShooter.setAutoShoot(true);
            else if(mControls.getAutoShootoff()) mShooter.setAutoShoot(false);
            
            mShooter.setTurretManualScanOutput(mControls.getTurretManual() * .12);

            

          /*  if (mControls.getHoodTuningPositiveButton()) {
                mShooter.setHoodManualScanOutput(0.4);
            } else if (mControls.getHoodTuningNegativeButton()) {
                mShooter.setHoodManualScanOutput(-0.4);
            } else {
                mShooter.setHoodManualScanOutput(0.0);
            }*/

            mShooter.setHoodManualScanOutput(mControls.getHoodTuningAdjustment());

            if(mControls.addBeltBall()) mRobotState.setIntakeBalls(1);
            else if(mControls.subtractBeltBall()) mRobotState.setIntakeBalls(-1);

            if(mControls.addFeederBall()) mRobotState.setFeederBalls(1);
            else if(mControls.subtractFeederBall()) mRobotState.setFeederBalls(-1);

            if(mControls.fillBalls()) mRobotState.fillBalls();

            if(mControls.addCSVValue()) mShooter.addHoodCSV();
        }
            if(mControls.getStartClimb()){
                 mAutoModeExecuter.start();
                climbing=true;
            }
            else if(mControls.getStopClimb()){
                 mAutoModeExecuter.stop();
                 climbing=false;
            }

            mLeveller.set(mControls.getBalanceJog());
        
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

   
   // int period = 0;
    @Override
    public void testInit() {
    


    }
  
    @Override
    public void testPeriodic() {
        
        mLeveller.set(mControls.getBalanceJog());
        if(Math.abs(mControls.getLiftJog())>.1)mLift.setOpenLoop(mControls.getLiftJog());
        else if(mControls.getStartClimb()) mLift.setDesiredHeight(35);
        else mLift.setOpenLoop(0);


        if(Math.abs(mControls.getWinchJog())>.1)mWinch.setOpenLoop(mControls.getWinchJog());
        else if(mControls.getStopClimb()) mWinch.setDesiredHeight(10);
        else mWinch.setOpenLoop(0);


        allPeriodic();
    }

  /**
     * Helper function that is called in all periodic functions
     */
    public void allPeriodic() {
        
        mSubsystemManager.outputToSmartDashboard();
        if(mHoodTuningMode) mSubsystemManager.writeToLog();
        mEnabledLooper.outputToSmartDashboard();
        mRobotState.outputToSmartDashboard();
        
    }
   
}
