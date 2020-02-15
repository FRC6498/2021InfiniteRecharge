package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.CANCoder;
import com.kauailabs.navx.frc.AHRS;

import frc.robot.Constants;
import frc.robot.Kinematics;
import frc.robot.RobotState;
import frc.robot.loops.Loop;
import frc.robot.loops.Looper;
import frc.lib.util.AdaptivePurePursuitController;
import frc.lib.util.DriveSignal;
import frc.lib.util.Path;
import frc.lib.util.RigidTransform2d;
import frc.lib.util.Rotation2d;

import frc.lib.util.SynchronousPID;

import frc.lib.util.Path;
import frc.lib.util.Translation2d;
import frc.lib.util.Path.Waypoint;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The robot's drivetrain, which implements the Superstructure abstract class.
 * The drivetrain has several states and builds on the abstract class by
 * offering additional control methods, including control by path and velocity.
 * 
 * @see Subsystem.java
 */
public class Drive extends Subsystem {
    protected static final int kVelocityControlSlot = 0;
    protected static final int kBaseLockControlSlot = 1;

    private static Drive instance_ = new Drive();
    private double mLastHeadingErrorDegrees;

    public static Drive getInstance() {
        return instance_;
    }

    // The robot drivetrain's various states
    public enum DriveControlState {
        OPEN_LOOP, BASE_LOCKED, VELOCITY_SETPOINT, VELOCITY_HEADING_CONTROL, PATH_FOLLOWING_CONTROL
    }

    private  TalonFX leftMaster_, rightMaster_, leftChild_ ,rightChild_;
    //private CANCoder leftEncoder, rightEncoder;
    private boolean isBrakeMode_ = true;
    private Solenoid shifter_;

    private final AHRS gyro_;
    
   

    private DriveControlState driveControlState_;
    private VelocityHeadingSetpoint velocityHeadingSetpoint_;
    private AdaptivePurePursuitController pathFollowingController_;
    private SynchronousPID velocityHeadingPid_;

    // The main control loop (an implementation of Loop), which cycles
    // through different robot states
    private final Loop mLoop = new Loop() {
        @Override
        public void onStart() {
            setOpenLoop(DriveSignal.NEUTRAL);
            pathFollowingController_ = null;
            setBrakeMode(false);
        }

        @Override
        public void onLoop() {
            synchronized (Drive.this) {
               
                switch (driveControlState_) {
                case OPEN_LOOP:
                    return;
                case BASE_LOCKED:
                    return;
                case VELOCITY_SETPOINT:
                    // Talons are updating the control loop state
                    return;
                case VELOCITY_HEADING_CONTROL:
                    updateVelocityHeadingSetpoint();
                    return;
                case PATH_FOLLOWING_CONTROL:
                    updatePathFollower();
                    if (isFinishedPath()) {
                        stop();
                    }
                    break;
               
                default:
                    System.out.println("Unexpected drive control state: " + driveControlState_);
                    break;
                }
            }
        }

        @Override
        public void onStop() {
            setOpenLoop(DriveSignal.NEUTRAL);
        }
    };

    // The constructor instantiates all of the drivetrain components when the
    // robot powers up
    private Drive() {
        leftMaster_ = new TalonFX(Constants.kLeftDriveMasterId);
        leftChild_ = new TalonFX(Constants.kLeftDriveChildId);
        
        rightMaster_ = new TalonFX(Constants.kRightDriveMasterId);
        rightChild_ = new TalonFX(Constants.kRightDriveChildId);

        leftMaster_.configOpenloopRamp(Constants.kDriveRampRate);
        rightMaster_.configOpenloopRamp(Constants.kDriveRampRate);
        leftChild_.configOpenloopRamp(Constants.kDriveRampRate);
        rightChild_.configOpenloopRamp(Constants.kDriveRampRate);
       
        rightChild_.follow(rightMaster_);
        leftChild_.follow(leftMaster_);

        //leftEncoder = new CANCoder(Constants.kLeftEncoderId);
        //rightEncoder = new CANCoder(Constants.kRightEncoderId);



        gyro_ = new AHRS(SPI.Port.kMXP);
      
       
        // Get status at 100Hz
        leftMaster_.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 10);
        rightMaster_.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 10);


        shifter_=new Solenoid(Constants.shiftSolenoidId);

       
        
        setBrakeMode(false);

        // Set up the encoders
        leftMaster_.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        
        leftMaster_.setSensorPhase(false);
        leftMaster_.setInverted(false);
        leftChild_.setInverted(false);
        
        rightMaster_.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
       
        rightMaster_.setSensorPhase(false);
        rightMaster_.setInverted(true);
        rightChild_.setInverted(true);
        

        // Load velocity control gains
        leftMaster_ = tuneLoops(leftMaster_, kVelocityControlSlot, Constants.kDriveVelocityKp, Constants.kDriveVelocityKi, Constants.kDriveVelocityKd, Constants.kDriveVelocityKf, Constants.kDriveVelocityIZone, Constants.kDriveVelocityRampRate);
       
        rightMaster_= tuneLoops(rightMaster_, kVelocityControlSlot, Constants.kDriveVelocityKp, Constants.kDriveVelocityKi, Constants.kDriveVelocityKd, Constants.kDriveVelocityKf, Constants.kDriveVelocityIZone, Constants.kDriveVelocityRampRate);
        
        // Load base lock control gains
        leftMaster_= tuneLoops(leftMaster_, kBaseLockControlSlot, Constants.kDriveBaseLockKp, Constants.kDriveBaseLockKi, Constants.kDriveBaseLockKd, Constants.kDriveBaseLockKf, Constants.kDriveBaseLockIZone, Constants.kDriveBaseLockRampRate);
       
        rightMaster_=tuneLoops(rightMaster_, kBaseLockControlSlot, Constants.kDriveBaseLockKp, Constants.kDriveBaseLockKi, Constants.kDriveBaseLockKd, Constants.kDriveBaseLockKf, Constants.kDriveBaseLockIZone, Constants.kDriveBaseLockRampRate);
       
        velocityHeadingPid_ = new SynchronousPID(Constants.kDriveHeadingVelocityKp, Constants.kDriveHeadingVelocityKi,
                Constants.kDriveHeadingVelocityKd);
        velocityHeadingPid_.setOutputRange(-15,15);//-30, 30);

        setOpenLoop(DriveSignal.NEUTRAL);
    }

    public static TalonFX tuneLoops(TalonFX talon, int id, double P, double I, double D, double F,int Izone,double Ramp){

        talon.config_kP(id, P);
        talon.config_kI(id, I);
        talon.config_kD(id, D);
        talon.config_kF(id, F);
        talon.config_IntegralZone(id, Izone);
        talon.configClosedloopRamp(Ramp);
        

        return talon;
    }

    /*public Loop getLoop() {
        return mLoop;
    }*/

   /* protected synchronized void setLeftRightPower(double left, double right) {
        leftMaster_.set(left);
        rightMaster_.set(-right);
    }*/

    public synchronized void setOpenLoop(DriveSignal signal) {
        if (driveControlState_ != DriveControlState.OPEN_LOOP) {
           
            driveControlState_ = DriveControlState.OPEN_LOOP;
        }
        leftMaster_.set(ControlMode.PercentOutput, signal.leftMotor); 
        rightMaster_.set(ControlMode.PercentOutput, signal.rightMotor);
    }

    public synchronized void setBaseLockOn() {
        if (driveControlState_ != DriveControlState.BASE_LOCKED) {
            leftMaster_.selectProfileSlot(kBaseLockControlSlot, 0);
           // leftMaster_.changeControlMode(CANTalon.TalonControlMode.Position);
            leftMaster_.configAllowableClosedloopError(kBaseLockControlSlot, Constants.kDriveBaseLockAllowableError);
            leftMaster_.set(ControlMode.Position, leftMaster_.getSelectedSensorPosition());
            rightMaster_.selectProfileSlot(kBaseLockControlSlot, 0);
           // rightMaster_.changeControlMode(CANTalon.TalonControlMode.Position);
            rightMaster_.configAllowableClosedloopError(kBaseLockControlSlot, Constants.kDriveBaseLockAllowableError);
            rightMaster_.set(ControlMode.Position, rightMaster_.getSelectedSensorPosition());
            driveControlState_ = DriveControlState.BASE_LOCKED;
            setBrakeMode(true);
        }
        
    }

    public synchronized void setVelocitySetpoint(double left_inches_per_sec, double right_inches_per_sec) {
        configureTalonsForSpeedControl();
        driveControlState_ = DriveControlState.VELOCITY_SETPOINT;
        updateVelocitySetpoint(left_inches_per_sec, right_inches_per_sec);
    }

    public synchronized void setVelocityHeadingSetpoint(double forward_inches_per_sec, Rotation2d headingSetpoint) {
        if (driveControlState_ != DriveControlState.VELOCITY_HEADING_CONTROL) {
            configureTalonsForSpeedControl();
            driveControlState_ = DriveControlState.VELOCITY_HEADING_CONTROL;
            velocityHeadingPid_.reset();
        }
        velocityHeadingSetpoint_ = new VelocityHeadingSetpoint(forward_inches_per_sec, forward_inches_per_sec,
                headingSetpoint);
        updateVelocityHeadingSetpoint();
    }

    
    /**
     * The robot follows a set path, which is defined by Waypoint objects.
     * 
     * @param Path
     *            to follow
     * @param reversed
     * @see frc.lib.util/Path.java
     */
    public synchronized void followPath(Path path, boolean reversed) {
        if (driveControlState_ != DriveControlState.PATH_FOLLOWING_CONTROL) {
            configureTalonsForSpeedControl();
            driveControlState_ = DriveControlState.PATH_FOLLOWING_CONTROL;
            velocityHeadingPid_.reset();
        }
        pathFollowingController_ = new AdaptivePurePursuitController(Constants.kPathFollowingLookahead,
                Constants.kPathFollowingMaxAccel, Constants.kLooperDt, path, reversed, 0.25);
        updatePathFollower();
    }

    /**
     * @return Returns if the robot mode is Path Following Control and the set
     *         path is complete.
     */
    public synchronized boolean isFinishedPath() {
        return (driveControlState_ == DriveControlState.PATH_FOLLOWING_CONTROL && pathFollowingController_.isDone())
                || driveControlState_ != DriveControlState.PATH_FOLLOWING_CONTROL;
    }

    /**
     * Path Markers are an optional functionality that name the various
     * Waypoints in a Path with a String. This can make defining set locations
     * much easier.
     * 
     * @return Set of Strings with Path Markers that the robot has crossed.
     */
    public synchronized Set<String> getPathMarkersCrossed() {
        if (driveControlState_ != DriveControlState.PATH_FOLLOWING_CONTROL) {
            return null;
        } else {
            return pathFollowingController_.getMarkersCrossed();
        }
    }

    public double getLeftDistanceInches() {
        return rotationsToInches(leftMaster_.getSelectedSensorPosition()/Constants.kDriveTicksPerRotation);
    }

    public double getRightDistanceInches() {
        return rotationsToInches(rightMaster_.getSelectedSensorPosition()/Constants.kDriveTicksPerRotation);
    }

    public double getLeftVelocityInchesPerSec() {
        return rpmToInchesPerSecond(leftMaster_.getSelectedSensorVelocity()/Constants.kDriveTicksPerRotation*10*60);
    }

    public double getRightVelocityInchesPerSec() {
        return rpmToInchesPerSecond(rightMaster_.getSelectedSensorVelocity()/Constants.kDriveTicksPerRotation*10*60);
    }

    public AHRS getGyro() {
        return gyro_;
    }

    public synchronized Rotation2d getGyroAngle() {
        return Rotation2d.fromDegrees(-gyro_.getAngle());
    }

    boolean isHighGear_=false;
    public boolean isHighGear() {
        return isHighGear_;
    }

   public void setHighGear(boolean high_gear) {
        isHighGear_ = high_gear;
        shifter_.set(!high_gear);
    }

    public synchronized void resetEncoders() {
        leftMaster_.setSelectedSensorPosition(0);
        rightMaster_.setSelectedSensorPosition(0);
    }

    public synchronized DriveControlState getControlState() {
        return driveControlState_;
    }

    public synchronized VelocityHeadingSetpoint getVelocityHeadingSetpoint() {
        return velocityHeadingSetpoint_;
    }

    @Override
    public synchronized void stop() {
        setOpenLoop(DriveSignal.NEUTRAL);
    }

    @Override
    public void outputToSmartDashboard() {
        SmartDashboard.putNumber("left_distance", getLeftDistanceInches());
        SmartDashboard.putNumber("right_distance", getRightDistanceInches());
        SmartDashboard.putNumber("left_velocity", getLeftVelocityInchesPerSec());
        SmartDashboard.putNumber("right_velocity", getRightVelocityInchesPerSec());
        SmartDashboard.putNumber("left_error", leftMaster_.getClosedLoopError());
        SmartDashboard.putNumber("right_error", rightMaster_.getClosedLoopError());
        SmartDashboard.putNumber("gyro_angle", getGyro().getAngle());
        if(getControlState() == DriveControlState.VELOCITY_HEADING_CONTROL) SmartDashboard.putNumber("heading set point", getVelocityHeadingSetpoint().headingSetpoint_.getDegrees());
        //SmartDashboard.putNumber("gyro_center", getGyro().getCenter());
        SmartDashboard.putNumber("heading_error", mLastHeadingErrorDegrees);
     
       // SmartDashboard.putBoolean("line_sensor1", lineSensor1_.get());
        //SmartDashboard.putBoolean("line_sensor2", lineSensor2_.get());
    }

    @Override
    public synchronized void zeroSensors() {
        resetEncoders();
        gyro_.reset();
    }

    private void configureTalonsForSpeedControl() {
        if (driveControlState_ != DriveControlState.VELOCITY_HEADING_CONTROL
                && driveControlState_ != DriveControlState.VELOCITY_SETPOINT
                && driveControlState_ != DriveControlState.PATH_FOLLOWING_CONTROL) {
            //leftMaster_.changeControlMode(CANTalon.TalonControlMode.Speed);
            leftMaster_.selectProfileSlot(kVelocityControlSlot,0);
            leftMaster_.configAllowableClosedloopError(kVelocityControlSlot,Constants.kDriveVelocityAllowableError);
            //rightMaster_.changeControlMode(CANTalon.TalonControlMode.Speed);
            rightMaster_.selectProfileSlot(kVelocityControlSlot,0);
            rightMaster_.configAllowableClosedloopError(kVelocityControlSlot,Constants.kDriveVelocityAllowableError);
           
            setBrakeMode(true);
        }
    }

    private synchronized void updateVelocitySetpoint(double left_inches_per_sec, double right_inches_per_sec) {
        if (driveControlState_ == DriveControlState.VELOCITY_HEADING_CONTROL
                || driveControlState_ == DriveControlState.VELOCITY_SETPOINT
                || driveControlState_ == DriveControlState.PATH_FOLLOWING_CONTROL
                ) {
            leftMaster_.set(ControlMode.Velocity,inchesPerSecondToRpm(left_inches_per_sec)/60/10*Constants.kDriveTicksPerRotation);
            rightMaster_.set(ControlMode.Velocity, inchesPerSecondToRpm(right_inches_per_sec)/60/10*Constants.kDriveTicksPerRotation);
        } else {
            System.out.println("Hit a bad velocity control state");
            leftMaster_.set(ControlMode.Disabled,0);
            rightMaster_.set(ControlMode.Disabled,0);
        }
    }

    private void updateVelocityHeadingSetpoint() {
        Rotation2d actualGyroAngle = getGyroAngle();

        mLastHeadingErrorDegrees = velocityHeadingSetpoint_.getHeading().rotateBy(actualGyroAngle.inverse())
                .getDegrees();

        double deltaSpeed = velocityHeadingPid_.calculate(mLastHeadingErrorDegrees);
        updateVelocitySetpoint(velocityHeadingSetpoint_.getLeftSpeed() + deltaSpeed / 2,
                velocityHeadingSetpoint_.getRightSpeed() - deltaSpeed / 2);
    }

    private void updatePathFollower() {
        RigidTransform2d robot_pose = RobotState.getInstance().getLatestFieldToVehicle().getValue();
        RigidTransform2d.Delta command = pathFollowingController_.update(robot_pose, Timer.getFPGATimestamp());
        Kinematics.DriveVelocity setpoint = Kinematics.inverseKinematics(command);

        // Scale the command to respect the max velocity limits
        double max_vel = 0.0;
        max_vel = Math.max(max_vel, Math.abs(setpoint.left));
        max_vel = Math.max(max_vel, Math.abs(setpoint.right));
        if (max_vel > Constants.kPathFollowingMaxVel) {
            double scaling = Constants.kPathFollowingMaxVel / max_vel;
            setpoint = new Kinematics.DriveVelocity(setpoint.left * scaling, setpoint.right * scaling);
        }
        updateVelocitySetpoint(setpoint.left, setpoint.right);
    }

  
   


    private static double rotationsToInches(double rotations) {
        return rotations * (Constants.kDriveWheelDiameterInches * Math.PI);
    }

    private static double rpmToInchesPerSecond(double rpm) {
        return rotationsToInches(rpm) / 60;
    }

    private static double inchesToRotations(double inches) {
        return inches / (Constants.kDriveWheelDiameterInches * Math.PI);
    }

    private static double inchesPerSecondToRpm(double inches_per_second) {
        return inchesToRotations(inches_per_second) * 60;
    }

    public void setBrakeMode(boolean on) {
        if (isBrakeMode_ != on) {
            if(on){
                leftMaster_.setNeutralMode(NeutralMode.Brake);
                rightMaster_.setNeutralMode(NeutralMode.Brake);
            }else{
                leftMaster_.setNeutralMode(NeutralMode.Coast);
                rightMaster_.setNeutralMode(NeutralMode.Coast);
            }
          
            isBrakeMode_ = on;
        }
    }

    /**
     * VelocityHeadingSetpoints are used to calculate the robot's path given the
     * speed of the robot in each wheel and the polar coordinates. Especially
     * useful if the robot is negotiating a turn and to forecast the robot's
     * location.
     */
    public static class VelocityHeadingSetpoint {
        private final double leftSpeed_;
        private final double rightSpeed_;
        private final Rotation2d headingSetpoint_;

        // Constructor for straight line motion
        public VelocityHeadingSetpoint(double leftSpeed, double rightSpeed, Rotation2d headingSetpoint) {
            leftSpeed_ = leftSpeed;
            rightSpeed_ = rightSpeed;
            headingSetpoint_ = headingSetpoint;
        }

        public double getLeftSpeed() {
            return leftSpeed_;
        }

        public double getRightSpeed() {
            return rightSpeed_;
        }

        public Rotation2d getHeading() {
            return headingSetpoint_;
        }
    }

    @Override
    public void registerEnabledLoops(Looper in) {
        in.register(mLoop);
    }
}
