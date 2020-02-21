package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.loops.Loop;
import frc.robot.loops.Looper;
import frc.lib.util.ContinuousRotationServo;
import frc.lib.util.MA3AnalogEncoder;

import frc.lib.util.Rotation2d;
import frc.lib.util.SynchronousPID;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The Hood assembly controls the launch angle of the ball. Additionally, it
 * must be lowered whenever the robot is crossing a defense. There are several
 * parameters accessible to the rest of the robot code: the hood angle, whether
 * or not the hood is stowed.
 * 
 * The ball is first picked up with the Intake then is fed to the Flywheel with
 * the HoodRoller. The Turret controls the direction that the ball is fired at.
 * Finally, the Hood controls the output angle and, conversely, trajectory.
 * 
 * This is a member of the Superstructure superclass.
 * 
 * @see Flywheel
 * @see Intake
 * @see HoodRoller
 * @see Turret
 * @see Superstructure
 */

 
public class Hood extends Subsystem {
    private ContinuousRotationServo servo_;
    
    private MA3AnalogEncoder encoder_;
    private boolean has_homed_;
    private SynchronousPID pid_;

    private static Hood instance_ = new Hood();

    public static Hood getInstance() {
        return instance_;
    }


    enum ControlMode {
        HOMING, OPEN_LOOP, POSITION
    }

    ControlMode control_mode_=ControlMode.HOMING;

    Loop mLoop = new Loop() {
        static final double kHomingTimeSeconds = 2.0;
        ControlMode last_iteration_control_mode_ = ControlMode.POSITION;
        double homing_start_time_ = 0;

        @Override
        public void onLoop() {
            synchronized (Hood.this) {
                if (control_mode_ == ControlMode.HOMING) {
                    if (control_mode_ != last_iteration_control_mode_) {
                        startHoming();
                        homing_start_time_ = Timer.getFPGATimestamp();
                    } else if (Timer.getFPGATimestamp() >= homing_start_time_ + kHomingTimeSeconds) {
                        stopHoming(true);
                    }
                } else if (control_mode_ == ControlMode.POSITION) {
                    set(pid_.calculate(getAngle()));
                }
                last_iteration_control_mode_ = control_mode_;
            }
        }

        @Override
        public void onStart() {
            synchronized (Hood.this) {
                if (!has_homed_) {
                    control_mode_ = ControlMode.HOMING;
                }
                pid_.setSetpoint(Constants.kHoodNeutralAngle);
            }
        }

        @Override
        public void onStop() {
            synchronized (Hood.this) {
                if (control_mode_ == ControlMode.HOMING) {
                    stopHoming(false);
                }
            }
        }
    };

    private Hood() {
        servo_ = new ContinuousRotationServo(Constants.kHoodServoPWM);

        
 
        encoder_ = new MA3AnalogEncoder(Constants.kHoodEncoderAnalog);

        

        pid_ = new SynchronousPID(Constants.kHoodKp, Constants.kHoodKi, Constants.kHoodKd);
        pid_.setDeadband(Constants.kHoodDeadband);
        pid_.setInputRange(Constants.kMinHoodAngle, Constants.kMaxHoodAngle);
        

        has_homed_ = false;
        pid_.setSetpoint(Constants.kHoodNeutralAngle);
        control_mode_ = ControlMode.OPEN_LOOP;
    }

   

    /**
     * Sets the angle of the hood to a specified angle.
     * 
     * @param A
     *            set angle
     */
    public synchronized void setDesiredAngle(Rotation2d angle) {
        if (control_mode_ != ControlMode.HOMING && control_mode_ != ControlMode.POSITION) {
            control_mode_ = ControlMode.POSITION;
            pid_.reset();
        }
        pid_.setSetpoint(angle.getDegrees());
    }

    /**
     * Gets the current angle of the hood.
     * 
     * @return The hood's current angle.
     */
    public synchronized double getAngle() {
        return -encoder_.getContinuousAngleDegrees() *20 / 560 + Constants.kMinHoodAngle;
    }

    private synchronized void set(double power) {
        power=-power;
        if(control_mode_==ControlMode.HOMING) servo_.set(power);
        else{
            double angle = getAngle();
            if(power<0&&angle<Constants.kMaxHoodAngle-1) servo_.set(power); //going up

            else if(power>0&&angle>Constants.kMinHoodAngle+1) servo_.set(power); //going down

            else servo_.set(0);


        }

    }

    public synchronized void setOpenLoop(double power) {
        if (control_mode_ != ControlMode.HOMING) {


            set(power);
            control_mode_ = ControlMode.OPEN_LOOP;
        }
    }

    /**
     * Sets the hood state such that it begins retracting
     */
    public synchronized void homeSystem() {
        control_mode_ = ControlMode.HOMING;
    }

    /**
     * Makes the hood assembly begin to retract, or home.
     */
    synchronized void startHoming() {
        control_mode_ = ControlMode.HOMING;
        set(-0.9);
    }

    /**
     * Changes the control state of the Hood assembly if the hood is fully
     * retracted. If not, the Hood state is set to Open Loop.
     * 
     * @param If
     *            the hood has fully retracted.
     */
    synchronized void stopHoming(boolean success) {
        if (success) {
            has_homed_ = true;
            control_mode_ = ControlMode.POSITION;
            encoder_.zero();
        } else {
            control_mode_ = ControlMode.OPEN_LOOP;
        }
        set(0);
    }

    public synchronized boolean hasHomed() {
        return has_homed_;
    }

    public synchronized double getSetpoint() {
        return pid_.getSetpoint();
    }

    /**
     * @return If the hood position is within a set tolerance to a specified
     *         value.
     */
    public synchronized boolean isOnTarget() {
        return (has_homed_ && control_mode_ == ControlMode.POSITION
                && Math.abs(pid_.getError()) < Constants.kHoodOnTargetTolerance);
    }

  

    @Override
    public void outputToSmartDashboard() {
        SmartDashboard.putBoolean("has_hood_homed", has_homed_);
        SmartDashboard.putNumber("hood_angle", getAngle());
        SmartDashboard.putNumber("hood_setpoint", pid_.getSetpoint());
        SmartDashboard.putBoolean("hood_on_target", isOnTarget());
        SmartDashboard.putNumber("hood_error", pid_.getSetpoint() - getAngle());
    }

    @Override
    public synchronized void stop() {
        pid_.reset();
        
        control_mode_ = ControlMode.OPEN_LOOP;
        set(0);
    }

    @Override
    public synchronized void zeroSensors() {
        //encoder_.zero();
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
}
