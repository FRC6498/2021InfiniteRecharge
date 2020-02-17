package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.loops.Loop;
import frc.robot.loops.Looper;
import frc.lib.util.Rotation2d;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The Turret subsystem controls the direction the ball is fired. On the Turret
 * assembly is the Hood and Flywheel. The Turret can only rotate within 240
 * degrees (+/- 120), and mechanical bumper switches indicate when the
 * mechanical limits are reached. This is part of the Superstructure superclass.
 * 
 * The ball is first picked up with the Intake then is fed to the Flywheel with
 * the HoodRoller. The Turret controls the direction that the ball is fired at.
 * Finally, the Hood controls the output angle and trajectory of the shot.
 * 
 * @see Flywheel
 * @see Hood
 * @see HoodRoller
 * @see Intake
 * @see Superstructure
 */
public class Turret extends Subsystem {
    private TalonFX falcon_;

    private boolean has_homed_=false;

    private static Turret instance_ = new Turret();

    public static Turret getInstance() {
        return instance_;
    }

 

    Loop mLoop = new Loop() {
       
  
      

        @Override
        public void onLoop() {
            synchronized (Turret.this) {
                
               
               if(!has_homed_) handleHoming();

                checkLimits();
            }
        }

        @Override
        public void onStart() {
            synchronized (Turret.this) {
               if(!has_homed_) startHoming();
                
            }
        }

        @Override
        public void onStop() {
            synchronized (Turret.this) {
               stop();
            }
        }
    };


    private Turret() {
        // The turret has one Talon.
        falcon_ = new TalonFX(Constants.kTurretFalconId);
        falcon_.setNeutralMode(NeutralMode.Brake);
        falcon_.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector,LimitSwitchNormal.NormallyOpen);
  
      
        falcon_.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 10);//.setStatusFrameRateMs(StatusFrameRate.Feedback, 10);
        falcon_.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);//setFeedbackDevice(CANTalon.FeedbackDevice.CtreMagEncoder_Relative);
      

        falcon_.config_kP(0, Constants.kTurretKp);
        falcon_.config_kI(0, Constants.kTurretKi);
        falcon_.config_kD(0, Constants.kTurretKd);
        falcon_.config_kF(0, Constants.kTurretKf);
        falcon_.config_IntegralZone(0, Constants.kTurretIZone);
        falcon_.configClosedloopRamp(Constants.kTurretRampRate);
        falcon_.configOpenloopRamp(Constants.kTurretRampRate);

        falcon_.configPeakOutputReverse(-Constants.kTurretMaxPercentOut);
        falcon_.configPeakOutputForward(Constants.kTurretMaxPercentOut);

        falcon_.configNominalOutputForward(0);
        falcon_.configNominalOutputReverse(0);

        falcon_.configAllowableClosedloopError(0, Constants.kTurretAllowableError);

        
      //  falcon_.sensorre(true);

        falcon_.setInverted(TalonFXInvertType.Clockwise);

        // We use soft limits to make sure the turret doesn't try to spin too
        // far.
        falcon_.configForwardSoftLimitEnable(true);//.enableForwardSoftLimit(true);
        falcon_.configReverseSoftLimitEnable(true);
        falcon_.configForwardSoftLimitThreshold((int) (Constants.kSoftMaxTurretAngle / (360.0 / Constants.kTurretTicksPerRotation)));//setForwardSoftLimit(Constants.kSoftMaxTurretAngle / (360.0 * Constants.kTurretRotationsPerTick));
        falcon_.configReverseSoftLimitThreshold((int) (Constants.kSoftMinTurretAngle / (360.0 / Constants.kTurretTicksPerRotation)));//setReverseSoftLimit(Constants.kSoftMinTurretAngle / (360.0 * Constants.kTurretRotationsPerTick));
    }

    


    
    private synchronized void startHoming(){
        has_homed_=false;
            setOpenLoop(-.12);
            falcon_.overrideSoftLimitsEnable(false);
    }

    private synchronized void handleHoming(){

        if(checkLimits()){
            setOpenLoop(0);
            has_homed_=true;
            falcon_.overrideSoftLimitsEnable(true);
            setDesiredAngle(Rotation2d.fromDegrees(0));
        }

    }
  




    // Set the desired angle of the turret (and put it into position control
    // mode if it isn't already).
    public synchronized void setDesiredAngle(Rotation2d angle) {
        //talon_.changeControlMode(CANTalon.TalonControlMode.Position);

        if(has_homed_) falcon_.set(ControlMode.Position, angle.getRadians() / (2 * Math.PI / Constants.kTurretTicksPerRotation));
    }

    // Manually move the turret (and put it into vbus mode if it isn't already).
    public synchronized void setOpenLoop(double speed) {
        //talon_.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
        falcon_.set(ControlMode.PercentOutput, speed);
    }

    // Tell the Talon it is at a given position.
    public synchronized void reset(Rotation2d actual_rotation) {
        falcon_.setSelectedSensorPosition((int) (actual_rotation.getRadians() / (2 * Math.PI / Constants.kTurretTicksPerRotation))); //setPosition(actual_rotation.getRadians() / (2 * Math.PI * Constants.kTurretRotationsPerTick));
    }

    public synchronized Rotation2d getAngle() {
        return Rotation2d.fromRadians( falcon_.getSelectedSensorPosition()/Constants.kTurretTicksPerRotation * 2 * Math.PI);
    }

    private synchronized boolean checkLimits(){
        if (getForwardLimitSwitch()) {
            reset(Rotation2d.fromDegrees(Constants.kHardMaxTurretAngle));
            System.out.println("Turret max");
            return true;
        } else if (getReverseLimitSwitch()) {
            reset(Rotation2d.fromDegrees(Constants.kHardMinTurretAngle));
            System.out.println("Turret min");
            return true;
        }else return false;
    }

    public synchronized boolean getForwardLimitSwitch() {
        return (falcon_.isFwdLimitSwitchClosed()==1) ? true : false;
    }

    public synchronized boolean getReverseLimitSwitch() {
        return (falcon_.isRevLimitSwitchClosed()==1) ? true : false;
    }

    public synchronized double getSetpoint() {
       if(falcon_.getControlMode() == ControlMode.Position) return falcon_.getClosedLoopTarget() / Constants.kTurretTicksPerRotation * 360.0;
       else return 0;
    }

    private synchronized double getError() {
        return getAngle().getDegrees() - getSetpoint();
    }

    // We are "OnTarget" if we are in position mode and close to the setpoint.
    public synchronized boolean isOnTarget() {
        return (Math.abs(getError()) < Constants.kTurretOnTargetTolerance);
    }

   

    @Override
    public synchronized void stop() {
        setOpenLoop(0);
    }

    @Override
    public void outputToSmartDashboard() {
        SmartDashboard.putNumber("turret_error", getError());
        SmartDashboard.putNumber("turret_angle", getAngle().getDegrees());
        SmartDashboard.putNumber("turret_raw_angle", falcon_.getSelectedSensorPosition());
        SmartDashboard.putNumber("turret_setpoint", getSetpoint());
        SmartDashboard.putBoolean("turret_fwd_limit", getForwardLimitSwitch());
        SmartDashboard.putBoolean("turret_rev_limit", getReverseLimitSwitch());
        SmartDashboard.putBoolean("turret_on_target", isOnTarget());
    }

    @Override
    public void zeroSensors() {
       // reset(new Rotation2d());
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
