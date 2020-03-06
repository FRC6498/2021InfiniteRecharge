/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.loops.Loop;
import frc.robot.loops.Looper;


/**
 * The Lift controls the ascent/descent of the elevator that carries the levelling unit.
 */
public class Lift extends Subsystem
{
    private TalonFX falcon_;
    private AnalogInput sensor_;


    private static Lift instance_ = new Lift();

    public static Lift getInstance() {
        return instance_;
    }

 

    Loop mLoop = new Loop() {
       
  
      

        @Override
        public void onLoop() {
            synchronized (Lift.this) {
                
                seesBarUpdater();
               
            }
        }

        @Override
        public void onStart() {
            synchronized (Lift.this) {
                
            }
        }

        @Override
        public void onStop() {
            synchronized (Lift.this) {
               stop();
            }
        }
    };


    private Lift() {
        // The turret has one Talon.
        falcon_ = new TalonFX(Constants.kLiftFalconId);
        falcon_.setNeutralMode(NeutralMode.Brake);
        //falcon_.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector,LimitSwitchNormal.NormallyOpen);
  
      
        falcon_.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 10);//.setStatusFrameRateMs(StatusFrameRate.Feedback, 10);
        falcon_.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);//setFeedbackDevice(CANTalon.FeedbackDevice.CtreMagEncoder_Relative);
      

        falcon_.config_kP(0, Constants.kLiftFalconKp);
        falcon_.config_kI(0, Constants.kLiftFalconKi);
        falcon_.config_kD(0, Constants.kLiftFalconKd);
        falcon_.config_kF(0, Constants.kLiftFalconKf);
       //falcon_.config_IntegralZone(0, Constants.kLiftFalcon);
        falcon_.configClosedloopRamp(.1);
        falcon_.configOpenloopRamp(.1);

        falcon_.configMotionAcceleration(Constants.kLiftMaxAcceleration);
        falcon_.configMotionCruiseVelocity(Constants.kLiftMaxVelocity);


       
      // falcon_.configPeakOutputReverse(-Constants.kTurretMaxPercentOut);
      //  falcon_.configPeakOutputForward(Constants.kTurretMaxPercentOut);

       // falcon_.configNominalOutputForward(0);
     //  falcon_.configNominalOutputReverse(0);

       // falcon_.configAllowableClosedloopError(0, Constants.kTurretAllowableError);

        
      //  falcon_.sensorre(true);

     //   falcon_.setInverted(TalonFXInvertType.Clockwise);

        // We use soft limits to make sure the turret doesn't try to spin too
        // far.
        falcon_.configForwardSoftLimitEnable(true);//.enableForwardSoftLimit(true);
       falcon_.configReverseSoftLimitEnable(true);
        falcon_.configForwardSoftLimitThreshold((int) ((Constants.kLiftSoftLimit-Constants.kLiftStartingHeight)*Constants.kLiftTicksPerInch));
        falcon_.configReverseSoftLimitThreshold((int) 0);

        reset();

        sensor_ = new AnalogInput(Constants.kBarSensorChannel);
    }




    public synchronized void setDesiredHeight(double height) {
        //talon_.changeControlMode(CANTalon.TalonControlMode.Position);

        falcon_.set(ControlMode.MotionMagic, (height-Constants.kLiftStartingHeight)*Constants.kLiftTicksPerInch);
    }

    // Manually move the turret (and put it into vbus mode if it isn't already).
    public synchronized void setOpenLoop(double speed) {
        //talon_.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
        falcon_.set(ControlMode.PercentOutput, speed);
    }

    // Tell the Talon it is at a given position.
    public synchronized void reset() {
        falcon_.setSelectedSensorPosition(0);
    }

    public synchronized double getHeight() {
        return falcon_.getSelectedSensorPosition()/Constants.kLiftTicksPerInch + Constants.kLiftStartingHeight;
    }

    public synchronized double getSetpoint() {
       if(falcon_.getControlMode() == ControlMode.MotionMagic) return falcon_.getClosedLoopTarget() / Constants.kLiftTicksPerInch+Constants.kLiftStartingHeight;
       else return 0;
    }

    private synchronized double getError() {
        return getHeight() - getSetpoint();
    }

    // We are "OnTarget" if we are in position mode and close to the setpoint.
    public synchronized boolean isOnTarget() {
        return (Math.abs(getError()) < Constants.kLiftTargetThreshold);
    }

    boolean seesBarValue=false;
    public boolean seesBarRaw(){
        return sensor_.getVoltage()>Constants.kBarSensorThreshold;
    }

    double seenTime = 0;
    private void seesBarUpdater(){

        double now = Timer.getFPGATimestamp();

        if(seesBarRaw()) {
          if(seenTime==0) seenTime=now;
  
  
        }else{
          seenTime=0;
        }
  
          seesBarValue = seenTime!=0&&(now-seenTime)>Constants.kBarSensorTimeThreshold;
          
    }

    public synchronized boolean seesBar(){
        return seesBarValue;
    }

    @Override
    public synchronized void stop() {
        setOpenLoop(0);
    }

    @Override
    public void outputToSmartDashboard() {
        SmartDashboard.putNumber("lift_error", getError());
        SmartDashboard.putNumber("lift_height", getHeight());
        SmartDashboard.putNumber("lift_setpoint", getSetpoint());
        SmartDashboard.putBoolean("lift_on_target", isOnTarget());
        SmartDashboard.putNumber("lift_bar_sensor", sensor_.getVoltage());
        SmartDashboard.putBoolean("lift_sees_bar", seesBar());
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

