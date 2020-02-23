package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.loops.Looper;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The flywheel has several parameters: the RPM speed, the setpoint, and the RPM
 * tolerance. When told to, the flywheel will try to spin up to the setpoint
 * within the set RPM tolerance.
 * 
 * The ball is first picked up with the Intake then is fed to the Flywheel with
 * the HoodRoller. The Turret controls the direction that the ball is fired at.
 * Finally, the Hood controls the output angle and, conversely, trajectory.
 * 
 * This is a member of the Superstructure superclass.
 * 
 * @see Intake
 * @see Hood
 * @see HoodRoller
 * @see Turret
 * @see Superstructure
 */

 
public class Flywheel extends Subsystem {
    CANSparkMax neo_;
    CANEncoder encoder_;
    CANPIDController pid_;

    private static Flywheel instance_ = new Flywheel();

    public static Flywheel getInstance() {
        return instance_;
    }

    private Flywheel() {
        neo_ = new CANSparkMax(Constants.kFlyWheelNeoId, MotorType.kBrushless);
        encoder_ = neo_.getEncoder();
       
        pid_ = neo_.getPIDController();
  
      /*  pid_.setP(Constants.kFlywheelKp);
        pid_.setI(Constants.kFlywheelKi);
        pid_.setD(Constants.kFlywheelKd);
        pid_.setFF(Constants.kFlywheelKf);
        pid_.setIZone(Constants.kFlywheelIZone);
        neo_.setClosedLoopRampRate(Constants.kFlywheelRampRate);
        neo_.setOpenLoopRampRate(Constants.kFlywheelRampRate);
*/
        neo_.setIdleMode(IdleMode.kCoast);

    }

    public synchronized double getRpm() {
        return encoder_.getVelocity();
    }
    double rpmSetpoint=0;
    /**
     * Sets the RPM of the flywheel. The flywheel will then spin up to the set
     * speed within a preset tolerance.
     * 
     * @param Set
     *            flywheel RPM
     */
   public  synchronized void setRpm(double rpm) {
        pid_.setReference(rpm, ControlType.kVelocity);
        rpmSetpoint=rpm;
    }

    synchronized void setOpenLoop(double speed) {
        neo_.set(speed);
    }

    public synchronized double getSetpoint() {
        return rpmSetpoint;
    }

    /**
     * @return If the flywheel RPM is within the tolerance to the specified set
     *         point.
     */
    public synchronized boolean isOnTarget() {
        return (Math.abs(getRpm() - getSetpoint()) < Constants.kFlywheelOnTargetTolerance);
    }

    @Override
    public synchronized void stop() {
        setOpenLoop(0);
    }

    @Override
    public void outputToSmartDashboard() {
        //SmartDashboard.putNumber("flywheel_rpm", getRpm());
       // SmartDashboard.putNumber("flywheel_setpoint", getSetpoint());
       // SmartDashboard.putBoolean("flywheel_on_target", isOnTarget());
       // SmartDashboard.putNumber("flywheel_master_current", neo_.getOutputCurrent());
       
    }

    @Override
    public void zeroSensors() {
        // no-op
    }

    @Override
    public void registerEnabledLoops(Looper in) {
        //in.register(mLoop);
    }


    double startTestTime=0;
    @Override
    public boolean test(double now) {
        if(startTestTime==0){
            startTestTime = now;
        }

        double timeElapsed = now-startTestTime;

        if(timeElapsed<3){
           
            setRpm(Constants.kFlywheelGoodBallRpmSetpoint);
        }else if(!isOnTarget()){

        
        }else{
            stop();
            startTestTime=0;
            return true;
        }


        return false;
    }
}
