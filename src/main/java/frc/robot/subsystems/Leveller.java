/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import frc.robot.Constants;
import frc.robot.loops.Looper;

/**
 * Add your docs here.
 */
public class Leveller extends Subsystem
{
    private static Leveller sInstance = null;

    public static Leveller getInstance()
    {
        if (sInstance == null) 
        {
            sInstance = new Leveller();    
        }
        return sInstance;
    }


    private VictorSPX motor_;

    private Leveller(){
        motor_=new VictorSPX(Constants.kLevelVictorId);
    }

    public void set(double speed){
        motor_.set(ControlMode.PercentOutput, speed);
    }

    @Override
    public void stop()
    {
        set(0);
    }

    @Override
    public void zeroSensors()
    {

    }

    @Override
    public void outputToSmartDashboard()
    {

    }

    @Override
    public void registerEnabledLoops(Looper looper)
    {

    }

    @Override
    public boolean test(double now) {
        // TODO Auto-generated method stub
        return true;
    }
}
