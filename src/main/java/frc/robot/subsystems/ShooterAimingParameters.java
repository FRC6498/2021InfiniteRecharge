package frc.robot.subsystems;

import frc.lib.util.Rotation2d;

/**
 * A container class to specify the shooter angle. It contains the desired
 * range, the turret angle, and the computer vision's track's ID.
 */
public class ShooterAimingParameters {
    public double range;
    Rotation2d turret_angle;
    

    public ShooterAimingParameters(double range, Rotation2d turret_angle) {
        this.range = range;
        this.turret_angle = turret_angle;
     
    }

    public double getRange() {
        return range;
    }

    public Rotation2d getTurretAngle() {
        return turret_angle;
    }

  
}
