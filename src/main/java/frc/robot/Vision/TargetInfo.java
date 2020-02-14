package frc.robot.Vision;

/**
 * A container class for Targets detected by the vision system, containing the
 * location in three-dimensional space.
 */
public class TargetInfo {
    /*protected double x = 1.0;
    protected double y;
    protected double z;

    public TargetInfo(double y, double z) {
        this.y = y;
        this.z = z;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getZ() {
        return z;
    }*/

    protected double yaw, pitch;

    public TargetInfo(double yaw, double pitch){
        this.yaw = yaw;
        this.pitch = pitch;
    }

    public double getPitch(){
        return pitch;
    }

    public double getYaw(){
        return yaw;
    }
}