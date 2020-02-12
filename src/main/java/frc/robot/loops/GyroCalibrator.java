package frc.robot.loops;

import frc.robot.subsystems.Drive;
import frc.lib.util.ADXRS453_Gyro;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Timer;

/**
 * Routine that recalibrates the gyroscope every 5 seconds. The gyroscope helps
 * the robot drive in a straight line despite obstacles and compensates for
 * bumps and obstacles.
 */
public class GyroCalibrator implements Loop {
    AHRS mGyro = Drive.getInstance().getGyro();

    double mCalibrationStartTime = 0;

    @Override
    public void onStart() {
        // no-op
    }

    @Override
    public void onLoop() {
        /*double now = Timer.getFPGATimestamp();
        // Keep re-calibrating the gyro every 5 seconds
        if (now - mCalibrationStartTime > 5) {
            mGyro.ca;
            System.out.println("Gyro calibrated, new zero is " + mGyro.getCenter());
            mCalibrationStartTime = now;
            mGyro.startCalibrate();
        }*/
    }

    @Override
    public void onStop() {
        //mGyro.cancelCalibrate();
    }

}
