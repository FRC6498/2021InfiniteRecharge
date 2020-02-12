package frc.robot;

import frc.lib.util.InterpolatingDouble;
import frc.lib.util.InterpolatingTreeMap;
import edu.wpi.first.wpilibj.Solenoid;

/**
 * A list of constants used by the rest of the robot code. This include physics
 * constants as well as constants determined through calibrations.
 */
public class Constants {
   



   
    // Wheels
    public static double kDriveWheelDiameterInches = 6; // Measured on
                                                          // 4/23/2016
    public static double kTrackLengthInches = 8;//11.75;//8.265;
    public static double kTrackWidthInches = 26.5;//23.8
    public static double kTrackEffectiveDiameter = (kTrackWidthInches * kTrackWidthInches
            + kTrackLengthInches * kTrackLengthInches) / kTrackWidthInches;
    public static double kTrackScrubFactor = 0.5;

    // Drive constants
    public static double kDriveLowGearMaxSpeedInchesPerSec = 12.0 * 7.0;

    public static double kDriveTicksPerRotation = 2048*26;//4096*3;//360/3; 

    public static double kDriveRampRate=.3;

  
    
  

    public static double kLooperDt = 0.01;

    // CONTROL LOOP GAINS

   

    // PID gains for drive velocity loop (LOW GEAR)
    // Units: error is 4096 counts/rev. Max output is +/- 1023 units.
    public static double kDriveVelocityKp = 0.1;//1.0;
    public static double kDriveVelocityKi = 0.0;
    public static double kDriveVelocityKd = 6.0;
    public static double kDriveVelocityKf = 0.05;//.5
    public static int kDriveVelocityIZone = 0;
    public static double kDriveVelocityRampRate = 0.0;
    public static int kDriveVelocityAllowableError = 0;

    // PID gains for drive base lock loop
    // Units: error is 4096 counts/rev. Max output is +/- 1023 units.
    public static double kDriveBaseLockKp = 0.05;//0.5;
    public static double kDriveBaseLockKi = 0;
    public static double kDriveBaseLockKd = 0;
    public static double kDriveBaseLockKf = 0;
    public static int kDriveBaseLockIZone = 0;
    public static double kDriveBaseLockRampRate = 0;
    public static int kDriveBaseLockAllowableError = 1;//10;

    // PID gains for constant heading velocity control
    // Units: Error is degrees. Output is inches/second difference to
    // left/right.
    public static double kDriveHeadingVelocityKp = 2;//4;//6;//4.0; // 6.0;
    public static double kDriveHeadingVelocityKi = 0.0;
    public static double kDriveHeadingVelocityKd = 50;//50.0;

    // Path following constants
    public static double kPathFollowingLookahead = 24.0; // inches
    public static double kPathFollowingMaxVel = 120.0; // inches/sec
    public static double kPathFollowingMaxAccel = 80.0; // inches/sec^2

   
    //Auto alignment with Load Station
    public static double kCameraHeight = 25.75;
    public static double kCameraAngle = 30;
    public static double kTargetHeight = 16.5;
    public static double kDistanceToBumber = 0;
   

   
   
    // Do not change anything after this line!
    // Port assignments should match up with the spreadsheet here:
    // https://docs.google.com/spreadsheets/d/1O2Szvp3cC3gO2euKjqhdmpsyd54t6eB2t9jzo41G2H4
    // Talons
    // (Note that if multiple talons are dedicated to a mechanism, any sensors
    // are attached to the master)
    // Motors
   
    public static final int kLeftDriveMasterId = 1;
    public static final int kLeftDriveChildId = 2;
   
    public static final int kRightDriveMasterId = 3;
    public static final int kRightDriveChildId = 4;

    //public static final int kLeftEncoderId =1;
    //public static final int kRightEncoderId = 2;
   

    // SOLENOIDS
    public static final int shiftSolenoidId = 3;

    // Analog Inputs
   

    /**
     * Make an {@link Solenoid} instance for the single-number ID of the
     * solenoid
     * 
     * @param solenoidId
     *            One of the kXyzSolenoidId constants
     */
    public static Solenoid makeSolenoidForId(int solenoidId) {
        return new Solenoid(solenoidId / 8, solenoidId % 8);
    }

    // DIGITAL IO
   

   


   

}
