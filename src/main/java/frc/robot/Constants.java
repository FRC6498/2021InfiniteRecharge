package frc.robot;

import frc.lib.util.InterpolatingDouble;
import frc.lib.util.InterpolatingTreeMap;
import edu.wpi.first.wpilibj.Solenoid;

/**
 * A list of constants used by the rest of the robot code. This include physics
 * constants as well as constants determined through calibrations.
 */
public class Constants {
   

    public static double kCenterOfTargetHeight = 90; // inches TODO: get this

    // Pose of the turret frame w.r.t. the vehicle frame
    public static double kTurretXOffset = 6;//-7.376;
    public static double kTurretYOffset = 0.0;
    public static double kTurretAngleOffsetDegrees = 0.0;

    // Pose of the camera frame w.r.t. the turret frame
    public static double kCameraXOffset = -4.0589;//-6.454;
    public static double kCameraYOffset = 0.0;
    public static double kCameraZOffset = 27;
    public static double kCameraPitchAngleDegrees = 30;//35.75; // calibrated 4/22
    public static double kCameraYawAngleDegrees = 0;//-1.0;
    public static double kCameraDeadband = 0.0;

    // Goal tracker constants
    public static double kMaxGoalTrackAge = 0.3;
    public static double kMaxTrackerDistance = 18.0;
    public static double kCameraFrameRate = 90.0;
    //public static double kTrackReportComparatorStablityWeight = 1.0;
    //public static double kTrackReportComparatorAgeWeight = 1.0;
    //public static double kTrackReportComparatorSwitchingWeight = 3.0;
    //public static double kTrackReportComparatorDistanceWeight = 2.0; // Unused

   
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

    public static final double regularTurnReduction=.85;//.7;

    public static final double kDriveSwivelReduction=1;//.85;

  
    // Hood constants
    public static double kMinHoodAngle = 45;//42.48;
    public static double kMaxHoodAngle = 78;//55;//75.96;//71.42;
    public static double kHoodNeutralAngle = 46.5;//42.5;
    public static double kHoodOnTargetTolerance = 0.4;
    //public static double kHoodGearReduction = 20/564; 

      // Turret constants
    public static double kHardMaxTurretAngle = 45;//135+13.35;
    public static double kHardMinTurretAngle = -135-13.25;
    public static double kSoftMaxTurretAngle = 134+13.35;
    public static double kSoftMinTurretAngle = -90;//-134-13.25;
    public static double kTurretOnTargetTolerance = 0.8;
    public static double kTurretTicksPerRotation = (2048*(40/10)*(40/20)*(314/40));

    // Flywheel constants
    public static double kFlywheelOnTargetTolerance = 100.0;
    public static double kFlywheelGoodBallRpmSetpoint = 5700;//6200  different setpoints based on ball quality, for now same
    public static double kFlywheelBadBallRpmSetpoint = kFlywheelGoodBallRpmSetpoint;

    // Auto aiming/shooter constants
    public static double kAutoAimMinRange = 10.0;
    public static double kAutoAimMaxRange = 220.0;
    public static double kAutoShootMaxDriveSpeed = 18.0;
    public static double kAutoAimPredictionTime = 0.25;
    public static int kAutoAimMinConsecutiveCyclesOnTarget = 3;
    public static double kShootActuationTime = 0.75;

    //Indexer Constants
    public static double kIndexRampRate = .1;
    public static double kIndexOnlySpeed = .5;
    public static double kIndexandBeltSpeed = .8;
    
    //Feeder Belt Constatns
    public static double kFeederBeltRampRate=.1;
    public static double kFeederBeltLoadingSpeed=.6;
    public static double kFeederBeltDeployingSpeed=1;

    //Feeder Flywheel Constatns
    public static double kFeederFlywheelShootRPM = 3000;
    public static double kFeederFlywheelActuationTime = 1;
    public static double kFeederFlywheelOnTargetTolerance = 200;

    //Intake Constatns
    public static double kIntakeGroundSpeed = .65;
    public static boolean kIntakeVelocityCompensation=true;
    public static double kIntakeVelocityRateOfChange = .05;
    public static double kIntakeCurrentRateOfChange = .05;
    public static double kIntakeGroundCurrentThreshold = 20;
    public static double kIntakeGroundTimeThreshold = 1;
    public static double kIntakeActuationTime = 1;
    public static double kIntakePlowSpeed = .75;
    
    //Belt Clamp Constants
    public static double kBeltClampConveySpeed=.6;
    public static double kBeltClampAgitationPeriod = 1;
    public static double kBeltClampAgitationTime = 3;
    public static double kBeltClampAgitationSpeed = .2;
    public static double kBeltClampConveyTimeout = 15;
    public static double kBeltClampIntakeClampTime = 2.5;

     

    public static double kLooperDt = 0.01;

    // CONTROL LOOP GAINS

   // PID gains for hood position loop
    // Units: error is degrees of hood rotation. Max output is +/- 1.0.
    // Loop runs at 100Hz
    public static double kHoodKp = 0.1;//0.1;
    public static double kHoodKi = 0.0;
    public static double kHoodKd = 0.0;
    public static double kHoodDeadband = 0.3; // degrees

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

    // PID gains for turret position loop
    // Units: error is 4096 counts/rev. Max output is +/- 1023 units.
    public static double kTurretKp = .25;//0.7;
    public static double kTurretKi = 0.0005;
    public static double kTurretKd = 0.0;
    public static double kTurretKf = 0;
    public static int kTurretIZone = (int) (1023.0 / kTurretKp);
    public static double kTurretRampRate = .1;
    public static int kTurretAllowableError = 80;
    public static double kTurretMaxPercentOut = .25;//.4;

    // PID gains for flywheel velocity loop
    // Units: error is (4096 counts/rev)/100ms. Max output is +/- 1023 units.
    public static double kFlywheelKp = .0003;//0.00005;
    public static double kFlywheelKi = 0.000001;
    public static double kFlywheelKd = 0;//0.5;
    public static double kFlywheelKf = 0.000165;
    public static int kFlywheelIZone = (int) (1023.0 / kFlywheelKp);
    public static double kFlywheelRampRate = .1;
    public static int kFlywheelAllowableError = 0;

    // PID gains for feeder flywheel velocity loop
    // Units: error is (4096 counts/rev)/100ms. Max output is +/- 1023 units.
    public static double kFeederFlywheelKp = 0.00001;
    public static double kFeederFlywheelKi = 0.0;
    public static double kFeederFlywheelKd = 0;//0.5;
    public static double kFeederFlywheelKf = 0.000165;
    public static int kFeederFlywheelIZone = (int) (1023.0 / kFlywheelKp);
    public static double kFeederFlywheelRampRate = .1;
    public static int kFeederFlywheelAllowableError = 0;

   
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
   
    public static final int kLeftDriveMasterId = 3;
    public static final int kLeftDriveChildId = 4;
   
    public static final int kRightDriveMasterId = 1;
    public static final int kRightDriveChildId = 2;


    public static final int kTurretFalconId = 10;

    public static final int kFlyWheelNeoId = 14;

    public static int kIndexFalconId = 12;

    public static int kFeederBeltVictorId = 7;

    public static int kFeederFlywheelNeoId = 15;

    public static int kIntakeTalonId = 5;

    public static int kBeltClampVictorId = 8;
   

    // SOLENOIDS
    public static final int shiftSolenoidId = 3;
    public static final int intakeSolenoidId = 0;
    public static final int beltClampSolenoidId = 2;

    // Analog Inputs
    public static final int kHoodEncoderAnalog = 0;

    
    // DIGITAL IO
   
    public static final int kHoodServoPWM = 0;

    public static final int kIndexSensorPort = 0;

    public static final int kFeederBeltSensorPort = 1;

    
    
   

    // Shooter Operational consts
    public static final double kOldBallHoodAdjustment = 2.4;
    public static final double kNewBallHoodAdjustment = 0.7;
    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kHoodAutoAimMap = new InterpolatingTreeMap<>();

    static {
        /* Tune 4/23 with 6200 rpm setpoint (inch,angle)*/
        kHoodAutoAimMap.put(new InterpolatingDouble(60.0), new InterpolatingDouble(42.5));
        kHoodAutoAimMap.put(new InterpolatingDouble(70.0), new InterpolatingDouble(44.5));
        kHoodAutoAimMap.put(new InterpolatingDouble(75.0), new InterpolatingDouble(46.8));
        kHoodAutoAimMap.put(new InterpolatingDouble(80.0), new InterpolatingDouble(48.0));
        kHoodAutoAimMap.put(new InterpolatingDouble(85.0), new InterpolatingDouble(49.0));
        kHoodAutoAimMap.put(new InterpolatingDouble(90.0), new InterpolatingDouble(50.5));
        kHoodAutoAimMap.put(new InterpolatingDouble(100.0), new InterpolatingDouble(52.5));
        kHoodAutoAimMap.put(new InterpolatingDouble(110.0), new InterpolatingDouble(54.5));
        kHoodAutoAimMap.put(new InterpolatingDouble(120.0), new InterpolatingDouble(56.5));
        kHoodAutoAimMap.put(new InterpolatingDouble(130.0), new InterpolatingDouble(57.8));
        kHoodAutoAimMap.put(new InterpolatingDouble(140.0), new InterpolatingDouble(59.0));
        kHoodAutoAimMap.put(new InterpolatingDouble(150.0), new InterpolatingDouble(59.8));
        kHoodAutoAimMap.put(new InterpolatingDouble(160.0), new InterpolatingDouble(60.5));
        kHoodAutoAimMap.put(new InterpolatingDouble(170.0), new InterpolatingDouble(61.0));
        kHoodAutoAimMap.put(new InterpolatingDouble(180.0), new InterpolatingDouble(61.5));
    }

    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kFlywheelAutoAimMap = new InterpolatingTreeMap<>();

    static {
        kFlywheelAutoAimMap.put(new InterpolatingDouble(75.0),
                new InterpolatingDouble(Constants.kFlywheelGoodBallRpmSetpoint));
        kFlywheelAutoAimMap.put(new InterpolatingDouble(Constants.kAutoAimMaxRange),
                new InterpolatingDouble(Constants.kFlywheelGoodBallRpmSetpoint));
    }
   

}
