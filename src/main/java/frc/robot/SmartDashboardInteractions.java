package frc.robot;

import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.modes.*;
import frc.robot.subsystems.ShooterAimingParameters;
import frc.lib.util.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.json.simple.JSONArray;

/**
 * Controls the interactive elements of SmartDashboard.
 *
 * Keeps the network tables keys in one spot and enforces autonomous mode
 * invariants.
 */
public class SmartDashboardInteractions {

    private static final String HOOD_TUNING_MODE = "Hood Tuning Mode";
    private static final String OUTPUT_TO_SMART_DASHBOARD = "Output To SmartDashboard";


    private static final String AUTO_OPTIONS = "auto_options";
    private static final String SELECTED_AUTO_MODE = "selected_auto_mode";
    private static final String SELECTED_AUTO_LANE = "selected_auto_lane";

    private static final String AUTO_BALLS_WORN = "auto_balls_worn";

    private static final AutonOption DEFAULT_MODE = AutonOption.SHOOT_THREE_GET_TRENCH_SHOOT;
    private static final AutonLane DEFAULT_LANE = AutonLane.LANE_1;

    public void initWithDefaults() {
        SmartDashboard.putBoolean(HOOD_TUNING_MODE, false);
        SmartDashboard.putBoolean(OUTPUT_TO_SMART_DASHBOARD, true);
      

        JSONArray autoOptionsArray = new JSONArray();
        for (AutonOption autonOption : AutonOption.values()) {
            autoOptionsArray.add(autonOption.name);
        }
        SmartDashboard.putString(AUTO_OPTIONS, autoOptionsArray.toString());
        SmartDashboard.putString(SELECTED_AUTO_MODE, DEFAULT_MODE.name);
        SmartDashboard.putString(SELECTED_AUTO_LANE, DEFAULT_LANE.numberString);
        SmartDashboard.putBoolean(AUTO_BALLS_WORN, false);
    }

    public boolean isInHoodTuningMode() {
        return SmartDashboard.getBoolean(HOOD_TUNING_MODE, false);
    }

    public boolean shouldLogToSmartDashboard() {
        return SmartDashboard.getBoolean(OUTPUT_TO_SMART_DASHBOARD, true);
    }

   

    public AutoModeBase getSelectedAutonMode() {
        String autoModeString = SmartDashboard.getString(SELECTED_AUTO_MODE, DEFAULT_MODE.name);
        AutonOption selectedOption = DEFAULT_MODE;
        for (AutonOption autonOption : AutonOption.values()) {
            if (autonOption.name.equals(autoModeString)) {
                selectedOption = autonOption;
                break;
            }
        }

        String autoLaneString = SmartDashboard.getString(SELECTED_AUTO_LANE, DEFAULT_LANE.numberString);
        AutonLane selectedLane = DEFAULT_LANE;
        for (AutonLane autonLane : AutonLane.values()) {
            if (autonLane.numberString.equals(autoLaneString)) {
                selectedLane = autonLane;
            }
        }

        return createAutoMode(selectedOption, selectedLane);
    }

    public boolean areAutoBallsWorn() {
        return SmartDashboard.getBoolean(AUTO_BALLS_WORN, false);
    }

    /**
     * I don't trust {@link SendableChooser} to manage {@link AutoModeBase}
     * objects directly, so use this enum to project us from WPILIb.
     */
    enum AutonOption {

        SHOOT_THREE_GET_TRENCH_SHOOT("STGTS"),
        GET_TWO_SHOOT_FIVE("GTSF"),
        SHOOT_THREE("ST"),
        SHOOT_THREE_MOVE_TRENCH("STMT"),
        SHOOT_THREE_MOVE_FORWARD("STMF"),
        SHOOT_THREE_MOVE_BACKWARD("STMB"),
        STAND_STILL("Stand Still"), //
        TEST_DRIVE("TEST ONLY Driving");

        public final String name;

        AutonOption(String name) {
            this.name = name;
        }
    }

    enum AutonLane {
        LANE_1(160, "1"), LANE_2(205, "2"), LANE_3(160, "3");

        public final double distanceToDrive;
        public final String numberString;

        AutonLane(double distanceToDrive, String numberString) {
            this.distanceToDrive = distanceToDrive;
            this.numberString = numberString;
        }
    }

    private ShooterAimingParameters getAimingHintForLane(AutonLane lane) {
        if (lane == AutonLane.LANE_1) {
            return new ShooterAimingParameters(8*12, Rotation2d.fromDegrees(0));
        } else if (lane == AutonLane.LANE_2) {
            return new ShooterAimingParameters(150.0, Rotation2d.fromDegrees(-30));
        } else if (lane == AutonLane.LANE_3) {
            return new ShooterAimingParameters(140.0, Rotation2d.fromDegrees(-15));
        } else { /* if (lane == AutonLane.LANE_5) */
            return new ShooterAimingParameters(140.0, Rotation2d.fromDegrees(25));
        }
    }

    private double getDistanceForAutoLane(AutonLane lane){
        switch(lane){

            case LANE_1:
                return 25;
            case LANE_2:
                return 35+25;
            default:
                return 25;
        }
    }

    private AutoModeBase createAutoMode(AutonOption autonOption, AutonLane autonLane) {
        switch (autonOption) {
    
        case SHOOT_THREE_GET_TRENCH_SHOOT:
            return new ShootThreeGetTrenchShootMode(getDistanceForAutoLane(autonLane));
        case SHOOT_THREE:
            return new ShootThreeMode();
        case SHOOT_THREE_MOVE_TRENCH:
            return new ShootThreeMoveTrenchMode(getDistanceForAutoLane(autonLane));
        case SHOOT_THREE_MOVE_FORWARD:
            return new ShootThreeMoveForwardMode();
        case SHOOT_THREE_MOVE_BACKWARD:
            return new ShootThreeMoveBackwardMode();
        case GET_TWO_SHOOT_FIVE:
            return new GetTwoShootFiveMode(getDistanceForAutoLane(autonLane));
        case TEST_DRIVE:
            return new AutoModeBase() {
                @Override
                protected void routine() throws AutoModeEndedException {
                    throw new RuntimeException("Expected exception!!!");
                }
            };

        case STAND_STILL: // fallthrough
        default:
            System.out.println("ERROR: unexpected auto mode: " + autonOption);
            return new StandStillMode();
        }
    }
}
