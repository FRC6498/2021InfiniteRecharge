package frc.robot.loops;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotState;
import frc.robot.StateMachines.Shooter;
import frc.robot.Vision.TargetInfo;
import frc.robot.Vision.TurretCam;
import frc.robot.subsystems.Turret;

/**
 * This function adds vision updates (from the Nexus smartphone) to a list in
 * RobotState. This helps keep track of goals detected by the vision system. The
 * code to determine the best goal to shoot at and prune old Goal tracks is in
 * GoalTracker.java
 * 
 * @see GoalTracker.java
 */
public class VisionProcessor implements Loop {
    static VisionProcessor instance_ = new VisionProcessor();
   
    RobotState robot_state_ = RobotState.getInstance();

    public static VisionProcessor getInstance() {
        return instance_;
    }

    VisionProcessor() {
    }

    @Override
    public void onStart() {
    }

    @Override
    public void onLoop() {
        
       if(TurretCam.isTarget()) robot_state_.addVisionUpdate(Timer.getFPGATimestamp(), new TargetInfo(TurretCam.getTx(), TurretCam.getTy()));
    }

    @Override
    public void onStop() {
        // no-op
    }

}

