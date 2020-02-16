package frc.robot;

import java.util.Map;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.util.InterpolatingDouble;
import frc.lib.util.InterpolatingTreeMap;
import frc.lib.util.RigidTransform2d;
import frc.lib.util.Rotation2d;
import frc.lib.util.Translation2d;
//import frc.robot.GoalTracker.TrackReport;
import frc.robot.Vision.TargetInfo;
import frc.robot.subsystems.ShooterAimingParameters;

/**
 * RobotState keeps track of the poses of various coordinate frames throughout
 * the match. A coordinate frame is simply a point and direction in space that
 * defines an (x,y) coordinate system. Transforms (or poses) keep track of the
 * spatial relationship between different frames.
 * 
 * Robot frames of interest (from parent to child):
 * 
 * 1. Field frame: origin is where the robot is turned on
 * 
 * 2. Vehicle frame: origin is the center of the robot wheelbase, facing
 * forwards
 * 
 * 3. Turret fixed frame: origin is the center of the turret when the turret is
 * at 0 degrees rotation relative to the vehicle frame
 * 
 * 4. Turret rotating frame: origin is the center of the turret as it rotates
 * 
 * 5. Camera frame: origin is the center of the camera imager as it rotates with
 * the turret
 * 
 * 6. Goal frame: origin is the center of the goal (note that orientation in
 * this frame is arbitrary). Also note that there can be multiple goal frames.
 * 
 * As a kinematic chain with 6 frames, there are 5 transforms of interest:
 * 
 * 1. Field-to-vehicle: This is tracked over time by integrating encoder and
 * gyro measurements. It will inevitably drift, but is usually accurate over
 * short time periods.
 * 
 * 2. Vehicle-to-turret-fixed: This is a constant.
 * 
 * 3. Vehicle-to-turret-rotating: This is a pure rotation, and is tracked over
 * time using the turret encoder.
 * 
 * 4. Turret-rotating-to-camera: This is a constant.
 * 
 * 5. Camera-to-goal: This is a pure translation, and is measured by the vision
 * system.
 */

public class RobotState {
    private static RobotState instance_ = new RobotState();

    public static RobotState getInstance() {
        return instance_;
    }

    public static final int kObservationBufferSize = 100;
    public static final double kMaxTargetAge = 0.4;

   
    public static final RigidTransform2d kVehicleToTurretFixed = new RigidTransform2d(
            new Translation2d(Constants.kTurretXOffset, Constants.kTurretYOffset),
            Rotation2d.fromDegrees(Constants.kTurretAngleOffsetDegrees));

    public static final RigidTransform2d kTurretRotatingToCamera = new RigidTransform2d(
            new Translation2d(Constants.kCameraXOffset, Constants.kCameraYOffset), new Rotation2d());
    

    // FPGATimestamp -> RigidTransform2d or Rotation2d
    protected InterpolatingTreeMap<InterpolatingDouble, RigidTransform2d> field_to_vehicle_;
    protected RigidTransform2d.Delta vehicle_velocity_;
    protected InterpolatingTreeMap<InterpolatingDouble, Rotation2d> turret_rotation_;
    protected GoalTrack goal_track;
    protected Rotation2d camera_pitch_correction_;
    protected Rotation2d camera_yaw_correction_;
    protected double differential_height_;
    

    protected RobotState() {
        reset(0, new RigidTransform2d(), new Rotation2d());
    }

    public synchronized void reset(double start_time, RigidTransform2d initial_field_to_vehicle, Rotation2d initial_turret_rotation) {
        field_to_vehicle_ = new InterpolatingTreeMap<>(kObservationBufferSize);
        field_to_vehicle_.put(new InterpolatingDouble(start_time), initial_field_to_vehicle);
        vehicle_velocity_ = new RigidTransform2d.Delta(0, 0, 0);
        turret_rotation_ = new InterpolatingTreeMap<>(kObservationBufferSize);
        turret_rotation_.put(new InterpolatingDouble(start_time), initial_turret_rotation);
        goal_track = new GoalTrack(0, new Translation2d());
       
        differential_height_ = Constants.kCenterOfTargetHeight - Constants.kCameraZOffset;

        //totalNumberBallsCounted=0;
    }

    public synchronized RigidTransform2d getFieldToVehicle(double timestamp) {
        return field_to_vehicle_.getInterpolated(new InterpolatingDouble(timestamp));
    }

    public synchronized Map.Entry<InterpolatingDouble, RigidTransform2d> getLatestFieldToVehicle() {
        return field_to_vehicle_.lastEntry();
    }

    public synchronized RigidTransform2d getPredictedFieldToVehicle(double lookahead_time) {
        return getLatestFieldToVehicle().getValue().transformBy(
                RigidTransform2d.fromVelocity(new RigidTransform2d.Delta(vehicle_velocity_.dx * lookahead_time,
                        vehicle_velocity_.dy * lookahead_time, vehicle_velocity_.dtheta * lookahead_time)));
    }

  
    public synchronized Rotation2d getTurretRotation(double timestamp) {
        return turret_rotation_.getInterpolated(new InterpolatingDouble(timestamp));
    }

    public synchronized Map.Entry<InterpolatingDouble, Rotation2d> getLatestTurretRotation() {
        return turret_rotation_.lastEntry();
    }

    public synchronized RigidTransform2d getFieldToTurretRotated(double timestamp) {
        InterpolatingDouble key = new InterpolatingDouble(timestamp);
        return field_to_vehicle_.getInterpolated(key).transformBy(kVehicleToTurretFixed)
                .transformBy(RigidTransform2d.fromRotation(turret_rotation_.getInterpolated(key)));
    }

    public synchronized RigidTransform2d getFieldToCamera(double timestamp) {
        return getFieldToTurretRotated(timestamp).transformBy(kTurretRotatingToCamera);
    }

    public synchronized RigidTransform2d getCaptureTimeFieldToGoal() {
        RigidTransform2d rv = new RigidTransform2d();
        //for (TrackReport report : goal_tracker_.getTracks()) {
         synchronized(this) {  rv=RigidTransform2d.fromTranslation(goal_track.getSmoothedPosition());}
        //}
        return rv;
    }

    public synchronized ShooterAimingParameters getAimingParameters(double current_timestamp) {
        ShooterAimingParameters rv;
        //List<TrackReport> reports = goal_tracker_.getTracks();
        //Collections.sort(reports, comparator);

        // turret fixed (latest) -> vehicle (latest) -> field
        RigidTransform2d latest_turret_fixed_to_field = getPredictedFieldToVehicle(Constants.kAutoAimPredictionTime)
                .transformBy(kVehicleToTurretFixed).inverse();

       
            if (current_timestamp - goal_track.getLatestTimestamp() > kMaxTargetAge) {
                //System.out.println("Max target age "+goal_track.getLatestTimestamp()+ " current: " + current_timestamp);
                return null;
            }
            // turret fixed (latest) -> vehicle (latest) -> field -> goals  so getting where turret is relative to field origin then transforming by the field origin to the goal
            RigidTransform2d latest_turret_fixed_to_goal;
            synchronized(this){   latest_turret_fixed_to_goal = latest_turret_fixed_to_field
                    .transformBy(RigidTransform2d.fromTranslation(goal_track.getSmoothedPosition()));
          }
            // We can actually disregard the angular portion of this pose. It is
            // the bearing that we care about!
            rv=new ShooterAimingParameters(latest_turret_fixed_to_goal.getTranslation().norm(), //get hypot
            new Rotation2d(latest_turret_fixed_to_goal.getTranslation().getX(), //get
                    latest_turret_fixed_to_goal.getTranslation().getY(), true)); //angle

           //System.out.println("Param: "+rv.getRange()+ " angle: "+rv.getTurretAngle());
        
        return rv;
    }

    public synchronized void addFieldToVehicleObservation(double timestamp, RigidTransform2d observation) {
        field_to_vehicle_.put(new InterpolatingDouble(timestamp), observation);
    }

    public synchronized void addTurretRotationObservation(double timestamp, Rotation2d observation) {
        turret_rotation_.put(new InterpolatingDouble(timestamp), observation);
    }

    public synchronized void addObservations(double timestamp, RigidTransform2d field_to_vehicle, Rotation2d turret_rotation, RigidTransform2d.Delta velocity) {
        addFieldToVehicleObservation(timestamp, field_to_vehicle);
        addTurretRotationObservation(timestamp, turret_rotation);
        vehicle_velocity_ = velocity;
    }

    public void addVisionUpdate(double timestamp, TargetInfo target) {
        Translation2d field_to_goal = new Translation2d();
        RigidTransform2d field_to_camera = getFieldToCamera(timestamp);
       
                //counter clock wise is positivie thats why target yaw is inverted
                //pretty sure x is positive forward, y is positive left
            
                double yaw = -target.getYaw() + Constants.kCameraYawAngleDegrees;
                double pitch = target.getPitch() + Constants.kCameraPitchAngleDegrees;

                double xr = (differential_height_)/Math.tan(pitch*(Math.PI/180)); //convert degrees to radians

                double yr = xr*Math.tan(yaw*(Math.PI/180)); //conert degrees to radians
             
                    double distance = Math.hypot(xr, yr);// * scaling;
                    Rotation2d angle = new Rotation2d(xr, yr, true);
                    field_to_goal=field_to_camera
                            .transformBy(RigidTransform2d
                                    .fromTranslation(new Translation2d(distance * angle.cos(), distance * angle.sin())))
                            .getTranslation();
            
        
        synchronized(this){
           goal_track.tryUpdate(timestamp, field_to_goal);
        }
           // System.out.println("time: "+timestamp+ " x: "+field_to_goal.getX()+" y: "+field_to_goal.getY());
        
    
    }

    public synchronized void resetVision() {
        goal_track.reset();
    }

    public RigidTransform2d generateOdometryFromSensors(double left_encoder_delta_distance,
            double right_encoder_delta_distance, Rotation2d current_gyro_angle) {
        RigidTransform2d last_measurement = getLatestFieldToVehicle().getValue();
        return Kinematics.integrateForwardKinematics(last_measurement, left_encoder_delta_distance,
                right_encoder_delta_distance, current_gyro_angle);
    }




    
    private int totalBallsInFeeder=0;
    private int totalBallsInBelt=0;

    public synchronized int getTotalBalls(){
        return totalBallsInFeeder+totalBallsInBelt;
    }

    public synchronized void setIntakeBalls(int number){
        totalBallsInBelt+=number;
    }

    public synchronized void setFeederBalls(int number){
        totalBallsInFeeder+=number;
    }

    public synchronized int getFeederBalls(){
        return totalBallsInFeeder;
    }

    public synchronized int getBeltBalls(){
        return totalBallsInBelt;
    }

    public synchronized void fillBalls(){
        totalBallsInBelt = 5-getFeederBalls();
    }

    public synchronized int ballsNeeded(){
        return 5-getTotalBalls();
    }

    





    public void outputToSmartDashboard() {
        RigidTransform2d odometry = getLatestFieldToVehicle().getValue();
        SmartDashboard.putNumber("robot_pose_x", odometry.getTranslation().getX());
        SmartDashboard.putNumber("robot_pose_y", odometry.getTranslation().getY());
        SmartDashboard.putNumber("robot_pose_theta", odometry.getRotation().getDegrees());
        RigidTransform2d pose = getCaptureTimeFieldToGoal();
        
            // Only output first goal
           SmartDashboard.putNumber("goal_pose_x", pose.getTranslation().getX());
            SmartDashboard.putNumber("goal_pose_y", pose.getTranslation().getY());

        SmartDashboard.putNumber("belt balls", getBeltBalls());
        SmartDashboard.putNumber("feeder balls", getFeederBalls());
        
    }
}
