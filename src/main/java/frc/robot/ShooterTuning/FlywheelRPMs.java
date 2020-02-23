package frc.robot.ShooterTuning;

import frc.lib.util.InterpolatingDouble;
import frc.lib.util.InterpolatingTreeMap;

public class FlywheelRPMs {
   
    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kFlywheelAutoAimMap = new InterpolatingTreeMap<>();

    static {
        kFlywheelAutoAimMap.put(new InterpolatingDouble(60.0),
                new InterpolatingDouble(5200.0));
        kFlywheelAutoAimMap.put(new InterpolatingDouble(300.0),
                new InterpolatingDouble(5900.0));
    }
}