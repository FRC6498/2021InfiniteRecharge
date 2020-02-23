package frc.robot.ShooterTuning;
import frc.lib.util.InterpolatingDouble;
import frc.lib.util.InterpolatingTreeMap;
public class HoodAngles {
	public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kHoodAutoAimMap = new InterpolatingTreeMap<>();
 	static {
		/*
		Sat, 22 Feb 2020 20:24:30 -0600
		Tuned at Castle
		(inch, angle)
		*/
		kHoodAutoAimMap.put(new InterpolatingDouble(242.8519739583336), new InterpolatingDouble(0.0));
		kHoodAutoAimMap.put(new InterpolatingDouble(243.2057067642466), new InterpolatingDouble(0.0));
		kHoodAutoAimMap.put(new InterpolatingDouble(242.852123274681), new InterpolatingDouble(0.0));
		kHoodAutoAimMap.put(new InterpolatingDouble(243.60379081908067), new InterpolatingDouble(0.0));
		kHoodAutoAimMap.put(new InterpolatingDouble(242.8520989946217), new InterpolatingDouble(0.0));
		kHoodAutoAimMap.put(new InterpolatingDouble(242.8521233556694), new InterpolatingDouble(0.0));
	}
}
