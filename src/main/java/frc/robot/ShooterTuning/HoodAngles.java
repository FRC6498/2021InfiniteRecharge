package frc.robot.ShooterTuning;
import frc.lib.util.InterpolatingDouble;
import frc.lib.util.InterpolatingTreeMap;
public class HoodAngles {
	public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kHoodAutoAimMap = new InterpolatingTreeMap<>();
 	static {
		/*
		Sat, 22 Feb 2020 20:55:53 -0600
		Castle
		(inch, angle)
		*/
		kHoodAutoAimMap.put(new InterpolatingDouble(62.4), new InterpolatingDouble(46.17));
		kHoodAutoAimMap.put(new InterpolatingDouble(124.9), new InterpolatingDouble(58.5));
		kHoodAutoAimMap.put(new InterpolatingDouble(183.7), new InterpolatingDouble(67.3));
		kHoodAutoAimMap.put(new InterpolatingDouble(237.4), new InterpolatingDouble(69.1));
		kHoodAutoAimMap.put(new InterpolatingDouble(292.0), new InterpolatingDouble(69.3));
	}
}
