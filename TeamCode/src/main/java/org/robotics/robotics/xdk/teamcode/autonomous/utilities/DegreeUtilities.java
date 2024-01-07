package org.robotics.robotics.xdk.teamcode.autonomous.utilities;

public class DegreeUtilities {

    public static double degFrom(
            final double current,
            final double targetDeg
    ) {
        double d1 = (targetDeg - current);
        double d2 = (360 - targetDeg + current);

        if (Math.abs(d1) < Math.abs(d2)) {
            return -d1;
        } else {
            return -d2;
        }
    }
}