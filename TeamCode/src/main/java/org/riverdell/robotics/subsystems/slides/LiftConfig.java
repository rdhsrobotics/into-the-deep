package org.riverdell.robotics.subsystems.slides;

import com.acmerobotics.dashboard.config.Config;


@Config
public class LiftConfig {
    public static int MAX_EXTENSION = 770;
    public static double kP = 0.02;
    public static double kI = 0.0;
    public static double kD = 0.0015;
    public static double f_g = 0.06;
}
