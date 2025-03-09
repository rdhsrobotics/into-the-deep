package org.riverdell.robotics.autonomous.impl.tests;

import com.acmerobotics.dashboard.config.Config;

import org.riverdell.robotics.autonomous.movement.PositionChangeTolerance;

@Config
public class TestConfig {
    public static double x = 0.0;
    public static double y = 4.0;
    public static double turn = 90.0;
    public static double automaticDeath = 50000;
    public static boolean extendoOut = false;
    public static boolean telemetry = true;

    public static PositionChangeTolerance positionChangeTolerance = new PositionChangeTolerance();
}
