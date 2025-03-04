package org.riverdell.robotics.autonomous.impl.tests;

import com.acmerobotics.dashboard.config.Config;

import org.riverdell.robotics.autonomous.movement.PositionChangeTolerance;

@Config
public class TestConfig {

    public static double x = -11.8;
    public static double y = 22.07;
    public static double turn = 48.13;
    public static double automaticDeath = 50000;
    public static boolean extendoOut = false;
    public static boolean telemetry = true;

    public static PositionChangeTolerance positionChangeTolerance = new PositionChangeTolerance();
}
