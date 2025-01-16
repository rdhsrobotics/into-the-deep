package org.riverdell.robotics.autonomous.movement;

public class FrictionCompensationFunctionParameters {
    public double m;
    public double a;
    public double F_d;
    public double sharpness;

    public FrictionCompensationFunctionParameters(double m, double a, double F_d, double sharpness) {
        this.m = m;
        this.F_d = F_d;
        this.sharpness = sharpness;
        this.a = a;
    }
}
