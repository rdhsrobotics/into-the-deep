package org.riverdell.robotics.utilities.motionprofile;

public class ProfileConstraints {
    public double accel;
    public double decel;
    public double velo;

    public ProfileConstraints() {
        this(10.0, 10.0, 10.0);
    }

    public ProfileConstraints(double velo, double accel) {
        this(accel, accel, velo);
    }

    public ProfileConstraints(double velo, double accel, double decel) {
        this.velo = Math.abs(velo);
        this.accel = Math.abs(accel);
        this.decel = Math.abs(decel);
    }

    public void convert(double factor) {
        this.velo *= factor;
        this.accel *= factor;
        this.decel *= factor;
    }
}