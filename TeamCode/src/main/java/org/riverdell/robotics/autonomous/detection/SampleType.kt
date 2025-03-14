package org.riverdell.robotics.autonomous.detection;

import org.opencv.core.Scalar;

public enum SampleType {
    Blue(
        new Scalar(90.0, 100.0, 100.0),
        new Scalar(130.0, 255.0, 255.0)
    ),
    Yellow(
        new Scalar(20.0, 100.0, 100.0),
        new Scalar(30.0, 255.0, 255.0)
    ),
    Red(
        new Scalar(0.0, 100.0, 100.0),
        new Scalar(10.0, 255.0, 255.0)
    );

    private final Scalar colorRangeMinimum;
    private final Scalar colorRangeMaximum;

    SampleType(Scalar colorRangeMinimum, Scalar colorRangeMaximum) {
        this.colorRangeMinimum = colorRangeMinimum;
        this.colorRangeMaximum = colorRangeMaximum;
    }

    public Scalar getColorRangeMinimum() {
        return colorRangeMinimum;
    }

    public Scalar getColorRangeMaximum() {
        return colorRangeMaximum;
    }
}
