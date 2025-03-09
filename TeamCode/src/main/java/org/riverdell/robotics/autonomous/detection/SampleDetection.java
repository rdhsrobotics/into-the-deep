package org.riverdell.robotics.autonomous.detection;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.jetbrains.annotations.NotNull;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.riverdell.robotics.autonomous.movement.geometry.Pose;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;

import javax.annotation.Nullable;

@Config
public class SampleDetection implements CameraStreamSource, VisionProcessor {

    public static double PICKUP_Y_ADJUSTMENT = -5.0;
    public static double PICKUP_X_ADJUSTMENT = 1.5;

    public static double TURN_FACTOR = 0.15;
    public static double TURN_FACTOR_D_GAIN = -0.0001;

    public static double FRAME_CENTER_X = 640.0;
    public static double FRAME_CENTER_Y = 480.0;

    public static double X_GUIDANCE_SCALE = 0.015;
    public static double Y_GUIDANCE_SCALE = 0.015;

    // dont tune
    public static double MIN_SAMPLE_AREA = 200000.0;
/*
    public static double SAMPLE_AREA_AVERAGE_DURING_HOVER = 500000.0;
    public static double SAMPLE_AREA_AVERAGE_DURING_HOVER_MAX_DEVIATION = 100000.0;*/

    // dont tune
    public static double SAMPLE_TRACKING_DISTANCE = 1000.0;

    public static double SAMPLE_AUTOSNAP_RADIUS = 50.0;
    public static double SAMPLE_AUTOSNAP_LIFETIME = 1000L;

    // AtomicReference to store the last frame as a Bitmap
    private final AtomicReference<Bitmap> lastFrame = new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

    private @Nullable Vector2d guidanceVector = null;

    @Nullable
    public Vector2d getGuidanceVector() {
        return guidanceVector;
    }

    private double previousRotationAngle = 0.0;
    public double guidanceRotationAngle = 0.0;
    private double sampleArea = 0.0;

    private Point autoSnapCenterLock = null;
    private long autoSnapHeartbeat = 0L;

    public static double MIN_B = 0.0;
    public static double MIN_G = 0.0;
    public static double MIN_R = 0.0;

    public static double MAX_B = 0.0;
    public static double MAX_G = 0.0;
    public static double MAX_R = 0.0;

    private SampleType detectionType = SampleType.Yellow;
    private Supplier<Double> currentWristPosition = () -> 0.0;

    private double targetWristPosition = 0.49;

    public double getTargetWristPosition() {
        return targetWristPosition;
    }

    public void supplyCurrentWristPosition(Supplier<Double> currentWristPosition) {
        this.currentWristPosition = currentWristPosition;
    }

    public void setDetectionType(SampleType sampleType) {
        this.detectionType = sampleType;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        // Initialize the last frame with the correct size
        lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
    }

    public static int SET_TO_1_IF_COLOR_TUNING = 1;

    @Override
    public Object processFrame(Mat input, long captureTimeNanos) {
        Mat hsvMat = new Mat();
        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);

        // Create a mask for the specified color range
        Mat colorMask = new Mat();
        if (SET_TO_1_IF_COLOR_TUNING == 0) {
            Core.inRange(
                    hsvMat,
                    new Scalar(MIN_B, MIN_G, MIN_R),
                    new Scalar(MAX_B, MAX_G, MAX_R),
                    colorMask
            );
        } else {
            Core.inRange(
                    hsvMat,
                    detectionType.getColorRangeMinimum(),
                    detectionType.getColorRangeMaximum(),
                    colorMask
            );
        }

        // Detect the sample object in the specified mask
        Point sampleCenter = detectSample(input, colorMask);
        Imgproc.circle(input, new Point(FRAME_CENTER_X, FRAME_CENTER_Y), (int) SAMPLE_TRACKING_DISTANCE, new Scalar(128, 0, 0));

        // Annotate the detected sample with bounding box and angle
        if (sampleCenter != null) {
            annotateBoundingBox(input, sampleCenter);
        } else {
            guidanceVector = null;
        }

        // Convert the processed frame (Mat) to a Bitmap for FTC dashboard display
        Bitmap bitmap = Bitmap.createBitmap(input.width(), input.height(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(SET_TO_1_IF_COLOR_TUNING == 1 ? input : colorMask, bitmap);

        // Update the last frame
        lastFrame.set(bitmap);

        return input;
    }

    private Point detectSample(Mat input, Mat blueMask) {
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(blueMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        Rect boundingBox = null;
        Point sampleCenter = null;
        double minDistance = SAMPLE_TRACKING_DISTANCE;

        // Loop through contours to find the largest contour (which should be the sample)
        for (MatOfPoint contour : contours) {
            Rect localBoundingBox = Imgproc.boundingRect(contour);
            Point localSampleCenter = new Point((localBoundingBox.x + localBoundingBox.width / 2.0), (localBoundingBox.y + localBoundingBox.height / 2.0));
            double distance = Math.hypot(localSampleCenter.x - FRAME_CENTER_X, localSampleCenter.y - FRAME_CENTER_Y);
            double area = localBoundingBox.area();

            if (area < MIN_SAMPLE_AREA || distance > minDistance) {
                continue;
            }

            boundingBox = localBoundingBox;
            sampleCenter = localSampleCenter;
            minDistance = distance;

            sampleArea = localBoundingBox.area();
            guidanceRotationAngle = calculateRotationAngle(contour);

            if (autoSnapCenterLock != null) {
                if (Math.hypot(localSampleCenter.x - autoSnapCenterLock.x, localSampleCenter.y - autoSnapCenterLock.y) < SAMPLE_AUTOSNAP_RADIUS) {
                    Imgproc.circle(input, localSampleCenter, (int) SAMPLE_AUTOSNAP_RADIUS, new Scalar(0, 128, 0));

                    autoSnapHeartbeat = System.currentTimeMillis();
                    autoSnapCenterLock = localSampleCenter;
                    break;
                }

                if (System.currentTimeMillis() - autoSnapHeartbeat < SAMPLE_AUTOSNAP_LIFETIME) {
                    return autoSnapCenterLock;
                }
            }


            // add a rectangle showing we detected this sample
            Imgproc.rectangle(input, localBoundingBox, new Scalar(117, 38, 2), 1);
        }

        // Draw a rectangle around the detected sample
        if (boundingBox != null) {
            if (autoSnapCenterLock == null) {
                autoSnapCenterLock = sampleCenter;
                autoSnapHeartbeat = System.currentTimeMillis();
            }

            Imgproc.rectangle(input, boundingBox, new Scalar(117, 38, 191), 5);
        }

        return sampleCenter;
    }

    public @NotNull Pose getTargetPose(@NotNull Pose currentPose) {
        if (guidanceVector == null) {
            return currentPose;
        }

        return currentPose.addOnlyTranslational(guidanceVector);
    }

    public Vector2d calculateGuidanceVector(Point sampleCenter) {
        // Calculate the difference in pixels
        double xDiff = sampleCenter.x - FRAME_CENTER_X; // Positive means right, negative means left
        double yDiff = sampleCenter.y - FRAME_CENTER_Y; // Positive means down, negative means up

        // Convert pixel differences to applicable units
        double xMovement = xDiff * X_GUIDANCE_SCALE;
        double yMovement = yDiff * Y_GUIDANCE_SCALE;

        return new Vector2d(xMovement, yMovement); // Return guidance vector
    }

    private void annotateBoundingBox(Mat input, Point sampleCenter) {

        // Annotate the angle and servo position on the frame
        targetWristPosition = calculateServoPosition(currentWristPosition.get(), guidanceRotationAngle);

        Imgproc.putText(input, "Rotate: " + String.format("%.2f", guidanceRotationAngle) + " degrees", new Point(sampleCenter.x - 50, sampleCenter.y - 20),
                Imgproc.FONT_HERSHEY_SIMPLEX, 2, new Scalar(0, 0, 0), 3);

        Imgproc.putText(input, "Servo: " + String.format("%.2f", targetWristPosition), new Point(sampleCenter.x - 50, sampleCenter.y + 40),
                Imgproc.FONT_HERSHEY_SIMPLEX, 2, new Scalar(0, 0, 0), 3);

        Vector2d guidance = guidanceVector = calculateGuidanceVector(sampleCenter).plus(new Vector2d(PICKUP_X_ADJUSTMENT, PICKUP_Y_ADJUSTMENT));
        Imgproc.putText(input, "Area: " + sampleArea, new Point(sampleCenter.x - 50, sampleCenter.y + 150),
                Imgproc.FONT_HERSHEY_SIMPLEX, 2, new Scalar(0, 0, 0), 3);

        Imgproc.putText(input, "Guidance: " + guidance, new Point(guidanceVector.getX(), guidanceVector.getY()),
                Imgproc.FONT_HERSHEY_SIMPLEX, 2, new Scalar(0, 0, 0), 3);
    }

    private double calculateRotationAngle(MatOfPoint contour) {
        // Convert contour to MatOfPoint2f for the minAreaRect function
        MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());

        // Get the minimum area bounding rectangle for the contour
        RotatedRect rotRect = Imgproc.minAreaRect(contour2f);

        // Get the angle of the bounding rectangle
        double angle = rotRect.angle;

        // If width < height, the object is closer to vertical and the angle needs to be adjusted to correct to vertical orientation
        if (rotRect.size.width > rotRect.size.height) {
            angle += 90.0;
        }

        // Normalize the angle to [-90, 90] to get the smallest rotation required
        if (angle > 90) {
            angle -= 180.0;
        }

        return angle;
    }

    private double calculateServoPosition(double current, double rotationAngle) {
//        // Calculate the derivative (change) in rotation angle
//        double derivative = rotationAngle - previousRotationAngle;
//
//        // Store the current angle for the next calculation
//        previousRotationAngle = rotationAngle;

        // Calculate the servo adjustment based on the P and D terms
        double servoAdjustment = TURN_FACTOR * rotationAngle / 180;
        //double servoAdjustment = rotationAngle;

        // Apply the adjustment to the current servo position
        double newServoPosition = current + servoAdjustment;

        // Ensure the servo position stays within valid bounds [0, 1]
        newServoPosition = Range.clip(newServoPosition, 0.0, 1.0);

        return newServoPosition;
    }

    @Override
    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
        continuation.dispatch(bitmapConsumer -> {
            // Pass the last frame (Bitmap) to the consumer
            bitmapConsumer.accept(lastFrame.get());
        });
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        // Optional: If you want to draw additional information on the canvas
    }
}
