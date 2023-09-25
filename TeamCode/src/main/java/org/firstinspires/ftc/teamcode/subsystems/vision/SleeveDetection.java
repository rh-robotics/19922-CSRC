package org.firstinspires.ftc.teamcode.subsystems.vision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

/**
 * OpenCvPipeline to detect sleeves given color ranges (PowerPlay! code)
 */
public class SleeveDetection extends OpenCvPipeline {
    // Lower and upper boundaries for colors
    private static final Scalar
            lower_white_bounds = new Scalar(200, 200, 200, 255),
            upper_white_bounds = new Scalar(255, 255, 255, 255),
            lower_black_bounds = new Scalar(0, 0, 0, 0),
            upper_black_bounds = new Scalar(100, 100, 100, 255),
            lower_green_bounds = new Scalar(0, 70, 0, 255),
            upper_green_bounds = new Scalar(80, 255, 80, 255);
    static int X = 145; //145
    static int Y = 168;
    // TOPLEFT anchor point for the bounding box
    private static final Point SLEEVE_TOPLEFT_ANCHOR_POINT = new Point(X, Y);
    static int W = 30;//30

    /*
    white  = Parking Left
    black    = Parking Middle
    green = Parking Right
     */
    // Width and height for the bounding box
    public static int REGION_WIDTH = W;
    static int H = 50;//50
    public static int REGION_HEIGHT = H;
    // Color definitions
    private final Scalar
            WHITE = new Scalar(255, 255, 255),
            BLACK = new Scalar(0, 0, 0),
            GREEN = new Scalar(0, 255, 0);
    private final Mat whiMat = new Mat();
    private final Mat blaMat = new Mat();
    private final Mat greMat = new Mat();
    // Anchor point definitions
    Point sleeve_pointA = new Point(
            SLEEVE_TOPLEFT_ANCHOR_POINT.x,
            SLEEVE_TOPLEFT_ANCHOR_POINT.y);
    Point sleeve_pointB = new Point(
            SLEEVE_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            SLEEVE_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
    // Running variable storing the parking position
    int position = 0;
    // Percent and mat definitions
    private double whiPercent, blaPercent, grePercent;
    private Mat blurredMat = new Mat();

    // TODO: Update param. descriptions

    /**
     * Constructor for SleeveDetection class
     *
     * @param boundX The bound in the X direction for camera
     * @param boundY The bound in the Y direction for camera
     * @param width  Width
     * @param height Height
     */
    public SleeveDetection(int boundX, int boundY, int width, int height) {
        X = boundX;
        Y = boundY;
        W = width;
        H = height;
    }

    @Override
    public Mat processFrame(Mat input) {
        // Noise reduction
        Imgproc.blur(input, blurredMat, new Size(5, 5));
        blurredMat = blurredMat.submat(new Rect(sleeve_pointA, sleeve_pointB));

        // Apply Morphology
        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
        Imgproc.morphologyEx(blurredMat, blurredMat, Imgproc.MORPH_CLOSE, kernel);

        // Gets channels from given source mat
        Core.inRange(blurredMat, lower_white_bounds, upper_white_bounds, whiMat);
        Core.inRange(blurredMat, lower_black_bounds, upper_black_bounds, blaMat);
        Core.inRange(blurredMat, lower_green_bounds, upper_green_bounds, greMat);

        // Gets color specific values
        whiPercent = Core.countNonZero(whiMat);
        blaPercent = Core.countNonZero(blaMat);
        grePercent = Core.countNonZero(greMat);

        // Calculates the highest amount of pixels being covered on each side
        double maxPercent = Math.max(whiPercent, Math.max(blaPercent, grePercent));

        // Checks all percentages, will highlight bounding box in camera preview
        // based on what color is being detected
        if (maxPercent == whiPercent) {
            position = 2;
            Imgproc.rectangle(
                    input,
                    sleeve_pointA,
                    sleeve_pointB,
                    WHITE,
                    2
            );
        } else if (maxPercent == blaPercent) {
            position = 1;
            Imgproc.rectangle(
                    input,
                    sleeve_pointA,
                    sleeve_pointB,
                    BLACK,
                    2
            );
        } else if (maxPercent == grePercent) {
            position = 3;
            Imgproc.rectangle(
                    input,
                    sleeve_pointA,
                    sleeve_pointB,
                    GREEN,
                    2
            );
        }

        // Memory cleanup
        blurredMat.release();
        whiMat.release();
        blaMat.release();
        greMat.release();

        return input;
    }

    /** Returns an enum being the current position where the robot will park */
    public int getPosition() {
        return position;
    }

    public enum ParkingPosition {
        LEFT,
        CENTER,
        RIGHT
    }
}