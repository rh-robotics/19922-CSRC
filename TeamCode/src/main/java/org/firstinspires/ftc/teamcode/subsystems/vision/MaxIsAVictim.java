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
public class MaxIsAVictim extends OpenCvPipeline {
    // Lower and upper boundaries for colors
    private static final Scalar
            lower_blue_bounds = new Scalar(30, 30, 200, 255),
            upper_blue_bounds = new Scalar(70, 70, 255, 255),
            lower_red_bounds = new Scalar(100, 0, 0 ,255),
            upper_red_bounds = new Scalar(255, 80, 80 ,255),
            lower_none_bounds = new Scalar(70, 100, 80 ,255),
            upper_none_bounds = new Scalar(100, 255, 200 ,255);

    int  i;
    public MaxIsAVictim(int iteration) {
        i  =iteration;
    }
    static int X = 145; //145
    static int Y = 0;
    // TOPLEFT anchor point for the bounding box


    private static  Point SLEEVE_TOPLEFT_ANCHOR_POINT = new Point(X, Y);
    static int W = 100;//30

    // Width and height for the bounding box
    public static int REGION_WIDTH = W;
    static int H = 80;//50
    public static int REGION_HEIGHT = H;
    // Color definitions
    private final Scalar
            BLUE = new Scalar(0, 0, 255),
            RED = new Scalar(255, 0, 0),
            NONE = new Scalar(255,255,255);
    private final Mat bluMat = new Mat();
    private final Mat redMat = new Mat();
    private final Mat noneMat = new Mat();

    // Anchor point definitions
    /*Point sleeve_pointA = new Point(
            SLEEVE_TOPLEFT_ANCHOR_POINT.x,
            SLEEVE_TOPLEFT_ANCHOR_POINT.y); */
   /* Point sleeve_pointB = new Point(
            SLEEVE_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            SLEEVE_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);*/
    // Running variable storing the parking position
    int position = 0;
    // Percent and mat definitions
    private double bluPercent, redPercent, nonePercent;
    private Mat blurredMat = new Mat();

    // TODO: Update param. descriptions

    /**
     * Constructor for SleeveDetection class
     *
     */


    @Override
    public Mat processFrame(Mat input) {




        // Noise reduction
        Rect sleeveRect;
        Imgproc.blur(input, blurredMat, new Size(5, 5));
        if (i ==1) {
             sleeveRect = new Rect(0,0,50,80);
            blurredMat = blurredMat.submat(sleeveRect);
        }
        else if (i == 2){
             sleeveRect = new Rect(80,0,50,80);
            blurredMat = blurredMat.submat(sleeveRect);
        }
        else {
             sleeveRect = new Rect(160,0,50,80);
            blurredMat = blurredMat.submat(sleeveRect);
        }

        // Apply Morphology
        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
        Imgproc.morphologyEx(blurredMat, blurredMat, Imgproc.MORPH_CLOSE, kernel);

        // Gets channels from given source mat
        Core.inRange(blurredMat, lower_blue_bounds, upper_blue_bounds, bluMat);
        Core.inRange(blurredMat, lower_red_bounds, upper_red_bounds, redMat);
        Core.inRange(blurredMat, lower_none_bounds, upper_none_bounds, noneMat);

        // Gets color specific values
        bluPercent = Core.countNonZero(bluMat);
        redPercent = Core.countNonZero(redMat);
        nonePercent = Core.countNonZero(noneMat);

        // Calculates the highest amount of pixels being covered on each side
        double maxPercent = Math.max(bluPercent, redPercent);

        // Checks all percentages, will highlight bounding box in camera preview
        // based on what color is being detected
        if (maxPercent == bluPercent) {
            position = 2;
            Imgproc.rectangle(
                    input,
                    sleeveRect,
                    BLUE,
                    2
            );
        } else if (maxPercent == redPercent) {
            position = 1;
            Imgproc.rectangle(
                    input,
                    sleeveRect,
                    RED,
                    2
            );
        }
        else{
            position = 0;
            Imgproc.rectangle(
                    input,
                    sleeveRect,
                    NONE,
                    2);
        }
        // Memory cleanup
        blurredMat.release();
        bluMat.release();
        redMat.release();
        noneMat.release();

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