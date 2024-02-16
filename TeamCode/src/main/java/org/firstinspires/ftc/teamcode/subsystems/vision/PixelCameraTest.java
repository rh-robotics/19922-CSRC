package org.firstinspires.ftc.teamcode.subsystems.vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class PixelCameraTest extends OpenCvPipeline {
    static final Rect LEFT_ROI = new Rect(
            new Point(15, 20),
            new Point(100, 80));
    static final Rect RIGHT_ROI = new Rect(
            new Point(205, 20),
            new Point(239, 80));
    static final Rect MID_ROI = new Rect(
            new Point(110, 20)
            ,
            new Point(200, 80));
    static double PERCENT_COLOR_THRESHOLD = 0.3;
    Telemetry telemetry;
    Mat mat = new Mat();
    private Location location;

    public PixelCameraTest(Telemetry t) {
        telemetry = t;
    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Scalar lowHSV = new Scalar(23, 50, 70);
        Scalar highHSV = new Scalar(32, 255, 255);
        //this is for blue, different vales will be needed to check for red
        //now its red

        Core.inRange(mat, lowHSV, highHSV, mat);

        Mat left = mat.submat(LEFT_ROI);
        Mat right = mat.submat(RIGHT_ROI);
        Mat middle = mat.submat(MID_ROI);

        double leftValue = Core.sumElems(left).val[0] / LEFT_ROI.area() / 255;
        double rightValue = Core.sumElems(right).val[0] / RIGHT_ROI.area() / 255;
        double midValue = Core.sumElems(middle).val[0] / MID_ROI.area() / 255;

        left.release();
        right.release();
        middle.release();

        telemetry.addData("Left raw value", (int) Core.sumElems(left).val[0]);
        telemetry.addData("Right raw value", (int) Core.sumElems(right).val[0]);
        telemetry.addData("Middle raw value", (int) Core.sumElems(middle).val[0]);

        telemetry.addData("Left percentage", Math.round(leftValue * 100) + "%");
        telemetry.addData("Right percentage", Math.round(rightValue * 100) + "%");
        telemetry.addData("Middle percentage", Math.round(midValue * 100) + "%");

        boolean stoneLeft = leftValue > PERCENT_COLOR_THRESHOLD;
        boolean stoneRight = rightValue > PERCENT_COLOR_THRESHOLD;
        boolean stoneMid = midValue > PERCENT_COLOR_THRESHOLD;

        if (stoneLeft && stoneRight) {
            location = Location.NOT_FOUND;
            telemetry.addData("Skystone Location", "not found");
        } else if (stoneLeft && stoneMid) {
            location = Location.NOT_FOUND;
            telemetry.addData("Skystone Location", "not found");
        } else if (stoneRight && stoneMid) {
            location = Location.NOT_FOUND;
            telemetry.addData("Skystone Location", "not found");
        } else if (stoneLeft) {
            location = Location.RIGHT;
            telemetry.addData("Box Location", "right");
        } else if (stoneMid) {
            location = Location.MIDDLE;
            telemetry.addData("Box Location", "right");
        } else {
            location = Location.LEFT;
            telemetry.addData("Box Location", "left");
        }
        telemetry.update();

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        Scalar colorStone = new Scalar(0, 0, 255);
        Scalar colorSkystone = new Scalar(0, 255, 0);

        Imgproc.rectangle(mat, LEFT_ROI, location == Location.LEFT ? colorSkystone : colorStone);
        Imgproc.rectangle(mat, RIGHT_ROI, location == Location.RIGHT ? colorSkystone : colorStone);
        Imgproc.rectangle(mat, MID_ROI, location == Location.MIDDLE ? colorSkystone : colorStone);

        return mat;
    }

    public Location getLocation() {
        return location;
    }

    public enum Location {
        LEFT,
        RIGHT,
        MIDDLE,
        NOT_FOUND
    }
}