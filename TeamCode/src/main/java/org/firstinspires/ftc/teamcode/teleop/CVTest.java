package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystems.vision.SleeveDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name="Vision Test ", group="Auto")
public class CVTest extends LinearOpMode {
    OpenCvCamera camera;
    String webcamName = "Webcam 1";
    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
        PixelCameraTest detector = new PixelCameraTest(telemetry);
        camera.setPipeline(detector);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode) {}
        });


        waitForStart();
            switch (detector.getLocation()) {
                case LEFT:
                    telemetry.addData("Object Position", "Left");
                    break;
                case RIGHT:
                    telemetry.addData("Object Position", "Right");
                    break;
                case MIDDLE:
                    telemetry.addData("Object Position", "Middle");
                    break;
                case NOT_FOUND:
                    telemetry.addData("Object Position", "Unknown!!!");

            }
            telemetry.update();



    }
}
