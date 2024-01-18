package org.firstinspires.ftc.teamcode.subsystems.vision;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "testing")
public class visionTest2 extends LinearOpMode {

    maxLikesLemons maxLikesLemons = new maxLikesLemons();
    OpenCvCamera camera;
    String webcamName = "webcam";
    int i = 1;
    int lCount = 0;
    int mCount = 0;
    int rCount = 0;


    @Override
    public void runOpMode() throws InterruptedException {


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
        MaxIsAVictim maxIsAVictimL = new MaxIsAVictim(1);
        MaxIsAVictim maxIsAVictimM = new MaxIsAVictim(2);
        MaxIsAVictim maxIsAVictimR = new MaxIsAVictim(3);

        camera.setPipeline(maxIsAVictimL);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                if (i > 3) {
                    i = 1;
                }
                camera.startStreaming(320, 240, OpenCvCameraRotation.SENSOR_NATIVE);
                if (i == 1) {
                    camera.setPipeline(maxIsAVictimL);
                    if (maxIsAVictimL.getPosition() == 1) ;
                    {
                        lCount++;
                    }

                } else if (i == 2) {
                    camera.setPipeline(maxIsAVictimM);
                    if (maxIsAVictimM.getPosition() == 1) ;
                    {
                        mCount++;
                    }
                } else {
                    camera.setPipeline(maxIsAVictimR);
                    if (maxIsAVictimR.getPosition() == 1) ;
                    {
                        rCount++;
                    }
                }
                i++;
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        while (!isStarted()) {
            if (i > 3) {
                i = 1;
            }
            if (i == 1) {
                camera.setPipeline(maxIsAVictimL);
                if (maxIsAVictimL.getPosition() == 1) ;
                {
                    lCount++;
                }

            } else if (i == 2) {
                camera.setPipeline(maxIsAVictimM);
                if (maxIsAVictimM.getPosition() == 1) ;
                {
                    mCount++;
                }
            } else {
                camera.setPipeline(maxIsAVictimR);
                if (maxIsAVictimR.getPosition() == 1) ;
                {
                    rCount++;
                }
            }
            i++;


            telemetry.addData("Left: ", lCount);
            telemetry.addData("Mid: ", mCount);
            telemetry.addData("Right: ", rCount);
            if (Math.max(rCount, Math.max(mCount, lCount)) == lCount) {
                telemetry.addData("Pos", "Left");
            } else if (Math.max(rCount, Math.max(mCount, lCount)) == mCount) {
                telemetry.addData("Pos", "Middle");
            } else if (Math.max(rCount, Math.max(mCount, lCount)) == rCount) {
                telemetry.addData("Pos", "Right");
            } else {
                telemetry.addData("Pos", "Unknown");
            }
            if (rCount > 10000 || lCount > 10000 || mCount > 10000) {
                lCount = 0;
                mCount = 0;
                rCount = 0;
            }
            telemetry.update();
        }

        waitForStart();

    }
}