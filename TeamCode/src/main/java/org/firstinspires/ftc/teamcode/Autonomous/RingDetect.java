package org.firstinspires.ftc.teamcode.Autonomous;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Autonomous.AutoOmniDriveTrainV1;
import org.firstinspires.ftc.teamcode.vision.EasyOpenCV;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

@Autonomous(name = "Ring Detect!", group = "Linear Opmode")
public class RingDetect extends EasyOpenCV
{

    private AutoOmniDriveTrainV1 autoOmni;

    OpenCvInternalCamera phoneCam;
    SkystoneDeterminationPipeline pipeline = new SkystoneDeterminationPipeline();



    @Override
    public void runOpMode()  {
        this.autoOmni = new AutoOmniDriveTrainV1(this.hardwareMap, this.telemetry);
       this.autoOmni.initialize(hardwareMap, telemetry);


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new SkystoneDeterminationPipeline();
        phoneCam.setPipeline(pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                phoneCam.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }
        });

        waitForStart();

        while (opModeIsActive())
        {

            sleep(1000);
            if (pipeline.position == SkystoneDeterminationPipeline.RingPosition.FOUR) {
                lineDetect();

            }

            if (pipeline.position == SkystoneDeterminationPipeline.RingPosition.ONE) {
                lineDetect();
            }

            if (pipeline.position == SkystoneDeterminationPipeline.RingPosition.NONE) {
                lineDetect();
            }


            this.autoOmni.colorDetect();


            telemetry.addData("Hue", this.autoOmni.hsvValues[0]);
            telemetry.addData("Blue ", this.autoOmni.colorRight.blue());
            telemetry.addData("Hue", this.autoOmni.hsvValues[0]);
            telemetry.addData("Left Red Val", this.autoOmni.colorLeft.red());
            telemetry.addData("Right Red Val", this.autoOmni.colorRight.red());
            telemetry.addData("Left Blue Val", this.autoOmni.colorLeft.blue());
            telemetry.addData("Right Blue Val", this.autoOmni.colorRight.blue());
            telemetry.addData("Left Green Val", this.autoOmni.colorLeft.green());
            telemetry.addData("Right Green Val", this.autoOmni.colorRight.green());
            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.addData("Position", pipeline.position);
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }
    }
    public void lineDetect(){
//        this.autoOmni.crab(-1000, 0.4);
//        sleep(100);
//        this.autoOmni.move(4000, 0.4);

        while(this.autoOmni.colorLeft.red() < 200 || this.autoOmni.colorRight.red() < 100){
            if(this.autoOmni.colorLeft.red() > 200 && this.autoOmni.colorRight.red() > 100){
                break;
            }
            if(this.autoOmni.colorLeft.red() > 200 ){
                this.autoOmni.rightCorrect(0.1);
            }
            if (this.autoOmni.colorRight.red() > 100)
            {
                this.autoOmni.leftCorrect(0.1);
            }
            if(this.autoOmni.colorLeft.red() > 200 && this.autoOmni.colorRight.red() > 100){
                break;
            }

            if (this.autoOmni.colorLeft.red() < 200 && this.autoOmni.colorRight.red() < 100){
                this.autoOmni.movePower(0.3);
            }
        }

        this.autoOmni.stopNow();



    }

    public static class SkystoneDeterminationPipeline extends OpenCvPipeline
    {
        /*
         * An enum to define the skystone position
         */
        public enum RingPosition
        {
            FOUR,
            ONE,
            NONE
        }

        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        /*
         * The core values which define the location and size of the sample regions
         */
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(195,50);


        //WE HAVE DOUBLED THIS FOR TESTING, THE ORIGINAL VALUES ARE 35 AND 25!!!!!!!!! -Shawn
        static final int REGION_WIDTH = 80;
        static final int REGION_HEIGHT = 70;

        final int FOUR_RING_THRESHOLD = 140;
        final int ZERO_RING_THRESHOLD = 128;

        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        /*
         * Working variables
         */
        Mat region1_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1;

        // Volatile since accessed by OpMode thread w/o synchronization
        public volatile RingPosition position = RingPosition.FOUR;

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        @Override
        public void init(Mat firstFrame)
        {
            inputToCb(firstFrame);

            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
        }

        @Override
        public Mat processFrame(Mat input)
        {
            inputToCb(input);

            avg1 = (int) Core.mean(region1_Cb).val[0];

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            position = RingPosition.NONE; // Record our analysis
            if(avg1 > FOUR_RING_THRESHOLD){
                position = RingPosition.FOUR;
            }else if (avg1 < ZERO_RING_THRESHOLD){
                position = RingPosition.NONE;
            }else{
                position = RingPosition.ONE;
            }

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill

            return input;
        }

        public int getAnalysis()
        {
            return avg1;
        }
    }
}