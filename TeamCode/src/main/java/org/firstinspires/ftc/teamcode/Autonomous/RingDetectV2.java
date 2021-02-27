package org.firstinspires.ftc.teamcode.Autonomous;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuforiaUltimateGoalNavigation;
import org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuforiaUltimateGoalNavigationWebcam;
import org.firstinspires.ftc.teamcode.Autonomous.AutoOmniDriveTrainV1;
import org.firstinspires.ftc.teamcode.vision.EasyOpenCV;
import org.firstinspires.ftc.teamcode.vision.Vuforia_Identifier;
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



@Autonomous(name = "Ring Detect V2", group = "Linear Opmode")
public class RingDetectV2 extends EasyOpenCV
{

    private AutoOmniDriveTrainV1 autoOmni;
    private Vuforia_Identifier vuforiaDetect;
    OpenCvInternalCamera phoneCam;
    SkystoneDeterminationPipeline pipeline = new SkystoneDeterminationPipeline();
    ConceptVuforiaUltimateGoalNavigationWebcam pictureDetect;

    private static final int SLEEP_TIME = 20;





    @Override
    public void runOpMode()  {

        this.autoOmni = new AutoOmniDriveTrainV1(this.hardwareMap, this.telemetry);
        this.autoOmni.initialize(hardwareMap, telemetry);
        this.vuforiaDetect = new Vuforia_Identifier();
        this.vuforiaDetect.initVuforia();
        this.pictureDetect = new ConceptVuforiaUltimateGoalNavigationWebcam();


                telemetry.setAutoClear(false);





        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
//        pipeline = new SkystoneDeterminationPipeline();
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
        SkystoneDeterminationPipeline.RingPosition ringPosition = null;
        if (opModeIsActive())
        {
            sleep(1000);

            if (pipeline.position == SkystoneDeterminationPipeline.RingPosition.FOUR) {
                ringPosition = SkystoneDeterminationPipeline.RingPosition.FOUR;
            }

            if (pipeline.position == SkystoneDeterminationPipeline.RingPosition.ONE) {
                ringPosition = SkystoneDeterminationPipeline.RingPosition.ONE;
            }

            if (pipeline.position == SkystoneDeterminationPipeline.RingPosition.NONE) {
                ringPosition = SkystoneDeterminationPipeline.RingPosition.NONE;
            }

            moveToLine();

//            this.autoOmni.movePower(0.2);
//            sleep(3000);
//            this.autoOmni.stopNow();
//            sleep(50);
            this.autoOmni.initDriveMotors(hardwareMap, telemetry);
            lineDetect();
            sleep(SLEEP_TIME);
            this.autoOmni.move(-550, 0.3);

            this.autoOmni.propel();
            sleep(100);
            this.autoOmni.propel();
            sleep(100);
            this.autoOmni.propel();
            sleep(100);
            this.autoOmni.propel();
//            this.autoOmni.tripleShoot();
            this.autoOmni.launchStop();
            this.autoOmni.initDriveMotors(hardwareMap, telemetry);
            lineDetect();

            switch (ringPosition){
                case NONE:
                    telemetry.addData(String.valueOf(ringPosition), "ZERO Rings Detected");
                    this.autoOmni.move(400, 0.4);
                    sleep(SLEEP_TIME);
                    this.autoOmni.crab(-700, 0.4);
                    this.autoOmni.autoTowerHand();

                    break;
                case ONE:
                    telemetry.addData(String.valueOf(ringPosition), "ONE Ring Detected");
                    this.autoOmni.crab(550,0.4);
                    sleep(SLEEP_TIME);
                    this.autoOmni.move(1400, 0.4);
                    this.autoOmni.autoTowerHand();
                    this.autoOmni.move(-1200, 0.6);
                    break;
                case FOUR:
                    telemetry.addData(String.valueOf(ringPosition), "FOUR Rings Detected");
                    this.autoOmni.move(2200, 0.4);
                    sleep(SLEEP_TIME);
                    this.autoOmni.crab(-700, 0.4);
                    this.autoOmni.move(-2000, 0.6);
                    break;
                default:
                    telemetry.addData(String.valueOf(ringPosition), "NO Rings Detected");

            }




            this.autoOmni.stopNow();





//            this.vuforiaDetect.vuforiaLine();

        }
        while(opModeIsActive()){
            sleep(100);
        }

    }

    private void moveToLine(){
        this.autoOmni.move(100, 0.3);
        sleep(SLEEP_TIME);
        this.autoOmni.crab(-450, 0.4);

        this.autoOmni.launch();

        sleep(SLEEP_TIME);
        this.autoOmni.move(2600, 0.4);
        sleep(SLEEP_TIME);
        this.autoOmni.crab(600, 0.4);
        this.autoOmni.stopNow();
    }

    private boolean isLeftWhite(){
        return this.autoOmni.colorLeft.red() > 600 && this.autoOmni.colorLeft.green() > 600
                && this.autoOmni.colorLeft.blue() > 600;
    }

    private boolean isRightWhite(){
        return this.autoOmni.colorRight.red() > 300 && this.autoOmni.colorLeft.green() > 300
                && this.autoOmni.colorRight.blue() > 300;
    }

    public void lineDetect(){
        boolean leftDone = false;
        boolean rightDone = false;
        this.autoOmni.movePower(0.2);
//        telemetry.setAutoClear(false);
//        telemetry.addData("Detecting Line", leftDone);
//        telemetry.update();
        while(!(leftDone && rightDone) && opModeIsActive()){
//            telemetry.addData("In While Loop", leftDone);
//            telemetry.addData("In While Loop", rightDone);
//            telemetry.update();

            if(this.isLeftWhite() && !leftDone ){
                this.autoOmni.rightCorrect(0.1);
                leftDone = true;
//                telemetry.addData("Left Done?", leftDone);
//                telemetry.update();
            }
            if (this.isRightWhite() && !rightDone)
            {
                this.autoOmni.leftCorrect(0.1);
                rightDone = true;
//                telemetry.addData("Right Done?", rightDone);
//                telemetry.update();
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
