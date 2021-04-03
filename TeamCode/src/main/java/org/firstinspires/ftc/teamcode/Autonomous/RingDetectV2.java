package org.firstinspires.ftc.teamcode.Autonomous;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

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
public class RingDetectV2 extends LinearOpMode//extends EasyOpenCV
{

    private static final double DEFAULT_POWER = 0.4;
    protected AutoOmniDriveTrainV1 autoOmni;
    protected Vuforia_Identifier vuforiaDetect;
    OpenCvInternalCamera phoneCam;
    private static final double CAMERA_X_POSITION = 195;
    private static final double CAMERA_Y_POSITION = 50;

    SkystoneDeterminationPipeline pipeline = null;

    ConceptVuforiaUltimateGoalNavigationWebcam pictureDetect;
    public ColorSensor propellorColor;

    static final int SLEEP_TIME = 35;

    protected void createPipeline(double xPosition, double yPosition){
        pipeline = new SkystoneDeterminationPipeline(xPosition, yPosition);
    }

    protected void initOpMode(double xPosition, double yPosition){
        this.createPipeline(xPosition, yPosition);
        this.autoOmni = new AutoOmniDriveTrainV1(this.hardwareMap, this.telemetry);
        this.autoOmni.initialize(hardwareMap, telemetry);
        this.vuforiaDetect = new Vuforia_Identifier();
        this.vuforiaDetect.initVuforia();
        this.pictureDetect = new ConceptVuforiaUltimateGoalNavigationWebcam();
        this.propellorColor = hardwareMap.get(ColorSensor.class, "propellor_color");

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
    }


    protected SkystoneDeterminationPipeline.RingPosition getRingPosition(){
        SkystoneDeterminationPipeline.RingPosition ringPosition = null;

        sleep(1000);

        if (pipeline.position == SkystoneDeterminationPipeline.RingPosition.FOUR) {
            ringPosition = SkystoneDeterminationPipeline.RingPosition.FOUR;
            telemetry.addData("4 Rings Detected!", ringPosition);
            telemetry.update();
        }

        if (pipeline.position == SkystoneDeterminationPipeline.RingPosition.ONE) {
            ringPosition = SkystoneDeterminationPipeline.RingPosition.ONE;
            telemetry.addData("1 Ring Detected!", ringPosition);
            telemetry.update();
        }

        if (pipeline.position == SkystoneDeterminationPipeline.RingPosition.NONE) {
            ringPosition = SkystoneDeterminationPipeline.RingPosition.NONE;
            telemetry.addData("0 Rings Detected!", ringPosition);
            telemetry.update();
        }
        return ringPosition;
    }

    @Override
    public void runOpMode()  {

        initOpMode(CAMERA_X_POSITION, CAMERA_Y_POSITION);

        waitForStart();

       runAutonomous();

        while(opModeIsActive()){
            sleep(100);
        }
    }

    protected void runAutonomous(){
        if (opModeIsActive())
        {

            SkystoneDeterminationPipeline.RingPosition ringPosition = getRingPosition();

            moveToLine();

            this.autoOmni.initDriveMotors(hardwareMap, telemetry);

            lineDetect();
            sleep(SLEEP_TIME);

            //Backup to get to launch position
            this.autoOmni.move(-250, 0.3);

            sleep(SLEEP_TIME);


            this.propel();
            this.propel();
            this.propel();
            this.propel();

            this.autoOmni.launchStop();
            this.autoOmni.initDriveMotors(hardwareMap, telemetry);
            lineDetect();

            switch (ringPosition){
                case NONE:
                    telemetry.addData(String.valueOf(ringPosition), "ZERO Rings Detected");
                    this.autoOmni.move(300, this.getDefaultPower());
                    sleep(SLEEP_TIME);
                    this.autoOmni.crab(-400, this.getDefaultPower());
                    this.autoOmni.autoTowerHand();
                    sleep(200);
                    this.autoOmni.crab(200, this.getDefaultPower());

                    break;
                case ONE:
                    telemetry.addData(String.valueOf(ringPosition), "ONE Ring Detected");
                    this.autoOmni.crab(700,this.getDefaultPower());
                    sleep(SLEEP_TIME);
                    this.autoOmni.move(1400, this.getDefaultPower());
                    this.autoOmni.autoTowerHand();
                    sleep(200);
                    this.autoOmni.crab(200, this.getDefaultPower());
                    this.autoOmni.move(-900, 0.6);
                    break;
                case FOUR:
                    telemetry.addData(String.valueOf(ringPosition), "FOUR Rings Detected");
                    this.autoOmni.move(2250, this.getDefaultPower());
                    sleep(SLEEP_TIME);
                    this.autoOmni.crab(-475, this.getDefaultPower());
                    this.autoOmni.autoTowerHand();
                    sleep(200);
                    this.autoOmni.crab(200, this.getDefaultPower());
                    this.autoOmni.move(-2000, 0.6);
                    break;
                default:
                    telemetry.addData(String.valueOf(ringPosition), "NO Rings Detected");

            }
            this.autoOmni.stopNow();
        }


    }

    protected double getDefaultPower(){
        return DEFAULT_POWER;

    }

    protected void leaveBase(){
        this.autoOmni.move(100, 0.3);
        sleep(SLEEP_TIME);
        //Crab to avoid rings
        this.autoOmni.crab(-450, this.getDefaultPower());
        sleep(SLEEP_TIME);
    }

    protected void getCloseToLine(){
        this.autoOmni.move(2700, this.getDefaultPower());
        sleep(SLEEP_TIME);
    }

    protected void crabToShootPos(){
        this.autoOmni.crab(650, this.getDefaultPower());
        this.autoOmni.stopNow();
    }

    protected void launch(){
        this.autoOmni.launch();
    }

    protected void moveToLine(){
        //Leave Base
        leaveBase();
        //Start Launcher Motors
        launch();
        //Move close enough to the white line
        getCloseToLine();
        //Crab to line for shooting rings
        crabToShootPos();
    }

    protected final void crabToBlue(){
        telemetry.setAutoClear(false);
        while (opModeIsActive()) {
            if (this.autoOmni.colorRight.blue() < 200) {
                this.autoOmni.crabPower(0.1);
                telemetry.addData("Left Red", this.autoOmni.colorLeft.red());
                telemetry.addData("Left Green", this.autoOmni.colorLeft.green());
                telemetry.addData("Left Blue", this.autoOmni.colorLeft.blue());
                telemetry.update();
            } else {
                this.autoOmni.stopNow();
                break;
            }
            sleep(25);
        }
    }

    private boolean isLeftWhite(){
        return this.autoOmni.colorLeft.red() > 600 && this.autoOmni.colorLeft.green() > 600
                && this.autoOmni.colorLeft.blue() > 600;
    }

    private boolean isRightWhite(){
        return this.autoOmni.colorRight.red() > 300 && this.autoOmni.colorLeft.green() > 300
                && this.autoOmni.colorRight.blue() > 300;
    }

    public void lineDetect() {
        boolean leftDone = false;
        boolean rightDone = false;
        this.autoOmni.movePower(0.1);
        while(!(leftDone && rightDone) && opModeIsActive()){
            if(this.isLeftWhite() && !leftDone ){
                this.autoOmni.rightCorrect(0.1);
                leftDone = true;
            }
            if (this.isRightWhite() && !rightDone)
            {
                this.autoOmni.leftCorrect(0.1);
                rightDone = true;
            }

        }
        this.autoOmni.stopNow();
    }

    private boolean isColor(){
        return this.propellorColor.blue() > 100 || this.propellorColor.green() > 100 || this.propellorColor.red() > 100;
    }
    protected void propel(){
        autoOmni.propeller.setPosition(0.01);
        sleep(100);
        while (!isColor()) {
            sleep(5);
        }
        autoOmni.propeller.setPosition(0.5);
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
//        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(195,50);
        private Point REGION1_TOPLEFT_ANCHOR_POINT = null;

        //WE HAVE DOUBLED THIS FOR TESTING, THE ORIGINAL VALUES ARE 35 AND 25!!!!!!!!! -Shawn
        static final int REGION_WIDTH = 80;
        static final int REGION_HEIGHT = 70;

        final int FOUR_RING_THRESHOLD = 140;
        final int ZERO_RING_THRESHOLD = 128;

//        Point region1_pointA = new Point(
//                REGION1_TOPLEFT_ANCHOR_POINT.x,
//                REGION1_TOPLEFT_ANCHOR_POINT.y);
//        Point region1_pointB = new Point(
//                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
//                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        Point region1_pointA = null;
        Point region1_pointB = null;

        /*
         * Working variables
         */
        Mat region1_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();

        int avg1;

        // Volatile since accessed by OpMode thread w/o synchronization
        public volatile RingPosition position = RingPosition.FOUR;
        public SkystoneDeterminationPipeline(double xPosition, double yPosition){
            super();
            this.REGION1_TOPLEFT_ANCHOR_POINT = new Point(xPosition, yPosition);
            this.region1_pointA = new Point(
                    REGION1_TOPLEFT_ANCHOR_POINT.x,
                    REGION1_TOPLEFT_ANCHOR_POINT.y);
            this.region1_pointB = new Point(
                    REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                    REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
        }
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
