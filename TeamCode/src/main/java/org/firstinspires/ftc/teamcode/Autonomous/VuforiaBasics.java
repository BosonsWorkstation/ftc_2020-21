package org.firstinspires.ftc.teamcode.Autonomous;

import android.icu.text.UFormat;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.teamcode.R;


@Autonomous(name = "VuforiaBasics")
public class VuforiaBasics extends LinearOpMode {

    VuforiaLocalizer vuforiaLocalizer;
    VuforiaLocalizer.Parameters parameters;
    VuforiaTrackables visionTargets;
    VuforiaTrackables target;
    VuforiaTrackableDefaultListener listener;
    OpenGLMatrix lastKnownLocation;
    OpenGLMatrix phonelocation;
    public static final String VUFORIA_KEY = "Ad/93Wv/////AAABmcg6QmLviUmMnbMCYrQjy3korvz10+kHAA1/XifbTNTCgiUL/mfrfkC6ag6F0Ii4pxn6CXTEOPupW6u3H/hFZpwbuHEWxmCxmsyhCg3vJRl9gQbVMehyVIO4GUAYIf3b7FeLNCj7QCLLYg4NBdYy76cf3jO1Hs7ioyOhAyPeFPag5hE1pD8WV41QN7SCJ+hDSq+uOjUfZLPUVNwz0L7YID7f5D3agcY0OEq10xaS/dtZN4mpeQg6ElehC2Yhofgj+nsQZtu9V/LRCgdEyKSxyJyvqYF2dKiXdINi0tOHQrJd9++p+nYHc2J5eieGt8vP9CLMg0PeZl/QOjw3h0BULvTO1D7VHlP/ff3kk6Mfe2Po";

    private float robotX = 0;
    private float robotY = 0;

    private float robotAngle = 0;

    public void runOpMode() throws InterruptedException{

        setupVuforia();

        lastKnownLocation = createMatrix(0,0,0,0,0,0);

        waitForStart();

        visionTargets.activate();

        while(opModeIsActive()){

            OpenGLMatrix latestLocation = listener.getUpdatedRobotLocation();

            if(latestLocation != null)
            {
                lastKnownLocation = latestLocation;
            }

            float[] coordinates = lastKnownLocation.getTranslation().getData();

            robotX = coordinates[0];
            robotY = coordinates[1];
            robotAngle = Orientation.getOrientation
                    (lastKnownLocation, AxesReference.EXTRINSIC, AxesOrder.XYX, AngleUnit.DEGREES).thirdAngle;

            telemetry.addData("Robot Location", robotAngle);
            telemetry.addData("Tracking", listener.isVisible());
            telemetry.addData("Last Known Location", formatMatrix(lastKnownLocation));

            telemetry.update();

            idle();
        }
    }
    public void setupVuforia(){
        parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        vuforiaLocalizer = ClassFactory.createVuforiaLocalizer(parameters);
        parameters.useExtendedTracking = false;

        WebcamName eyes;
        eyes = hardwareMap.get(WebcamName.class, "eyes");
        parameters.cameraName = eyes;

        visionTargets = vuforiaLocalizer.loadTrackablesFromAsset("UltimateGoal");

        VuforiaTrackable blueTower = visionTargets.get(0);
        blueTower.setName("Blue Tower Goal");
        blueTower.setLocation(createMatrix(0, 400, 0, 90, 0, 90));

        phonelocation = createMatrix(0, 200, 0, 90,0, 0);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        listener = (VuforiaTrackableDefaultListener) blueTower.getListener();


        listener.setPhoneInformation(phonelocation, parameters.cameraDirection);


        }
    public OpenGLMatrix createMatrix(float x, float y, float z, float u, float v, float w){
        return OpenGLMatrix.translation(x, y, z).
                multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES, u, v, w));
    }

    public String formatMatrix(OpenGLMatrix matrix)
    {
        return matrix.formatAsTransform();
    }

}
