package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


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

    public void runOpMode() throws InterruptedException{




        waitForStart();

        while(opModeIsActive()){
            idle();
        }
    }
    public void SetupVuforia(){

    }

}
