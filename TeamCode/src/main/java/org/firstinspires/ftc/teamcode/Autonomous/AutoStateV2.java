package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;


@Autonomous(name = "Auto State V2", group = "Linear Opmode")
public class AutoStateV2 extends AutoStateV1 {

    private static final int CAMERA_X_POSITION = 20;
    private static final int CAMERA_Y_POSITION = 50;

    @Override
    public void runOpMode() {
        initOpMode(CAMERA_X_POSITION, CAMERA_Y_POSITION);

        waitForStart();
        runAutonomous();

        while(opModeIsActive()){
            sleep(100);
        }
    }

    @Override
    protected void crabToBlue(){
        this.autoOmni.crab(getCorrectedDistance(-200), 0.6);
        this.autoOmni.initDriveMotors(hardwareMap, telemetry);
        this.crabToBlue(false);
    }

    @Override
    protected void moveToLine(SkystoneDeterminationPipeline.RingPosition ringPosition){
        switch (ringPosition){
            case NONE:
                this.autoOmni.move(getCorrectedDistance(2950), 0.6);
                break;
            case ONE:
            case FOUR:
                this.autoOmni.move(getCorrectedDistance(2650), this.getDefaultPower(ringPosition));
                break;

//            case FOUR:
//                this.autoOmni.diagonal(HeadingEnum.NORTH_EAST, 2000, this.getDefaultPower());
//                sleep(SLEEP_TIME);
//                this.autoOmni.diagonal(HeadingEnum.NORTH_WEST, 3100, this.getDefaultPower());
//                sleep(SLEEP_TIME);
//                this.autoOmni.crab(-400, this.getDefaultPower());
        }
    }

    @Override
    protected void getBackToWhiteLine(SkystoneDeterminationPipeline.RingPosition ringPosition){
        switch (ringPosition){
            case NONE:
                this.autoOmni.crab(getCorrectedDistance(1400), this.getDefaultPower(ringPosition));
                break;
            case ONE:
                this.autoOmni.crab(getCorrectedDistance(200), this.getDefaultPower(ringPosition));
                sleep(SLEEP_TIME);
                this.autoOmni.move(getCorrectedDistance(-1100), this.getDefaultPower(ringPosition));
                break;
            case FOUR:
                this.autoOmni.diagonal(HeadingEnum.SOUTH_EAST, getCorrectedDistance(3000), this.getDefaultPower(ringPosition));
                this.autoOmni.move(getCorrectedDistance(-400), this.getDefaultPower(ringPosition));
                break;
            default:
                telemetry.addData(String.valueOf(ringPosition), "NO Rings Detected");

        }
    }

}

