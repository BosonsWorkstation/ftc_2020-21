package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto State V1", group = "Linear Opmode")
public class AutoStateV1 extends RingDetectV2{

    private static final int CAMERA_X_POSITION = 75;
    private static final int CAMERA_Y_POSITION = 50;

    private static final double DEFAULT_POWER = 0.8;

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
    protected void leaveBase(){
        this.autoOmni.move(850, this.getDefaultPower());
        sleep(SLEEP_TIME);
        //Crab to avoid rings
        this.autoOmni.crab(-850, this.getDefaultPower());
        sleep(SLEEP_TIME);
    }

    @Override
    protected void getCloseToLine(){
        this.autoOmni.move(1950, 0.6);
        sleep(SLEEP_TIME);
    }

    @Override
    protected double getDefaultPower(){
        return DEFAULT_POWER;
    }
    @Override
    protected void crabToShootPos(){

    }

    @Override
    protected void launch(){
        this.autoOmni.powerLaunch();
    }

    @Override
    protected void runAutonomous(){
        SkystoneDeterminationPipeline.RingPosition ringPosition = getRingPosition();

        moveToLine();

        this.autoOmni.initDriveMotors(hardwareMap, telemetry);

        crabToBlue();

        this.autoOmni.crab(250, 0.2);

        this.autoOmni.initDriveMotors(hardwareMap, telemetry);

        lineDetect();
        sleep(SLEEP_TIME);

        this.autoOmni.move(-150, 0.2);
        sleep(SLEEP_TIME);

        this.autoOmni.rotate(120, 0.2);
        this.propel();
        sleep(SLEEP_TIME);
        this.autoOmni.rotate(75, 0.2);
        this.propel();
        sleep(SLEEP_TIME);
        this.autoOmni.rotate(75, 0.2);
        this.propel();
        sleep(SLEEP_TIME);
    }
}
