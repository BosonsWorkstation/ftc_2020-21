package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;


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
    protected void leaveBase(SkystoneDeterminationPipeline.RingPosition ringPosition){
        this.autoOmni.move(getCorrectedDistance(850), this.getDefaultPower(ringPosition));
        sleep(SLEEP_TIME);
        //Crab to avoid rings
        this.autoOmni.crab(getCorrectedDistance(-850), this.getDefaultPower(ringPosition));
        sleep(SLEEP_TIME);
    }

    @Override
    protected void getCloseToLine(){
        this.autoOmni.move(getCorrectedDistance(1950), 0.6);
        sleep(SLEEP_TIME);
    }


    protected double getDefaultPower(SkystoneDeterminationPipeline.RingPosition ringPosition){
        //only 4 rings is power 1, others are 0.8
//        return ringPosition == SkystoneDeterminationPipeline.RingPosition.FOUR  ? 1:DEFAULT_POWER;

        //only 0 rings is power 0.8, others are 1
        return ringPosition == SkystoneDeterminationPipeline.RingPosition.NONE ? DEFAULT_POWER:0.9;
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
    protected void shootRings(){
        this.autoOmni.rotate(getCorrectedDistance(145), 0.4);
        this.propel();
//        sleep(SLEEP_TIME);
        this.autoOmni.rotate(getCorrectedDistance(65), 0.4);
        this.propel();
//        sleep(SLEEP_TIME);
        this.autoOmni.rotate(getCorrectedDistance(50), 0.4);
        this.propel();
        sleep(100);
        this.autoOmni.launchStop();
    }

    protected void shootRings(SkystoneDeterminationPipeline.RingPosition ringPosition){
        if(ringPosition == SkystoneDeterminationPipeline.RingPosition.FOUR){
            super.shootRings();
        }
        else{
            this.shootRings();
        }
    }

    @Override
    protected void runAutonomous(){
        SkystoneDeterminationPipeline.RingPosition ringPosition = getRingPosition();

        moveToLine(ringPosition);

        this.autoOmni.initDriveMotors();
        this.lineDetect(true, 0.1);

        this.autoOmni.move(getCorrectedDistance(-300), getDefaultPower());
        //Start Launcher Motors
        launch();
        sleep(100);
        this.autoOmni.crab(getCorrectedDistance(-300), 0.6);
        crabToBlue();

        this.autoOmni.crab(getCorrectedDistance(200), 0.3);

        this.autoOmni.initDriveMotors();
        lineDetect(true, 0.08);
        sleep(SLEEP_TIME);


        this.autoOmni.move(getCorrectedDistance(-250), 0.3);
        sleep(SLEEP_TIME);

       this.shootRings();

//       if(ringPosition != SkystoneDeterminationPipeline.RingPosition.NONE){
//           this.autoOmni.rotate(getCorrectedDistance(-200), 0.3);
//           sleep(SLEEP_TIME);
////        this.autoOmni.move(getCorrectedDistance(-100), this.getDefaultPower(ringPosition));
//
//           this.autoOmni.initDriveMotors(hardwareMap, telemetry);
//           this.lineDetect(true, 0.08);
//           sleep(SLEEP_TIME);
//       }


        this.dropWobble(ringPosition, 1);

        //low power
        this.wobbleDropThread(autoOmni, 4400, true);

        //high power
//        this.wobbleDropThread(autoOmni, 3700, true);

        this.autoOmni.initDriveMotors();
        if(ringPosition == SkystoneDeterminationPipeline.RingPosition.ONE){
            this.lineDetect(false, 0.15);
        }
        else{
            this.lineDetect(false, 0.08);
        }
        sleep(SLEEP_TIME);
        this.getSecondWobble(ringPosition);

        this.dropWobble(ringPosition, 2);
    }
}
