package org.firstinspires.ftc.teamcode.TeleOp;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;



import android.graphics.drawable.GradientDrawable;


@TeleOp(name = "Ultimate Teleop V1", group = "Linear Opmode")
public class UltimateTeleopV1 extends LinearOpMode {
    private OmniDriveTrainV2 driveTrain2;

    private static final OmniDriveTrainV2.DirectionEnum direction = OmniDriveTrainV2.DirectionEnum.SOUTH;


    @Override
    public void runOpMode() throws InterruptedException {
        this.driveTrain2 = new OmniDriveTrainV2(this.hardwareMap, this.telemetry, direction);

        this.driveTrain2.initializeGyro(hardwareMap, telemetry);
        this.driveTrain2.initializeMotors(hardwareMap, telemetry);
//        this.blockArm = new SkyStoneStacker(this.hardwareMap, this.telemetry);


        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".

//
//        telemetry.addData("Mode", "calibrating...");
        telemetry.update();
        this.driveTrain2.getHeading();

        waitForStart();


        while (opModeIsActive()) {


            double crabValue = 0;
            double moveValue = 0;
            double turnValue = 0;

            crabValue = -gamepad1.left_stick_x;
            moveValue = gamepad1.left_stick_y;
            turnValue = gamepad1.right_stick_x;

            if (Math.abs(moveValue) < 0.1 && Math.abs(crabValue) < 0.1 && Math.abs(crabValue) < 0.1) {
                this.driveTrain2.stop();
            }


            this.driveTrain2.drive(crabValue, moveValue, turnValue);
            idle();

            if (gamepad1.a){
                this.driveTrain2.resetAngle();
            }

            if (gamepad2.left_trigger > 0.3){
                this.driveTrain2.intake();
            }

            if(gamepad2.left_bumper){
                this.driveTrain2.outtake();
            }

            if(gamepad2.left_trigger < 0.3 && !gamepad2.left_bumper){
                this.driveTrain2.intakeStop();
            }

            if(gamepad2.right_trigger > 0.3){
                this.driveTrain2.propel();
                this.driveTrain2.launch();
            }
            else{
                this.driveTrain2.launchStop();
            }
        }
        telemetry.update();





        telemetry.update();
        Thread.sleep(50);


    }

}


