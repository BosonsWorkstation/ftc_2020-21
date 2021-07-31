package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp (name = "Frenzy Teleop", group = "Linear Opmode")
public class FrenzyTeleop extends LinearOpMode {
    private FrenzyDriveTrain driveTrain;
    private static final FrenzyDriveTrain.DirectionEnum direction = FrenzyDriveTrain.DirectionEnum.SOUTH;

    @Override
    public void runOpMode() throws InterruptedException {
        this.driveTrain = new FrenzyDriveTrain(this.hardwareMap, this.telemetry, direction);

        this.driveTrain.initializeGyro(hardwareMap, telemetry);
        this.driveTrain.initializeMotors(hardwareMap, telemetry);

        this.driveTrain.getHeading();
        this.driveTrain.resetAngle();

        waitForStart();
        while (opModeIsActive()) {

            double crabValue = 0;
            double moveValue = 0;
            double turnValue = 0;
            double maxPower = 0.8;


            if (gamepad1.left_bumper) {
                maxPower = 0.4;

                crabValue = -gamepad1.left_stick_x / 1.5;
                moveValue = gamepad1.left_stick_y / 1.5;
                turnValue = -gamepad1.right_stick_x / 4;
            } else {
                maxPower = 0.8;

                crabValue = -gamepad1.left_stick_x * 1.5;
                moveValue = gamepad1.left_stick_y * 1.5;
                turnValue = -gamepad1.right_stick_x;
            }


            if (Math.abs(moveValue) < 0.1 && Math.abs(crabValue) < 0.1 && Math.abs(crabValue) < 0.1) {
                this.driveTrain.stopNow();
            }


            this.driveTrain.drive(crabValue, moveValue, turnValue, maxPower);
            idle();

            //GAMEPAD 1
            if (gamepad1.a){
                this.driveTrain.resetAngle();
            }


            //GAMEPAD 2
        }
    }
}
