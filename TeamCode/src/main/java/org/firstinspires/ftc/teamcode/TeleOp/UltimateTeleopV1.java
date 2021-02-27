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



        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".

//
//        telemetry.addData("Mode", "calibrating...");
//        telemetry.update();
        this.driveTrain2.getHeading();
        this.driveTrain2.resetAngle();

        waitForStart();
        startPropellorControl(driveTrain2);


        while (opModeIsActive()) {


            double crabValue = 0;
            double moveValue = 0;
            double turnValue = 0;
            double maxPower = 0.8;




            if(gamepad1.left_bumper){
                maxPower = 0.4;

                crabValue = -gamepad1.left_stick_x / 1.5;
                moveValue = gamepad1.left_stick_y / 1.5;
                turnValue = -gamepad1.right_stick_x / 4;
            }
            else{
                maxPower = 0.8;

                crabValue = -gamepad1.left_stick_x * 1.5;
                moveValue = gamepad1.left_stick_y * 1.5;
                turnValue = -gamepad1.right_stick_x;
            }



            if (Math.abs(moveValue) < 0.1 && Math.abs(crabValue) < 0.1 && Math.abs(crabValue) < 0.1) {
                this.driveTrain2.stop();
            }


            this.driveTrain2.drive(crabValue, moveValue, turnValue, maxPower);
            idle();

            if(gamepad2.left_bumper){
                this.driveTrain2.towerOpen();
            }

            if(gamepad2.right_bumper){
                this.driveTrain2.towerClose();
            }

            if (gamepad1.a){
                this.driveTrain2.resetAngle();
            }

            if(gamepad2.b){
                this.driveTrain2.knockIntake();
            }

            if(gamepad2.x){
                this.driveTrain2.unknockIntake();
            }

            if (gamepad1.right_trigger > 0.3){
                this.driveTrain2.intake();
            }

            if(gamepad1.left_trigger > 0.3){
                this.driveTrain2.outtake();
            }

            if(gamepad1.left_trigger < 0.3 && gamepad1.right_trigger < 0.3){
                this.driveTrain2.intakeStop();
            }


            if(gamepad2.right_trigger > 0.3){
              this.driveTrain2.launch();
            }

            if(gamepad2.left_trigger > 0.3){
                this.driveTrain2.powerLaunch();
            }

            if(gamepad2.right_trigger < 0.3 && gamepad2.left_trigger < 0.3){
                this.driveTrain2.launchStop();
            }

            if(gamepad2.left_stick_y > 0.2 || gamepad2.left_stick_y < -0.2){
              this.driveTrain2.intakePower(gamepad2.left_stick_y);
            }



            if(gamepad2.dpad_right){
                this.driveTrain2.towerHandDown();
            }
            else if(gamepad2.dpad_left){
                this.driveTrain2.towerHandUp();
            }
            else{
                this.driveTrain2.towerHandStop();
            }
            if(gamepad2.dpad_up){
                this.driveTrain2.lightsRed();
            }

            if(gamepad2.dpad_down){
                this.driveTrain2.lightsOff();
            }
//            boolean turning = gamepad1.right_bumper;
            if(gamepad1.right_bumper){
                this.driveTrain2.turnTime(100);
//                turning = true;
            }

        }




        telemetry.addData("Intake Power", this.driveTrain2.intake.getPower());
        telemetry.update();
        Thread.sleep(50);


    }
    private void startPropellorControl(final OmniDriveTrainV2 driveTrain2) {
        Thread t1 = new Thread(new Runnable(){
           @Override
           public void run() {
               boolean buttonPressed = false;
               while (opModeIsActive()) {

                   if(gamepad2.y){
                       if(!buttonPressed) {
                           driveTrain2.propeller.setPosition(0.01);
                           sleep(770);
                           driveTrain2.propeller.setPosition(0.5);
                           sleep(100);
                       }
                       else{
                           sleep(20);
                       }
                       buttonPressed = true;
                   }
                   if(gamepad2.a){
                       driveTrain2.propeller.setPosition(0.99);
                       sleep(770);
                       driveTrain2.propeller.setPosition(0.5);
                       sleep(100);
                   }
                   else {
                       sleep(20);
                       buttonPressed = false;
                   }
               }
           }
        });
        t1.start();

    }


}


