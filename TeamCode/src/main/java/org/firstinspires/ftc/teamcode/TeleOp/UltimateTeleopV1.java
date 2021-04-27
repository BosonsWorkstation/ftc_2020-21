package org.firstinspires.ftc.teamcode.TeleOp;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Autonomous.AutoOmniDriveTrainV1;


import android.graphics.drawable.GradientDrawable;


@TeleOp(name = "Ultimate Teleop V1", group = "Linear Opmode")
public class UltimateTeleopV1 extends LinearOpMode {
    private OmniDriveTrainV2 driveTrain2;
    public ColorSensor propellorColor;

    private static final OmniDriveTrainV2.DirectionEnum direction = OmniDriveTrainV2.DirectionEnum.SOUTH;


    @Override
    public void runOpMode() throws InterruptedException {
        this.driveTrain2 = new OmniDriveTrainV2(this.hardwareMap, this.telemetry, direction);
        this.propellorColor = hardwareMap.get(ColorSensor.class, "propellor_color");

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

            //TESTINGGGG
            if(gamepad2.left_stick_button){
                this.driveTrain2.towerServoUp();
            }
            if (gamepad2.right_stick_button){
                this.driveTrain2.towerServoDown();
            }

            if(gamepad2.right_bumper){
                this.driveTrain2.towerClose();
            }

            if (gamepad1.a){
                this.driveTrain2.resetAngle();
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

//            if(gamepad2.left_stick_y > 0.2 || gamepad2.left_stick_y < -0.2){
//              this.driveTrain2.intakePower(gamepad2.left_stick_y);
//            }


            // Luke's preference
//            if(gamepad2.dpad_up){
//                this.driveTrain2.towerHandDown();
//            }
//            else if(gamepad2.dpad_down){
//                this.driveTrain2.towerHandUp();
//            }


            // Shawn's preference
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


            if(gamepad1.dpad_up){
                telemetry.setAutoClear(true);
                telemetry.addData("Left Red", this.driveTrain2.colorRight.red());
                telemetry.addData("Left Green", this.driveTrain2.colorRight.green());
                telemetry.addData("Left Blue", this.driveTrain2.colorRight.blue());

                telemetry.addData("Right Red", this.driveTrain2.colorLeft.red());
                telemetry.addData("Right Green", this.driveTrain2.colorLeft.green());
                telemetry.addData("Right Blue", this.driveTrain2.colorLeft.blue());
                telemetry.update();
            }

//            if(gamepad1.left_bumper){
////                launch();
////
////
////                crabToBlue();
////
////                this.autoOmni.crab(200, 0.2);
////
////                this.autoOmni.initDriveMotors(hardwareMap, telemetry);
////                lineDetect(true);
////                sleep(SLEEP_TIME);
////
////
////                this.autoOmni.move(-150, 0.2);
////                sleep(SLEEP_TIME);
//
//                this.driveTrain2.launch();
//
//                this.driveTrain2.initMotors();
//                this.driveTrain2.crabToBlue(false);
//
//                this.driveTrain2.autoCrab(200, 0.2);
//
//                this.driveTrain2.initMotors();
//                this.driveTrain2.lineDetect(true);
//                sleep(10);
//
//                this.driveTrain2.move(-150, 0.2);
//            }
//
//            if(gamepad1.right_bumper){
//                this.driveTrain2.autoShoot();
//            }

            this.isColor();
//            Thread.sleep(50);
        }






//        telemetry.addData("Intake Power", this.driveTrain2.intake.getPower());
//        telemetry.update();



    }


//    private boolean isColor(){
//        telemetry.addData("blue", propellorColor.blue());
//        telemetry.addData("green", propellorColor.green());
//        telemetry.addData("red", propellorColor.red());
//        return this.propellorColor.blue() > 40 || this.propellorColor.green() > 40 || this.propellorColor.red() > 40;
//    }

    private boolean isColor(){
        return this.propellorColor.blue() > 100 || this.propellorColor.green() > 100 || this.propellorColor.red() > 100;
    }

    private void runPropellorToColor(){
        sleep(100);
        long startTime = System.currentTimeMillis();
        while (!isColor()) {
            if(System.currentTimeMillis() - startTime > 2000){
                break;
            }
            sleep(5);
        }
        driveTrain2.propeller.setPosition(0.5);
    }

    private void propel(){
        driveTrain2.propeller.setPosition(0.01);
        this.runPropellorToColor();
    }

    private void reversePropel(){
        driveTrain2.propeller.setPosition(0.99);
        this.runPropellorToColor();
    }

    private void startPropellorControl(final OmniDriveTrainV2 driveTrain2) {
        Thread t1 = new Thread(new Runnable() {
            @Override
            public void run() {
                telemetry.setAutoClear(false);
                while (opModeIsActive()) {
                    if (gamepad2.y || gamepad1.y) {
                        propel();
                    } else if (gamepad2.a) {
                        reversePropel();
                    } else {
                        sleep(20);
                    }
                }
            }
        });
        t1.start();
    }

//    private void startPropellorControl(final OmniDriveTrainV2 driveTrain2) {
//        Thread t1 = new Thread(new Runnable(){
//           @Override
//           public void run() {
//               boolean buttonPressed = false;
//               telemetry.setAutoClear(false);
//               while (opModeIsActive()) {
//                   if (gamepad2.y) {
//                       if (!buttonPressed) {
////                           driveTrain2.propeller.setPosition(0.01);
//                           driveTrain2.propeller.setPosition(0.1);
//                           sleep(100);
//                           long startTime = System.currentTimeMillis();
//                           while (!isColor() || System.currentTimeMillis() - startTime < 2000) {
//                               sleep(5);
//                               telemetry.addData("waiting for color sensor", isColor());
//                               telemetry.update();
//
//                           }
//                           driveTrain2.propeller.setPosition(0.5);
//                           telemetry.update();
//                       } else {
//                           sleep(20);
//                       }
//                       buttonPressed = true;
//                   }
//                   if (gamepad2.a) {
//                       driveTrain2.propeller.setPosition(0.99);
//                       sleep(100);
//                       long startTime = System.currentTimeMillis();
//                       while (!isColor() || System.currentTimeMillis() - startTime < 2000) {
//                           sleep(5);
//                       }
//                       driveTrain2.propeller.setPosition(0.5);
//                   } else {
//                       sleep(20);
//                       buttonPressed = false;
//                   }
//               }
//               while (opModeIsActive()) {
//
//                   if(gamepad2.y){
//                       if(!buttonPressed) {
//                           driveTrain2.propeller.setPosition(0.01);
//                           sleep(770);
//                           driveTrain2.propeller.setPosition(0.5);
//                           sleep(100);
//                       }
//                       else{
//                           sleep(20);
//                       }
//                       buttonPressed = true;
//                   }
//                   if(gamepad2.a){
//                       driveTrain2.propeller.setPosition(0.99);
//                       sleep(770);
//                       driveTrain2.propeller.setPosition(0.5);
//                       sleep(100);
//                   }
//                   else {
//                       sleep(20);
//                       buttonPressed = false;
//                   }
//               }
//           }
//        });
//        t1.start();

//    }


}


