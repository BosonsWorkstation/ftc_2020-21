package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.TeleOp.OmniDriveTrainV2;

@TeleOp(name = "Propellor", group = "Linear Opmode")
public class Propellor extends LinearOpMode {
    private OmniDriveTrainV2 driveTrain2;

    private static final OmniDriveTrainV2.DirectionEnum direction = OmniDriveTrainV2.DirectionEnum.SOUTH;
    public ColorSensor propellorColor;

    @Override
    public void runOpMode() throws InterruptedException {
        this.driveTrain2 = new OmniDriveTrainV2(this.hardwareMap, this.telemetry, direction);
        this.propellorColor = hardwareMap.get(ColorSensor.class, "propellor_color");

        waitForStart();
        telemetry.addData("starting robot", "now");
        startPropellorControl(driveTrain2);

        while (opModeIsActive()) {
            sleep(200);
        }


    }
    private boolean isColor(){
        return this.propellorColor.blue() > 100 || this.propellorColor.green() > 100 || this.propellorColor.red() > 100;
    }

    private void startPropellorControl(final OmniDriveTrainV2 driveTrain2) {
        Thread t1 = new Thread(new Runnable() {
            @Override
            public void run() {
                boolean buttonPressed = false;
                while (opModeIsActive()) {

                    if (gamepad2.y) {
                        if (!buttonPressed) {
                            telemetry.addData("propellor", .01);
                            driveTrain2.propeller.setPosition(0.01);
                            while (!isColor()) {
                                telemetry.addData("Color Blue", propellorColor.blue());
                                telemetry.addData("Color Red", propellorColor.red());
                                telemetry.addData("Color Green", propellorColor.green());

                                sleep(5);
                            }
                            telemetry.addData("propellor", .5);
                            driveTrain2.propeller.setPosition(0.5);
                            telemetry.update();
                        } else {
                            sleep(20);
                        }
                        buttonPressed = true;
                    }
                    if (gamepad2.a) {
                        driveTrain2.propeller.setPosition(0.99);
                        while (!isColor()) {
                            sleep(5);
                        }
                        driveTrain2.propeller.setPosition(0.5);
                    } else {
                        sleep(20);
                        buttonPressed = false;
                    }
                }
            }
        });
        t1.start();
    }
}