package org.firstinspires.ftc.teamcode.TeleOp;

import android.hardware.HardwareBuffer;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp (name = "Mecanum Test", group = "Linear Opmode")
public class MecanumTest extends LinearOpMode {


    private FrenzyDriveTrain driveTrain;
    private static final FrenzyDriveTrain.DirectionEnum direction = FrenzyDriveTrain.DirectionEnum.SOUTH;
    //   private static final FrenzyDriveTrain.DirectionEnum direction = FrenzyDriveTrain.DirectionEnum.EAST;
    protected DcMotor front_left_wheel = null;
    protected DcMotor back_left_wheel = null;
    protected DcMotor back_right_wheel = null;
    protected DcMotor front_right_wheel = null;

    @Override
    public void runOpMode() throws InterruptedException {
        front_left_wheel = hardwareMap.dcMotor.get("Front_Left_Wheel");
        back_left_wheel = hardwareMap.dcMotor.get("Back_Left_Wheel");
        back_right_wheel = hardwareMap.dcMotor.get("Back_Right_Wheel");
        front_right_wheel = hardwareMap.dcMotor.get("Front_Right_Wheel");

        front_left_wheel.setDirection(DcMotor.Direction.REVERSE);
        back_left_wheel.setDirection(DcMotor.Direction.REVERSE);
        front_right_wheel.setDirection(DcMotor.Direction.FORWARD);
        back_right_wheel.setDirection(DcMotor.Direction.FORWARD);

        front_left_wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left_wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right_wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right_wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.left_stick_y > 0.1 || gamepad1.left_stick_y < 0.1) {
                front_left_wheel.setPower(gamepad1.left_stick_y);
                back_left_wheel.setPower(gamepad1.left_stick_y);
                back_right_wheel.setPower(gamepad1.left_stick_y);
                front_right_wheel.setPower(gamepad1.left_stick_y);
            }
            if (gamepad1.left_stick_x > 0.1) {
                front_left_wheel.setPower(gamepad1.left_stick_x);
                back_left_wheel.setPower(-gamepad1.left_stick_x);
                back_right_wheel.setPower(gamepad1.left_stick_x);
                front_right_wheel.setPower(-gamepad1.left_stick_x);
            }
            if (gamepad1.left_stick_x < 0.1) {
                front_left_wheel.setPower(gamepad1.left_stick_x);
                back_left_wheel.setPower(-gamepad1.left_stick_x);
                back_right_wheel.setPower(gamepad1.left_stick_x);
                front_right_wheel.setPower(-gamepad1.left_stick_x);
            }
            if (gamepad1.right_stick_x > 0.1 || gamepad1.right_stick_x < 0.1) {
                front_left_wheel.setPower(-gamepad1.right_stick_y);
                back_left_wheel.setPower(-gamepad1.right_stick_y);
                back_right_wheel.setPower(gamepad1.right_stick_y);
                front_right_wheel.setPower(gamepad1.right_stick_y);
            }

            telemetry.addData("FrontLeft", front_left_wheel.getPower());
            telemetry.addData("FrontRight", front_right_wheel.getPower());
            telemetry.addData("BackLeft", back_left_wheel.getPower());
            telemetry.addData("BackRight", back_right_wheel.getPower());

            telemetry.update();


        }
    }
}
