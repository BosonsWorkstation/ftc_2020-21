package org.firstinspires.ftc.teamcode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "TESTBOT TELE", group = "Linear Opmode")
public class TESTBOT_TELE extends LinearOpMode{

    protected DcMotor leftWheel;
    protected DcMotor rightWheel;

    @Override
    public void runOpMode() throws InterruptedException {
        this.leftWheel = hardwareMap.dcMotor.get("Left_Wheel");
        this.rightWheel = hardwareMap.dcMotor.get("Right_Wheel");
        this.rightWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        this.leftWheel.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();

        while(opModeIsActive()){
            this.leftWheel.setPower(-(gamepad1.left_stick_y/2));
            this.rightWheel.setPower(-(gamepad1.right_stick_y/2));
        }
    }
}
