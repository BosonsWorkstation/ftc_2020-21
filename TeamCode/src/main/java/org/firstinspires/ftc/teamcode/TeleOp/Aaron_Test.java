package org.firstinspires.ftc.teamcode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Aaron Test", group = "Linear Opmode")

public class Aaron_Test extends LinearOpMode {
    protected DcMotor test_wheel;
    protected ColorSensor colorSensor;

    @Override
    public void runOpMode(){
        this.test_wheel = hardwareMap.dcMotor.get("Test_Motor");
        this.colorSensor = hardwareMap.colorSensor.get("Color_Sensor");

        waitForStart();

        while(opModeIsActive()){


            this.telemetry.addData("Red: ", this.colorSensor.red());
            this.telemetry.addData("Green: ", this.colorSensor.green());
            this.telemetry.addData("Blue: ", this.colorSensor.blue());
            telemetry.update();

            if(this.colorSensor.blue() > 100 && this.colorSensor.blue() < 500){
                test_wheel.setPower(0.5);
            }
            if(this.colorSensor.blue() > 500){
                test_wheel.setPower(-0.5);
            }
            if(this.colorSensor.blue() < 100){
                test_wheel.setPower(0);
            }
        }
    }
}
