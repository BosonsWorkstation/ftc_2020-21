package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.TeleOp.OmniDriveTrainV2;

public class AutoOmniDriveTrainV1 extends AutoOmniOther{
    int position;
    private static final double MOTOR_POWER = 0.5;
    private static final int TICKS_PER_REVOLUTION = 280;
    private static final double DISTANCE_PER_REVOLUTION = 4 * Math.PI;
    private static final double CRAB_POWER = .2;
//    Telemetry.Item currentPositionTel;
//    Telemetry.Item targetValueTel;



    public AutoOmniDriveTrainV1(HardwareMap hardwareMap, Telemetry telemetry){
        super(hardwareMap,telemetry);
//        currentPositionTel = telemetry.addData("Current Position", 0);
//        targetValueTel = telemetry.addData("Target ValueL", 0);

    }


    public void initMotors(){
        this.initMotor(frontRightWheel);

        this.initMotor(frontLeftWheel);

        this.initMotor(backLeftWheel);

        this.initMotor(backRightWheel);
    }

    private void initMotor(DcMotor motor) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public int getTargetValue(int distance){
        int targetValue = Math.round((float)(Math.abs(distance)/DISTANCE_PER_REVOLUTION)*TICKS_PER_REVOLUTION);
//        telemetry.clear();
//        targetValueTel.setValue("%d", targetValue);
//        telemetry.update();
        return targetValue;
    }

    public void  moveDistance (int distance) throws InterruptedException {
        int direction = distance > 0 ? -1 : 1;
        int targetValue = getTargetValue(distance);
        int currentPosition = 0;
        this.setMoveMotorDirection(direction);
        boolean done = false;


        this.stopNow();
    }


    public void newMoveDistance(int distance, double power) throws InterruptedException{
        int frontLeftPosition = frontLeftWheel.getCurrentPosition() ;
        int frontRightPosition = frontRightWheel.getCurrentPosition();
        int backLeftPosition = backLeftWheel.getCurrentPosition();
        int backRightPosition = backRightWheel.getCurrentPosition();

        frontLeftWheel.setTargetPosition(frontLeftPosition + distance);
        frontRightWheel.setTargetPosition(frontRightPosition + distance);
        backLeftWheel.setTargetPosition(backLeftPosition + distance);
        backRightWheel.setTargetPosition(backRightPosition + distance);

        frontLeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeftWheel.setPower(power);
        frontRightWheel.setPower(power);
        backLeftWheel.setPower(power);
        backRightWheel.setPower(power);

        while(frontLeftWheel.isBusy() || frontRightWheel.isBusy() || backLeftWheel.isBusy() || backRightWheel.isBusy()){
            telemetry.addData("Back Left", backLeftWheel.getCurrentPosition());
            telemetry.addData("Back Right", backRightWheel.getCurrentPosition());
            telemetry.addData("Front Left", frontLeftWheel.getCurrentPosition());
            telemetry.addData("Front Right", frontRightWheel.getCurrentPosition());
            telemetry.update();
        }

        stop();
    }

    public void crabRight(int distance, double power){
        int frontLeftPosition = frontLeftWheel.getCurrentPosition() ;
        int frontRightPosition = frontRightWheel.getCurrentPosition();
        int backLeftPosition = backLeftWheel.getCurrentPosition();
        int backRightPosition = backRightWheel.getCurrentPosition();

        frontLeftWheel.setTargetPosition(frontLeftPosition + distance);
        frontRightWheel.setTargetPosition(frontRightPosition + distance);
        backLeftWheel.setTargetPosition(backLeftPosition + distance);
        backRightWheel.setTargetPosition(backRightPosition + distance);

        frontLeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeftWheel.setPower(power);
        frontRightWheel.setPower(power);
        backLeftWheel.setPower(power);
        backRightWheel.setPower(power);

        while(frontLeftWheel.isBusy() || frontRightWheel.isBusy() || backLeftWheel.isBusy() || backRightWheel.isBusy()){
            telemetry.addData("Back Left", backLeftWheel.getCurrentPosition());
            telemetry.addData("Back Right", backRightWheel.getCurrentPosition());
            telemetry.addData("Front Left", frontLeftWheel.getCurrentPosition());
            telemetry.addData("Front Right", frontRightWheel.getCurrentPosition());
            telemetry.update();
        }

        stop();
    }

}
