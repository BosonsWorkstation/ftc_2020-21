package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

public class FrenzyDriveTrain {

    private final Telemetry telemetry;

    public FrenzyDriveTrain(HardwareMap hardwareMap, Telemetry telemetry, FrenzyDriveTrain.DirectionEnum direction) {
        this.telemetry = telemetry;
    }

    float rotate_angle = 0;
    double reset_angle = 0;

    protected DcMotor front_left_wheel = null;
    protected DcMotor back_left_wheel = null;
    protected DcMotor back_right_wheel = null;
    protected DcMotor front_right_wheel = null;

    private static BNO055IMU imu;

    public void initializeDriveMotors(HardwareMap hardwareMap) {
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


        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        imu.initialize(parameters);
    }

    public enum DirectionEnum{
        NORTH(90), SOUTH(-90), EAST(180), WEST(0);
        private double correction;
        DirectionEnum(double correction) {
            this.correction = correction;
        }
        public double getCorrection() {
            return correction;
        }
    }

    public void simpleF(double simplePower){
        front_left_wheel.setPower(-simplePower);
        back_left_wheel.setPower(-simplePower);
        back_right_wheel.setPower(-simplePower);
        front_right_wheel.setPower(-simplePower);
    }
    public void simpleL(double simplePower){
        front_left_wheel.setPower(simplePower);
        back_left_wheel.setPower(-simplePower);
        back_right_wheel.setPower(simplePower);
        front_right_wheel.setPower(-simplePower);
    }
    public void simpleB(double simplePower){
        front_left_wheel.setPower(simplePower);
        back_left_wheel.setPower(simplePower);
        back_right_wheel.setPower(simplePower);
        front_right_wheel.setPower(simplePower);
    }
    public void simpleR(double simplePower){
        front_left_wheel.setPower(-simplePower);
        back_left_wheel.setPower(simplePower);
        back_right_wheel.setPower(-simplePower);
        front_right_wheel.setPower(simplePower);
    }
    public void simpleRotate(double simplePower){
        front_left_wheel.setPower(-simplePower);
        back_left_wheel.setPower(-simplePower);
        back_right_wheel.setPower(simplePower);
        front_right_wheel.setPower(simplePower);
    }

    public void stopNow(){
        front_left_wheel.setPower(0);
        back_left_wheel.setPower(0);
        back_right_wheel.setPower(0);
        front_right_wheel.setPower(0);
    }

    public void reset_angle(){
        this.reset_angle = this.getHeading() + this.reset_angle;
    }



    public void drive(double crabValue, double moveValue, double turnValue, double maxPower) {
//        double Protate = gamepad1.right_stick_x / 4;
//        double stick_x = gamepad1.left_stick_x * Math.sqrt(Math.pow(1 - Math.abs(Protate), 2) / 2); //Accounts for Protate when limiting magnitude to be less than 1
//        double stick_y = gamepad1.left_stick_y * Math.sqrt(Math.pow(1 - Math.abs(Protate), 2) / 2);
        double Protate = turnValue;
        double stick_x = crabValue * Math.sqrt(Math.pow(1-Math.abs(Protate), 2)/2); //Accounts for Protate when limiting magnitude to be less than 1
        double stick_y = moveValue * Math.sqrt(Math.pow(1-Math.abs(Protate), 2)/2);
        double theta = 0;
        double Px = 0;
        double Py = 0;

        double gyroAngle = getHeading() * Math.PI / 180; //Converts gyroAngle into radians
        if (gyroAngle <= 0) {
            gyroAngle = gyroAngle + (Math.PI / 2);
        } else if (0 < gyroAngle && gyroAngle < Math.PI / 2) {
            gyroAngle = gyroAngle + (Math.PI / 2);
        } else if (Math.PI / 2 <= gyroAngle) {
            gyroAngle = gyroAngle - (3 * Math.PI / 2);
//            gyroAngle = gyroAngle - (Math.PI / 2);
        }
        gyroAngle = -1 * gyroAngle;

//        if (gamepad1.right_bumper) { //Disables gyro, sets to -Math.PI/2 so front is defined correctly.
//            gyroAngle = -Math.PI / 2;
//        }
        //MOVEMENT
        theta = Math.atan2(stick_y, stick_x) - gyroAngle - (Math.PI / 2);
        Px = Math.sqrt(Math.pow(stick_x, 2) + Math.pow(stick_y, 2)) * (Math.sin(theta + Math.PI / 4));
        Py = Math.sqrt(Math.pow(stick_x, 2) + Math.pow(stick_y, 2)) * (Math.sin(theta - Math.PI / 4));

        telemetry.addData("crab val", crabValue);
        telemetry.addData("move val", moveValue);
        telemetry.addData("turn val", turnValue);
        telemetry.addData("Magnitude", Math.sqrt(Math.pow(stick_x, 2) + Math.pow(stick_y, 2)));
        telemetry.addData("Front Left", Py - Protate);
        telemetry.addData("Back Left", Px - Protate);
        telemetry.addData("Back Right", Py + Protate);
        telemetry.addData("Front Right", Px + Protate);

        if(Math.abs(crabValue) < 0.2 && Math.abs(moveValue) < 0.2){
            front_left_wheel.setPower(-Protate);
            back_left_wheel.setPower(-Protate);
            back_right_wheel.setPower(Protate);
            front_right_wheel.setPower(Protate);
        }
        else{
            front_left_wheel.setPower(Py - Protate);
            back_left_wheel.setPower(Px - Protate);
            back_right_wheel.setPower(Py + Protate);
            front_right_wheel.setPower(Px + Protate);
        }
        telemetry.update();

    }

    public double getHeading(){
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double heading = angles.firstAngle;
        if(heading < -180) {
            heading = heading + 360;
        }
        else if(heading > 180){
            heading = heading - 360;
        }
        heading = heading - reset_angle;
        return heading;
    }
}