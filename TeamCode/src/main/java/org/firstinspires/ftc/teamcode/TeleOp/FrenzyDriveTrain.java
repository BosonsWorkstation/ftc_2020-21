package org.firstinspires.ftc.teamcode.TeleOp;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Autonomous.HeadingEnum;

import static java.lang.Thread.sleep;

public class FrenzyDriveTrain {
    protected DcMotor backRightWheel;
    protected DcMotor backLeftWheel;
    protected DcMotor frontRightWheel;
    protected DcMotor frontLeftWheel;

    protected ColorSensor colorLeft;
    protected ColorSensor colorRight;
    private static BNO055IMU imu;
    private static boolean gyroInitialized = false;

    private static final double MAX_POWER = 0.8;
    private Telemetry telemetry;
    float rotate_angle = 0;
    double reset_angle = 0;
    private double correction_factor = 0;

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

    public FrenzyDriveTrain(HardwareMap hardwareMap, Telemetry telemetry, FrenzyDriveTrain.DirectionEnum direction) {
        this.telemetry = telemetry;
        this.initializeGyro(hardwareMap, telemetry);
        this.initializeMotors(hardwareMap, telemetry);
        this.correction_factor = direction.getCorrection();
    }

    public void initializeMotors(HardwareMap hardwareMap, Telemetry telemetry) {
        //drive motors
        this.backLeftWheel = hardwareMap.dcMotor.get("Back_Left_Wheel");
        this.backRightWheel = hardwareMap.dcMotor.get("Back_Right_Wheel");
        this.frontLeftWheel = hardwareMap.dcMotor.get("Front_Left_Wheel");
        this.frontRightWheel = hardwareMap.dcMotor.get("Front_Right_Wheel");
        //attachment motors

    }

    public void initializeGyro(HardwareMap hardwareMap, Telemetry telemetry) {
//        if(!gyroInitialized) {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu.initialize(parameters);
//        }
//        gyroInitialized = true;
    }

    public void resetAngle(){
        reset_angle = getHeading() + reset_angle;
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
        return heading + correction_factor;

    }

    public void resetDriveMotors(){
        frontLeftWheel.setDirection(DcMotor.Direction.FORWARD);
        backLeftWheel.setDirection(DcMotor.Direction.FORWARD);
        frontRightWheel.setDirection(DcMotor.Direction.REVERSE);
        backRightWheel.setDirection(DcMotor.Direction.REVERSE);
    }

    public void useDriveEncoders(){
        frontLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void noDriveEncoders(){
        frontLeftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void driveRunToPosition(){
        frontLeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void driveSetPower(double power){
        frontLeftWheel.setPower(power);
        frontRightWheel.setPower(power);
        backLeftWheel.setPower(power);
        backRightWheel.setPower(power);
    }

    public void stopNow(){
        frontRightWheel.setPower(0);
        frontLeftWheel.setPower(0);
        backRightWheel.setPower(0);
        backLeftWheel.setPower(0);
    }

    public void stopSoft(double power){
        while(power > Math.abs(0.05)){
            power = power * 0.8;
//            try {
//                sleep(25);
//            } catch (InterruptedException e) {
//                Thread.currentThread().interrupt();
//            }
            driveSetPower(power);
        }
        stopNow();
    }

    //START MAIN DRIVE COMPONENTS
    public void autoRotate(int distance, double power){
        useDriveEncoders();

        int frontLeftPosition = frontLeftWheel.getCurrentPosition() ;
        int frontRightPosition = frontRightWheel.getCurrentPosition();
        int backLeftPosition = backLeftWheel.getCurrentPosition();
        int backRightPosition = backRightWheel.getCurrentPosition();

        frontLeftWheel.setTargetPosition(frontLeftPosition + distance);
        frontRightWheel.setTargetPosition(frontRightPosition + distance);
        backLeftWheel.setTargetPosition(backLeftPosition + distance);
        backRightWheel.setTargetPosition(backRightPosition + distance);

        driveRunToPosition();

        driveSetPower(power);

        while(frontLeftWheel.isBusy() && frontRightWheel.isBusy() && backLeftWheel.isBusy() && backRightWheel.isBusy()){
            try {
                sleep(5);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        stopNow(); //TODO CHANGE ALL TO stopSoft??

        //TODO Add noDriveEncoders??
    }

    public void autoMove(int distance, double power){
        useDriveEncoders();

        int frontLeftPosition = frontLeftWheel.getCurrentPosition() ;
        int frontRightPosition = frontRightWheel.getCurrentPosition();
        int backLeftPosition = backLeftWheel.getCurrentPosition();
        int backRightPosition = backRightWheel.getCurrentPosition();

        frontLeftWheel.setTargetPosition(frontLeftPosition - distance);
        frontRightWheel.setTargetPosition(frontRightPosition + distance);
        backLeftWheel.setTargetPosition(backLeftPosition - distance);
        backRightWheel.setTargetPosition(backRightPosition + distance);

        driveRunToPosition();

        driveSetPower(power);

        while(frontLeftWheel.isBusy() && frontRightWheel.isBusy() && backLeftWheel.isBusy() && backRightWheel.isBusy()){
            try {
                sleep(5);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        stopNow(); //TODO CHANGE ALL TO stopSoft??

        //TODO Add noDriveEncoders??
    }

    public void autoCrab(int distance, double power){
        useDriveEncoders();

        int frontLeftPosition = frontLeftWheel.getCurrentPosition() ;
        int frontRightPosition = frontRightWheel.getCurrentPosition();
        int backLeftPosition = backLeftWheel.getCurrentPosition();
        int backRightPosition = backRightWheel.getCurrentPosition();

        frontLeftWheel.setTargetPosition(frontLeftPosition - distance);
        frontRightWheel.setTargetPosition(frontRightPosition - distance);
        backLeftWheel.setTargetPosition(backLeftPosition + distance);
        backRightWheel.setTargetPosition(backRightPosition + distance);

        driveRunToPosition();

        driveSetPower(power);

        while(frontLeftWheel.isBusy() && frontRightWheel.isBusy() && backLeftWheel.isBusy() && backRightWheel.isBusy()){
            try {
                sleep(5);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        stopNow(); //TODO CHANGE ALL TO stopSoft??

        //TODO Add noDriveEncoders??
    }

    public void diagonal(HeadingEnum direction, int distance, double power){
        useDriveEncoders();
        //1 = forward + left; 2 = forward + right; 3 = backward + left; 4 = backward + right
        int frontLeftPosition = frontLeftWheel.getCurrentPosition() ;
        int frontRightPosition = frontRightWheel.getCurrentPosition();
        int backLeftPosition = backLeftWheel.getCurrentPosition();
        int backRightPosition = backRightWheel.getCurrentPosition();

        switch(direction){
            case SOUTH_EAST:
                frontRightWheel.setTargetPosition(frontRightPosition - distance);
                backLeftWheel.setTargetPosition(backLeftPosition + distance);

                frontRightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backLeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                frontRightWheel.setPower(power);
                backLeftWheel.setPower(power);

                break;
            case SOUTH_WEST:
                frontLeftWheel.setTargetPosition(frontLeftPosition + distance);
                backRightWheel.setTargetPosition(backRightPosition - distance);

                frontLeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backRightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                frontLeftWheel.setPower(power);
                backRightWheel.setPower(power);

                break;
            case NORTH_EAST:
                frontLeftWheel.setTargetPosition(frontLeftPosition - distance);
                backRightWheel.setTargetPosition(backRightPosition + distance);

                frontLeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backRightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                frontLeftWheel.setPower(power);
                backRightWheel.setPower(power);

                break;
            case NORTH_WEST:
                frontRightWheel.setTargetPosition(frontRightPosition + distance);
                backLeftWheel.setTargetPosition(backLeftPosition - distance);

                frontRightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backLeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                frontRightWheel.setPower(power);
                backLeftWheel.setPower(power);

                break;
        }

        while(frontLeftWheel.isBusy() && frontRightWheel.isBusy() && backLeftWheel.isBusy() && backRightWheel.isBusy()){
            try {
                Thread.sleep(20);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        stopNow();
//        stopSoft();
    }

    public void drive(double moveValue, double crabValue, double turnValue, double maxPower) {

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
        }
        gyroAngle = -1 * gyroAngle;


        theta = Math.atan2(stick_y, stick_x) - gyroAngle - (Math.PI / 2);
        Px = Math.sqrt(Math.pow(stick_x, 2) + Math.pow(stick_y, 2)) * (Math.sin(theta + Math.PI / 4));
        Py = Math.sqrt(Math.pow(stick_x, 2) + Math.pow(stick_y, 2)) * (Math.sin(theta - Math.PI / 4));


        frontLeftWheel.setPower(Py - Protate);
        backLeftWheel.setPower(Px - Protate);
        backRightWheel.setPower(Py + Protate);
        frontRightWheel.setPower(Px + Protate);


        telemetry.update();
    }

    public void  movePower (double power){
        frontLeftWheel.setPower(-power);
        frontRightWheel.setPower(power);
        backLeftWheel.setPower(-power);
        backRightWheel.setPower(power);
    }

    public void crabPower (double power){
        frontLeftWheel.setPower(-power);
        frontRightWheel.setPower(-power);
        backLeftWheel.setPower(power);
        backRightWheel.setPower(power);
    }

    public void rotatePower (double power){
        frontLeftWheel.setPower(power);
        frontRightWheel.setPower(power);
        backLeftWheel.setPower(power);
        backRightWheel.setPower(power);
    }
}
