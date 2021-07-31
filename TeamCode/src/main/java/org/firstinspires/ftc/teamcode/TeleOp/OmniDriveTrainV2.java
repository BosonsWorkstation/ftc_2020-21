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


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import static java.lang.Thread.sleep;


public class OmniDriveTrainV2 {

    protected DcMotor backRightWheel;
    protected DcMotor backLeftWheel;
    protected DcMotor frontRightWheel;
    protected DcMotor frontLeftWheel;
    protected DcMotor towerHand;
    protected Servo towerGrasp;
    protected DcMotor intake;
    protected DcMotor launcherL;
    protected DcMotor launcherR;
    protected Servo propeller;
    protected Servo intakeServo;
    protected Servo towerServo;
    protected  RevBlinkinLedDriver lights;
    protected ColorSensor colorLeft;
    protected ColorSensor colorRight;
    protected ColorSensor propellorColor;


    private static BNO055IMU imu;
    private static boolean gyroInitialized = false;

    private static final double MAX_POWER = 0.8;
    private Telemetry telemetry;
    float rotate_angle = 0;
    double reset_angle = 0;
    private double correction_factor = 0;

//    HardwareMap hardwareMap = null;

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

    public OmniDriveTrainV2(HardwareMap hardwareMap, Telemetry telemetry, DirectionEnum direction) {
        this.telemetry = telemetry;
        this.initializeGyro(hardwareMap, telemetry);
        this.initializeMotors(hardwareMap, telemetry);
        this.correction_factor = direction.getCorrection();
    }

    public void initializeMotors(HardwareMap hardwareMap, Telemetry telemetry) {
        this.backLeftWheel = hardwareMap.dcMotor.get("Back_Left_Wheel");
        this.backRightWheel = hardwareMap.dcMotor.get("Back_Right_Wheel");
        this.frontLeftWheel = hardwareMap.dcMotor.get("Front_Left_Wheel");
        this.frontRightWheel = hardwareMap.dcMotor.get("Front_Right_Wheel");
        this.towerHand = hardwareMap.dcMotor.get("Tower_Hand");
        this.intake = hardwareMap.dcMotor.get("Intake");
        this.launcherL = hardwareMap.dcMotor.get("Launcher_Left");
        this.launcherR = hardwareMap.dcMotor.get("Launcher_Right");
        this.lights = hardwareMap.get(RevBlinkinLedDriver.class, "lights");
        this.launcherL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.launcherR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.propeller = hardwareMap.servo.get("Propeller");
        this.intakeServo = hardwareMap.servo.get("Intake_Servo");
        this.towerGrasp = hardwareMap.servo.get("Tower_Grasp");
        this.towerGrasp.setPosition(0.5);
        this.towerServo = hardwareMap.servo.get("Tower_Servo");
        this.colorLeft = hardwareMap.get(ColorSensor.class, "Color_Left");
        this.colorRight = hardwareMap.get(ColorSensor.class, "Color_Right");
        this.propellorColor = hardwareMap.get(ColorSensor.class, "propellor_color");

        //OLD MOTORS
//        frontLeftWheel.setDirection(DcMotor.Direction.REVERSE);
//        backLeftWheel.setDirection(DcMotor.Direction.REVERSE);
//        frontRightWheel.setDirection(DcMotor.Direction.FORWARD);
//        backRightWheel.setDirection(DcMotor.Direction.FORWARD);
            this.initializeMotors();

        }

    public void initializeMotors(){
        //NEW MOTORS
        frontLeftWheel.setDirection(DcMotor.Direction.FORWARD);
        backLeftWheel.setDirection(DcMotor.Direction.FORWARD);
        frontRightWheel.setDirection(DcMotor.Direction.REVERSE);
        backRightWheel.setDirection(DcMotor.Direction.REVERSE);

        launcherL.setDirection(DcMotor.Direction.REVERSE);
        launcherR.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.REVERSE);
        frontLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        towerHand.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

    public void towerServoUp(){
        towerServo.setPosition(0);
    }
    public void towerServoDown(){
        towerServo.setPosition(0.5);
    }

    public void stop(){
        frontRightWheel.setPower(0);
        frontLeftWheel.setPower(0);
        backRightWheel.setPower(0);
        backLeftWheel.setPower(0);
    }

    public void lightsOff(){
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
    }
    public void lightsRed(){
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_RED);
    }

    public void resetAngle(){

        reset_angle = getHeading() + reset_angle;

    }




    public void driveSimple(double crabPower, double movePower){

        this.frontLeftWheel.setPower(Range.clip(movePower + crabPower, -MAX_POWER, MAX_POWER));
        telemetry.addData("Front Left :", this.frontLeftWheel.getPower());
        this.frontRightWheel.setPower(Range.clip(movePower - crabPower, -MAX_POWER, MAX_POWER));
        telemetry.addData("Front Right: ", this.frontRightWheel.getPower());
        this.backRightWheel.setPower(Range.clip(movePower + crabPower, -MAX_POWER, MAX_POWER));
        telemetry.addData("Back Right: ", this.backRightWheel.getPower());
        this.backLeftWheel.setPower(Range.clip(movePower - crabPower, -MAX_POWER, MAX_POWER));
        telemetry.addData("Back Left: ", this.backLeftWheel.getPower());


    }

    public void turnTime(int milliseconds) throws InterruptedException{
        this.frontLeftWheel.setPower(0.6);
        this.frontRightWheel.setPower(0.6);
        this.backLeftWheel.setPower(0.6);
        this.backRightWheel.setPower(-0.6);

        Thread.sleep(milliseconds);

        this.frontLeftWheel.setPower(0);
        this.frontRightWheel.setPower(0);
        this.backLeftWheel.setPower(0);
        this.backRightWheel.setPower(0);
        Thread.sleep(500);
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


    public void outtake(){
        intake.setPower(0.8);
    }

    public void intakeStop(){
        intake.setPower(0);
    }

    public void intake(){
        intake.setPower(-0.8);
    }

    public void intakePower(double power){
        intake.setPower(power);
    }

    public void towerOpen(){
        this.towerGrasp.setPosition(0.5);
    }
    public void towerClose(){
        this.towerGrasp.setPosition(1);
    }



    public void launch(){
//        launcherL.setPower(0.55);
//       launcherR.setPower(0.55);

        launcherL.setPower(8.00);
        launcherR.setPower(0.55);


//       launcherL.setPower(0.53);
//       launcherR.setPower(0.53);
//        launcherL.setPower(0.3);
//        launcherR.setPower(0.3);
    }

    public void launchStop(){
        launcherL.setPower(0);
        launcherR.setPower(0);

    }

    public void powerLaunch(){
//        launcherL.setPower(0.4);
//        launcherR.setPower(0.4);
        launcherL.setPower(0.37);
        launcherR.setPower(0.37);
    }


    private boolean isColor(){
        return this.propellorColor.blue() > 100 || this.propellorColor.green() > 100 || this.propellorColor.red() > 100;
    }

    private void runPropellorToColor() throws InterruptedException {
        sleep(100);
        long startTime = System.currentTimeMillis();
        while (!isColor()) {
            if(System.currentTimeMillis() - startTime > 2000){
                break;
            }
            sleep(5);
        }
        this.propeller.setPosition(0.5);
    }

//    public void propel() throws InterruptedException {
//        propeller.setPosition(0.01);
//        Thread.sleep(770);
//        propeller.setPosition(0.5);
//        Thread.sleep(100);
//    }

    public void knockIntake(){
        intakeServo.setPosition(0.1);
    }

    public void unknockIntake(){
        intakeServo.setPosition(1);
    }

    public void towerHandUp(){
        towerHand.setPower(0.2);
    }

    public void towerHandDown(){
        towerHand.setPower(-0.2);
    }

    public void towerHandStop(){
        towerHand.setPower(0);
    }

    public void initMotors(){
        frontLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void autoCrabPower(double power){
        frontLeftWheel.setPower(-power);
        frontRightWheel.setPower(-power);
        backLeftWheel.setPower(power);
        backRightWheel.setPower(power);
    }

    public void autoRotate(int distance, double power){
        int frontLeftPosition = frontLeftWheel.getCurrentPosition() ;
        int frontRightPosition = frontRightWheel.getCurrentPosition();
        int backLeftPosition = backLeftWheel.getCurrentPosition();
        int backRightPosition = backRightWheel.getCurrentPosition();

        frontLeftWheel.setTargetPosition(frontLeftPosition - distance);
        frontRightWheel.setTargetPosition(frontRightPosition + distance);
        backLeftWheel.setTargetPosition(backLeftPosition - distance);
        backRightWheel.setTargetPosition(backRightPosition + distance);

        frontLeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeftWheel.setPower(power);
//        frontRightWheel.setPower(power - correctionPower);
        frontRightWheel.setPower(power);
        backLeftWheel.setPower(power);
        backRightWheel.setPower(power);

        while(frontLeftWheel.isBusy() && frontRightWheel.isBusy() && backLeftWheel.isBusy() && backRightWheel.isBusy()){
            try {
                sleep(5);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        stopNow();

//        this.initializeMotors(hardwareMap, telemetry);
    }

    public void move(int distance, double power){
        int frontLeftPosition = frontLeftWheel.getCurrentPosition() ;
        int frontRightPosition = frontRightWheel.getCurrentPosition();
        int backLeftPosition = backLeftWheel.getCurrentPosition();
        int backRightPosition = backRightWheel.getCurrentPosition();

        frontLeftWheel.setTargetPosition(frontLeftPosition - distance);
        frontRightWheel.setTargetPosition(frontRightPosition + distance);
        backLeftWheel.setTargetPosition(backLeftPosition - distance);
        backRightWheel.setTargetPosition(backRightPosition + distance);

        frontLeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeftWheel.setPower(power);
//        frontRightWheel.setPower(power - correctionPower);
        frontRightWheel.setPower(power);
        backLeftWheel.setPower(power);
        backRightWheel.setPower(power);

        while(frontLeftWheel.isBusy() && frontRightWheel.isBusy() && backLeftWheel.isBusy() && backRightWheel.isBusy()){
            try {
                sleep(5);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        stopNow();
    }

    public void autoCrab(int distance, double power){
        int frontLeftPosition = frontLeftWheel.getCurrentPosition() ;
        int frontRightPosition = frontRightWheel.getCurrentPosition();
        int backLeftPosition = backLeftWheel.getCurrentPosition();
        int backRightPosition = backRightWheel.getCurrentPosition();

        frontLeftWheel.setTargetPosition(frontLeftPosition - distance);
        frontRightWheel.setTargetPosition(frontRightPosition - distance);
        backLeftWheel.setTargetPosition(backLeftPosition + distance);
        backRightWheel.setTargetPosition(backRightPosition + distance);

        frontLeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeftWheel.setPower(power);
//        frontRightWheel.setPower(power - correctionPower);
        frontRightWheel.setPower(power);
        backLeftWheel.setPower(power);
        backRightWheel.setPower(power);

        while(frontLeftWheel.isBusy() && frontRightWheel.isBusy() && backLeftWheel.isBusy() && backRightWheel.isBusy()){
            try {
                sleep(5);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        stopNow();
    }

    protected void stopNow(){
        frontLeftWheel.setPower(0);
        frontRightWheel.setPower(0);
        backLeftWheel.setPower(0);
        backRightWheel.setPower(0);
    }

    protected void crabToBlue(boolean right) throws InterruptedException {
        double power = right ? 0.2:-0.2;
        boolean done = false;
        while (!done) {
            if (this.colorRight.blue() < 180) {
                this.autoCrabPower(power);
            } else {
                this.stopNow();
                done = true;
            }
            Thread.sleep(5);
        }
    }

    private boolean isLeftWhite(){
        return this.colorLeft.red() > 600 && this.colorLeft.green() > 600
                && this.colorLeft.blue() > 600;
    }

    private boolean isRightWhite(){
        return this.colorRight.red() > 300 && this.colorLeft.green() > 300
                && this.colorRight.blue() > 300;
    }

    public void rightCorrect(double power){

        frontLeftWheel.setPower(0);
        frontRightWheel.setPower(power);
        backLeftWheel.setPower(0);
        backRightWheel.setPower(power);
    }

    public void leftCorrect(double power){
        frontLeftWheel.setPower(-power);
        frontRightWheel.setPower(0);
        backLeftWheel.setPower(-power);
        backRightWheel.setPower(0);
    }

    public void lineDetect(boolean forward) {
        boolean leftDone = false;
        boolean rightDone = false;
        double power = forward ? 0.1: -0.1;
        this.movePower(power);
        while(!(leftDone && rightDone)){
            if(this.isLeftWhite() && !leftDone ){
                this.rightCorrect(power);
                leftDone = true;
            }
            if (this.isRightWhite() && !rightDone)
            {
                this.leftCorrect(power);
                rightDone = true;
            }

        }
        this.stopNow();
    }

    public void  movePower (double power)  {
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


    private void propel() throws InterruptedException {
        propeller.setPosition(0.01);
        this.runPropellorToColor();
    }

    public void autoShoot() throws InterruptedException {
        this.autoRotate(75, 0.2);
        this.propel();
        this.autoRotate(35, 0.2);
        this.propel();
        this.autoRotate(25, 0.2);
        this.propel();
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

    public void test() {
        int a = 2;
        a++;
    }

}