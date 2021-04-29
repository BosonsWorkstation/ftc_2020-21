package org.firstinspires.ftc.teamcode.Autonomous;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.TeleOp.OmniDriveTrainV2;

public class AutoOmniDriveTrainV1{
    protected DcMotor backRightWheel;
    protected DcMotor backLeftWheel;
    protected DcMotor frontRightWheel;
    protected DcMotor frontLeftWheel;
    protected DcMotor launcherL;
    protected DcMotor launcherR;
    protected Servo propeller;
    protected Servo intakeServo;
    protected Servo towerServo;
    protected DcMotor towerHand;
    protected Servo towerGrasp;
    protected  RevBlinkinLedDriver lights;
    protected ColorSensor colorLeft;
    protected ColorSensor colorRight;
    protected ColorSensor colorBack;
    float hsvValues[] = {0F, 0F, 0F};
    final float values[] = hsvValues; //these hsv values are for alphaColor()

    private BNO055IMU imu;
    private double lastPower = 0;
    private static final double INCREMENT = 0.1;
    private static final double MIN_DIFF = 0.05;
    private static final double MAX_POWER = 0.5;
    public boolean GO_SLOW = false;
    protected Telemetry telemetry;
    private Telemetry.Item leftFrontTelemetry;
    private Telemetry.Item rightFrontTelemetry;
    private Telemetry.Item leftBackTelemetry;
    private Telemetry.Item rightBackTelemetry;
    private Telemetry.Item usePowerTelemetry;

    int position;
    double getLeftColor;
    private static final double MOTOR_POWER = 0.5;
    private static final int TICKS_PER_REVOLUTION = 280;
    private static final double DISTANCE_PER_REVOLUTION = 4 * Math.PI;
    private static final double CRAB_POWER = .2;
//    private static final double correctionPower = 0.05;
    private static final double correctionPower = 0;
    private static boolean gyroInitialized = false;
//    Telemetry.Item currentPositionTel;
//    Telemetry.Item targetValueTel;

//    private OmniDriveTrainV2 omniTrain;


    public AutoOmniDriveTrainV1(HardwareMap hardwareMap, Telemetry telemetry){
//        super(hardwareMap,telemetry);
//        currentPositionTel = telemetry.addData("Current Position", 0);
//        targetValueTel = telemetry.addData("Target ValueL", 0);
        this.telemetry = telemetry;
        initializeGyro(hardwareMap, telemetry);
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

    public void initDriveMotors(HardwareMap hardwareMap, Telemetry telemetry){
        this.backLeftWheel = hardwareMap.dcMotor.get("Back_Left_Wheel");
        this.backRightWheel = hardwareMap.dcMotor.get("Back_Right_Wheel");
        this.frontLeftWheel = hardwareMap.dcMotor.get("Front_Left_Wheel");
        this.frontRightWheel = hardwareMap.dcMotor.get("Front_Right_Wheel");
        this.initDriveMotors();
    }

    public void initDriveMotors(){
        this.initMotor(frontLeftWheel);

        this.initMotor(frontRightWheel);

        this.initMotor(backLeftWheel);

        this.initMotor(backRightWheel);
    }

    private void initLauncher(HardwareMap hardwareMap, Telemetry telemetry){
        this.launcherL = hardwareMap.dcMotor.get("Launcher_Left");
        this.launcherR = hardwareMap.dcMotor.get("Launcher_Right");
        this.launcherL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.launcherR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcherL.setDirection(DcMotor.Direction.REVERSE);
        this.propeller = hardwareMap.servo.get("Propeller");
    }

    private void initIntake(HardwareMap hardwareMap, Telemetry telemetry){
        this.intakeServo = hardwareMap.servo.get("Intake_Servo");
    }

    private void initTowerHand(HardwareMap hardwareMap, Telemetry telemetry){
        this.towerHand = hardwareMap.dcMotor.get("Tower_Hand");
        this.towerGrasp = hardwareMap.servo.get("Tower_Grasp");
        this.towerGrasp.setPosition(0.5);
        this.towerServo = hardwareMap.servo.get("Tower_Servo");
    }

    private void initLights(HardwareMap hardwareMap, Telemetry telemetry){
        this.lights = hardwareMap.get(RevBlinkinLedDriver.class, "lights");
    }

    private void initColor(HardwareMap hardwareMap, Telemetry telemetry){
        this.colorLeft = hardwareMap.get(ColorSensor.class, "Color_Left");
        this.colorRight = hardwareMap.get(ColorSensor.class, "Color_Right");
        this.colorBack = hardwareMap.get(ColorSensor.class, "Color_Back");
    }

    public void initialize(HardwareMap hardwareMap, Telemetry telemetry){

        this.initDriveMotors(hardwareMap,telemetry);
        this.initLauncher(hardwareMap, telemetry);
        this.initIntake(hardwareMap, telemetry);
        this.initTowerHand(hardwareMap, telemetry);
        this.initLights(hardwareMap, telemetry);
        this.initColor(hardwareMap, telemetry);

        }

    public void initMotor(DcMotor motor) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setDirection(DcMotor.Direction.REVERSE);
    }

    public int getTargetValue(int distance){
        int targetValue = Math.round((float)(Math.abs(distance)/DISTANCE_PER_REVOLUTION)*TICKS_PER_REVOLUTION);
//        telemetry.clear();
//        targetValueTel.setValue("%d", targetValue);
//        telemetry.update();
        return targetValue;
    }

    public void towerServoUp(){
        towerServo.setPosition(0.5);
    }
    public void towerServoDown(){
        towerServo.setPosition(0);
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

    public void rightStop(){
        frontRightWheel.setPower(0);
        backRightWheel.setPower(0);
    }
    public void leftStop(){
        frontLeftWheel.setPower(0);
        backLeftWheel.setPower(0);
    }



    public void  movePower (double power)  {
        frontLeftWheel.setPower(-power);
        frontRightWheel.setPower(power);
//        backLeftWheel.setPower(-power - correctionPower);
        backLeftWheel.setPower(-power);
        backRightWheel.setPower(power);
    }

    public void crabPower (double power){
        frontLeftWheel.setPower(-power);
        frontRightWheel.setPower(-power);
        backLeftWheel.setPower(power);
        backRightWheel.setPower(power);
    }

    public void towerHandUp(){
        towerHand.setPower(1);
    }

    public void towerHandDown(){
        towerHand.setPower(-0.8);
    }

    public void towerHandStop(){
        towerHand.setPower(0);
    }

    public void stopSoft(){
        double power = frontRightWheel.getPower();
        while (power > 0){
            power = power - 0.05;
            if(power < 0){
                break;
            }
            else{
                backLeftWheel.setPower(power);
                backRightWheel.setPower(power);
                frontLeftWheel.setPower(power);
                frontRightWheel.setPower(power);
            }
            sleep(20);
        }
        backLeftWheel.setPower(0);
        backRightWheel.setPower(0);
        frontLeftWheel.setPower(0);
        frontRightWheel.setPower(0);
    }

//    public void stopSoft(){
//        this.stopSoft(this.backLeftWheel);
//        this.stopSoft(this.backRightWheel);
//        this.stopSoft(this.frontLeftWheel);
//        this.stopSoft(this.frontRightWheel);
//    }


    public void rotate(int distance, double power) {
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
//        frontRightWheel.setPower(power - correctionPower);
        frontRightWheel.setPower(power);
        backLeftWheel.setPower(power);
        backRightWheel.setPower(power);

        while(frontLeftWheel.isBusy() && frontRightWheel.isBusy() && backLeftWheel.isBusy() && backRightWheel.isBusy()){
            sleep(10);
        }

        stopNow();
//        stopSoft();
    }
        public void stopNow(){
        frontRightWheel.setPower(0);
        frontLeftWheel.setPower(0);
        backRightWheel.setPower(0);
        backLeftWheel.setPower(0);
        lastPower = 0;
    }

    public void crab(int distance, double power){
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
        backLeftWheel.setPower(power - correctionPower);
        backRightWheel.setPower(power - correctionPower);

        while(frontLeftWheel.isBusy() && frontRightWheel.isBusy() && backLeftWheel.isBusy() && backRightWheel.isBusy()){
            sleep(5);
        }

        stopNow();
//        stopSoft();
    }

    public void knockIntake(){
        intakeServo.setPosition(0.1);
    }

    public void unknockIntake(){
        intakeServo.setPosition(1);
    }


    public void move(int distance, double power){
        this.move(distance, power, true);
    }


    public void move(int distance, double power, boolean slowDown){
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

        frontLeftWheel.setPower(power + correctionPower);
        frontRightWheel.setPower(power);
        backLeftWheel.setPower(power);
        backRightWheel.setPower(power);

        while(frontLeftWheel.isBusy() && frontRightWheel.isBusy() && backLeftWheel.isBusy() && backRightWheel.isBusy()){
            sleep(5);
            if(slowDown){
                if (frontRightWheel.getPower() > 0.2 && Math.abs(distance) > 400){
                    if (Math.abs(frontRightWheel.getTargetPosition() - frontRightWheel.getCurrentPosition()) < 400) {
                        frontLeftWheel.setPower(frontLeftWheel.getPower() * 0.8);
                        frontRightWheel.setPower(frontRightWheel.getPower() * 0.8);
                        backLeftWheel.setPower(backLeftWheel.getPower() * 0.8);
                        backRightWheel.setPower(backRightWheel.getPower() * 0.8);
                    }
                }
            }
        }

        stopNow();
//        stopSoft();
    }

    private void sleep(long ms){
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public void towerOpen(){
        this.towerGrasp.setPosition(0.5);
    }
    public void towerClose(){
        this.towerGrasp.setPosition(1);
    }

    public void powerLaunch(){
//        this.launcherR.setPower(0.32);
//        this.launcherL.setPower(0.32);

        this.launcherR.setPower(0.35);
        this.launcherL.setPower(0.35);
    }

    public void launch() {
//        this.launcherL.setPower(0.53);
//        this.launcherR.setPower(0.53);
        this.launcherL.setPower(0.53);
        this.launcherR.setPower(0.53);
    }
    public void launchStop(){
        this.launcherL.setPower(0);
        this.launcherR.setPower(0);
    }


    public void propel(){


        try {
            propeller.setPosition(0.01);
            Thread.sleep(770);
            propeller.setPosition(0.5);
        }
        catch (InterruptedException e){
            Thread.currentThread().interrupt();
        }




    }

    public void colorDetect(){
        Color.RGBToHSV((int) (colorRight.red() * 255.00),
                (int) (colorRight.green() * 255.00),
                (int) (colorRight.blue() * 255.00),
                hsvValues);
    }

    public void tripleShoot(){
        try {
            propeller.setPosition(0.1);
            Thread.sleep(3120);
            propeller.setPosition(0.5);

        }
        catch (InterruptedException e){
            Thread.currentThread().interrupt();
        }


    }

    public void autoTowerHand(boolean up){
        this.autoTowerHand(2000, up);
    }

    public void autoTowerHand(long time, boolean up){
        try {
            if(up){
                this.towerHandUp();
            }
            else{
                this.towerHandDown();
            }
            Thread.sleep(time);
            this.towerHandStop();

        }
        catch (InterruptedException e){
            Thread.currentThread().interrupt();
        }

    }

    public void diagonal(HeadingEnum direction, int distance, double power){
        this.initDriveMotors();
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



    public void lightsGreen(){
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
    }

    public void lightsRed(){
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_RED);
    }

    public void lightsBlue(){
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_BLUE);
    }

}

