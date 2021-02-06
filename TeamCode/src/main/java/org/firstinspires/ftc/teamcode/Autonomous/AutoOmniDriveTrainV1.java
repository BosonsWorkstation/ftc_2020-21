package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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
    protected DcMotor towerHand;
    protected  RevBlinkinLedDriver lights;

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
    private static final double MOTOR_POWER = 0.5;
    private static final int TICKS_PER_REVOLUTION = 280;
    private static final double DISTANCE_PER_REVOLUTION = 4 * Math.PI;
    private static final double CRAB_POWER = .2;
    private static final double correctionPower = 0.24;
//    Telemetry.Item currentPositionTel;
//    Telemetry.Item targetValueTel;

//    private OmniDriveTrainV2 omniTrain;


    public AutoOmniDriveTrainV1(HardwareMap hardwareMap, Telemetry telemetry){
//        super(hardwareMap,telemetry);
//        currentPositionTel = telemetry.addData("Current Position", 0);
//        targetValueTel = telemetry.addData("Target ValueL", 0);

    }


    private void initDriveMotors(HardwareMap hardwareMap, Telemetry telemetry){
        this.backLeftWheel = hardwareMap.dcMotor.get("Back_Left_Wheel");
        this.backRightWheel = hardwareMap.dcMotor.get("Back_Right_Wheel");
        this.frontLeftWheel = hardwareMap.dcMotor.get("Front_Left_Wheel");
        this.frontRightWheel = hardwareMap.dcMotor.get("Front_Right_Wheel");
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
    }

    private void initLights(HardwareMap hardwareMap, Telemetry telemetry){
        this.lights = hardwareMap.get(RevBlinkinLedDriver.class, "lights");
    }

    public void initMotors(HardwareMap hardwareMap, Telemetry telemetry){

        this.initDriveMotors(hardwareMap,telemetry);
        this.initLauncher(hardwareMap, telemetry);
        this.initIntake(hardwareMap, telemetry);
        this.initTowerHand(hardwareMap, telemetry);
        this.initLights(hardwareMap, telemetry);
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

    public void  moveDistance (int distance)  {
        int direction = distance > 0 ? -1 : 1;
        int targetValue = getTargetValue(distance);
        int currentPosition = 0;
//        this.setMoveMotorDirection(direction);
        boolean done = false;


        this.stopNow();
    }
    public void towerHandUp(){
        towerHand.setPower(-0.8);
    }

    public void towerHandDown(){
        towerHand.setPower(0.8);
    }

    public void towerHandStop(){
        towerHand.setPower(0);
    }


    public void rotate(int distance, double power) throws InterruptedException{
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
        frontRightWheel.setPower(power - correctionPower);
//        frontRightWheel.setPower(power);
        backLeftWheel.setPower(power);
        backRightWheel.setPower(power);

        while(frontLeftWheel.isBusy() && frontRightWheel.isBusy() && backLeftWheel.isBusy() && backRightWheel.isBusy()){
            telemetry.addData("Back Left", backLeftWheel.getCurrentPosition());
            telemetry.addData("Back Right", backRightWheel.getCurrentPosition());
            telemetry.addData("Front Left", frontLeftWheel.getCurrentPosition());
            telemetry.addData("Front Right", frontRightWheel.getCurrentPosition());
            telemetry.update();
        }

        stopNow();
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
        frontRightWheel.setPower(power - correctionPower);
//        frontRightWheel.setPower(power);
        backLeftWheel.setPower(power);
        backRightWheel.setPower(power);

        while(frontLeftWheel.isBusy() && frontRightWheel.isBusy() && backLeftWheel.isBusy() && backRightWheel.isBusy()){
            telemetry.addData("Back Left", backLeftWheel.getCurrentPosition());
            telemetry.addData("Back Right", backRightWheel.getCurrentPosition());
            telemetry.addData("Front Left", frontLeftWheel.getCurrentPosition());
            telemetry.addData("Front Right", frontRightWheel.getCurrentPosition());
            telemetry.update();
        }

        stopNow();
    }

    public void knockIntake(){
        intakeServo.setPosition(0.1);
    }

    public void unknockIntake(){
        intakeServo.setPosition(1);
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
        frontRightWheel.setPower(power - correctionPower);
//        frontRightWheel.setPower(power);
        backLeftWheel.setPower(power);
        backRightWheel.setPower(power);

        while(frontLeftWheel.isBusy() && frontRightWheel.isBusy() && backLeftWheel.isBusy() && backRightWheel.isBusy()){
            telemetry.addData("Back Left", backLeftWheel.getCurrentPosition());
            telemetry.addData("Back Right", backRightWheel.getCurrentPosition());
            telemetry.addData("Front Left", frontLeftWheel.getCurrentPosition());
            telemetry.addData("Front Right", frontRightWheel.getCurrentPosition());
            telemetry.update();
        }

        stopNow();
    }

    public void launch() {
        this.launcherL.setPower(0.53);
        this.launcherR.setPower(0.53);
    }
    public void launchStop(){
        this.launcherL.setPower(0);
        this.launcherR.setPower(0);
    }


    public void propel(){


        try {
            propeller.setPosition(0.1);
            Thread.sleep(790);
            propeller.setPosition(0.5);
        }
        catch (InterruptedException e){
            Thread.currentThread().interrupt();
        }




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

