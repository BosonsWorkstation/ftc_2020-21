package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
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
    protected DcMotor intake;
    protected DcMotor launcherL;
    protected DcMotor launcherR;
    protected Servo propeller;
    protected Servo intakeServo;



    private static BNO055IMU imu;
    private static boolean gyroInitialized = false;

    private static final double MAX_POWER = 0.8;
    private Telemetry telemetry;
    float rotate_angle = 0;
    double reset_angle = 0;
    private double correction_factor = 0;

    private Telemetry.Item leftFrontTelemetry;
    private Telemetry.Item rightFrontTelemetry;
    private Telemetry.Item leftBackTelemetry;
    private Telemetry.Item rightBackTelemetry;
    private Telemetry.Item usePowerTelemetry;

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
        this.leftFrontTelemetry = telemetry.addData("LF Power", frontLeftWheel.getPower());
        this.rightFrontTelemetry = telemetry.addData("RF Power", frontRightWheel.getPower());
        this.leftBackTelemetry = telemetry.addData("LF Power", backLeftWheel.getPower());
        this.rightBackTelemetry = telemetry.addData("RB Power", backRightWheel.getPower());
        this.usePowerTelemetry = telemetry.addData("usePower", 0);
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
        this.launcherL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.launcherR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.propeller = hardwareMap.servo.get("Propeller");
        this.intakeServo = hardwareMap.servo.get("Intake_Servo");
        frontLeftWheel.setDirection(DcMotor.Direction.REVERSE);
        backLeftWheel.setDirection(DcMotor.Direction.REVERSE);
        frontRightWheel.setDirection(DcMotor.Direction.FORWARD);
        backRightWheel.setDirection(DcMotor.Direction.FORWARD);
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
        if(!gyroInitialized) {
            imu = hardwareMap.get(BNO055IMU.class, "imu");
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.mode = BNO055IMU.SensorMode.IMU;
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.loggingEnabled = false;

            imu.initialize(parameters);
        }
        gyroInitialized = true;
    }

    public void stop(){

        frontRightWheel.setPower(0);
        frontLeftWheel.setPower(0);
        backRightWheel.setPower(0);
        backLeftWheel.setPower(0);
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
        intake.setPower(-0.8);
    }

    public void intakeStop(){
        intake.setPower(0);
    }

    public void intake(){
        intake.setPower(0.7);
    }

    public void launch(){
       launcherL.setPower(0.43);
       launcherR.setPower(0.43);
    }

    public void launchStop(){
        launcherL.setPower(0);
        launcherR.setPower(0);

    }

    public void powerLaunch(){
        launcherL.setPower(0.37);
        launcherR.setPower(0.37);
    }

    public void propel() throws InterruptedException {
        propeller.setPosition(0.1);
        Thread.sleep(790);
        propeller.setPosition(0.5);
        Thread.sleep(600);
    }

    public void knockIntake(){
        intakeServo.setPosition(0.1);
    }

    public void unknockIntake(){
        intakeServo.setPosition(1);
    }

    public void towerHandUp(){
        towerHand.setPower(0.6);
    }

    public void towerHandDown(){
        towerHand.setPower(-0.6);
    }

    public void towerHandStop(){
        towerHand.setPower(0);
    }


    public void drive(double moveValue, double crabValue, double turnValue) {

        double Protate = turnValue/4;
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




        //MOVEMENT
        theta = Math.atan2(stick_y, stick_x) - gyroAngle - (Math.PI / 2);
        Px = Math.sqrt(Math.pow(stick_x, 2) + Math.pow(stick_y, 2)) * (Math.sin(theta + Math.PI / 4));
        Py = Math.sqrt(Math.pow(stick_x, 2) + Math.pow(stick_y, 2)) * (Math.sin(theta - Math.PI / 4));


//        telemetry.addData("servo bruh", intakeServo.getPosition());
//        telemetry.addData("Stick_X", stick_x);
//        telemetry.addData("Stick_Y", stick_y); 
//        telemetry.addData("Magnitude",  Math.sqrt(Math.pow(stick_x, 2) + Math.pow(stick_y, 2)));
//        telemetry.addData("Front Left", Py - Protate);
//        telemetry.addData("Back Left", Px - Protate);
//        telemetry.addData("Back Right", Py + Protate);
//        telemetry.addData("Front Right", Px + Protate);

        if (launcherL.getPower() > 0.4){
            telemetry.addData("GO!!!!!!!", launcherL.getPower());
        }
        else{
            telemetry.addData("wait", launcherL.getPower());
        }

        frontLeftWheel.setPower(Py - Protate);
        backLeftWheel.setPower(Px - Protate);
        backRightWheel.setPower(Py + Protate);
        frontRightWheel.setPower(Px + Protate);

        telemetry.update();
    }

}