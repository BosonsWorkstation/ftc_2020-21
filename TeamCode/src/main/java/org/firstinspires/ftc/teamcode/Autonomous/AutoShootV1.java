
package org.firstinspires.ftc.teamcode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "AutoShootV1", group = "Linear Opmode")
public class AutoShootV1 extends LinearOpMode {
    private AutoOmniDriveTrainV1 autoOmni;
    private static final long INITIAL_WAIT = 1000;

    public void runOpMode() throws InterruptedException {
        this.autoOmni = new AutoOmniDriveTrainV1(this.hardwareMap, this.telemetry);
        this.autoOmni.initMotors();
        boolean parked = false;

        waitForStart();

        while (opModeIsActive() ){




            Thread.sleep(INITIAL_WAIT);



            if(!parked) {

                this.autoOmni.launch();
//            this.autoOmni.move(500, 0.4);
            Thread.sleep(300);
//            this.autoOmni.crab(500, 0.4);
//            Thread.sleep(300);
            this.autoOmni.move(2800, 0.4);
            Thread.sleep(300);
            this.autoOmni.crab(-625, 0.4);
            Thread.sleep(300);
            this.autoOmni.rotate(100, 0.4);

            this.autoOmni.propel();
            Thread.sleep(300);
            this.autoOmni.propel();
            Thread.sleep(300);
            this.autoOmni.propel();
            Thread.sleep(300);
            this.autoOmni.propel();


//            this.autoOmni.towerHandUp();
//            Thread.sleep(500);

            this.autoOmni.launchStop();

            this.autoOmni.move(650, 0.4);
            this.autoOmni.towerHandUp();
//            Thread.sleep(4000);
//            this.autoOmni.towerHandStop();
//            Thread.sleep(100);
            Thread.sleep(100);
            this.autoOmni.knockIntake();
            Thread.sleep(100);
            this.autoOmni.unknockIntake();
            }

            Thread.sleep(50);
            parked = true;
        }


    }
}
