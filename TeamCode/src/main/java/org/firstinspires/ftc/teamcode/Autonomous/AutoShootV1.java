
package org.firstinspires.ftc.teamcode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "AutoShootV1", group = "Linear Opmode")
public class AutoShootV1 extends LinearOpMode {
    private AutoOmniDriveTrainV1 autoOmni;
    private static final long INITIAL_WAIT = 1000;
    @Override
    public void runOpMode() {
        this.autoOmni = new AutoOmniDriveTrainV1(this.hardwareMap, this.telemetry);
        this.autoOmni.initialize(hardwareMap, telemetry);
        boolean parked = false;

        waitForStart();

        while (opModeIsActive() ){




            sleep(INITIAL_WAIT);



            if(!parked) {


//            this.autoOmni.move(500, 0.4);
                sleep(300);
//            this.autoOmni.crab(500, 0.4);
//            Thread.sleep(300);
//                this.autoOmni.move(4000, 0.4);
                this.autoOmni.move(2850, 0.4);
                sleep(1000);
                this.autoOmni.crab(-500, 0.4);
                sleep(1000);
//            this.autoOmni.rotate(100, 0.4);
                sleep(400);

                this.autoOmni.launch();

                sleep(4000);

            this.autoOmni.propel();
            sleep(2000);
            this.autoOmni.propel();
            sleep(2000);
            this.autoOmni.propel();
            sleep(2000);
            this.autoOmni.propel();


//            this.autoOmni.towerHandUp();
//            Thread.sleep(500);

            this.autoOmni.launchStop();

            this.autoOmni.move(650, 0.4);
            this.autoOmni.towerHandUp();
            sleep(4000);
            this.autoOmni.towerHandStop();
            sleep(100);
            sleep(100);
            this.autoOmni.knockIntake();
            sleep(100);
            this.autoOmni.unknockIntake();
            }

            sleep(50);
            parked = true;
        }


    }
}
