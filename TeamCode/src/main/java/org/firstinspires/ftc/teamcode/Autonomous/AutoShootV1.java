
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
            this.autoOmni.move(500, 0.4);
            Thread.sleep(300);
            this.autoOmni.crab(500, 0.4);
            Thread.sleep(300);
            this.autoOmni.move(2400, 0.4);
            Thread.sleep(300);
            this.autoOmni.crab(-500, 0.4);
            Thread.sleep(300);

            this.autoOmni.propel();
            Thread.sleep(300);
            this.autoOmni.propel();
            Thread.sleep(300);
            this.autoOmni.propel();
            Thread.sleep(300);
            this.autoOmni.propel();

            this.autoOmni.move(400, 0.4);

            }

            Thread.sleep(50);
            parked = true;
        }


    }
}
