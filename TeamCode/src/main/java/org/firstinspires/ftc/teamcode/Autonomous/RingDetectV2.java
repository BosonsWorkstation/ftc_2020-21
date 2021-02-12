package org.firstinspires.ftc.teamcode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Autonomous.AutoOmniDriveTrainV1;

@Autonomous(name = "Ring Detect V2", group = "Linear Opmode")
@Disabled
public class RingDetectV2 extends RingDetect{
   private RingDetect detectionClass;
   private AutoOmniDriveTrainV1 autoOmniTrain;




    public void runOpMode() {

    waitForStart();

    while (opModeIsActive()) {






        telemetry.addData("Analysis", pipeline.getAnalysis());
        telemetry.addData("Position", pipeline.position);
        telemetry.update();
    }
    }
}