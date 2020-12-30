package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.TeleOp.OmniDriveTrainV2;

@Autonomous
@Disabled
public class Autonomous1 extends LinearOpMode
{
    private AutonomousClass driveTrain2;

    private static final AutonomousClass.DirectionEnum direction = AutonomousClass.DirectionEnum.SOUTH;


    @Override
    public void runOpMode()
    {
 //       this.driveTrain2 = new AutonomousClass(this.hardwareMap, this.telemetry, direction);

        this.driveTrain2.getHeading();

        waitForStart();

        while (opModeIsActive())
        {


        }

    }


}
