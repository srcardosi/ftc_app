package org.firstinspires.ftc.teamcode.Subsystems.Transitioning;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by seancardosi on 12/18/17.
 */
@Autonomous(name = "TestTransitioner", group = "MemeMachine")
public class TestTransitioner extends LinearOpMode {//test OpMode to automatically switch from autonomous to TeleOp


    public void runOpMode() throws InterruptedException {

        telemetry.addData("Initializing Here", true);
        telemetry.update();


        waitForStart();

        AutoTransitioner.transitionOnStop(this, "SeanTeleOp");

    }
}