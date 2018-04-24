package org.firstinspires.ftc.teamcode.MemeMachine;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MemeMachine.MemeMachineDrivetrain;

/**
 * Created by seancardosi on 2/6/18.
 */
@TeleOp(name = "SeanTeleOp", group = "MemeMachine")
public class MemeMachineTeleOp extends OpMode {


    MemeMachineDrivetrain robot;

    public void init() {

    robot.init();

    }

    public void loop() {

        //----------------------------------------------=+(Drivetrain)+=----------------------------------------------\\

        robot.SwerveDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

        //----------------------------------------------=+(Drivetrain)+=----------------------------------------------\\


    }
}
