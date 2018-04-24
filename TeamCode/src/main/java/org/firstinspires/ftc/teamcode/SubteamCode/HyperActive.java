package org.firstinspires.ftc.teamcode.SubteamCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by seancardosi on 3/23/18.
 */
@TeleOp(name = "HyperActive", group = "HyperActive")
public class HyperActive extends OpMode {

    DcMotor left, right, lift;
    Servo srvoL, srvoR;

    public void init() {


        left = hardwareMap.dcMotor.get("left");
        right = hardwareMap.dcMotor.get("right");
        right.setDirection(DcMotor.Direction.REVERSE);
        lift = hardwareMap.dcMotor.get("lift");

        srvoL = hardwareMap.servo.get("L");
        srvoL.setDirection(Servo.Direction.REVERSE);
        srvoR = hardwareMap.servo.get("R");
    }

    public void loop() {

        left.setPower(-gamepad1.left_stick_y);
        right.setPower(-gamepad1.right_stick_y);


        if (gamepad1.y) {
            lift.setPower(.2);
        } else if (gamepad1.a) {
            lift.setPower(-.2);
        } else {
            lift.setPower(0);
        }

        if (gamepad1.x) {
            srvoL.setPosition(.75);
            srvoR.setPosition(.75);

        } else if (gamepad1.b) {
            srvoL.setPosition(0);
            srvoR.setPosition(0);
        }

    }
}
