package org.firstinspires.ftc.teamcode.SeansPlayground;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by seancardosi on 2/10/2018.
 */


@TeleOp(name = "SwerveTest", group= "SwerveTest")

public class SwerveDrive extends OpMode {

    DcMotor leftBackMotor;
    DcMotor leftFrontMotor;
    DcMotor rightBackMotor;
    DcMotor rightFrontMotor;

    CRServo leftBackServo;
    CRServo leftFrontServo;
    CRServo rightBackServo;
    CRServo rightFrontServo;

    public double A = 0;
    public double B = 0;
    public double C = 0;
    public double D = 0;

    public double length = 16;
    public double width = 16;

    public double vehicleTranslationx = 0;
    public double vehicleTranslationy = 0;
    public double vehicleRotation = 0;



    public void init() {


        leftBackMotor = hardwareMap.dcMotor.get("m1");
        leftFrontMotor = hardwareMap.dcMotor.get("m2");
        leftBackMotor.setDirection(DcMotor.Direction.FORWARD);
        leftFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        rightBackMotor = hardwareMap.dcMotor.get("m3");
        rightFrontMotor = hardwareMap.dcMotor.get("m4");
        rightBackMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        leftBackServo = hardwareMap.crservo.get("s1");
        leftFrontServo = hardwareMap.crservo.get("s2");
        leftBackServo.setDirection(CRServo.Direction.REVERSE);

        rightBackServo = hardwareMap.crservo.get("s3");
        rightFrontServo = hardwareMap.crservo.get("s4");
        rightBackServo.setDirection(CRServo.Direction.REVERSE);

    }


        public void loop() {

        vehicleTranslationx = gamepad1.right_stick_x;
        vehicleTranslationy = gamepad1.right_stick_y;

        vehicleRotation = gamepad1.left_stick_x;



        A = vehicleTranslationx - vehicleRotation*(length /2);
        B = vehicleTranslationx + vehicleRotation*(length/2);
        C = vehicleTranslationy - vehicleRotation*(width / 2);
        D = vehicleTranslationy + vehicleRotation*(width / 2);

        double wheelOneSpeed = Math.sqrt((Math.pow(B,2) + Math.pow(C,2)));
        double wheelOneAngle = (((Math.atan2(B,C) * (180 / Math.PI)) + 180) / 360);
        double wheelTwoSpeed = Math.sqrt((Math.pow(B,2) + Math.pow(D,2)));
        double wheelTwoAngle = (((Math.atan2(B,D) * (180 / Math.PI))+ 180) / 360);
        double wheelThreeSpeed = Math.sqrt((Math.pow(A,2) + Math.pow(D,2)));
        double wheelThreeAngle = (((Math.atan2(A,D) * (180 / Math.PI))+ 180) / 360);
        double wheelFourSpeed = Math.sqrt((Math.pow(A,2) + Math.pow(C,2)));
        double wheelFourAngle = (((Math.atan2(A,C) * (180 / Math.PI)) + 180) / 360);

        double max = 0;

        if (max < wheelOneSpeed){
            max = wheelOneSpeed;
        }

        if(max < wheelTwoSpeed){
            max = wheelTwoSpeed;
        }

        if(max < wheelThreeSpeed){
            max = wheelThreeSpeed;
        }

        if(max < wheelFourSpeed){
            max = wheelFourSpeed;
        }

        if(max > 1){
            wheelOneSpeed = wheelOneSpeed / max;
            wheelTwoSpeed = wheelTwoSpeed / max;
            wheelThreeSpeed = wheelThreeSpeed / max;
            wheelFourSpeed = wheelFourSpeed / max;
        }


        rightFrontMotor.setPower(wheelOneSpeed);
        rightFrontServo.setPower(wheelOneAngle);

        leftFrontMotor.setPower(wheelTwoSpeed);
        leftFrontServo.setPower(wheelTwoAngle);

        rightBackMotor.setPower(wheelFourSpeed);
        rightBackServo.setPower(wheelFourAngle);

        leftBackMotor.setPower(wheelThreeSpeed);
        leftBackServo.setPower(wheelThreeAngle);



    }
}