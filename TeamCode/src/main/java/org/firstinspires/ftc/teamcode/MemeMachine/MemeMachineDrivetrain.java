package org.firstinspires.ftc.teamcode.MemeMachine;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by seancardosi on 2/6/18.
 */

public class MemeMachineDrivetrain {// TODO: Still needs support for field relative drive and for testing without it

    DcMotor leftBackMotor;
    DcMotor leftFrontMotor;
    DcMotor rightBackMotor;
    DcMotor rightFrontMotor;

    CRServo leftBackServo;
    CRServo leftFrontServo;
    CRServo rightBackServo;
    CRServo rightFrontServo;

    private final BNO055IMU imu;

    private double headingOffset = 0.0;
    private Orientation angles;

    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;



    public MemeMachineDrivetrain(final HardwareMap _hardwareMap, final Telemetry _telemetry) {
        hardwareMap = _hardwareMap;
        telemetry = _telemetry;

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


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit            = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit            = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled       = true;
        parameters.useExternalCrystal   = true;
        parameters.mode                 = BNO055IMU.SensorMode.IMU;
        parameters.loggingTag           = "IMU";
        imu                             = hardwareMap.get(BNO055IMU.class, "imuINT");

        imu.initialize(parameters);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

    }

    public void init() {
        leftBackMotor.setPower(0);
        leftFrontMotor.setPower(0);
        rightBackMotor.setPower(0);
        rightFrontMotor.setPower(0);

        leftBackServo.setPower(0);
        leftFrontServo.setPower(0);
        rightBackServo.setPower(0);
        rightFrontServo.setPower(0);
    }

    public void loop() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
    }


    private double getRawHeading() {
        return angles.firstAngle;
    }


    public double getHeading() {
        return (getRawHeading() - headingOffset) % (2.0 * Math.PI);
    }


    public void resetHeading() {
        headingOffset = getRawHeading();
    }


    final double interAxleLength = 16;
    final double interAxleWidth = 16;

    public void SwerveDrive(double x1, double y1, double x2){
        loop();

        double r = Math.sqrt ((interAxleLength * interAxleLength) + (interAxleWidth * interAxleWidth));
        y1 *= -1;

        double a = x1 - x2 * (interAxleLength / r);
        double b = x1 + x2 * (interAxleLength / r);
        double c = y1 - x2 * (interAxleWidth / r);
        double d = y1 + x2 * (interAxleWidth / r);

        double backRightSpeed = Math.sqrt ((a * a) + (d * d));
        double backLeftSpeed = Math.sqrt ((a * a) + (c * c));
        double frontRightSpeed = Math.sqrt ((b * b) + (d * d));
        double frontLeftSpeed = Math.sqrt ((b * b) + (c * c));

        double backRightAngle = Math.atan2 (a, d) / Math.PI;
        double backLeftAngle = Math.atan2 (a, c) / Math.PI;
        double frontRightAngle = Math.atan2 (b, d) / Math.PI;
        double frontLeftAngle = Math.atan2 (b, c) / Math.PI;

        leftBackMotor.setPower(backLeftSpeed);
        leftFrontMotor.setPower(frontLeftSpeed);
        rightBackMotor.setPower(backRightSpeed);
        rightFrontMotor.setPower(frontRightSpeed);

        leftBackServo.setPower(backLeftAngle);
        leftFrontServo.setPower(frontLeftAngle);
        rightBackServo.setPower(backRightAngle);
        rightFrontServo.setPower(frontRightAngle);
    }
}
