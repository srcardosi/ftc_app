package org.firstinspires.ftc.teamcode.SeansPlayground;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 * Created by seancardosi on 4/10/18.
 */
@TeleOp(name = "Position Integration V2", group = "V2")
public class PositionIntegrationV2 extends OpMode {

    public BNO055IMU gyro;
    public Orientation gyro_angle;
    public Position position;
    public Velocity velocity;
    public Acceleration accel1;
    public Acceleration accel2;
    ElapsedTime elapsedTime = new ElapsedTime();

    @Override
    public void init() {

        gyro = hardwareMap.get(BNO055IMU.class, "imuINT");
        BNO055IMU.Parameters param = new BNO055IMU.Parameters();
        param.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        param.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        param.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        param.loggingEnabled      = true;
        param.loggingTag          = "IMU";

        gyro.initialize(param);
        gyro_angle = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

        gyro.startAccelerationIntegration(position, velocity, 1000);

        double accelOld;
        double accelNew;
    }

    @Override
    public void loop() {

        double accelOld = accel1.xAccel;

//        accelOld = accelNew;


        telemetry.addData("Position", position);

        telemetry.update();
    }
}
