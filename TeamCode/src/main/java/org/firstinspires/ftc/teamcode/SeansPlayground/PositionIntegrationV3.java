package org.firstinspires.ftc.teamcode.SeansPlayground;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
@TeleOp(name = "Position Integration V3", group = "V3")
public class PositionIntegrationV3 extends OpMode {

    public BNO055IMU gyro;
    public Orientation gyro_angle;
    public Position position;
    public Velocity velocity;

    @Override
    public void init() {

        gyro = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters param              = new BNO055IMU.Parameters();
        param.angleUnit                         = BNO055IMU.AngleUnit.DEGREES;
        param.accelUnit                         = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        param.accelerationIntegrationAlgorithm  = new JustLoggingAccelerationIntegrator();
        param.calibrationDataFile               ="BNO055IMUCalibration.json";
        param.mode                              = BNO055IMU.SensorMode.IMU;
        param.useExternalCrystal                = true;
        param.loggingEnabled                    = true;
        param.loggingTag                        = "AdaFruitIMU";
        gyro_angle = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        gyro.startAccelerationIntegration(position, velocity, 1000);
        gyro.initialize(param);
    }

    @Override
    public void loop() {

        telemetry.addData("Position", position.toUnit(DistanceUnit.CM));
        telemetry.addData("Velocity", velocity.toUnit(DistanceUnit.CM));
        telemetry.addData("Acceleration", gyro.getAcceleration());
        telemetry.update();
    }
}
