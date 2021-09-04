package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import java.lang.reflect.Member;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import com.qualcomm.robotcore.hardware.GyroSensor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;
import java.util.Locale;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Rotatetoangle", group="")

public class Rotatetoangle extends LinearOpMode{

    private DcMotor D1 = null;
    private DcMotor D2 = null;
    private DcMotor D3 = null;
    private DcMotor D4 = null;
    private DcMotor D5 = null;
    private DcMotor D6 = null;
    private DcMotor D7 = null;
    private DcMotor D8 = null;
    private BNO055IMU imu;
    
    private Orientation angles;
    private Acceleration gravity;
    
    private final double TargetHeading = 0.0;
    private final double CurrentHeading = 0.0;
    private final double Tolerance = 2.0;
    private final double Motormin = -1.0;
    private final double Motormax = 1.0;
    private final double Yaw_P = 0.005;
    private final double Yaw_I = 0.0;
    private final double Yaw_D = 0.0;
    
    
    @Override
    public void runOpMode()
    {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        D1 = hardwareMap.get(DcMotor.class, "FL");
        D2 = hardwareMap.get(DcMotor.class, "FR");
        D3 = hardwareMap.get(DcMotor.class, "BL");
        D4 = hardwareMap.get(DcMotor.class, "BR");
        D5 = hardwareMap.get(DcMotor.class, "Collector");
        D6 = hardwareMap.get(DcMotor.class, "Conveyor");
        D7 = hardwareMap.get(DcMotor.class, "Shooter");
        D8 = hardwareMap.get(DcMotor.class, "Wobble");
        
        D1.setDirection(DcMotor.Direction.REVERSE);
        D2.setDirection(DcMotor.Direction.REVERSE);
        D3.setDirection(DcMotor.Direction.FORWARD);
        D4.setDirection(DcMotor.Direction.FORWARD);
        
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit            = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled       = true;
        parameters.useExternalCrystal   = true;
        parameters.mode                 = BNO055IMU.SensorMode.IMU;
        parameters.loggingTag           = "IMU";
        imu                             = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);
        telemetry.setMsTransmissionInterval(100);
        
        waitForStart();
    }
    
    void sendTelemetry()
    {
        telemetry.addData("Status", imu.getSystemStatus().toString());
        telemetry.addData("Calib", imu.getCalibrationStatus().toString());
        telemetry.addData("Heading", formatAngle(angles.angleUnit, angles.firstAngle));
        telemetry.addData("Roll", formatAngle(angles.angleUnit, angles.secondAngle));
        telemetry.addData("Pitch", formatAngle(angles.angleUnit, angles.thirdAngle));
        telemetry.addData("Grav", gravity.toString());
        
        telemetry.update();
    }

    String formatAngle(AngleUnit angleUnit, double angle)
    {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees)
    {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}