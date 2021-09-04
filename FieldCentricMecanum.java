package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import java.util.Locale;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.util.List;
import java.util.Vector;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="FieldCentricMecanum", group="")

public class FieldCentricMecanum extends LinearOpMode {

    private DcMotor D1 = null;
    private DcMotor D2 = null;
    private DcMotor D3 = null;
    private DcMotor D4 = null;
    private DcMotor D5 = null;
    private DcMotor D6 = null;
    private DcMotor D7 = null;
    private DcMotor D8 = null;
    
    Servo S1;
    double servoPosition = 1.0;

    
    static final double kP = 0.03;
    static final double kI = 0.00;
    static final double kD = 0.00;
    static final double kF = 0.00;
    
    private BNO055IMU imu;
    private Orientation angles;
    private Acceleration gravity;
    ElapsedTime timer = new ElapsedTime();
    int jun = 0;
    int stage = 0;
   @Override
    public void runOpMode() {
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
        S1 = hardwareMap.servo.get("S1");
        S1.setPosition(servoPosition);
        
        
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

        while (opModeIsActive()) {
           double leftX = gamepad1.left_stick_x;
           double leftY = gamepad1.left_stick_y;
           double rightX = gamepad1.right_stick_x;
           double rightY = gamepad1.right_stick_y;
           double ML = Math.hypot(leftY, leftX);
           double MR = Math.hypot(rightY, rightX);
           double degree = Math.atan2(leftY, leftX) *180/Math.PI + 90;
               if (degree<0) {
                   degree += 360;
               }
           double M1 = leftX  - leftY + rightX;
           double M2 = leftX  + leftY + rightX;
           double M3 = leftX  + leftY - rightX;
           double M4 = leftX  - leftY - rightX;
           

           D1.setPower(M1);
           D2.setPower(M2);
           D3.setPower(M3);
           D4.setPower(M4);
           //Code for the collector and conveyor
           if (gamepad2.left_stick_y >= .01){    
            D8.setPower(-gamepad2.left_stick_y);
           } else if (gamepad2.left_stick_y <= -.01){
            D8.setPower(-gamepad2.left_stick_y);   
           } else {
            D8.setPower(0);
           }
            
           if (gamepad2.right_trigger == 1.0){
               D5.setPower(-5);
               D6.setPower(-5);
           }
           if (gamepad2.right_bumper){
               D5.setPower(0);
               D6.setPower(0);
           }
           //Code for shooter
           
           
           if (gamepad2.left_trigger == 1.0){
              timer.reset();
              stage = 1;
               
           }

           // conveyor down
           if (stage == 1){
               D6.setPower(5);
               D7.setPower(0);
               if (timer.time() >= 0.20){
                   timer.reset();
                   stage = 2;
               }
           }
           // shooter on
           if (stage == 2){
               D6.setPower(0);
               D7.setPower(-5);
               if (timer.time() >= 1.0){
                   timer.reset();
                   stage = 3;
               }
           }
           // conveyor up, everything off
           if (stage == 3){
               D6.setPower(-1);
           }
           if (gamepad2.left_bumper){
                D6.setPower(0);
                D7.setPower(0);
                stage = 4;
           }
           if (gamepad2.x){
                servoPosition = 1.0;
                S1.setPosition(servoPosition);
                
           }
           if (gamepad2.y){
                servoPosition = 0.5;
                S1.setPosition(servoPosition);
                
           }
            angles  = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity = imu.getGravity();
            sendTelemetry();           


           
        }
    }
     void sendTelemetry()
    {
        telemetry.addData("Status", imu.getSystemStatus().toString());
        telemetry.addData("Calib", imu.getCalibrationStatus().toString());
        telemetry.addData("Heading", formatAngle(angles.angleUnit, angles.firstAngle));
        telemetry.addData("Roll", formatAngle(angles.angleUnit, angles.secondAngle));
        telemetry.addData("Pitch", formatAngle(angles.angleUnit, angles.thirdAngle));
        telemetry.addData("Angles", angles.toString());
        telemetry.addData("Grav", gravity.toString());
        telemetry.addData("time", timer.toString());
        telemetry.addData("stage", stage); //INTS CAN BE PRINTED!!!
        telemetry.addData("jun",servoPosition);
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