package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.Vector;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import java.util.List;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@TeleOp(name="BasicMecanum", group="")

public class BasicMecanum extends LinearOpMode {

    private DcMotor D1 = null;
    private DcMotor D2 = null;
    private DcMotor D3 = null;
    private DcMotor D4 = null;
    private DcMotor D5 = null;
    private DcMotor D6 = null;
    private DcMotor D7 = null;
    private DcMotor D8 = null;
    
    static final double kP = 0.03;
    static final double kI = 0.00;
    static final double kD = 0.00;
    static final double kF = 0.00;
    
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
        
        D1.setDirection(DcMotor.Direction.REVERSE);
        D2.setDirection(DcMotor.Direction.REVERSE);
        D3.setDirection(DcMotor.Direction.FORWARD);
        D4.setDirection(DcMotor.Direction.FORWARD);
        
        waitForStart();

        while (opModeIsActive()) {

           double degree = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) *180/Math.PI + 90;
           if (degree<0) {
               degree += 360;
           }
           
           double Magnitude = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
           double rightX = gamepad1.right_stick_x;

           double M1 = gamepad1.left_stick_x  - gamepad1.left_stick_y + rightX;
           double M2 = gamepad1.left_stick_x  + gamepad1.left_stick_y + rightX;
           double M3 = gamepad1.left_stick_x  + gamepad1.left_stick_y - rightX;
           double M4 = gamepad1.left_stick_x  - gamepad1.left_stick_y - rightX;
           

           D1.setPower(M1);
           D2.setPower(M2);
           D3.setPower(M3);
           D4.setPower(M4);
           
           telemetry.addData("Degree", degree);
           telemetry.addData("M", Magnitude);
           
           telemetry.addData("M1", M1);
           telemetry.addData("M2", M2);
           telemetry.addData("M3", M3);
           telemetry.addData("M4", M4);
           telemetry.update();
           
        }
    }
}
