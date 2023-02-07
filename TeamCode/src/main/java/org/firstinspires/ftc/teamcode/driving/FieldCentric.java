package org.firstinspires.ftc.teamcode.driving;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@TeleOp(name = "PID Field Centric")
public class FieldCentric extends OpMode {

    private Motor fL, fR, bL, bR;
    private MecanumDrive drive;
    private GamepadEx driverOp;

    private RevIMU imu;

    public static double kP, kI, kD;
    private PIDFController headingPID;

    private double heading = 0;
    private ElapsedTime elapsedTime;

    FtcDashboard dashboard;
    Telemetry dashboardTelemetry;

    @Override
    public void init() {
        fL = new Motor(hardwareMap, "leftFront");
        fR = new Motor(hardwareMap, "rightFront");
        bL = new Motor(hardwareMap, "leftRear");
        bR = new Motor(hardwareMap, "rightRear");

        fL.setInverted(true);
        fR.setInverted(true);
        bL.setInverted(true);
        bR.setInverted(true);

        drive = new MecanumDrive(fL, fR, bL, bR);

        driverOp = new GamepadEx(gamepad1);

        imu = new RevIMU(hardwareMap);
        imu.init();

        headingPID = new PIDFController(kP, kI, kD, 0);

        elapsedTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

        dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();
    }

    double offsetAngle = 0;
    double prevHeading = 0;

    @Override
    public void loop() {
//        headingPID.setPIDF(kP, kI, kD, 0);
//
//        double deltaTime = elapsedTime.time();
//        elapsedTime.reset();

//        heading -= 180 * driverOp.getRightX() * deltaTime;
//
//        double curAng = imu.getRotation2d().getDegrees();
//        double turnPower = headingPID.calculate(curAng + offsetAngle, heading);
//
//
//
//        prevHeading = curAng;
//
//        if (curAng - prevHeading > 180){
//            curAng = offsetAngle + 360;
//            turnPower = headingPID.calculate(curAng + offsetAngle, heading);
//        }
//        else  if (curAng - prevHeading < -180){
//            curAng = offsetAngle  - 360;
//            turnPower = headingPID.calculate(curAng + offsetAngle, heading);
//        }

        drive.driveFieldCentric(
                driverOp.getLeftX(),
                driverOp.getLeftY(),
                driverOp.getRightX(),
                imu.getRotation2d().getDegrees(),
                true
        );

        dashboardTelemetry.addData("current", imu.getRotation2d().getDegrees()  + offsetAngle);
        dashboardTelemetry.addData("target", heading);
        dashboardTelemetry.update();

        telemetry.addData("current", imu.getRotation2d().getDegrees()  + offsetAngle);
        telemetry.addData("target", heading);
        telemetry.update();
    }

    @Override
    public void stop() {

    }
}
//if (curAng - prevHeading > 180)
//        {
//        offsetAngle += 360;
//        turnPower = headingPID.calculate(curAng + offsetAngle, -heading);
//        }
//        if (curAng - prevHeading <= -180)
//        {
//        offsetAngle -= 360;
//        turnPower = headingPID.calculate(curAng + offsetAngle, -heading);
//        }