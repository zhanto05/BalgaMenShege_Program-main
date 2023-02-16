package org.firstinspires.ftc.teamcode.driving;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
@TeleOp(name = "PID Field Centric")
public class FieldCentric extends OpMode {
    private BHI260IMU imu;
    private double heading = 0;
    private ElapsedTime elapsedTime;
    @Override
    public void init() {
        imu = hardwareMap.get(BHI260IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(parameters);
        elapsedTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        imu.resetYaw();

    }
    double  n;
    double delta;
    double yawPrev;
    double yaw;
    double offsetAngle = 0;
    double prevHeading = 0;

    @Override
    public void loop() {
        yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        //(100);
        delta = yaw - yawPrev;
        if(delta > 600){
            n++;
        }
        else if(delta < -600){
            n--;
        }
        telemetry.addData("angle", imu.getRobotYawPitchRollAngles());
        telemetry.addData("n", n);
        yawPrev = yaw;
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