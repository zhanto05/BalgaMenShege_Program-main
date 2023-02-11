package org.firstinspires.ftc.teamcode.driving;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.drivebase.HDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Config
@TeleOp(name = "fake and gay")
public class lightSensorsTest extends LinearOpMode {

    ColorSensor front;
    ColorSensor right;
    ColorSensor back;
    ColorSensor left;

    Servo balls;


    @Override
    public void runOpMode() {
        balls = hardwareMap.get(Servo.class, "lift");
        front = hardwareMap.get(ColorSensor.class, "front");
        right = hardwareMap.get(ColorSensor.class, "right");
        left = hardwareMap.get(ColorSensor.class, "left");
        back = hardwareMap.get(ColorSensor.class, "back");


        //PIDFController ang = new PIDFController(kP, kI, kD, 0);
        ElapsedTime timer = new ElapsedTime();
        GamepadEx driverOp = new GamepadEx(gamepad1);

        int trg = 1; //1 right, 2 fwd, 3 left, 4 back
        int prevtrg = 1;
        int dir = 1;
        int targetVal = 3500;

        if (isStopRequested()) return;
        waitForStart();
        while (opModeIsActive()){
            if(gamepad1.dpad_down){
                prevtrg = trg;
                trg = 4;
            }
            if(gamepad1.dpad_right){
                prevtrg = trg;
                trg = 1;
            }
            if(gamepad1.dpad_left){
                prevtrg = trg;
                trg = 3;
            }
            if(gamepad1.dpad_up){
                prevtrg = trg;
                trg = 2;
            }
            int reading = 0;
            switch(trg){
                case 2:
                    reading = front.alpha();
                    break;
                case 1:
                    reading = right.alpha();
                    break;
                case 4:
                    reading = back.alpha();
                    break;
                case 3:
                    reading = left.alpha();
                    break;
            }

            int error = targetVal - reading;
            double kP = 0.00012;
            if(trg > prevtrg){
                dir = 1;
            }
            if(trg < prevtrg){
                dir = -1;
            }

            double vel = kP * error * dir;

            if(gamepad1.right_bumper){
                balls.setPosition(0.75);
            }
            else if(gamepad1.left_bumper){
                balls.setPosition(0.25);
            }
            else{
                sex(vel);
            }



            telemetry.addData("left", left.alpha());
            telemetry.addData("back", back.alpha());
            telemetry.addData("right", right.alpha());
            telemetry.addData("front", front.alpha());
            telemetry.update();
        }
    }

    double clamp(double v, double bounds) {
        return clamp(v, -bounds, bounds);
    }

    void sex(double penis){
        double cock = clamp(penis, 1);
        cock *= 0.25;
        balls.setPosition(0.5+cock);
    }

    double clamp(double v, double min, double max) {
        if (v > max) {
            return max;
        } else if (v < min) {
            return min;
        } else {
            return v;
        }
    }
}