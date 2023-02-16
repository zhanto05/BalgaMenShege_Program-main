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
    import com.qualcomm.robotcore.hardware.AnalogInput;
    import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Config
@TeleOp(name = "fake and gay")
public class lightSensorsTest extends LinearOpMode {

    ColorSensor front; ColorSensor right; ColorSensor back; ColorSensor left;
    Servo tower;
    CRServo lift;


    @Override
    public void runOpMode() {
        lift = hardwareMap.get(CRServo.class, "lift");
        front = hardwareMap.get(ColorSensor.class, "front"); right = hardwareMap.get(ColorSensor.class, "right");
        left = hardwareMap.get(ColorSensor.class, "left"); back = hardwareMap.get(ColorSensor.class, "back");
        tower = hardwareMap.get(Servo.class, "tower");
        ElapsedTime timer = new ElapsedTime();
        GamepadEx driverOp = new GamepadEx(gamepad1);

        int trg = 1; //1 right, 2 fwd, 3 left, 4 back
        int prevtrg = 1;
        int dir = 1;
        int targetVal = 3000;

        if (isStopRequested()) return;
            waitForStart();
        int prevreading = 0;
        int deltareading = 0;
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

            if(trg > prevtrg){
                dir = 1;
            }
            if(trg < prevtrg){
                dir = -1;
            }

            int error = targetVal - reading;
            double kP = 0.0003;

            double vel = kP * error * dir;
            setTower(vel);

            prevreading = reading;
        }
    }

    void setTower(double v){
        double vScaled = clamp(v, 1);
        vScaled *= 0.25;
        tower.setPosition(0.5+vScaled);
    }
    double clamp(double v, double bounds) {
        return clamp(v, -bounds, bounds);
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