package org.firstinspires.ftc.teamcode.driving;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.drivebase.HDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Config
@TeleOp(name = "drive program prototype")
public class drivePrototype extends LinearOpMode {
    public static double kP = 0.018, kI = 0, kD = 0;
    public static double kPTower = 0.0003;
    ColorSensor right;
    ColorSensor back;
    ColorSensor left;
    Orientation lastAngles = new Orientation();
    double globalAngle;
    private Motor frontLeftMotor = null;
    private Motor backLeftMotor = null;
    private Motor frontRightMotor = null;
    private Motor backRightMotor = null;
    private Servo claw = null;
    private CRServo lift = null;
    private Servo tower = null;
    private ColorSensor front;
    private BNO055IMU imu = null;

    @Override
    public void runOpMode() {
        frontLeftMotor = new Motor(hardwareMap, "leftFront");
        backLeftMotor = new Motor(hardwareMap, "leftRear");
        frontRightMotor = new Motor(hardwareMap, "rightFront");
        backRightMotor = new Motor(hardwareMap, "rightRear");

        claw = hardwareMap.get(Servo.class, "claw");
        tower = hardwareMap.get(Servo.class, "tower");
        lift = hardwareMap.get(CRServo.class, "lift");
        front = hardwareMap.get(ColorSensor.class, "front");
        right = hardwareMap.get(ColorSensor.class, "right");
        left = hardwareMap.get(ColorSensor.class, "left");
        back = hardwareMap.get(ColorSensor.class, "back");
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        frontLeftMotor.setRunMode(Motor.RunMode.RawPower);
        frontRightMotor.setRunMode(Motor.RunMode.RawPower);
        backLeftMotor.setRunMode(Motor.RunMode.RawPower);
        backRightMotor.setRunMode(Motor.RunMode.RawPower);

        PIDFController ang = new PIDFController(kP, kI, kD, 0);
        HDrive drive = new HDrive(frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
//        imu = hardwareMap.get(BHI260IMU.class, "imu");
//        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
//        imu.initialize(parameters);

        imu.initialize(parameters);

        double savedHeading = 0.0;
        double targetHeading = 0.0;
        double turnRate = -180;
        double pHeading = 0.15;
        double errHeading = 0.0;
        double deltaHeading = 0.0;
        double depression = 0;

        int trgTower = 2; //1 right, 2 fwd, 3 left, 4 back
        int trgTowerPrev = 2;
        int dir = 1;
        int dir2 = 1;
        int targetTowerVal = 2850;
        int error = 0;
        int errorPrev = 0;
        int errorDelta = 0;
        int readingTower = 0;

        double lastHeading = 0.0;
        ElapsedTime timer = new ElapsedTime();
        double prevTime = 0;

        ElapsedTime tick = new ElapsedTime();
        GamepadEx driverOp = new GamepadEx(gamepad1);
        ang.setP(kP);
        ang.setI(kI);
        ang.setD(kD);

        if (isStopRequested())
            return;
        waitForStart();
        double deltaTime = 0;
        double timerS = 0;
        boolean lTLast = false;
        boolean clawOpen = false;
        double clawPos = 0;
        while (opModeIsActive()) {
            double xStick = driverOp.getLeftX();
            double yStick = driverOp.getLeftY();
            double velo = 1;
            if (Math.abs(xStick) > 0.75) {
                xStick = Math.signum(xStick);
            }
            if (Math.abs(yStick) > 0.75) {
                yStick = Math.signum(yStick);
            }
            velo = 1 - driverOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) * 0.7;
            boolean lT = gamepad2.left_trigger > 0.2;
            double liftVel = 0;
            if (!lTLast && lT) {
                clawOpen = !clawOpen;
            }
            if (gamepad2.left_bumper)
                clawOpen = !clawOpen;

            if (clawOpen) {
                clawPos = 0.8;
            } else {
                clawPos = 0.6;
            }
            claw.setPosition(clawPos);
            lTLast = lT;
            //set the target position for the tower based on driver input
            if (gamepad2.dpad_down) {
                trgTowerPrev = trgTower;
                trgTower = 4;
            }
            if (gamepad2.dpad_right) {
                trgTowerPrev = trgTower;
                trgTower = 1;
            }
            if (gamepad2.dpad_left) {
                trgTowerPrev = trgTower;
                trgTower = 3;
            }
            if (gamepad2.dpad_up) {
                trgTowerPrev = trgTower;
                trgTower = 2;
            }
            if (gamepad2.right_bumper) {
                liftVel = 1;
            } else if (gamepad2.right_trigger > 0.2) {
                liftVel = -0.775;
            } else {
                liftVel = 0;
            }

            //read from light sensor based on target
            switch (trgTower) {
                case 2:
                    readingTower = front.alpha();
                    break;
                case 1:
                    readingTower = right.alpha();
                    break;
                case 4:
                    readingTower = back.alpha();
                    break;
                case 3:
                    readingTower = left.alpha();
                    break;
            }
            //calculate absolute and relative heading
            double absoluteHeading = getAngle();

            double relativeHeading = absoluteHeading - savedHeading;
            if (gamepad1.b) {
                resetAngle();
                targetHeading = 0;
            }
            //set the target heading
            //if (driverOp.getRightY() * driverOp.getRightY() + driverOp.getRightX() * driverOp.getRightX() > 0.4) {
            //targetHeading = Math.toDegrees(Math.atan2(-driverOp.getRightX(), -driverOp.getRightY()));
            //}1
            if (Math.abs(driverOp.getRightX()) > 0.15) {
                targetHeading += driverOp.getRightX() * turnRate * deltaTime / 1000;
            }
            if (targetHeading < -180) {
                targetHeading += 360;
            } else if (targetHeading > 180) {
                targetHeading -= 360;
            }
            if ((relativeHeading > 90 && targetHeading < -90) || (relativeHeading < -90 && targetHeading > 90)) {
                dir2 = -1;
            } else {
                dir2 = 1;
            }
            if (Math.abs(ang.getPositionError()) > 300) {
                depression = 0.65;
            } else {
                depression = 1;
            }
            //lift turn direction
            if (trgTower > trgTowerPrev) {
                dir = 1;
                //targetTowerVal = -Math.abs(targetTowerVal);
            }
            if (trgTower < trgTowerPrev) {
                dir = -1;
                //targetTowerVal = Math.abs(targetTowerVal);
            }

            if (errorDelta < -2500) {
                dir = -1;
            }
//            if(relativeHeading > 0){
//
//            }
//            else {
//                relativeHeading += 360;
//            }

            //calculate error and pid
            errHeading = targetHeading - relativeHeading;
            double turn = ang.calculate(relativeHeading, targetHeading) * dir2 * depression;
            error = targetTowerVal - readingTower;
            double vel = kPTower * error * dir;

            //set velocities
            if (!gamepad2.y) {
                setTower(vel);
            } else {
                setTower(0);
            }
            drive.driveFieldCentric(-yStick * velo, xStick * velo, turn, relativeHeading);
            lift.setPower(liftVel);
            //calculate deltas
            deltaHeading = relativeHeading - lastHeading;
            lastHeading = relativeHeading;
            errorDelta = error - errorPrev;
            errorPrev = error;
            //tele
            telemetry.addLine("Heading");
            telemetry.addData("absolute", absoluteHeading);
            telemetry.addData("relative", relativeHeading);
            telemetry.addData("target", targetHeading);
            telemetry.addData("error", errHeading);
            telemetry.addData("kP * error * kms", kP * errHeading * depression);
            telemetry.addData("delta", deltaHeading);

            telemetry.addLine("");
            telemetry.addLine("");

            telemetry.addLine("Tower");
            telemetry.addData("target", targetTowerVal);
            telemetry.addData("error", error);
            telemetry.addData("kP * error", kPTower * error);
            telemetry.addData("error delta", errorDelta);

            telemetry.addLine("");
            telemetry.addLine("");

            telemetry.addLine("Claw");
            telemetry.addData("pos", claw.getPosition());
            telemetry.addLine("Dead test");

            telemetry.update();
            deltaTime = timer.milliseconds() - timerS;
            timerS = timer.milliseconds();
        }
    }

    void setTower(double v) {
        double vScaled = clamp(v, 1);
        vScaled *= 0.25;
        tower.setPosition(0.5 + vScaled);
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

    void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.YZX, AngleUnit.DEGREES);
        globalAngle = 0;
    }

    private double getAngle() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle_ = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle_ < -180)
            deltaAngle_ += 360;
        else if (deltaAngle_ > 180)
            deltaAngle_ -= 360;

        globalAngle += deltaAngle_;

        lastAngles = angles;

        return globalAngle;
    }

}