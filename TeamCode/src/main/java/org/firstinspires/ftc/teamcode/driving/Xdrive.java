package org.firstinspires.ftc.teamcode.driving;

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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Config
@TeleOp(name = "MainXDrive")
public class Xdrive extends LinearOpMode {
    private Motor frontLeftMotor = null;
    private Motor backLeftMotor = null;
    private Motor frontRightMotor = null;
    private Motor backRightMotor = null;
    private Servo claw = null;
    private CRServo tower = null;
    private CRServo lift = null;
    private BNO055IMU imu = null;

    private DigitalChannel lmao = null;

    Orientation lastAngles = new Orientation();
    double globalAngle;
    public static double kP = 0.04, kI = 0, kD = 0;

    @Override
    public void runOpMode() {
        frontLeftMotor = new Motor(hardwareMap, "leftFront");
        backLeftMotor = new Motor(hardwareMap, "leftRear");
        frontRightMotor = new Motor(hardwareMap, "rightFront");
        backRightMotor = new Motor(hardwareMap, "rightRear");
        //backRightMotor = hardwareMap.get(Motor.class, "rightRear");
        claw = hardwareMap.get(Servo.class, "claw");
        tower = hardwareMap.get(CRServo.class, "tower");
        lift = hardwareMap.get(CRServo.class, "lift");
        lmao = hardwareMap.get(DigitalChannel.class, "lmao");

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        PIDFController ang = new PIDFController(kP, kI, kD, 0);
        HDrive drive = new HDrive(frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor);

        lmao.setMode(DigitalChannel.Mode.OUTPUT);


        // Retrieve the IMU from the hardware map
        //imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        imu.initialize(parameters);

        double savedHeading = 0.0;
        double targetHeading = 0.0;
        double turnRate = -240;
        double pHeading = 0.15;
        double errHeading = 0.0;

        double deltaHeading = 0.0;
        double lastHeading = 0.0;

        ElapsedTime timer = new ElapsedTime();
        double prevTime = 0;
        ElapsedTime tick = new ElapsedTime();
        GamepadEx driverOp = new GamepadEx(gamepad1);

        if (isStopRequested()) return;
        waitForStart();
        while (opModeIsActive()) {
            lmao.setState(driverOp.getButton(GamepadKeys.Button.DPAD_LEFT));
            if (driverOp.getButton(GamepadKeys.Button.DPAD_LEFT)) {

            }
            else if (driverOp.getButton(GamepadKeys.Button.DPAD_RIGHT)) {
//                tower.setPower(-1);
            }
            else{
                //tower.setPower(0);
            }
            if (driverOp.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
                lift.setPower(1);
            }
            else if (driverOp.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
                lift.setPower(-1);
            }
            else{
                lift.setPower(0);
            }
            if(driverOp.getButton(GamepadKeys.Button.A)){
                claw.setPosition(0);
            }
            else if (driverOp.getButton(GamepadKeys.Button.Y)){
                claw.setPosition(1);
            }



            ang.setP(kP);
            ang.setI(kI);
            ang.setD(kD);
            double elapsedTime = timer.time() - prevTime;
            prevTime = timer.time();

            double absoluteHeading = getAngle();

            double relativeHeading;
            if (gamepad1.b) {
                savedHeading = absoluteHeading ;
            }

//            targetHeading -= gamepad1.right_stick_x * elapsedTime;

            relativeHeading = absoluteHeading - savedHeading;
            if (driverOp.getRightY() * driverOp.getRightY() + driverOp.getRightX() * driverOp.getRightX() > 0.4) {
                //pizdec xuyna
                //targetHeading += driverOp.getRightX() * turnRate * elapsedTime;

                //blyaaaaaaaaaa
                targetHeading = Math.toDegrees(Math.atan2(-driverOp.getRightX(), -driverOp.getRightY()));
            }

            if(targetHeading > 179){
                targetHeading = 179;
            }
            else if(targetHeading < -179){
                targetHeading = -179;
            }

//adilrofl
//            if(targetHeading > 210){
//                targetHeading -= 390;
//            }
//            if(targetHeading < -210){
//                targetHeading += 390;
//            }

            //dastanrofl
//            double turnedValue;
//            double needToTurn = 0;
//            if(targetHeading > 180 && absoluteHeading <=160){
//                turnedValue = getAngle();
//                needToTurn = 360 - turnedValue;
//                absoluteHeading = 0;
//            }else if(targetHeading > needToTurn && absoluteHeading <= needToTurn){
//                absoluteHeading = 0;
//            }

//            if (absoluteHeading > 180) {
//                absoluteHeading -= 360;
//            }
//            if (absoluteHeading < -180) {
//                absoluteHeading += 360;
//            }

            errHeading = targetHeading - relativeHeading;

            double turn = ang.calculate(relativeHeading, targetHeading);

            drive.driveFieldCentric(-driverOp.getLeftX(), -driverOp.getLeftY(), turn, relativeHeading);

            deltaHeading = absoluteHeading - lastHeading;
            lastHeading = absoluteHeading;


            telemetry.addData("cock", lift.getManufacturer());
            telemetry.addData("absolute heading", absoluteHeading);
            telemetry.addData("saved heading", savedHeading);
            telemetry.addData("relative heading", relativeHeading);
            telemetry.addData("target heading", targetHeading);
            telemetry.addData("error", errHeading);
            telemetry.addData("kP", kP);
            telemetry.addData("kP * error", kP * errHeading);
            telemetry.addData("delta", deltaHeading);
            telemetry.update();
        }
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
    void resetAngle(){
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }

    double getAngle(){
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;
//        if(deltaAngle < -180)
//            deltaAngle += 360;
//        else if(deltaAngle > 180)
//            deltaAngle -= 360;
        globalAngle+=deltaAngle;
        lastAngles = angles;

        return globalAngle;
    }
}


//// Adjust the orientation parameters to match your robot
//        BNO055IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
//                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD));
//// Without this, the REV Hub's orientation is assumed to be logo up / USB forward
//        imu.initialize(parameters);

//        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
//        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
//
//        frontLeftMotor.setInverted(true);
//        backLeftMotor.setInverted(true);


//        waitForStart();
//        while(opModeIsActive()) {
//
//            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
//            double x = gamepad1.left_stick_x;
//            double rx = gamepad1.right_stick_x;
//
//            frontLeftMotor.setPower(y + x + rx);
//            backLeftMotor.setPower(y - x + rx);
//            frontRightMotor.setPower(y - x - rx);
//            backRightMotor.setPower(y + x - rx);
//        }


//            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
//            double x = gamepad1.left_stick_x ; // Counteract imperfect strafing
//            double rx = gamepad1.right_stick_x;

//            if (y > 0.7) {
//                y = 1;
//            }
//            if (y < -0.7) {
//                y = -1;
//            }
//            if (x > 0.7) {
//                x = 1;
//            }
//            if (x < -0.7) {
//                x = -1;
//            }
//
//            y *= 0.8;
//            x *= 0.8;

//            double absoluteHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);


//            double rotVel = errHeading * pHeading;
//            clamp(rotVel, 1);

// Rotate the movement direction counter to the bot's rotation
//            double rotX = x * Math.cos(-relativeHeading) - y * Math.sin(-relativeHeading);
//            double rotY = x * Math.sin(-relativeHeading) + y * Math.cos(-relativeHeading);

//            drive.driveFieldCentric(x, y, rx, absoluteHeading);



//            double lenDriveVec = Math.hypot(x, y);
//            double angDrive = Math.atan2(x, y);
//
//            double angMotor = angDrive - Math.toRadians(relativeHeading);
//
//            double rotX = Math.sin(angMotor) * lenDriveVec;
//            double rotY = Math.cos(angMotor) * lenDriveVec;
//
//            // Denominator is the largest motor power (absolute value) or 1
//            // This ensures all the powers maintain the same ratio, but only when
//            // at least one is out of the range [-1, 1]
//            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
//            double frontLeftPower = (rotY + rotX + rx) / denominator;
//            double backLeftPower = (rotY - rotX + rx) / denominator;
//            double frontRightPower = (rotY - rotX - rx) / denominator;
//            double backRightPower = (rotY + rotX - rx) / denominator;


//            frontLeftMotor.set(frontLeftPower);
//            backLeftMotor.set(backLeftPower);
//            frontRightMotor.set(frontRightPower);
//            backRightMotor.set(backRightPower);
