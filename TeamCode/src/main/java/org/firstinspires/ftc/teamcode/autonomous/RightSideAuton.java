package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="auton right side", group="Linear Opmode")
public class RightSideAuton extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor liftMotor = null;
    private Servo claw = null;
    private TouchSensor touch = null;

    private DistanceSensor color = null;

    private BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle;

    private AprilTagsDetector tagsDetector;
    private AprilTagsDetector.Tag foundTag;
    @Override
    public void runOpMode() {
        tagsDetector = new AprilTagsDetector(hardwareMap, telemetry);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        foundTag = AprilTagsDetector.Tag.noTag;
        while(opModeInInit()){
            AprilTagsDetector.Tag temptag = tagsDetector.getTag();
            foundTag = temptag;
            telemetry.update();
        }

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive  = hardwareMap.get(DcMotor.class, "left");
        rightDrive = hardwareMap.get(DcMotor.class, "right");
        liftMotor = hardwareMap.get(DcMotor.class, "lift");
        claw = hardwareMap.get(Servo.class, "claw");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        color = hardwareMap.get(DistanceSensor.class, "color");
        touch = hardwareMap.get(TouchSensor.class, "touch");
//        boolean PrevState = false;
//        boolean liftBind = false;

        imu.initialize(parameters);

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        //claw.setDirection(CRServo.Direction.FORWARD);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        double junction = 0;
        // run until the end of the match (driver presses STOP)
        rightDriveAuton();
    }

    void resetDrive(){
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    void resetAngle(){
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }

    double getAngle(){
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;
        if(deltaAngle < -180)
            deltaAngle += 360;
        else if(deltaAngle > 180)
            deltaAngle -= 360;
        globalAngle+=deltaAngle;
        lastAngles = angles;

        return globalAngle;
    }

    void leftSideAuton(){
        double circ = 3.54 * 3.14;
        double k = 360/circ;
        double dErr = 0, dTarg = 0, dP = 0.00063, dSucc = 10, dCurr = 0, dMax = 0.7, dF = 0.10;
        double lErr = 0, lTarg = 0, lP = 0.015, lSucc = 10, lCurr = 0, lMax = 1;
        double tErr = 0, tTarg = 0, tP = 0.00925, tSucc = 1, tCurr = 0, tMax = 0.5, tF = 0.04;
        double corrP = 0.004;
        int step = 0;

        boolean turning = false;
        resetBase();
        resetAngle();
        boolean bebra = true;
        while(opModeIsActive() && bebra){
            switch(step){
                case 0:
                    turning = false;
                    dTarg = k * 82;
                    lTarg = 1000;
                    break;
                case 1:
                    turning = true;
                    tTarg = -45;
                    lTarg = 2100;
                    resetBase();
                    break;
                case 2:
                    turning = false;
                    dTarg = k * 3;
                    break;
                case 3:
                    claw.setPosition(1);
                    dTarg = k * -0.5;
                    break;
                case 4:
                    lTarg = 1000;
                    //tMax = 0.25;
                    turning = true;
                    tTarg = 90;
                    resetBase();
                    break;
                case 5:
                    tMax = 0.5;
                    turning = false;
                    lTarg = 0;
                    double mv = 0;
                    //mv = 0 if middle park, 1 or -1 for side park
                    if (foundTag == AprilTagsDetector.Tag.left) {
                        mv = 1;
                    } else if (foundTag == AprilTagsDetector.Tag.right) {
                        mv = -1;
                    } else {
                        mv = 0;
                    }
                    telemetry.addData("parking...", 1000);
                    dTarg = k * mv * 72;
                    break;
                case 6:
                    bebra = false;
                    break;
                default:
                    break;

            }

            if(!turning){
                dCurr = basePos();
            }
            tCurr = getAngle();
            lCurr = liftMotor.getCurrentPosition();

            dErr = dTarg - dCurr;
            tErr = tTarg - tCurr;
            lErr = lTarg - lCurr;
            telemetry.addData("step", step);

            liftMotor.setPower(clamp(lErr * lP, -lMax, lMax));

            if(!turning && (abs(dErr) > dSucc)){
                double tv = (dErr * dP) + Math.signum(dErr) * dF;
                double v = clamp(tv, -dMax, dMax);
                setBase(v - (tErr * corrP), v + (tErr * corrP));
                telemetry.addData("dErr", dErr);
            }
            else if(turning && (abs(tErr) > tSucc)){
                double tv = (tErr *tP) + Math.signum(tErr)*tF;
                double v = clamp(tv, -tMax, tMax);

                setBase(-v, v);
                telemetry.addData("tErr", tErr);
            }

            if(abs(dErr) < dSucc && abs(tErr) < tSucc && abs(lErr) < lSucc){
                step++;
            }
            telemetry.update();
        }
    }

    void rightDriveAuton(){
        double circ = 3.54 * 3.14;
        double k = 360/circ;
        double dErr = 0, dTarg = 0, dP = 0.0007, dSucc = 15, dCurr = 0, dMax = 0.7, dF = 0.10;
        double lErr = 0, lTarg = 0, lP = 0.0158, lSucc = 12, lCurr = 0, lMax = 1;
        double tErr = 0, tTarg = 0, tP = 0.009, tSucc = 1, tCurr = 0, tMax = 0.6, tF = 0.065;
        double corrP = 0.004;
        int step = -2;
        double mv = 0;
        boolean turning = false;
        resetBase();
        resetAngle();
        boolean bebra = true;
        while(opModeIsActive() && bebra){
            switch(step){
                case -2:
                    claw.setPosition(0.45);
                    dTarg = k * 20;
                    if (foundTag == AprilTagsDetector.Tag.left) {
                        mv = 1;
                    } else if (foundTag == AprilTagsDetector.Tag.right) {
                        mv = -1;
                    } else {
                        mv = 0;
                    }
                    break;
                case -1:
                    //claw.setPosition(0.45);
                    turning = false;
                    dTarg = k * 83.5;
                    lTarg = 1000;
                    break;
                case 0:
                    turning = true;
                    tTarg = 45;
                    lTarg = 1950;
                    break;
                case 1:
                    resetBase();
                    break;
                case 2:
                    turning = false;
                    dTarg = k * 8.5;
                    break;
                case 3:
                    lTarg = 1900;
                    claw.setPosition(1);
                    dTarg = 0;
                    break;
                case 4:
                    lTarg = 1000;
                    tMax = 0.35;
                    turning = true;
                    tTarg = 90;
                    resetBase();
                    break;
                case 5:
                    //tMax = 0.5;
                    turning = false;
                    lTarg = 0;

                    //mv = 0 if middle park, 1 or -1 for side park
                    if (foundTag == AprilTagsDetector.Tag.left) {
                        mv = 1;
                        telemetry.addData("mv = ",mv);
                    } else if (foundTag == AprilTagsDetector.Tag.right) {
                        mv = -1;
                        telemetry.addData("mv = ",mv);
                    } else {
                        mv = 0;
                        telemetry.addData("mv = ",mv);
                    }
                    dTarg = k * mv * 30;
                    break;
                case 6:
                    bebra = false;
                    break;
                default:
                    break;

            }

            if(!turning){
                dCurr = basePos();
            }
            tCurr = getAngle();
            lCurr = liftMotor.getCurrentPosition();

            dErr = dTarg - dCurr;
            tErr = tTarg - tCurr;
            lErr = lTarg - lCurr;
            telemetry.addData("step", step);
            telemetry.addData("mv", mv);
            liftMotor.setPower(clamp(lErr * lP, -lMax, lMax));

            if(!turning && (abs(dErr) > dSucc)){
                double tv = (dErr * dP) + Math.signum(dErr) * dF;
                double v = clamp(tv, -dMax, dMax);
                setBase(v - (tErr * corrP), v + (tErr * corrP));
                telemetry.addData("dErr", dErr);
            }
            else if(turning && (abs(tErr) > tSucc)){
                double tv = (tErr *tP) + Math.signum(tErr)*tF;
                double v = clamp(tv, -tMax, tMax);

                setBase(-v, v);
                telemetry.addData("tErr", tErr);
            }

            if(abs(dErr) < dSucc && abs(tErr) < tSucc && abs(lErr) < lSucc){
                step++;
            }
            telemetry.update();
        }
    }

    double abs(double a){
        return Math.abs(a);
    }

    double clamp(double v, double min, double max){
        if(v > max){
            return max;
        }
        else return Math.max(v, min);
    }

    void setBase(double v){
        leftDrive.setPower(v);
        rightDrive.setPower(v);
    }
    void setBase(double l, double r){
        leftDrive.setPower(l);
        rightDrive.setPower(r);
    }
    void resetBase(){
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    double basePos(){
        return (rightDrive.getCurrentPosition()+leftDrive.getCurrentPosition())/2;
    }

    void bebruh(double t){
        double p = 0.00325;
        double succ = 0.25;
        double max = 0.7;
        double error = t;
        int c = 0;
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        while(c < 30 && opModeIsActive()){
            error = t - getAngle();
            double v = clamp(p * error, -max, max);

            if(Math.abs(error) < succ){
                c++;
            }

            setBase(-v, v);
            telemetry.addData("angle", error);
            telemetry.addData("count", c);
            telemetry.update();
        }
    }

    class PIDController{
        double P, I, D;
        double target;
        //finish later

        PIDController(double p, double i, double d, double Target){
            P = p; I = i; d = D; target = Target;
        }

        double output(double point){
            double error = target - point;
            return P * error;
        }
    }
}
