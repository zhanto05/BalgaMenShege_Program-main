package org.firstinspires.ftc.teamcode.autonomous;

        import com.qualcomm.hardware.bosch.BNO055IMU;
        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import com.qualcomm.robotcore.eventloop.opmode.Disabled;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.Servo;
        import com.qualcomm.robotcore.hardware.TouchSensor;
        import com.qualcomm.robotcore.util.ElapsedTime;
        import com.qualcomm.robotcore.util.Range;

        import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
        import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
        import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
        import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

        import java.util.concurrent.TimeUnit;

/**
 * This file illustrates the concept of driving an autonomous path based on Gyro heading and encoder counts.
 * The code is structured as a LinearOpMode
 * <p>
 * The path to be followed by the robot is built from a series of drive, turn or pause steps.
 * Each step on the path is defined by a single function call, and these can be strung together in any order.
 * <p>
 * The code REQUIRES that you have encoders on the drive motors, otherwise you should use: RobotAutoDriveByTime;
 * <p>
 * This code ALSO requires that you have a BOSCH BNO055 IMU, otherwise you would use: RobotAutoDriveByEncoder;
 * This IMU is found in REV Control/Expansion Hubs shipped prior to July 2022, and possibly also on later models.
 * To run as written, the Control/Expansion hub should be mounted horizontally on a flat part of the robot chassis.
 * <p>
 * This sample requires that the drive Motors have been configured with names : left_drive and right_drive.
 * It also requires that a positive power command moves both motors forward, and causes the encoders to count UP.
 * So please verify that both of your motors move the robot forward on the first move.  If not, make the required correction.
 * See the beginning of runOpMode() to set the FORWARD/REVERSE option for each motor.
 * <p>
 * This code uses RUN_TO_POSITION mode for driving straight, and RUN_USING_ENCODER mode for turning and holding.
 * Note: You must call setTargetPosition() at least once before switching to RUN_TO_POSITION mode.
 * <p>
 * Notes:
 * <p>
 * All angles are referenced to the coordinate-frame that is set whenever resetHeading() is called.
 * In this sample, the heading is reset when the Start button is touched on the Driver station.
 * Note: It would be possible to reset the heading after each move, but this would accumulate steering errors.
 * <p>
 * The angle of movement/rotation is assumed to be a standardized rotation around the robot Z axis,
 * which means that a Positive rotation is Counter Clockwise, looking down on the field.
 * This is consistent with the FTC field coordinate conventions set out in the document:
 * ftc_app\doc\tutorial\FTC_FieldCoordinateSystemDefinition.pdf
 * <p>
 * Control Approach.
 * <p>
 * To reach, or maintain a required heading, this code implements a basic Proportional Controller where:
 * <p>
 * Steering power = Heading Error * Proportional Gain.
 * <p>
 * "Heading Error" is calculated by taking the difference between the desired heading and the actual heading,
 * and then "normalizing" it by converting it to a value in the +/- 180 degree range.
 * <p>
 * "Proportional Gain" is a constant that YOU choose to set the "strength" of the steering response.
 * <p>
 * Use Android Studio to Copy this Class, and Paste it into your "TeamCode" folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name = "Autonomous for blue aliance")
public class BlueAutonomous extends LinearOpMode {
    private final int MAX_LIFT_HEIGHT_ENCODER = 5700;

    /* Declare OpMode members. */
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private BNO055IMU imu = null;      // Control/Expansion Hub IMU
    private DcMotor lift = null;
    private Servo claw = null;
    private TouchSensor touch= null;

    private double robotHeading = 0;
    private double headingOffset = 0;
    private double headingError = 0;

    // These variable are declared here (as class members) so they can be updated in various methods,
    // but still be displayed by sendTelemetry()
    private double targetHeading = 0;
    private double driveSpeed = 0;
    private double turnSpeed = 0;
    private double leftSpeed = 0;
    private double rightSpeed = 0;
    private int leftTarget = 0;
    private int rightTarget = 0;

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double COUNTS_PER_MOTOR_REV = 537.7;   // eg: GoBILDA 312 RPM Yellow Jacket
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 3.54331;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // They can/should be tweaked to suit the specific robot drive train.
    static final double DRIVE_SPEED = 0.8;     // Max driving speed for better distance accuracy.
    static final double TURN_SPEED = 0.45;     // Max Turn speed to limit turn rate
    static final double HEADING_THRESHOLD = 1.0;    // How close must the heading get to the target before moving to next step.
    // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
    // Define the Proportional control coefficient (or GAIN) for "heading control".
    // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
    // Increase these numbers if the heading does not corrects strongly enough (eg: a heavy robot or using tracks)
    // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
    static final double P_TURN_GAIN = 0.02;     // Larger is more responsive, but also less stable
    static final double P_DRIVE_GAIN = 0.03;     // Larger is more responsive, but also less stable

    private AprilTagsDetector tagsDetector;
    private AprilTagsDetector.Tag foundTag;

    ElapsedTime timer;

    enum ClawState {
        ACTIVE,
        NOT_ACTIVE
    }

    enum LiftMovementDirection {
        UP,
        DOWN
    }

    @Override
    public void runOpMode() {
        tagsDetector = new AprilTagsDetector(hardwareMap, telemetry);
        timer = new ElapsedTime();

        // Initialize the drive system variables.
        leftDrive = hardwareMap.get(DcMotor.class, "left");
        rightDrive = hardwareMap.get(DcMotor.class, "right");
        lift = hardwareMap.get(DcMotor.class, "lift");
        claw = hardwareMap.get(Servo.class, "claw");
        touch = hardwareMap.get(TouchSensor.class, "touch");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        claw.setDirection(Servo.Direction.REVERSE);

        // define initialization values for IMU, and then initialize it.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Ensure the robot is stationary.  Reset the encoders and set the motors to BRAKE mode
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        foundTag = AprilTagsDetector.Tag.noTag;

        // Wait for the game to start (Display Gyro value while waiting)
        while (opModeInInit()) {
            handleClawBooting(ClawState.ACTIVE);
            AprilTagsDetector.Tag tempTag = tagsDetector.getTag();
            if (tempTag != AprilTagsDetector.Tag.noTag) {
                foundTag = tempTag;
            } else {
                foundTag = AprilTagsDetector.Tag.noTag;
            }
            telemetry.addData(">", "Robot Heading = %4.0f", getRawHeading());
            telemetry.update();
        }

        // Set the encoders for closed loop speed control, and reset the heading.
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        resetHeading();

        // Step through each leg of the path,
        // Notes:   Reverse movement is obtained by setting a negative distance (not speed)
        //          holdHeading() is used after turns to let the heading stabilize
        //          Add a sleep(2000) after any step to keep the telemetry data visible for review

        // drive to junction

//        driveStraight(DRIVE_SPEED, 30, 0);
//        turnToHeading(TURN_SPEED, 45);
//        holdHeading(TURN_SPEED, 45, 0.5);
//
//        handleLiftMovement(1.0);
//
//        ElapsedTime timer = new ElapsedTime();
//        while (timer.time(TimeUnit.MILLISECONDS) < 2500 && opModeIsActive())
//        {
//            if (lift.getCurrentPosition() >= MAX_LIFT_HEIGHT_ENCODER)
//                handleLiftMovement(0);
//        }
//
//        handleLiftMovement(0);
//
//        driveStraight(DRIVE_SPEED, 10, 45);
//        holdHeading(TURN_SPEED, 45, 1.5);
//
//        handleLiftMovement(-0.5);
//
//        timer.reset();
//        while (timer.time(TimeUnit.MILLISECONDS) < 1000 && opModeIsActive())
//        {
//        }
////        holdHeading(TURN_SPEED, 45, 0.2);
//        handleClawBooting(ClawState.NOT_ACTIVE);
//        handleLiftMovement(0);
//
//        holdHeading(TURN_SPEED, 45, 0.2);
//
//        timer.reset();
//        handleLiftMovement(-1);
//        while (timer.time(TimeUnit.MILLISECONDS) < 1800 && opModeIsActive())
//        {
//        }
//
//        handleLiftMovement(0);
//
//        driveStraight(DRIVE_SPEED, -10, 45);
//        turnToHeading(TURN_SPEED, 0);
//        holdHeading(TURN_SPEED, 0, 0.5);

        if (foundTag == AprilTagsDetector.Tag.left) {
            leftJunctionAndParking();
//            minLeft();
        } else if (foundTag == AprilTagsDetector.Tag.right) {
            rightJunctionAndParking();
//            minRight();
        } else {
            midJunctionAndParking();
//            minMid();
        }
    }

    private void rightJunctionAndParking() {
        driveStraight(DRIVE_SPEED, 30, 0);
        holdHeading(DRIVE_SPEED, 0, 0.5);
        turnToHeading(TURN_SPEED, -90);
        holdHeading(TURN_SPEED, -90, 0.5);
        driveStraight(DRIVE_SPEED, 20, -90);
    }

    private void midJunctionAndParking() {
        driveStraight(DRIVE_SPEED, 35, 0);
    }

    private void leftJunctionAndParking() {
        driveStraight(DRIVE_SPEED, 30, 0);
        turnToHeading(TURN_SPEED, 90);
        holdHeading(DRIVE_SPEED, 90, 0.3);
        driveStraight(DRIVE_SPEED, 26, 90);
        turnToHeading(TURN_SPEED, 0);
        driveStraight(DRIVE_SPEED, 15, 0);
    }

    private void minRight() {
        turnToHeading(TURN_SPEED, -90);
        holdHeading(TURN_SPEED, -90, 0.5);
        driveStraight(DRIVE_SPEED, 20, -90);
    }

    private void minMid() {
    }

    private void minLeft() {
        turnToHeading(TURN_SPEED, 90);
        holdHeading(DRIVE_SPEED, 90, 0.3);
        driveStraight(DRIVE_SPEED, 24, 90);
        turnToHeading(TURN_SPEED, 0);
//        driveStraight(DRIVE_SPEED, 15, 0);
    }

    private void handleLiftMovement(double speed) {
        lift.setPower(speed);
//        if (speed > 0) {
//            lift.setPower(speed);
//
//            while (opModeIsActive() && lift.getCurrentPosition() < MAX_LIFT_HEIGHT_ENCODER) {
//            }
//
//            lift.setPower(0);
//        } else {
//            lift.setPower(speed);
//            while (opModeIsActive() && !touch.isPressed()) {
//                lift.setPower(-0.8);
//            }
//            lift.setPower(0);
//        }
    }

    private void handleClawBooting(ClawState clawState) {
        if (clawState == ClawState.ACTIVE)
            claw.setPosition(0.25);
        else if (clawState == ClawState.NOT_ACTIVE)
            claw.setPosition(0);
    }

    /*
     * ====================================================================================================
     * Driving "Helper" functions are below this line.
     * These provide the high and low level methods that handle driving straight and turning.
     * ====================================================================================================
     */

    // **********  HIGH Level driving functions.  ********************

    /**
     * Method to drive in a straight line, on a fixed compass heading (angle), based on encoder counts.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the desired position
     * 2) Driver stops the opmode running.
     *
     * @param maxDriveSpeed MAX Speed for forward/rev motion (range 0 to +1.0) .
     * @param distance      Distance (in inches) to move from current position.  Negative distance means move backward.
     * @param heading       Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                      0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                      If a relative angle is required, add/subtract from the current robotHeading.
     */
    public void driveStraight(double maxDriveSpeed,
                              double distance,
                              double heading) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            int moveCounts = (int) (distance * COUNTS_PER_INCH);
            leftTarget = leftDrive.getCurrentPosition() + moveCounts;
            rightTarget = rightDrive.getCurrentPosition() + moveCounts;

            // Set Target FIRST, then turn on RUN_TO_POSITION
            leftDrive.setTargetPosition(leftTarget);
            rightDrive.setTargetPosition(rightTarget);

            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            moveRobot(maxDriveSpeed, 0);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (leftDrive.isBusy() && rightDrive.isBusy())) {

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    turnSpeed *= -1.0;

                // Apply the turning correction to the current driving speed.
                moveRobot(driveSpeed, turnSpeed);

                // Display drive status for the driver.
                sendTelemetry(true);
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            moveRobot(0, 0);
            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     * Method to spin on central axis to point in a new direction.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the heading (angle)
     * 2) Driver stops the opmode running.
     *
     * @param maxTurnSpeed Desired MAX speed of turn. (range 0 to +1.0)
     * @param heading      Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                     0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                     If a relative angle is required, add/subtract from current heading.
     */
    public void turnToHeading(double maxTurnSpeed, double heading) {

        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, P_DRIVE_GAIN);

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(false);
        }

        // Stop all motion;
        moveRobot(0, 0);
    }

    /**
     * Method to obtain & hold a heading for a finite amount of time
     * Move will stop once the requested time has elapsed
     * This function is useful for giving the robot a moment to stabilize it's heading between movements.
     *
     * @param maxTurnSpeed Maximum differential turn speed (range 0 to +1.0)
     * @param heading      Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                     0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                     If a relative angle is required, add/subtract from current heading.
     * @param holdTime     Length of time (in seconds) to hold the specified heading.
     */
    public void holdHeading(double maxTurnSpeed, double heading, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        // keep looping while we have time remaining.
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(false);
        }

        // Stop all motion;
        moveRobot(0, 0);
    }

    // **********  LOW Level driving functions.  ********************

    /**
     * This method uses a Proportional Controller to determine how much steering correction is required.
     *
     * @param desiredHeading   The desired absolute heading (relative to last heading reset)
     * @param proportionalGain Gain factor applied to heading error to obtain turning power.
     * @return Turning power needed to get to required heading.
     */
    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry

        // Get the robot heading by applying an offset to the IMU heading
        robotHeading = getRawHeading() - headingOffset;

        // Determine the heading current error
        headingError = targetHeading - robotHeading;

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180) headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    /**
     * This method takes separate drive (fwd/rev) and turn (right/left) requests,
     * combines them, and applies the appropriate speed commands to the left and right wheel motors.
     *
     * @param drive forward motor speed
     * @param turn  clockwise turning motor speed.
     */
    public void moveRobot(double drive, double turn) {
        driveSpeed = drive;     // save this value as a class member so it can be used by telemetry.
        turnSpeed = turn;      // save this value as a class member so it can be used by telemetry.

        leftSpeed = drive - turn;
        rightSpeed = drive + turn;

        // Scale speeds down if either one exceeds +/- 1.0;
        double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (max > 1.0) {
            leftSpeed /= max;
            rightSpeed /= max;
        }

        leftDrive.setPower(leftSpeed);
        rightDrive.setPower(rightSpeed);
    }

    /**
     * Display the various control parameters while driving
     *
     * @param straight Set to true if we are driving straight, and the encoder positions should be included in the telemetry.
     */
    private void sendTelemetry(boolean straight) {

        if (straight) {
            telemetry.addData("Motion", "Drive Straight");
            telemetry.addData("Target Pos L:R", "%7d:%7d", leftTarget, rightTarget);
            telemetry.addData("Actual Pos L:R", "%7d:%7d", leftDrive.getCurrentPosition(),
                    rightDrive.getCurrentPosition());
        } else {
            telemetry.addData("Motion", "Turning");
        }

        telemetry.addData("Angle Target:Current", "%5.2f:%5.0f", targetHeading, robotHeading);
        telemetry.addData("Error:Steer", "%5.1f:%5.1f", headingError, turnSpeed);
        telemetry.addData("Wheel Speeds L:R.", "%5.2f : %5.2f", leftSpeed, rightSpeed);
        telemetry.update();
    }

    /**
     * read the raw (un-offset Gyro heading) directly from the IMU
     */
    public double getRawHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    /**
     * Reset the "offset" heading back to zero
     */
    public void resetHeading() {
        // Save a new heading offset equal to the current raw heading.
        headingOffset = getRawHeading();
        robotHeading = 0;
    }
}
