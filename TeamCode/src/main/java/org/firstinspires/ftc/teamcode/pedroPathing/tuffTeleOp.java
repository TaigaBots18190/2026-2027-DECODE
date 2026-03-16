package org.firstinspires.ftc.teamcode.pedroPathing;


import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
@TeleOp
public class tuffTeleOp extends LinearOpMode {


    double slideEncoderValue;
    double tuneSpeedManually = 0.6;


    int linearState = 0;


    final double FEED_TIME_SECONDS = 0.20; //The feeder servos run this long when a shot is requested.
    final double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
    final double FULL_SPEED = -1.0;


    /*
     * When we control our launcher motor, we are using encoders. These allow the control system
     * to read the current speed of the motor and apply more or less power to keep it at a constant
     * velocity. Here we are setting the target, and minimum velocity that the launcher should run
     * at. The minimum velocity is a threshold for determining when to fire.
     */
    final double LAUNCHER_TARGET_VELOCITY = 1125;
    final double LAUNCHER_MIN_VELOCITY = 1075;
    private DcMotorEx launcher = null;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;


    ElapsedTime feederTimer = new ElapsedTime();


    private enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
    }


    private LaunchState launchState;


    // Setup a variable for each drive wheel to save power level for telemetry




    void launch(boolean shotRequested) {
        switch (launchState) {
            case IDLE:
                if (shotRequested) {
                    launchState = LaunchState.SPIN_UP;
                }
                break;
            case SPIN_UP:
                launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
                if (launcher.getVelocity() > LAUNCHER_MIN_VELOCITY) {
                    launchState = LaunchState.LAUNCH;
                }
                break;
            case LAUNCH:
                leftFeeder.setPower(FULL_SPEED);
                rightFeeder.setPower(FULL_SPEED);
                feederTimer.reset();
                launchState = LaunchState.LAUNCHING;
                break;
            case LAUNCHING:
                if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                    launchState = LaunchState.IDLE;
                    leftFeeder.setPower(STOP_SPEED);
                    rightFeeder.setPower(STOP_SPEED);
                }
                break;
        }
    }




    @Override
    public void runOpMode() throws InterruptedException {

        launchState = LaunchState.IDLE;

        ElapsedTime timeSinceAPressed = new ElapsedTime();
        ElapsedTime timeSinceBPressed = new ElapsedTime();

        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "flm");//Port 1 Control Hub//
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "frm");//Port 2 Control Hub//
        DcMotor backLeft = hardwareMap.get(DcMotor.class, "blm");//Port 4 Control Hub//
        DcMotor backRight = hardwareMap.get(DcMotor.class, "brm");//Port 3 Control Hub//
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");//Port 0 Expansion Hub//
        leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");//Port 0 Control Hub//
        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");//Port 1 Expansion Hub//

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        launcher.setZeroPowerBehavior(BRAKE);

        leftFeeder.setPower(STOP_SPEED);
        rightFeeder.setPower(STOP_SPEED);

        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));
        leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);
        telemetry.addData("Status", "Initialized");

        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        waitForStart();


        if (isStopRequested()) return;


        while (opModeIsActive()) {

            // Drive input
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            if (gamepad1.y) {
                launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
            } else if (gamepad1.b) { // stop flywheel
                launcher.setVelocity(STOP_SPEED);
            }


            /*
             * Now we call our "Launch" function.
             */
            launch(gamepad1.right_bumper);

            if (gamepad1.a && timeSinceAPressed.milliseconds() > 500) {
                tuneSpeedManually = Math.min(1, tuneSpeedManually + 0.1);
                timeSinceAPressed.reset();
            }

            if (gamepad1.x && timeSinceBPressed.milliseconds() > 500) {
                tuneSpeedManually = Math.max(0, tuneSpeedManually - 0.1);
                timeSinceBPressed.reset();
            }

            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            rotX = rotX * 1.1;

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            frontLeft.setPower(frontLeftPower * tuneSpeedManually);
            backLeft.setPower(backLeftPower * tuneSpeedManually);
            frontRight.setPower(frontRightPower * tuneSpeedManually);
            backRight.setPower(backRightPower * tuneSpeedManually);
        }
    }
}

