package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;
import android.util.Size;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class GPPRed extends LinearOpMode {

    private Limelight3A limelight;
    boolean detection = false;
    // Variable Initialization
    private DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor, turret;
    private DcMotorEx shooter1;
    private Servo hoodExtension, indexer, hinge;

    private DcMotor intake;
    private double pos1Intake = 0.0486; // .9
    private double pos2Intake = 0.2567; // .7499
    private double pos3Intake = 0.4561; // .5404
    private double pos1Shoot = 0.3559; // .6444
    private double pos2Shoot = 0.5643; // .4381
    private double pos3Shoot = 0.7634; // .2331
    private double TurretPosition = 0; // may need to change
    private int turretExtremeLeft = 1500; // may need to change
    private int turretExtremeRight = 0; // may need to change
    private String motif = "GPP";
    private String pattern = "";
    private Boolean goingLeft = true;
    private boolean track = false;
    private double kP = 5;
    private boolean resetPattern = false;
    private static boolean shooting = false;
    private boolean shooting2 = false;
    private int indexerState = 0;
    private int iteration = 0;
    private boolean stopShooting = true;
    private boolean shoot = false;
    public int comparison = 0;
    boolean centered = false;
    boolean centerControl = false;
    boolean turret123 = false;

    // Elapsed Times
    ElapsedTime rightTriggerDuration = new ElapsedTime();
    ElapsedTime rightBumperDuration = new ElapsedTime();
    ElapsedTime leftTriggerDuration = new ElapsedTime();
    ElapsedTime indexerTime = new ElapsedTime();
    ElapsedTime hingeTime = new ElapsedTime();
    ElapsedTime indexerTime2 = new ElapsedTime();
    ElapsedTime hingeTime2 = new ElapsedTime();
    ElapsedTime turretInterval = new ElapsedTime();

    ElapsedTime xTime = new ElapsedTime();
    ElapsedTime bTime = new ElapsedTime();
    ElapsedTime yTime = new ElapsedTime();
    ElapsedTime colorTime = new ElapsedTime();
    ElapsedTime leftTrigger = new ElapsedTime();

    public static int count(String str, Character targetChar) {
        int iter = 0;
        for (int i = 0; i < str.length(); i++) {
            if (str.charAt(i) == targetChar) {
                iter++;
            }
        }
        return iter;
    }
    public void runIntake(boolean bool) {
        if (bool) {
            intake.setPower(1); // May need to change direction
        } else {
            intake.setPower(0);
        }

    }
    public void turretTracker(boolean track) {
        if (!track) return;

        LLResult result1 = limelight.getLatestResult();

        double kP = 1.5;          // tune this
        double deadband = 1.0;    // degrees
        int maxStep = 40;         // encoder ticks per loop

        if (result1 != null && result1.isValid()) {

            double error = result1.getTx();

            if (Math.abs(error) < deadband) {
                error = 0;
            }

            double delta = kP * error;
            delta = Math.max(-maxStep, Math.min(delta, maxStep));

            TurretPosition = turret.getCurrentPosition() + delta;

            telemetry.addData("Turret", "Tracking");
        } else {

            // ONLY scan when no target
            if (turretInterval.milliseconds() > 100) {
                TurretPosition += goingLeft ? 30 : -30;

                if (TurretPosition >= turretExtremeLeft) goingLeft = false;
                if (TurretPosition <= turretExtremeRight) goingLeft = true;
            }

            telemetry.addData("Turret", "Scanning");
        }

        TurretPosition = Math.max(turretExtremeRight,
                Math.min(TurretPosition, turretExtremeLeft));

        turret.setTargetPosition((int) TurretPosition);
    }



    public void automated_shoot(boolean launch) {
        char green = 'G';
        char purple = 'P';
        if (launch) {
            shooting = true;
            stopShooting = false;
            shooter1.setVelocity(1500); // May have to change direction
            if (count(pattern, green) == 1 && count(pattern, purple) == 2 && !motif.isEmpty()) {
                int motifDetect = motif.indexOf(green);
                int patternDetect = pattern.indexOf(green);
                if (motifDetect == patternDetect) {
                    if (iteration == 0) {
                        if (!centerControl) {
                            indexer.setPosition(pos1Shoot);
                            if (indexerTime.milliseconds() > 750) {
                                hinge.setPosition(0.4);
                                if (hingeTime.milliseconds() > 1250) {
                                    hinge.setPosition(0);
                                    iteration += 1;
                                    hingeTime.reset();
                                    indexerTime.reset();
                                }
                            }
                        } else {
                            hinge.setPosition(0.4);
                            if (hingeTime.milliseconds() > 500) {
                                hinge.setPosition(0);
                                iteration += 1;
                                hingeTime.reset();
                                indexerTime.reset();
                            }
                        }

                    } else if (iteration == 1) {
                        indexer.setPosition(pos2Shoot);
                        if (indexerTime.milliseconds() > 1000) {
                            hinge.setPosition(0.4);
                            if (hingeTime.milliseconds() > 1500) {
                                hinge.setPosition(0);
                                iteration += 1;
                                hingeTime.reset();
                                indexerTime.reset();
                            }
                        }
                    } else if (iteration == 2) {
                        indexer.setPosition(pos3Shoot);
                        if (indexerTime.milliseconds() > 1000) {
                            hinge.setPosition(0.4);
                            if (hingeTime.milliseconds() > 1500) {
                                hinge.setPosition(0);
                                iteration += 1;
                                hingeTime.reset();
                                indexerTime.reset();
                                stopShooting = true;
                            }
                        }
                    }
                } else if (motifDetect == (patternDetect+1)%3) { // may have to change condition
                    if (iteration == 0) {
                        if (!centerControl) {
                            indexer.setPosition(pos3Shoot);
                            if (indexerTime.milliseconds() > 750) {
                                hinge.setPosition(0.4);
                                if (hingeTime.milliseconds() > 1250) {
                                    hinge.setPosition(0);
                                    iteration += 1;
                                    hingeTime.reset();
                                    indexerTime.reset();
                                }
                            }
                        } else {
                            hinge.setPosition(0.4);
                            if (hingeTime.milliseconds() > 500) {
                                hinge.setPosition(0);
                                iteration += 1;
                                hingeTime.reset();
                                indexerTime.reset();
                            }
                        }
                    } else if (iteration == 1) {
                        indexer.setPosition(pos1Shoot);
                        if (indexerTime.milliseconds() > 1000) {
                            hinge.setPosition(0.4);
                            if (hingeTime.milliseconds() > 1500) {
                                hinge.setPosition(0);
                                iteration += 1;
                                hingeTime.reset();
                                indexerTime.reset();
                            }
                        }
                    } else if (iteration == 2) {
                        indexer.setPosition(pos2Shoot);
                        if (indexerTime.milliseconds() > 1000) {
                            hinge.setPosition(0.4);
                            if (hingeTime.milliseconds() > 1500) {
                                hinge.setPosition(0);
                                iteration += 1;
                                hingeTime.reset();
                                indexerTime.reset();
                                stopShooting = true;
                            }
                        }
                    }
                } else {
                    if (iteration == 0) {
                        if (!centerControl) {
                            indexer.setPosition(pos2Shoot);
                            if (indexerTime.milliseconds() > 750) {
                                hinge.setPosition(0.4);
                                if (hingeTime.milliseconds() > 1250) {
                                    hinge.setPosition(0);
                                    iteration += 1;
                                    hingeTime.reset();
                                    indexerTime.reset();
                                }
                            }
                        } else {
                            hinge.setPosition(0.4);
                            if (hingeTime.milliseconds() > 500) {
                                hinge.setPosition(0);
                                iteration += 1;
                                hingeTime.reset();
                                indexerTime.reset();
                            }
                        }
                    } else if (iteration == 1) {
                        indexer.setPosition(pos3Shoot);
                        if (indexerTime.milliseconds() > 1000) {
                            hinge.setPosition(0.4);
                            if (hingeTime.milliseconds() > 1500) {
                                hinge.setPosition(0);
                                iteration += 1;
                                hingeTime.reset();
                                indexerTime.reset();
                            }
                        }
                    } else if (iteration == 2) {
                        indexer.setPosition(pos1Shoot);
                        if (indexerTime.milliseconds() > 1000) {
                            hinge.setPosition(0.4);
                            if (hingeTime.milliseconds() > 1500) {
                                hinge.setPosition(0);
                                iteration += 1;
                                hingeTime.reset();
                                indexerTime.reset();
                                stopShooting = true;
                            }
                        }
                    }
                }

                if (stopShooting) {
                    iteration = 0;
                    indexerState = 0;
                    resetPattern = true;
                    shooting = false;
                }

            } else {
                if (iteration == 0) {
                    if (!centerControl) {
                        indexer.setPosition(pos1Shoot);
                        if (indexerTime.milliseconds() > 750) {
                            hinge.setPosition(0.4);
                            if (hingeTime.milliseconds() > 1050) {
                                hinge.setPosition(0);
                                iteration += 1;
                                hingeTime.reset();
                                indexerTime.reset();
                            }
                        }
                    } else {
                        hinge.setPosition(0.4);
                        if (hingeTime.milliseconds() > 300) {
                            hinge.setPosition(0);
                            iteration += 1;
                            hingeTime.reset();
                            indexerTime.reset();
                        }

                    }
                }
                if (iteration == 1) {
                    indexer.setPosition(pos2Shoot);
                    if (indexerTime.milliseconds() > 750) {
                        hinge.setPosition(0.4);
                        if (hingeTime.milliseconds() > 1050) {
                            hinge.setPosition(0);
                            iteration += 1;
                            hingeTime.reset();
                            indexerTime.reset();
                        }
                    }
                }
                if (iteration == 2) {
                    indexer.setPosition(pos3Shoot);
                    if (indexerTime.milliseconds() > 750) {
                        hinge.setPosition(0.4);
                        if (hingeTime.milliseconds() > 1050) {
                            hinge.setPosition(0);
                            iteration += 1;
                            hingeTime.reset();
                            indexerTime.reset();
                            stopShooting = true;
                        }
                    }
                }
                if (stopShooting) {
                    iteration = 0;
                    indexerState = 0;
                    resetPattern = true;
                    shooting = false;
                    centerControl = false;
                }
            }
        } else {
            iteration = 0;
            stopShooting = true;
            resetPattern = false;
            hingeTime.reset();
            indexerTime.reset();
        }
        telemetry.addData("Iteration:", iteration);
    }

    public void runOpMode() {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(10);
        limelight.pipelineSwitch(2);
        limelight.start();


        PredominantColorProcessor colorSensor = new PredominantColorProcessor.Builder()
                .setRoi(ImageRegion.asUnityCenterCoordinates(0.2, -0.1, 0.3, -0.2)) // may have to change
                .setSwatches(
                        PredominantColorProcessor.Swatch.ARTIFACT_GREEN,
                        PredominantColorProcessor.Swatch.ARTIFACT_PURPLE,
                        PredominantColorProcessor.Swatch.BLACK,
                        PredominantColorProcessor.Swatch.WHITE,
                        PredominantColorProcessor.Swatch.YELLOW)
                .build();

        VisionPortal portal = new VisionPortal.Builder()
                .addProcessor(colorSensor)
                .setCameraResolution(new Size(320, 240))
                .setCamera(hardwareMap.get(WebcamName.class, "logi"))
                .build();


        frontLeftMotor = hardwareMap.get(DcMotor.class, "flm");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frm");
        backLeftMotor = hardwareMap.get(DcMotor.class, "blm");
        backRightMotor = hardwareMap.get(DcMotor.class, "brm");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        turret = hardwareMap.get(DcMotor.class, "turret");
        turret.setDirection(DcMotorSimple.Direction.REVERSE);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setTargetPosition(0);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPower(0.6);

        shooter1 = hardwareMap.get(DcMotorEx.class, "shoot1");
        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(160, 0, 0, 15));
        hoodExtension = hardwareMap.get(Servo.class, "s1");


        indexer = hardwareMap.get(Servo.class, "index");

        hinge = hardwareMap.get(Servo.class, "h");

        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);

        boolean intakeToggle = false;
        int loop = 0;
        int position = 0;
        boolean condition = false;
        int previousPosition = position;
        boolean positionControl = true;
        boolean ballSeen = false;
        boolean swit = false;
        boolean ishaan = false;

        String manual_shoot = "";
        /*
        Hood:
            2 Motors shooter wheel
            1 Servo that controls hood extension
            Limelight camera attached on top of the hooded shooter
        Turret:
            2 Standard Servos to control the turret
        Indexer:
            Controlled by a Standard Servo
            Regular camera with color detection ability
        Hinge:
            Standard Servo
        Intake:
            1 continuous rotation melonbotics super servo
        Drivetrain:
            4 motors
        */

        boolean hn = false;

        hinge.setPosition(0);
        hoodExtension.setPosition(0);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            char green = 'G';
            char purple = 'P';

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            LLResult result1 = limelight.getLatestResult();

            if (result1 != null && result1.isValid()) {
                if (Math.abs(result1.getTx()) < 1) {
                    gamepad1.rumble(100);
                    gamepad2.rumble(100);
                }
            }



            if ((gamepad1.yWasPressed() || gamepad2.yWasPressed())) {
                swit = !swit;
            }

            if (swit) {
                shooter1.setVelocity(1500);
                telemetry.addLine("High Vel");
            } else {
                shooter1.setVelocity(1080);
                telemetry.addLine("Low Vel");
            }


            if (resetPattern) {
                pattern = "";
                resetPattern = false;
                centerControl = false;
            }

            if ((gamepad1.aWasPressed() || gamepad2.aWasPressed()) && count(pattern, green) >= 1) {
                manual_shoot += "G";
            }
            if ((gamepad1.xWasPressed() || gamepad2.xWasPressed()) && count(pattern, purple) >= 1) {
                manual_shoot += "P";
            }
            //if (manual_shoot.isEmpty()) {
            //    indexerTime.reset();
            //    hingeTime.reset();
            //}


            if (!manual_shoot.isEmpty()) {
                shooting2 = true;
                if (positionControl) {
                    position = pattern.indexOf(manual_shoot.charAt(0));
                    centerControl = true;
                    positionControl = false;
                }
                if (position == 0) {
                    indexer.setPosition(pos1Shoot);
                    if (indexerTime2.milliseconds() > 1000) {
                        hinge.setPosition(1);
                        if (hingeTime2.milliseconds() > 1500) {
                            hinge.setPosition(0.65);
                            hingeTime2.reset();
                            indexerTime2.reset();
                            condition = true;
                        }
                    }
                } else if (position == 1) {
                    indexer.setPosition(pos2Shoot);
                    if (indexerTime2.milliseconds() > 1000) {
                        hinge.setPosition(1);
                        if (hingeTime2.milliseconds() > 1500) {
                            hinge.setPosition(0.65);
                            hingeTime2.reset();
                            indexerTime2.reset();
                            condition = true;
                        }
                    }
                } else if (position == 2) {
                    indexer.setPosition(pos3Shoot);
                    if (indexerTime2.milliseconds() > 1000) {
                        hinge.setPosition(1);
                        if (hingeTime2.milliseconds() > 1500) {
                            hinge.setPosition(0.65);
                            hingeTime2.reset();
                            indexerTime2.reset();
                            condition = true;
                        }
                    }
                }
                if (condition) {
                    condition = false;
                    positionControl = true;
                    shooting2 = false;
                    pattern = pattern.substring(0, position) + pattern.substring(position + 1);
                    manual_shoot = manual_shoot.substring(1);
                }
                if (manual_shoot.isEmpty()) {
                    centerControl = false;
                }
            } else {
                hingeTime2.reset();
                indexerTime2.reset();
            }


            if ((gamepad1.bWasPressed() || gamepad2.bWasPressed())) {
                if (!pattern.isEmpty()) {
                    pattern = pattern.substring(0, pattern.length()-1);
                }
            }
            if ((gamepad1.rightBumperWasPressed() || gamepad2.rightBumperWasPressed())) {
                shooting = true;
            }
            automated_shoot(shooting);
            if ((gamepad1.right_trigger > 0.5 || gamepad2.right_trigger > 0.5) && rightTriggerDuration.milliseconds() > 500) {
                intakeToggle = !intakeToggle;
                rightTriggerDuration.reset();
            }
            runIntake(intakeToggle);
            // Simple subset logic ends
            // Indexing Logic
            PredominantColorProcessor.Result result = colorSensor.getAnalysis();
            if (pattern.length() < 3 && !shooting && !shooting2 && !centerControl) {
                if (result.closestSwatch == PredominantColorProcessor.Swatch.ARTIFACT_GREEN && colorTime.milliseconds() > 1000) {
                    indexerState = (indexerState + 1) % 3;
                    pattern += "G";
                    colorTime.reset();
                } else if (result.closestSwatch == PredominantColorProcessor.Swatch.ARTIFACT_PURPLE && colorTime.milliseconds() > 1000) {
                    indexerState = (indexerState + 1) % 3;
                    pattern += "P";
                    colorTime.reset();
                }
            }

// Reset latch once ball leaves ROI

            if ((pattern.length() == 3 || ((gamepad1.left_trigger > 0.5 || gamepad2.left_trigger > 0.5) && leftTrigger.milliseconds() > 500)) && !centerControl) {
                if (count(pattern, green) == 1 && count(pattern, purple) == 2) {
                    int motifDetect = motif.indexOf(green);
                    int patternDetect = pattern.indexOf(green);
                    if (motifDetect == patternDetect) {
                        indexer.setPosition(pos1Shoot);
                    } else if (motifDetect == (patternDetect+1)%3) {
                        indexer.setPosition(pos3Shoot);
                    } else {
                        indexer.setPosition(pos2Shoot);
                    }
                } else {
                    indexer.setPosition(pos1Shoot);
                }
                centerControl = true;
                leftTrigger.reset();
            }

            if (!shooting && !shooting2 && !centerControl) {
                switch (indexerState) {
                    case 0:
                        indexer.setPosition(pos1Intake);
                        break;
                    case 1:
                        indexer.setPosition(pos2Intake);
                        break;
                    case 2:
                        indexer.setPosition(pos3Intake);
                        break;
                }
            }
            telemetry.addData("Pattern", pattern);
            telemetry.addData("Result:", result.closestSwatch);
            telemetry.addData("Manual-Shoot", manual_shoot);
            telemetry.addData("Turret Position", turret.getCurrentPosition());
            telemetry.update();
        }

    }
} // Have to check code again