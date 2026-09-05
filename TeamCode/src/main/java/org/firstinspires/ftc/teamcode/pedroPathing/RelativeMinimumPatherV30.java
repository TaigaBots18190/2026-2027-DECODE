package org.firstinspires.ftc.teamcode.pedroPathing;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator; // May need to use, idk yet
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="ClaudeVersion")
public class RelativeMinimumPatherV30 extends LinearOpMode {
    private Follower follower;
    private Limelight3A limelight;
    private DcMotor intake;
    private boolean intakeOn = false;

    private final List<PollenDetection> pollenList = new ArrayList<>();
    private final List<Coordinates> coords = new ArrayList<>();
    private final List<Double> projections = new ArrayList<>();
    private final List<Coordinates> points = new ArrayList<>();
    private final double mountingAngleDeg = 10; // Alpha
    private final double mountingHeight = 7; // H
    private final double limelightXOffset = 0;
    private final double limelightYOffset = 0;
    private final double intakeWidth = 8;

    public void runOpMode() {

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(8.78, 7.62, Math.toRadians(90)));

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(10);
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(3); // Make sure we configure our neural detector pipeline on 1
        limelight.start();

        if (intakeOn) {
            intake = hardwareMap.get(DcMotor.class, "intake");
        }

        boolean automatedDrive = false;
        boolean detection1 = false;

        // Non-blocking scan state -- replaces the old busy-wait timer loop
        boolean scanning = false;
        ElapsedTime scanTimer = new ElapsedTime();
        double scanTheta = 0;
        double scanRobotX = 0;
        double scanRobotY = 0;

        int checkin = 0;

        waitForStart();

        follower.startTeleopDrive(false);

        while (opModeIsActive()) {

            follower.update();

            if (intakeOn) {
                intake.setPower(1);
            }

            if (!automatedDrive) {
                follower.setTeleOpDrive(gamepad1.left_stick_y,
                        gamepad1.left_stick_x,
                        -gamepad1.right_stick_x,
                        false);
            }

            if (automatedDrive && ((gamepad1.bWasPressed() || !follower.isBusy()))) {
                follower.startTeleopDrive(false);
                detection1 = false;
                automatedDrive = false;
            }

            // Start a new scan when A is pressed and nothing else is already in progress
            if (gamepad1.aWasPressed() && checkin == 0) {
                checkin = 1;
                scanning = true;
                detection1 = false;
                pollenList.clear();
                scanTimer.reset();
                scanTheta = Math.toDegrees(follower.getHeading());
                scanRobotX = follower.getPose().getX();
                scanRobotY = follower.getPose().getY();
            }

            // Poll once per loop pass while a scan is active -- follower.update() and
            // driver control keep running every iteration, nothing freezes.
            if (scanning) {

                LLResult result = limelight.getLatestResult();
                boolean sawPollen = (result != null && result.isValid()
                        && !result.getDetectorResults().isEmpty());

                if (sawPollen) {
                    for (LLResultTypes.DetectorResult detection : result.getDetectorResults()) {
                        detection1 = true;
                        pollenList.add(
                                new PollenDetection(
                                        detection.getTargetXDegrees(),
                                        detection.getTargetYDegrees(),
                                        detection.getTargetArea()
                                )
                        );
                    }
                    scanning = false;
                } else if (scanTimer.milliseconds() >= 300) {
                    // Timed out without ever seeing pollen -- move on with an empty list
                    scanning = false;
                }

                if (!scanning) {

                    points.clear();
                    coords.clear();
                    projections.clear();

                    double meanX = 0;
                    double meanY = 0;
                    double sxx = 0;
                    double syy = 0;
                    double sxy = 0;
                    int iteration = 1;

                    // Determine the coordinates of each ball detected
                    for (PollenDetection pollen : pollenList) {

                        double tx = pollen.tx;
                        double ty = pollen.ty; // Might have to swap unary operator because the formula assumes when the target is above the crosshair, ty is positive

                        double d1 = mountingHeight / (Math.tan(Math.toRadians(mountingAngleDeg - ty)));
                        double d2 = d1 * Math.tan(Math.toRadians(tx));

                        double placeholder1 = Math.toRadians(scanTheta - (Math.toDegrees(Math.atan(d2 / d1))));
                        double sqrt = Math.sqrt(d1 * d1 + d2 * d2);

                        double x1 = Math.cos(placeholder1) * sqrt;
                        double y1 = Math.sin(placeholder1) * sqrt;

                        double heading = Math.toRadians(scanTheta);

                        double offsetX =
                                limelightXOffset * Math.cos(heading)
                                        - limelightYOffset * Math.sin(heading);

                        double offsetY =
                                limelightXOffset * Math.sin(heading)
                                        + limelightYOffset * Math.cos(heading);

                        coords.add(new Coordinates(scanRobotX + x1 + offsetX, scanRobotY + y1 + offsetY));
                    }

                    if (!coords.isEmpty()) {

                        for (Coordinates c : coords) {
                            meanX += c.x;
                            meanY += c.y;
                        }
                        meanX /= coords.size();
                        meanY /= coords.size();

                        for (Coordinates c : coords) {
                            double dx = c.x - meanX;
                            double dy = c.y - meanY;
                            sxx += dx * dx;
                            syy += dy * dy;
                            sxy += dx * dy;
                        }

                        double pcaAngle = 0.5 * Math.atan2(2 * sxy, sxx - syy);
                        double ux = Math.cos(pcaAngle);
                        double uy = Math.sin(pcaAngle);

                        double finalMeanY = meanY;
                        double finalMeanX = meanX;

                        coords.sort((a, b) -> {
                            double projA = (a.x - finalMeanX) * ux + (a.y - finalMeanY) * uy;
                            double projB = (b.x - finalMeanX) * ux + (b.y - finalMeanY) * uy;
                            return Double.compare(projA, projB);
                        });

                        double robotX = follower.getPose().getX();
                        double robotY = follower.getPose().getY();

                        Coordinates firstBall = coords.get(0);
                        Coordinates lastBall = coords.get(coords.size() - 1);
                        double distToFirst = Math.hypot(robotX - firstBall.x, robotY - firstBall.y);
                        double distToLast = Math.hypot(robotX - lastBall.x, robotY - lastBall.y);
                        if (distToLast < distToFirst) {
                            Collections.reverse(coords);
                        }

                        points.add(new Coordinates(follower.getPose().getX(), follower.getPose().getY()));

                        for (Coordinates c : coords) {
                            double projection = (c.x - meanX) * ux + (c.y - meanY) * uy;
                            projections.add(projection);
                            double projX = meanX + (projection) * ux;
                            double projY = meanY + (projection) * uy;
                            double dx = c.x - projX;
                            double dy = c.y - projY;
                            double ctrlXpoint = 0, ctrlYpoint = 0;
                            double perpendicularDistance = Math.sqrt(dx * dx + dy * dy);
                            if (perpendicularDistance > (intakeWidth / 2 - 2)) {
                                double excess = perpendicularDistance - (intakeWidth / 2 - 2);
                                double side = -(c.x - meanX) * uy + (c.y - meanY) * ux;
                                if (side < 0) {
                                    ctrlXpoint = projX + uy * excess;
                                    ctrlYpoint = projY - (ux * excess);
                                } else if (side > 0) {
                                    ctrlXpoint = projX - uy * excess;
                                    ctrlYpoint = projY + (ux * excess);
                                }
                                points.add(new Coordinates(ctrlXpoint, ctrlYpoint));
                            } else {
                                points.add(new Coordinates(projX, projY));
                            }
                        }

                        double endpointX = meanX + (Collections.max(projections) + 2) * ux;
                        double endpointY = meanY + (Collections.max(projections) + 2) * uy;
                        points.add(new Coordinates(endpointX, endpointY));

                        PathBuilder builder = follower.pathBuilder();

                        while (iteration < points.size()) {
                            if (iteration < points.size() - 1) {
                                Coordinates previous = new Coordinates(points.get(iteration - 1).x, points.get(iteration - 1).y);
                                Coordinates post = new Coordinates(points.get(iteration + 1).x, points.get(iteration + 1).y);
                                Coordinates ctrl = Coordinates.midpointControlPoint(previous, post, new Coordinates(points.get(iteration).x, points.get(iteration).y));
                                builder.addPath(new BezierCurve(new Pose(previous.x, previous.y), new Pose(ctrl.x, ctrl.y), new Pose(post.x, post.y)));
                            }
                            if (iteration == points.size() - 3) {
                                Coordinates previous = points.get(iteration + 1);
                                Coordinates post = points.get(iteration + 2);
                                builder.addPath(new BezierLine(new Pose(previous.x, previous.y), new Pose(post.x, post.y)));
                            }
                            iteration += 2;
                        }

                        builder.setTangentHeadingInterpolation();
                        PathChain path = builder.build();
                        follower.followPath(path);
                        automatedDrive = true;
                    }

                    checkin = 0;
                }
            }

            if (detection1) {
                telemetry.addLine("detected");
            } else {
                telemetry.addLine("not detected");
            }
            telemetry.update();
        }
    }
}
//