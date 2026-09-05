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


@TeleOp(name="ArtificialPather")
public class RelativeMinimumPatherV20 extends LinearOpMode{

    private Follower follower;
    private Limelight3A limelight;
    private DcMotor intake;
    private boolean intakeOn = false;

    private final List<PollenDetection> pollenList = new ArrayList<>();
    private final List<Coordinates> coords = new ArrayList<>();
    private final List<Double> projections = new ArrayList<>();
    private final List<Coordinates> points = new ArrayList<>();
    private final double mountingAngleDeg = 10; // Alpha
    private final double mountingHeight = 12.5; // H
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


        waitForStart();

        follower.startTeleopDrive(false);

        while (opModeIsActive()) {

            follower.update();

            if (intakeOn) {
                intake.setPower(1);
            }

            // Analysis starts here

            if (!automatedDrive) {
                follower.setTeleOpDrive(-gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x,
                        true);
            }

            if (automatedDrive && ((gamepad1.bWasPressed() || !follower.isBusy()))) {
                follower.startTeleopDrive(false);
                detection1 = false;
                automatedDrive = false;
            }



            if (gamepad1.aWasPressed()) {

                points.clear();
                pollenList.clear();
                coords.clear();
                projections.clear();

                double meanX = 0;
                double meanY = 0;

                double sxx = 0;
                double syy = 0;
                double sxy = 0;

                int iteration = 1;

                double theta = Math.toDegrees(follower.getHeading());
                // Detecting the pollen

                LLResult result = limelight.getLatestResult();

                if (result != null && result.isValid()) {

                    for (LLResultTypes.DetectorResult detection : result.getDetectorResults()) {

                        // Store the data of each detection in a list
                        detection1 = true;
                        pollenList.add(
                                new PollenDetection(
                                        detection.getTargetXDegrees(),
                                        detection.getTargetYDegrees(),
                                        detection.getTargetArea()
                                )
                        );
                    }
                }


                // Determine the coordinates of each ball detected

                for (PollenDetection pollen : pollenList) {

                    double tx = pollen.tx;
                    double ty = pollen.ty; // Might have to swap unary operator because the formula assumes when the target is above the crosshair, ty is positive

                    double d1 = mountingHeight/(Math.tan(Math.toRadians(mountingAngleDeg - ty)));
                    double d2 = d1*Math.tan(Math.toRadians(tx));

                    double placeholder1 = Math.toRadians(theta - (Math.toDegrees(Math.atan(d2 / d1))));
                    double sqrt = Math.sqrt(d1 * d1 + d2 * d2);

                    double x1 = Math.cos(placeholder1)* sqrt;
                    double y1 = Math.sin(placeholder1)* sqrt;

                    double heading = follower.getHeading();

                    double offsetX =
                            limelightXOffset * Math.cos(heading)
                                    - limelightYOffset * Math.sin(heading);

                    double offsetY =
                            limelightXOffset * Math.sin(heading)
                                    + limelightYOffset * Math.cos(heading);
                    // Add exceptions for grey areas
                    coords.add(new Coordinates(follower.getPose().getX()+x1+offsetX, follower.getPose().getY()+y1+offsetY));
                }

                // Add limits/clamps for points
                // Add state machine to switch from intake and outtake
                // Path Completion restriction (i.e. if the robot complese x% of the path and is close to the goal just switch to outtake)
                // Ultrasonic sensor to manuever around robots



                if (!coords.isEmpty()) {

                    // Principle Component Analysis (some weird ahh shi)

                    // Finding the centroid of the points (center/mean point)

                    for (Coordinates c : coords) {
                        meanX += c.x;
                        meanY += c.y;
                    }

                    meanX /= coords.size();
                    meanY /= coords.size();

                    // Calculating the variance and covariance between the x and y values

                    for (Coordinates c : coords) {

                        double dx = c.x - meanX;
                        double dy = c.y - meanY;

                        sxx += dx * dx;
                        syy += dy * dy;
                        sxy += dx * dy;

                    }

                    // Finding the so called eigenvector from the covariance matrix that contains the direction of the perpendicular regression plot

                    double pcaAngle =
                            0.5 * Math.atan2(
                                    2 * sxy,
                                    sxx - syy
                            );

                    // Converting the eigenvector to a direction vector (extrapolating the direction information)

                    double ux = Math.cos(pcaAngle);
                    double uy = Math.sin(pcaAngle);

                    double finalMeanY = meanY;
                    double finalMeanX = meanX;

                    coords.sort((a, b) -> {

                        double projA =
                                (a.x - finalMeanX) * ux
                                        + (a.y - finalMeanY) * uy;

                        double projB =
                                (b.x - finalMeanX) * ux
                                        + (b.y - finalMeanY) * uy;

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
                        double projection =
                                (c.x - meanX) * ux
                                        + (c.y - meanY) * uy;
                        projections.add(projection);
                        double projX = meanX + (projection) * ux;
                        double projY = meanY + (projection) * uy;
                        double dx = c.x - projX;
                        double dy = c.y - projY;
                        double ctrlXpoint, ctrlYpoint;
                        ctrlXpoint = 0;
                        ctrlYpoint = 0;
                        double perpendicularDistance =
                                Math.sqrt(dx * dx + dy * dy);
                        if (perpendicularDistance > (intakeWidth / 2 - 2)) {
                            double excess = perpendicularDistance - (intakeWidth / 2 - 2);
                            double side =
                                    -(c.x - meanX) * uy
                                            + (c.y - meanY) * ux;
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

                    double endpointX =
                            meanX + (Collections.max(projections) - 2) * ux;

                    double endpointY =
                            meanY + (Collections.max(projections) - 2) * uy;

                    points.add(new Coordinates(endpointX, endpointY));

                    PathBuilder builder = follower.pathBuilder();

                    if (points.size() == 3) {
                        Coordinates ball = coords.get(0);

                        double dx = ball.x - robotX;
                        double dy = ball.y - robotY;
                        double dist = Math.hypot(dx, dy);
                        double dirX = dx / dist;
                        double dirY = dy / dist;

                        double lineEndX = ball.x + 2 * dirX;
                        double lineEndY = ball.y + 2 * dirY;

                        builder.addPath(new BezierLine(new Pose(robotX, robotY), new Pose(lineEndX, lineEndY)));
                    } else {

                        while (iteration < points.size()) {
                            if (iteration < points.size() - 1) {
                                Coordinates previous = new Coordinates(points.get(iteration - 1).x, points.get(iteration - 1).y);
                                Coordinates post = new Coordinates(points.get(iteration + 1).x, points.get(iteration + 1).y);
                                Coordinates ctrl = Coordinates.midpointControlPoint(previous, post, new Coordinates(points.get(iteration).x, points.get(iteration).y));
                                builder.addPath(new BezierCurve(new Pose(previous.x, previous.y), new Pose(ctrl.x, ctrl.y), new Pose(post.x, post.y)));
                                builder.setBrakingStrength(0.1);
                                builder.setBrakingStart(0.1);
                            }
                            if (iteration == points.size() - 3) {
                                Coordinates previous = points.get(iteration + 1);
                                Coordinates post = points.get(iteration + 2);
                                builder.addPath(new BezierLine(new Pose(previous.x, previous.y), new Pose(post.x, post.y)));
                                builder.setBrakingStrength(0.1);
                                builder.setBrakingStart(0.1);
                            }
                            iteration += 2;
                        }
                    }

                    builder.setTangentHeadingInterpolation();

                    PathChain path = builder.build();

                    follower.followPath(path);

                    automatedDrive = true;

                }

            }

            if (detection1) {
                telemetry.addLine("detected");
            } else {
                telemetry.addLine("not detected");
            }
            telemetry.addData("xPos", follower.getPose().getX());
            telemetry.addData("yPos", follower.getPose().getY());
            telemetry.addData("Balls Detected:", pollenList.size());
            telemetry.update();

        }
    }

}
