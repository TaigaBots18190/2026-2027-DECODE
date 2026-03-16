package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
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


import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@TeleOp
public class ShooterTurretTesting extends LinearOpMode {

    private Limelight3A limelight;
    boolean detection = false;
    // Variable Initialization
    private DcMotorEx m1;
    private Servo hoodExtender, indexer, hinge;
    private double botPoseX = 0;
    private double botPoseY = 0;
    private double heading = 0;
    private String motif = "Undetected";
    double[] stepSize = {10, 1, 0.1, 0.01, 0.001, 0.0001};
    int stepIndex = 1;
    double F = 0;
    double P = 0;
    double targetVel = 1500;
    double curTargetVelocity = targetVel;

    public void runOpMode() {

        // HardwareMap init

        DcMotor turret = hardwareMap.get(DcMotor.class, "turret");
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {


            telemetry.addData("Turret Position: ", turret.getCurrentPosition());
            telemetry.update();


        }
    }
}