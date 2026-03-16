package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class shooterTesting extends LinearOpMode {

    double[] stepSize = {10, 1, 0.1, 0.01, 0.001, 0.0001};
    int stepIndex = 1;
    double F = 0;
    double P = 0;
    double targetVel = 1500;
    double curTargetVelocity = targetVel;




    public void runOpMode() {

        DcMotorEx m1 = hardwareMap.get(DcMotorEx.class, "shoot1");
        DcMotor light = hardwareMap.get(DcMotor.class, "l");
        m1.setDirection(DcMotorSimple.Direction.REVERSE); // may have to change to m1 depending on testing direction
        m1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(0, 0, 0, 0); // 14.4 is F,
        m1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        Servo hinge = hardwareMap.get(Servo.class, "h");

        Servo hoodExtender = hardwareMap.get(Servo.class, "s1");

        ElapsedTime activationInterval = new ElapsedTime();
        ElapsedTime activate1 = new ElapsedTime();
        ElapsedTime activate2 = new ElapsedTime();

        boolean shooterActive = true;
        boolean hinge1 = false;

        double servoPos = 0;

        telemetry.addLine("INIT COMPLETE 67!");
        telemetry.update();
        // 19.8
        waitForStart();

        light.setPower(1);

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double highVelocity = targetVel;
            double lowVelocity = targetVel - 600;

            if (gamepad1.right_trigger > 0.5) {
                servoPos -= 0.01;
            }

            if (gamepad1.left_trigger > 0.5) {
                servoPos += 0.01;
            }

            servoPos = Math.max(0.0, Math.min(1.0, servoPos));

            if (gamepad1.right_bumper && activate1.milliseconds() > 500) {
                targetVel += stepSize[stepIndex];
                activate1.reset();
            } else if (gamepad1.left_bumper && activate2.milliseconds() > 500) {
                targetVel -= stepSize[stepIndex];
                activate2.reset();
            }

            if (gamepad1.yWasPressed()) {
                shooterActive = !shooterActive;
            }

            if (gamepad1.leftBumperWasPressed()) {
                hinge1 = !hinge1;
            }

            if (!hinge1) {
                hinge.setPosition(0.09);
            } else {
                hinge.setPosition(0.4);
            }


            if (shooterActive) {
                curTargetVelocity = highVelocity;
                m1.setVelocity(highVelocity);
            } else {
                curTargetVelocity = lowVelocity;
                m1.setVelocity(lowVelocity);
            }

            hoodExtender.setPosition(servoPos);

            if (gamepad1.bWasPressed()) {
                stepIndex = (stepIndex + 1) % stepSize.length;
            }

            if (gamepad1.dpadLeftWasPressed()) {
                F -= stepSize[stepIndex];
            }
            if (gamepad1.dpadRightWasPressed()) {
                F += stepSize[stepIndex];
            }
            if (gamepad1.dpadUpWasPressed()) {
                P += stepSize[stepIndex];
            }
            if (gamepad1.dpadDownWasPressed()) {
                P -= stepSize[stepIndex];
            }

            pidfCoefficients = new PIDFCoefficients(P, 0, 0, F); // Testing
            m1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

            double curVelocity = Math.abs(m1.getVelocity());
            double error = curTargetVelocity - curVelocity;


            telemetry.addData("Hood Extender Pos: ", hoodExtender.getPosition());
            telemetry.addData("Target Velocity:", curTargetVelocity);
            telemetry.addData("Actual Velocity:", curVelocity);
            telemetry.addData("Error:", error);
            telemetry.addLine("---------------------------");
            telemetry.addData("P Value: ", P);
            telemetry.addData("F Value: ", F);
            telemetry.addData("Step Size: ", stepSize[stepIndex]);
            telemetry.update();


        }

    }

}
