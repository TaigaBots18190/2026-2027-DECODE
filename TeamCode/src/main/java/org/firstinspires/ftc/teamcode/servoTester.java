package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class servoTester extends LinearOpMode {

    public void runOpMode() {

        Servo tester = hardwareMap.get(Servo.class, "test");

        double pos = tester.getPosition();

        ElapsedTime testingInterval = new ElapsedTime();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            if (gamepad1.right_bumper) {
                pos += 0.01;
            }

            if (gamepad1.left_bumper) {
                pos -= 0.01;
            }

            tester.setPosition(pos);

            telemetry.addData("Tester Servo: ", tester.getPosition());
            telemetry.update();

        }
    }

}
