package org.firstinspires.ftc.teamcode.pedroPathing;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Test 2 Servos")
public class ServoTester2 extends LinearOpMode{

    ElapsedTime controller = new ElapsedTime();

    public void runOpMode() {

        Servo servo1 = hardwareMap.get(Servo.class, "a");
        Servo servo2 = hardwareMap.get(Servo.class, "b");
        servo1.setPosition(0.5);
        servo2.setPosition(0.5);
        double servoPos = 0.5;

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            if (gamepad1.right_bumper && controller.milliseconds() > 500) {
                servoPos += 0.005;
                controller.reset();
            }
            if (gamepad1.left_bumper && controller.milliseconds() > 500) {
                servoPos -= 0.005;
                controller.reset();
            }

            servo1.setPosition(servoPos);
            servo2.setPosition(servoPos);
            telemetry.addData("Servo1 Position", servo1.getPosition());
            telemetry.addData("Servo2 Position", servo2.getPosition());

        }
    }
}
