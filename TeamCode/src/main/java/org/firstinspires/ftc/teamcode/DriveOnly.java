package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "DriveOnly", group = "Rizzlords")
public class DriveOnly extends LinearOpMode {
    private DcMotor BottomLeft;

    private DcMotor TopRight;
    private DcMotor BottomRight;
    private DcMotor TopLeft;
//
    private double CurrRotation;

    private ElapsedTime runtime = new ElapsedTime();
    private int encoder;

    @Override
    public void runOpMode() throws InterruptedException {

        BottomLeft = hardwareMap.get(DcMotor.class, "Bottom Left");
        TopRight = hardwareMap.get(DcMotor.class, "Top Right");
        BottomRight = hardwareMap.get(DcMotor.class, "Bottom Right");
        TopLeft = hardwareMap.get(DcMotor.class, "Top Left");

        CurrRotation = 0;

        // initilization blocks, right motor = front right, left motor = front left, arm = back right, hand = back left
//        BottomLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                WheelControl();
                telemetry.update();
            }
        }
    }

    private void WheelControl() {
        double vertical;
        double horizontal;
        double pivot;
        TopRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BottomRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        TopLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BottomLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double speedDiv = (gamepad1.left_trigger > 0) ? 3.5 : 1;
        pivot = gamepad1.left_stick_x / speedDiv;
        horizontal = gamepad1.right_stick_x / speedDiv;
        vertical = -1 * gamepad1.right_stick_y / speedDiv;

        double topRight = -pivot + vertical - horizontal;
        double bottomRight = -pivot + vertical + horizontal;
        double topLeft = pivot + vertical + horizontal;
        double bottomLeft = -pivot + vertical + horizontal;

        TopRight.setPower(topRight);
        BottomRight.setPower(bottomRight);
        TopLeft.setPower(topLeft);
        BottomLeft.setPower(bottomLeft);

        telemetry.addData("BottomLeft", bottomLeft);
        telemetry.addData("TopRight", topRight);
        telemetry.addData("TopLeft", topLeft);
        telemetry.addData("bottomRight", bottomRight);
    }
}
