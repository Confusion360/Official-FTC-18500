package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp
public class servotest extends LinearOpMode {

    public static double position = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        // Servos for the intake
        CRServo intake = hardwareMap.crservo.get("intake");
        Servo rightSlideServo = hardwareMap.servo.get("fwdslide_r");
        Servo leftSlideServo = hardwareMap.servo.get("fwdslide_l");

        waitForStart();

        if (isStopRequested()) return;



        while (opModeIsActive()) {



            if(gamepad1.a){
                intake.setPower(1);
            } else if (gamepad1.b) {
                intake.setPower(-1);
            } else if (gamepad1.x) {
                intake.setPower(0);
                rightSlideServo.setPosition(0.5 -0.005);
                leftSlideServo.setPosition(1-0.5);
            }else if (gamepad1.y){
                rightSlideServo.setPosition(0.67 -0.005);
                leftSlideServo.setPosition(1-0.67);
            }

        }}}
