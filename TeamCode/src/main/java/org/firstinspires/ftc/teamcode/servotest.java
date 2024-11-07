package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp
public class servotest extends LinearOpMode {

    public static double pos = 0.5;

    //arm positions
    //0.18 - autonomous pickup
    //0.52 - outtake
    //0.83 - transfer position

    @Override
    public void runOpMode() throws InterruptedException {
        // Servos for the intake
        CRServo intake = hardwareMap.crservo.get("intake");
        Servo rightSlideServo = hardwareMap.servo.get("fwdslide_r");
        Servo leftSlideServo = hardwareMap.servo.get("fwdslide_l");

        Servo arm_l = hardwareMap.servo.get("ar");
        Servo arm_r = hardwareMap.servo.get("al");

        arm_l.setPosition(pos + 0.0);
        arm_r.setPosition(1-pos);

        waitForStart();

        if (isStopRequested()) return;



        while (opModeIsActive()) {

            arm_l.setPosition(pos + 0.0);
            arm_r.setPosition(1-pos);

//            if(gamepad1.a){
//                intake.setPower(1);
//            } else if (gamepad1.b) {
//                intake.setPower(-1);
//            } else if (gamepad1.x) {
//                intake.setPower(0);
//                rightSlideServo.setPosition(0.5 -0.005);
//                leftSlideServo.setPosition(1-0.5);
//            }else if (gamepad1.y){
//                rightSlideServo.setPosition(0.67 -0.005);
//                leftSlideServo.setPosition(1-0.67);
//            }

        }}}
