package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.*;
import com.arcrobotics.ftclib.command.SubsystemBase;

//http://192.168.43.1:8080/dash
@Config
@TeleOp
public class General extends LinearOpMode {

    //public static double slideServoPos = 0.5;
    //public static double slideServoOffset = 0.0;

    public static double armPos = 0.7;
    public static double outtakeHingePos = 0.5;
    public static double HorizontalslideServoPos = 0.5;
    public static int VerticalslidePos = 0;
    public static double intakeHingePos = 0.4;
    public static double claw = 0.5;
    public static double intake = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        //subsystems
        Arm arm = new Arm(hardwareMap, "al", "ar","claw", "claw_hinge");//, "claw", "clawrotate");       //arm and claw
        DriveTrain dt = new DriveTrain(imu, gamepad1, hardwareMap, "br", "bl", "fr", "fl");
        HorizontalSlides hSlides = new HorizontalSlides(hardwareMap, "fwdslide_r", "fwdslide_l","intake", "intHinge_r","intHinge_l");   //this includes intake
        VerticalSlides vSlides = new VerticalSlides(hardwareMap, "slide");

        // set start position of stuff
        //arm.moveArm(armPos);    //change if not config
        //hSlides.move(slideServoPos);
        //vSlides.reduce();

        waitForStart();

        if (isStopRequested()) return;
        hSlides.setHingePos(0.2);
        arm.moveArm(0.6);
        hSlides.move(0.5);
        while (opModeIsActive()) {

//            arm.moveArm(armPos);    //change if not config
              dt.update();
//            vSlides.moveToPos(VerticalslidePos);
//            hSlides.move(HorizontalslideServoPos);
//            arm.moveHinge(outtakeHingePos);
//            //hSlides.setIntHingePos(intakeHingePos);
//            hSlides.setHingePos(intakeHingePos);
//            arm.grab(claw);

            if (gamepad1.options) {     //reset yaw, but we probably wont use this since roadrunner
                imu.resetYaw();
            }

            if (gamepad1.right_trigger >= 0.2) {    // If the  button is pressed, raise the arm
                hSlides.move(0.66);
                sleep(300);
                hSlides.setHingePos(0.49);
                hSlides.intakeOn();
            } else if (gamepad1.left_trigger >= 0.2){
                hSlides.intakeOff();
                arm.release();
                hSlides.setHingePos(0.17);
                sleep(200);
                arm.moveArm(0.435);
                arm.moveHinge(0.43);
                sleep(250);
                hSlides.move(0.55);
                sleep(400);
                hSlides.move(0.5);
                sleep(600);
                arm.grab(1);
                sleep(500);
                arm.moveHinge(0.6);
                hSlides.move(0.6);
                sleep(500);
                arm.moveArm(0.7);
                sleep(100);
                hSlides.move(0.5);
            } else if (gamepad1.right_bumper) {
                arm.release();
                sleep(1000);
                arm.moveArm(0.7);
                vSlides.moveToPos(0);
            } else if (gamepad1.left_bumper) {
                hSlides.intakeEject();
            } else if (gamepad1.dpad_up) {
                vSlides.moveToPos(2300);
                arm.moveArm(0.8);
                arm.moveHinge(0.4);
            } else if (gamepad1.dpad_left) {
                vSlides.moveToPos(2030);
                arm.moveArm(0.5);
                arm.moveHinge(0);
            } else if (gamepad1.dpad_down) {
                vSlides.moveToPos(0);
                arm.moveArm(1);
                arm.moveHinge(0.45);
            }
            vSlides.showPos(telemetry);
        }
    }
}