package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
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
    public static double Horizontalslide = 0.5;
    public static int VerticalslidePos = 0;
    public static double intakeHingePos = 0.4;
    public static double claw = 0.5;
    //public static double intake = 0.0;

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
        GamepadEx gp2 = new GamepadEx(gamepad2);
        GamepadEx old_gamepad = gp2;

        //subsystems
        Arm arm = new Arm(hardwareMap, "al", "ar","claw", "claw_hinge");//, "claw", "clawrotate");       //arm and claw
        //DriveTrain dt = new DriveTrain(imu, gamepad2, hardwareMap, "br", "bl", "fr", "fl");
        HorizontalSlides hSlides = new HorizontalSlides(hardwareMap, "fwdslide_r", "fwdslide_l","intake", "intHinge_r","intHinge_l");   //this includes intake
        VerticalSlides vSlides = new VerticalSlides(hardwareMap, "slide");
        hSlides.setHingePos(intakeHingePos);

        // set start position of stuff
//        arm.moveArm(armPos);    //change if not config
//        arm.moveHinge(outtakeHingePos);
//        vSlides.moveToPos(VerticalslidePos);
//        arm.grab(claw);
//        hSlides.move(Horizontalslide);

        //hSlides max position - 0.63
        //hSlides min position - 0.5

        //intakeHinge max position - 0.49
        //intakeHinge min position

        //arm positions
        //0.18 - autonomous pickup
        //0.52 - outtake
        //0.83 - transfer position
        waitForStart();

        boolean drivingOn = false;
        Thread driving = new Thread(() -> {
            DriveTrain dt = new DriveTrain(imu, gamepad2, hardwareMap, "br", "bl", "fr", "fl");
            while (opModeIsActive()) {
                dt.update();
            }
        });

        if (isStopRequested()) return;
        hSlides.setHingePos(0.2);
        arm.moveArm(0.6);
        arm.moveHinge(outtakeHingePos);
        vSlides.moveToPos(VerticalslidePos);
        arm.grab(claw);

        while (opModeIsActive()) {

//            hSlides.setHingePos(intakeHingePos);
//            arm.moveArm(armPos);    //change if not config
//            arm.moveHinge(outtakeHingePos);
//            vSlides.moveToPos(VerticalslidePos);
//            arm.grab(claw);
//            hSlides.move(Horizontalslide);

            if (!drivingOn) {
                drivingOn = true;
                driving.start();
            }

            if (gamepad1.options) { imu.resetYaw(); }

            if (gamepad1.right_trigger >= 0.3) {    // HOVER, EXTEND, INTAKE OFF, ARM INTO PLACE
                hSlides.setHingePos(0.4);
                hSlides.move(0.66);
                sleep(300);
                hSlides.intakeOff();
                arm.moveArm(0.839);
                arm.moveHinge(0.43);
            } else if (gamepad1.right_bumper){      // INTAKE DOWN, INTAKE ON
                hSlides.setHingePos(0.49);
                hSlides.intakeOn();
            }else if (gamepad1.b){  //transfer
                hSlides.intakeOff();
                arm.release();
                hSlides.setHingePos(0.17);
                sleep(200);
                hSlides.move(0.48);
                sleep(800);
                arm.grab(1);
                sleep(500);
                hSlides.move(0.66);
                sleep(50);
                arm.moveHinge(0.6);
                sleep(500);
                arm.moveArm(0.52);
                sleep(100);
                hSlides.move(0.51);
            } else if (gamepad1.left_trigger >= 0.3) {     //drop sample
                arm.moveHinge(0.4);
                sleep(350);
                arm.moveArm(0.52);
                sleep(150);
                arm.release();
                sleep(300);
                arm.moveArm(0.6);
                vSlides.moveToPos(0);
            } else if (gp2.isDown(GamepadKeys.Button.A)) {
                hSlides.intakeEject();
            } else if (gp2.isDown(GamepadKeys.Button.B)){
                hSlides.intakeOff();
            } else if (gamepad1.dpad_up) {      // EXTEND VSLIDES
                vSlides.moveToPos(2300);
                arm.moveArm(0.55);
                arm.moveHinge(0.5);
            } else if (gamepad1.dpad_left) {    // IDK WHAT THIS IS FOR
                vSlides.moveToPos(2030);
                arm.moveArm(0.5);
                arm.moveHinge(0);
            } else if (gamepad1.dpad_down) {    // CONTRACT VSLIDES
                vSlides.moveToPos(0);
                arm.moveArm(0.52);
                arm.moveHinge(0.45);
            } else if (gamepad2.y) {                            //y - hinge in top position
                hSlides.setHingePos(0.17);
            }

            if (gp2.isDown(GamepadKeys.Button.DPAD_LEFT)) hSlides.setHingePos(0.4);
            else if (gp2.isDown(GamepadKeys.Button.DPAD_UP)) hSlides.setHingePos(0.46);
            else if (gp2.isDown(GamepadKeys.Button.DPAD_RIGHT)) hSlides.setHingePos(0.49);

            if (gp2.isDown(GamepadKeys.Button.X)) hSlides.move(0.536);
            else if (gp2.isDown(GamepadKeys.Button.Y)) hSlides.move(0.59);

            vSlides.showPos(telemetry);
            old_gamepad = gp2;
        }
    }
}