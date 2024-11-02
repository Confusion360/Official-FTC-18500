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

    @Override
    public void runOpMode() throws InterruptedException {

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
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

        while (opModeIsActive()) {

            arm.moveArm(armPos);    //change if not config
            //dt.update();
            vSlides.moveToPos(VerticalslidePos);
            hSlides.move(HorizontalslideServoPos);
            arm.moveHinge(outtakeHingePos);
            //hSlides.setIntHingePos(intakeHingePos);
            hSlides.setHingePos(intakeHingePos);
            arm.grab(claw);


            if (gamepad1.options) {     //reset yaw, but we probably wont use this since roadrunner
                imu.resetYaw();
            }

//            if (gamepad1.a) {    // If the  button is pressed, raise the arm
//                hSlides.move(0.67);
//                sleep(500);
//                hSlides.setHingePos(1);
//                hSlides.intakeOn();
//            } else if (gamepad1.b){
//                arm.moveArm(0.44);
//                arm.moveHinge(0.45);
//                sleep(500);
//                hSlides.setHingePos(0);
//                sleep(500);
//                hSlides.move(0.53);
//                arm.grab(1);
//                sleep(500);
//                hSlides.move(0.63);
//                sleep(500);
//                arm.moveArm(0.7);
//            }


            vSlides.showPos(telemetry);
        }
    }
}