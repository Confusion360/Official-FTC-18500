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
import org.firstinspires.ftc.teamcode.subsystems.ArmSystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.HorizontalSlides;

import com.arcrobotics.ftclib.command.SubsystemBase;

//http://192.168.43.1:8080/dash
@Config
@TeleOp
public class General extends LinearOpMode {

    public static double slideServoPos = 0.5;
    public static double slideServoOffset = 0.0;

    public static double armPos = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        //position for the slide max and min, idk the max value check that with someone
        int SLIDE_MIN_POSITION = 0;
        //I calculated revolutions using circumference and slide height change
        //I calculated tics per revolution by PPR
        int SLIDE_MAX_POSITION = 2250;

        // Motor for linear slide
        DcMotor slideMotor = hardwareMap.dcMotor.get("slide");

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        //subsystems
        ArmSystem arm = new ArmSystem(hardwareMap, "al", "ar", "claw");
        DriveTrain dt = new DriveTrain(imu, gamepad1, hardwareMap, "br", "bl", "fr", "fl");
        HorizontalSlides hSlides = new HorizontalSlides(hardwareMap, "fwdslide_r", "fwdslide_l");

        // set start position of stuff
        arm.moveArm(armPos);    //change if not config
        hSlides.move(slideServoPos, slideServoOffset);


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            arm.moveArm(armPos);    //change if not config
            dt.update();

            hSlides.move(slideServoPos, slideServoOffset);

            if (gamepad1.options) {     //reset yaw, but we probably wont use this since roadrunner
                imu.resetYaw();
            }

            // If the A button is pressed, raise the arm
            if (gamepad1.right_bumper) {
                slideMotor.setTargetPosition(SLIDE_MAX_POSITION);
                slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideMotor.setPower(0.5);
            }

            // If the B button is pressed, lower the arm
            if (gamepad1.left_bumper) {
                slideMotor.setTargetPosition(SLIDE_MIN_POSITION);
                slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideMotor.setPower(0.5);
            }


            // Get the current position of the armMotor
            double position = slideMotor.getCurrentPosition();

            // Get the target position of the armMotor
            double desiredPosition = slideMotor.getTargetPosition();

            // Show the position of the armMotor on telemetry
            telemetry.addData("Encoder Position", position);

            // Show the target position of the armMotor on telemetry
            telemetry.addData("Desired Position", desiredPosition);

            telemetry.update();
        }
    }
}