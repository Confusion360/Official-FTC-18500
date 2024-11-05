package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.Robot;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.button.*;
import org.firstinspires.ftc.teamcode.commands.*;
import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.MyRobot;

public class CmdOpMode extends CommandOpMode {
	Robot robot_teleOp = new MyRobot(MyRobot.OpModeType.TELEOP);
	//Robot robot_auto = new MyRobot(MyRobot.OpModeType.AUTO);

	@Override
	public void initialize() {
		//initialise hardware
		IMU imu = hardwareMap.get(IMU.class, "imu");
		IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
				RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
				RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
		imu.initialize(parameters);

		Arm arm = new Arm(hardwareMap, "al", "ar","claw", "claw_hinge");//, "claw", "clawrotate");       //arm and claw
		DriveTrain dt = new DriveTrain(imu, gamepad1, hardwareMap, "br", "bl", "fr", "fl");
		HorizontalSlides hSlides = new HorizontalSlides(hardwareMap, "fwdslide_r", "fwdslide_l","intake", "intHinge_r","intHinge_l");   //this includes intake
		VerticalSlides vSlides = new VerticalSlides(hardwareMap, "slide");

	}
}
