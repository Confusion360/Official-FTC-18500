package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.button.*;
import org.firstinspires.ftc.teamcode.commands.*;
import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.MyRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Command TeleOp")
public class CmdOpMode extends CommandOpMode {
	Robot robot_teleOp = new MyRobot(MyRobot.OpModeType.TELEOP);
	private IMU imu;
	private IMU.Parameters parameters;
	private GamepadEx gamepad1;

	//subsystem declare
	private Arm arm;
	private DriveTrain dt;
	private HorizontalSlides hSlides;
	private VerticalSlides vSlides;

	//Command Declare
	private DriveCommand drive_comm;
	private IntakeEject intakeEject_comm;
	private IntakeReady intakeReady_comm;
	private IntakeToArmTransfer transfer_comm;
	private ReleaseSample releaseSample_comm;
	private VerticalSlidesContract vSlidesContract_comm;
	private VerticalSlidesExtend vSlidesExtend_comm;
	private VerticalSlidesPrep vSlidesPrep_comm;

	//button declate
	Trigger intReady_btn;
	Button transfer_btn;
	Trigger release_btn;
	Button eject_btn;
	Button vExtend_btn;
	Button vContract_btn;
	Button vPrep_btn;

	@Override
	public void initialize() {
		//initialise hardware
		imu = hardwareMap.get(IMU.class, "imu");
		parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
				RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
				RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
		imu.initialize(parameters);
		gamepad1 = new GamepadEx(super.gamepad1);	//access the gamepad1 from OpMode

		//subsystem init
		arm = new Arm(hardwareMap, "al", "ar","claw", "claw_hinge");//, "claw", "clawrotate");       //arm and claw
		dt = new DriveTrain(imu, gamepad1, hardwareMap, "br", "bl", "fr", "fl");
		hSlides = new HorizontalSlides(hardwareMap, "fwdslide_r", "fwdslide_l","intake", "intHinge_r","intHinge_l");   //this includes intake
		vSlides = new VerticalSlides(hardwareMap, "slide");

		//command init
		drive_comm = new DriveCommand(dt);
		intakeEject_comm = new IntakeEject(hSlides);
		intakeReady_comm = new IntakeReady(hSlides);
		transfer_comm = new IntakeToArmTransfer(arm, hSlides);
		releaseSample_comm = new ReleaseSample(arm, vSlides);
		vSlidesContract_comm = new VerticalSlidesContract(arm, vSlides);
		vSlidesExtend_comm = new VerticalSlidesExtend(arm, vSlides);
		vSlidesPrep_comm = new VerticalSlidesPrep(vSlides, arm);

		//command scheduling
		intReady_btn = new Trigger(() -> gamepad1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5);		//weird syntax for triggers
		release_btn = new Trigger(() -> gamepad1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5);

		transfer_btn = (new GamepadButton(gamepad1, GamepadKeys.Button.A)).whenPressed(transfer_comm);
		eject_btn = (new GamepadButton(gamepad1, GamepadKeys.Button.RIGHT_BUMPER)).whenPressed(intakeEject_comm);

		//trigger handling
		intReady_btn.whenActive(intakeReady_comm);	//whenActive only schedules something once per time the condition is met
		release_btn.whenActive(releaseSample_comm);

		// Driving setup
		register(dt);
		register(arm);  // Register all subsystems
		register(hSlides);
		register(vSlides);
		dt.setDefaultCommand(drive_comm);
	}
}
