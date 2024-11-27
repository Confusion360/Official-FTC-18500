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
	//Robot robot_teleOp = new MyRobot(MyRobot.OpModeType.TELEOP);
	private IMU imu;
	private IMU.Parameters parameters;
	private GamepadEx gamepad1;
	private GamepadEx gamepad2;

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
	private Prep prep_comm;
	private FullTransfer fullTransfer_comm;
	private HorizontalSlidesOtherLength hSlidesExtend_comm;
	private HorizontalSlidesOtherLength hSlidesContract_comm;

	//button declare
	private Trigger intReady_btn;
	private Button transfer_btn;
	private Trigger release_btn;
	private Button eject_btn;
	private Button vExtend_btn;
	private Button vContract_btn;
	private Button vPrep_btn;
	private Button fullTransfer_btn;
	private Trigger hSlidesExtend_btn;
	private Trigger hSlidesContract_btn;

	@Override
	public void initialize() {
		//initialise hardware
		imu = hardwareMap.get(IMU.class, "imu");
		parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
				RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
				RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
		imu.initialize(parameters);
		//gamepad1 = new GamepadEx(super.gamepad1);	//access the gamepad1 from OpMode
		gamepad2 = new GamepadEx(super.gamepad2);

		//subsystem init
		arm = new Arm(hardwareMap, "al", "ar","claw", "claw_hinge");//, "claw", "clawrotate");       //arm and claw
		//dt = new DriveTrain(imu, gamepad2, hardwareMap, "br", "bl", "fr", "fl");
		hSlides = new HorizontalSlides(hardwareMap, "fwdslide_r", "fwdslide_l","intake", "intHinge_r","intHinge_l");   //this includes intake
		//vSlides = new VerticalSlides(hardwareMap, "slide", gamepad1);

		//command init
		drive_comm = new DriveCommand(dt);
		intakeEject_comm = new IntakeEject(hSlides);
		intakeReady_comm = new IntakeReady(hSlides);
		transfer_comm = new IntakeToArmTransfer(arm, hSlides, vSlides);
		releaseSample_comm = new ReleaseSample(arm, vSlides);
		vSlidesContract_comm = new VerticalSlidesContract(arm, vSlides);
		vSlidesExtend_comm = new VerticalSlidesExtend(arm, vSlides);
		prep_comm = new Prep(hSlides, arm);
		fullTransfer_comm = new FullTransfer(hSlides, arm, vSlides);
		hSlidesContract_comm = new HorizontalSlidesOtherLength(hSlides, false);
		hSlidesExtend_comm = new HorizontalSlidesOtherLength(hSlides, true);

		//command scheduling
		intReady_btn = new Trigger(() -> gamepad1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5);		//weird syntax for triggers
		release_btn = new Trigger(() -> gamepad1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5);
		hSlidesExtend_btn = new Trigger(() -> gamepad1.getRightY() > 0.6);
		hSlidesContract_btn = new Trigger(() -> gamepad1.getRightY() < 0.4);

		transfer_btn = (new GamepadButton(gamepad1, GamepadKeys.Button.A)).whenPressed(transfer_comm);
		eject_btn = (new GamepadButton(gamepad1, GamepadKeys.Button.RIGHT_BUMPER)).whenPressed(intakeEject_comm);
		vExtend_btn = (new GamepadButton(gamepad1, GamepadKeys.Button.DPAD_UP)).whenPressed(vSlidesExtend_comm);
		vContract_btn = (new GamepadButton(gamepad1, GamepadKeys.Button.DPAD_DOWN)).whenPressed(vSlidesContract_comm);
		//vPrep_btn = (new GamepadButton(gamepad1, GamepadKeys.Button.LEFT_BUMPER)).whenPressed(vSlidesPrep_comm);
		fullTransfer_btn = (new GamepadButton(gamepad1, GamepadKeys.Button.DPAD_RIGHT)).whenPressed(fullTransfer_comm);

		//trigger handling
		intReady_btn.whenActive(intakeReady_comm);	//whenActive only schedules something once per time the condition is met
		release_btn.whenActive(releaseSample_comm);
		hSlidesExtend_btn.whenActive(hSlidesExtend_comm);
		hSlidesContract_btn.whenActive(hSlidesContract_comm);

		// Driving setup

		//register(dt);
		register(arm);  // Register all subsystems
		register(hSlides);
		register(vSlides);
		//dt.setDefaultCommand(drive_comm);	//unless drivetrain is being used elsewhere (which it cannot be) do drive stuff
		schedule(prep_comm);
	}
}
