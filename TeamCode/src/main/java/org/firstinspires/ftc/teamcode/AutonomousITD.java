package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous(name = "BLUE_TEST_AUTO_PIXEL", group = "Autonomous")
public class AutonomousITD extends LinearOpMode {
	public class Lift {
		private DcMotorEx lift;

		public Lift(HardwareMap hardwareMap) {
			lift = hardwareMap.get(DcMotorEx.class, "liftMotor");
			lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
			lift.setDirection(DcMotorSimple.Direction.FORWARD);
		}

		public class LiftUp implements Action {
			private boolean initialized = false;

			@Override
			public boolean run(@NonNull TelemetryPacket packet) {
				if (!initialized) {
					lift.setPower(0.8);
					initialized = true;
				}

				double pos = lift.getCurrentPosition();
				packet.put("liftPos", pos);
				if (pos < 3000.0) {
					return true;
				} else {
					lift.setPower(0);
					return false;
				}
			}
		}
		public Action liftUp() {
			return new LiftUp();
		}

		public class LiftDown implements Action {
			private boolean initialized = false;

			@Override
			public boolean run(@NonNull TelemetryPacket packet) {
				if (!initialized) {
					lift.setPower(-0.8);
					initialized = true;
				}

				double pos = lift.getCurrentPosition();
				packet.put("liftPos", pos);
				if (pos > 100.0) {
					return true;
				} else {
					lift.setPower(0);
					return false;
				}
			}
		}
		public Action liftDown(){
			return new LiftDown();
		}
	}

	public class Claw {
		private Servo claw;

		public Claw(HardwareMap hardwareMap) {
			claw = hardwareMap.get(Servo.class, "claw");
		}

		public class CloseClaw implements Action {
			@Override
			public boolean run(@NonNull TelemetryPacket packet) {
				claw.setPosition(0.55);
				return false;
			}
		}
		public Action closeClaw() {
			return new CloseClaw();
		}

		public class OpenClaw implements Action {
			@Override
			public boolean run(@NonNull TelemetryPacket packet) {
				claw.setPosition(1.0);
				return false;
			}
		}
		public Action openClaw() {
			return new OpenClaw();
		}
	}

	@Override
	public void runOpMode() {
		Pose2d initialPose = new Pose2d(11.8, 61.7, Math.toRadians(90));
		MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

		// vision here that outputs position
		int visionOutputPosition = 1;

		TrajectoryActionBuilder firstAlign = drive.actionBuilder(initialPose)
				.strafeTo(new Vector2d(11.8+7.5, 61.7+(24-7.5)))	//7.5 is half the robot
				.turn(Math.toRadians(45));
				//.build();

		if (isStopRequested()) return;

		Action trajectoryActionChosen;
		trajectoryActionChosen = firstAlign.build();
		Actions.runBlocking(
				new SequentialAction(
						trajectoryActionChosen
				)
		);

	}
}