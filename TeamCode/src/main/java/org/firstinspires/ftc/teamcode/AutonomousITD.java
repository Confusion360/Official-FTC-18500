package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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

@Config
@Autonomous(name = "Autonomous IntoTheDeep", group = "Autonomous")
public class AutonomousITD extends LinearOpMode {
    //vertical slides class
    public class VerticalSlides {
        private DcMotor vslide;

        public VerticalSlides(HardwareMap hardwaremap){
            vslide = hardwaremap.get(DcMotor.class, "slide");
            vslide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            vslide.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        public class LiftUp implements Action {
            // checks if the lift motor has been powered on
            private boolean initialized = false;

            // actions are formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                if (!initialized) {
                    vslide.setPower(0.8);
                    initialized = true;
                }

                // checks lift's current position
                double pos = vslide.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < 2300.0) {
                    // true causes the action to rerun
                    return true;
                } else {
                    // false stops action rerun
                    vslide.setPower(0);
                    return false;
                }
                // overall, the action powers the lift until it surpasses
                // 2300 encoder ticks, then powers it off
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
                    vslide.setPower(-0.8);
                    initialized = true;
                }

                double pos = vslide.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos > 10.0) {
                    return true;
                } else {
                    vslide.setPower(0);
                    return false;
                }
            }
        }

        public Action liftDown() {
            return new LiftDown();
        }
    }

    // claw class
    public class Arm {
        private Servo arm_l;
        private Servo arm_r;
        private Servo claw;
        private Servo claw_hinge;

        public Arm(HardwareMap hMap) {
            claw = hardwareMap.get(Servo.class, "claw");
            arm_l = hMap.get(Servo.class, "al");
            arm_r = hMap.get(Servo.class, "ar");
            claw_hinge = hMap.get(Servo.class, "claw_hinge");
        }

        // within the Claw class
        public class CloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(1.0);
                return false;
            }
        }
        public Action closeClaw() {
            return new CloseClaw();
        }

        public class OpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(0.5);
                return false;
            }
        }
        public Action openClaw() {
            return new OpenClaw();
        }

        public class arm_down implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw_hinge.setPosition(0.3);
                arm_l.setPosition(1-0.175);
                arm_r.setPosition(0.175);
                return false;
            }
        }
        public Action Arm_Down() {
            return new arm_down();
        }

        public class arm_up implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw_hinge.setPosition(0.5);
                arm_l.setPosition(1-0.55);
                arm_r.setPosition(0.55);
                return false;
            }
        }
        public Action Arm_UP() {
            return new arm_up();
        }

        public class drop implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw_hinge.setPosition(0.4);
                new SleepAction(0.35);
                arm_l.setPosition(1-0.52);
                arm_r.setPosition(0.52);
                return false;
            }
        }
        public Action Drop() {
            return new drop();
        }



    }

    //Need to add arm + claw hinge

    //just init ihinges and hslides to make them not move

    public class TrackPositionAction implements Action {
        private final MecanumDrive drive;
        private Pose2d robotPosition;

        public TrackPositionAction(MecanumDrive drive) {
            this.drive = drive;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            this.robotPosition = drive.pose;
            return false;
        }

        public Pose2d getRobotPosition() {
            return robotPosition;
        }
    }

    public class Intake {
        private  Servo rightSlideServo;
        private  Servo leftSlideServo;
        private  Servo rightIntHinge;
        private  Servo leftIntHinge;

        public Intake(HardwareMap hMap) {
            rightSlideServo = hMap.get(Servo.class, "fwdslide_r");
            leftSlideServo = hMap.get(Servo.class, "fwdslide_l");
            rightIntHinge = hMap.get(Servo.class, "intHinge_r");
            leftIntHinge = hMap.get(Servo.class, "intHinge_l");
        }

        public class start_no_move implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                rightIntHinge.setPosition(0.4 + 0.02);
                leftIntHinge.setPosition(0.4);
                rightSlideServo.setPosition(0.5 -0.005);
                leftSlideServo.setPosition(1-0.5);
                return false;
            }
        }
        public Action Start_No_Move() {
            return new start_no_move();
        }
    }

    @Override
    public void runOpMode() {
        // instantiate your MecanumDrive at a particular pose.
        Pose2d initialPose = new Pose2d(-38, -61, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        // make a Claw instance
        Arm arm = new Arm(hardwareMap);
        // make a Lift instance
        VerticalSlides vslide = new VerticalSlides(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        TrackPositionAction track = new TrackPositionAction(drive);

        Pose2d basket = new Pose2d(-58.4, -55.48, Math.toRadians(-45));
        Vector2d basket_vec = new Vector2d(-58.4, -55.48);

        Action toBasket = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(-40, -38))	//7.5 is half the robot
                .turnTo(Math.toRadians(45))
                .strafeTo(basket_vec)
                .build();

        SequentialAction sampleIntoBasket = new SequentialAction(
                vslide.liftUp(),
                new SleepAction(0.35),
                arm.Drop(),
                new SleepAction(0.15),
                arm.openClaw(),
                new SleepAction(0.3),
                arm.Arm_UP(),
                new SleepAction(0.5),
                vslide.liftDown());
                //arm.openClaw()

        Action collectFirst = drive.actionBuilder(new Pose2d(-58.4, -55.48, Math.toRadians(45)))
                .turnTo(90)
                .strafeTo(new Vector2d(-35, -55.48))
                //.turnTo(Math.toRadians(90))
                //.strafeTo(new Vector2d(-35, -13))
                //.strafeTo(new Vector2d(-47, -13))
                .build();

        Action toBasket2 = drive.actionBuilder(new Pose2d(-47, -13, Math.toRadians(90)))
                .strafeTo(new Vector2d(-40, -38))
                .turn(Math.toRadians(-45))
                .strafeTo(basket.position)
                .build();

        SequentialAction grabSample = new SequentialAction(
                arm.Arm_Down(),
                new SleepAction(1.0),
                arm.closeClaw(),
                new SleepAction(0.8),
                arm.Arm_UP()
        );

        //Action park = drive.actionBuilder(basket);


        //clsoes claw at start
        Actions.runBlocking(arm.closeClaw());

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        intake.Start_No_Move(),
                        new SleepAction(4 ),
                        arm.Arm_UP(),
                        new SleepAction(4),
                        toBasket,
                        sampleIntoBasket
                        //collectFirst
                        //grabSample,
                        //toBasket2
                )
        );


    }
}
