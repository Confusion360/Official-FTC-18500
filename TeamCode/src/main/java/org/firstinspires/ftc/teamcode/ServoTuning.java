import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class ServoTuning extends OpMode {

    // Adjustable positions and offsets for each pair in the FTC Dashboard
    public static double intakeHingePosition = 0.5;
    public static double intakeHingeOffset = 0.05;

    public static double horizontalSlidePosition = 0.5;
    public static double horizontalSlideOffset = 0.05;

    public static double armPosition = 0.5;
    public static double armOffset = 0.05;

    private Servo rightIntakeHinge, leftIntakeHinge;
    private Servo rightHorizontalSlide, leftHorizontalSlide;
    private Servo armRight, armLeft;
    private FtcDashboard dashboard;

    @Override
    public void init() {
        // Hardware mapping
        rightIntakeHinge = hardwareMap.get(Servo.class, "IHinge_r");
        leftIntakeHinge = hardwareMap.get(Servo.class, "IHinge_l");
        rightHorizontalSlide = hardwareMap.get(Servo.class, "fwdslide_r");
        leftHorizontalSlide = hardwareMap.get(Servo.class, "fwdslide_l");
        armRight = hardwareMap.get(Servo.class, "ar");
        armLeft = hardwareMap.get(Servo.class, "al");

        // Initialize FTC Dashboard
        dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
    }

    @Override
    public void loop() {
        // Adjusted positions for each servo pair, considering offset
        rightIntakeHinge.setPosition(intakeHingePosition + intakeHingeOffset);
        leftIntakeHinge.setPosition(intakeHingePosition - intakeHingeOffset);

        rightHorizontalSlide.setPosition(horizontalSlidePosition + horizontalSlideOffset);
        leftHorizontalSlide.setPosition(horizontalSlidePosition - horizontalSlideOffset);

        armRight.setPosition(armPosition + armOffset);
        armLeft.setPosition(armPosition - armOffset);

        // Telemetry for feedback in the Dashboard
        telemetry.addData("Intake Hinge Position", intakeHingePosition);
        telemetry.addData("Intake Hinge Offset", intakeHingeOffset);
        telemetry.addData("Horizontal Slide Position", horizontalSlidePosition);
        telemetry.addData("Horizontal Slide Offset", horizontalSlideOffset);
        telemetry.addData("Arm Position", armPosition);
        telemetry.addData("Arm Offset", armOffset);
        telemetry.update();
    }
}
