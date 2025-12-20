package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.controllers.Sweeper.Sweeper_PID;
import org.firstinspires.ftc.teamcode.controllers.Trigger;

@Config
@TeleOp(name="Sweeper Tuner", group="Tuner")
public class SweeperTuner extends LinearOpMode {

    // ==================== CONFIGURATION PARAMETERS ====================
    // 这些参数可以在 FTC Dashboard 上实时调整
    public static double SWEEPER_POWER = 0.5;
    public static double SWEEPER_VELOCITY = 1000.0; // RPM
    public static double EAT_VELOCITY = 1200.0;
    public static double OUTPUT_VELOCITY = -1000.0;
    public static double GIVE_ARTIFACT_VELOCITY = 800.0;
    public static double SWEEPER_KP = 0.01;
    public static double SWEEPER_KI = 0.001;
    public static double SWEEPER_KD = 0.0001;
    public static double TRIGGER_OPEN_POSITION = 0.7;
    public static double TRIGGER_CLOSE_POSITION = 0.3;
    public static double TRIGGER_SERVO_SPEED = 0.5;

    // ==================== HARDWARE COMPONENTS ====================
    private Sweeper_PID sweeper;
    private Trigger trigger;

    // ==================== STATE VARIABLES ====================
    private enum SweeperMode {
        POWER_CONTROL,
        VELOCITY_CONTROL,
        EAT,
        OUTPUT,
        GIVE_ARTIFACT,
        STOP
    }

    private enum TriggerMode {
        OPEN,
        CLOSE,
        MANUAL
    }

    private SweeperMode currentSweeperMode = SweeperMode.STOP;
    private TriggerMode currentTriggerMode = TriggerMode.CLOSE;
    private ElapsedTime runtime = new ElapsedTime();
    private double manualTriggerPosition = 0.5;

    // ==================== INITIALIZATION ====================
    private void initHardware() {
        telemetry.addLine("Initializing Sweeper Tuner...");
        telemetry.update();

        // 初始化 Sweeper
        sweeper = new Sweeper_PID(hardwareMap, telemetry, "sweeperMotor", false);


        // 初始化 Trigger
        trigger = new Trigger(hardwareMap);

        telemetry.addLine("Hardware Initialized!");
        telemetry.update();
        sleep(500);
    }

    // ==================== TELEMETRY DISPLAY ====================
    private void updateTelemetry() {
        telemetry.clear();
        telemetry.addLine("====== SWEEPER TUNER ======");
        telemetry.addLine();

        // Sweeper 状态
        telemetry.addLine("=== SWEEPER ===");
        telemetry.addData("Mode", currentSweeperMode);
        telemetry.addData("Target Power", SWEEPER_POWER);
        telemetry.addData("Target Velocity", SWEEPER_VELOCITY);
        telemetry.addData("Current Speed (RPM)", sweeper.getCurrent_speed());
        telemetry.addData("Motor Power", sweeper.getPower());

        telemetry.addLine();

        // 控制说明
        telemetry.addLine("=== CONTROLS ===");
        telemetry.addLine("Sweeper Modes:");
        telemetry.addLine("  X = Power Control");
        telemetry.addLine("  Y = Velocity Control");
        telemetry.addLine("  A = Eat Mode");
        telemetry.addLine("  B = Output Mode");
        telemetry.addLine("  DPAD_UP = Give Artifact");
        telemetry.addLine("  DPAD_DOWN = Stop");
        telemetry.addLine();
        telemetry.addLine("Trigger Modes:");
        telemetry.addLine("  Left Bumper = Open");
        telemetry.addLine("  Right Bumper = Close");
        telemetry.addLine("  DPAD_LEFT/RIGHT = Manual Adjust");
        telemetry.addLine();
        telemetry.addLine("RT = Increase Power/Velocity");
        telemetry.addLine("LT = Decrease Power/Velocity");

        telemetry.update();
    }

    // ==================== SWEEPER CONTROL ====================
    private void controlSweeper() {
        // 模式切换
        if (gamepad1.b) {
            currentSweeperMode = SweeperMode.OUTPUT;
        } else if (gamepad1.dpad_up) {
            currentSweeperMode = SweeperMode.GIVE_ARTIFACT;
        } else if (gamepad1.dpad_down) {
            currentSweeperMode = SweeperMode.STOP;
        }

        // 实时调整参数
        if (gamepad1.right_trigger > 0.1) {
            if (currentSweeperMode == SweeperMode.POWER_CONTROL) {
                SWEEPER_POWER += 0.01;
                SWEEPER_POWER = Math.min(1.0, SWEEPER_POWER);
            } else if (currentSweeperMode == SweeperMode.VELOCITY_CONTROL) {
                SWEEPER_VELOCITY += 50;
            }
        }

        if (gamepad1.left_trigger > 0.1) {
            if (currentSweeperMode == SweeperMode.POWER_CONTROL) {
                SWEEPER_POWER -= 0.01;
                SWEEPER_POWER = Math.max(-1.0, SWEEPER_POWER);
            } else if (currentSweeperMode == SweeperMode.VELOCITY_CONTROL) {
                SWEEPER_VELOCITY -= 50;
            }
        }

        // 应用当前模式
        switch (currentSweeperMode) {
            case EAT:
                sweeper.Sweep(Sweeper_PID.EatVel);
                break;
            case OUTPUT:
                sweeper.Sweep(Sweeper_PID.OutputSpeed);
                break;
            case GIVE_ARTIFACT:
                sweeper.Sweep(Sweeper_PID.GiveTheArtifactVel);
                break;
            case STOP:
                sweeper.Sweep(0);
                break;
        }
    }

    // ==================== TRIGGER CONTROL ====================
    private void controlTrigger() {
        // 模式切换
        if (gamepad1.left_bumper) {
            currentTriggerMode = TriggerMode.OPEN;
        } else if (gamepad1.right_bumper) {
            currentTriggerMode = TriggerMode.CLOSE;
        }

        // 应用当前模式
        switch (currentTriggerMode) {
            case OPEN:
                // 假设Trigger类有setPosition方法
                // 如果没有，可能需要根据实际情况调整
                trigger.open(); // 使用默认的open方法
                break;
            case CLOSE:
                trigger.close(); // 使用默认的close方法
                break;
        }
    }

    // ==================== MAIN LOOP ====================
    @Override
    public void runOpMode() {
        initHardware();

        telemetry.addLine("Ready to start Sweeper Tuner!");
        telemetry.addLine("Press START to begin tuning");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            // 控制 Sweeper
            controlSweeper();

            // 控制 Trigger
            controlTrigger();

            // 更新 PID 参数（如果支持）
            // 这里假设Sweeper_PID类有设置PID参数的方法
            // sweeper.setPID(SWEEPER_KP, SWEEPER_KI, SWEEPER_KD);

            // 更新遥测
            updateTelemetry();

            // 避免过快的循环
            sleep(40);
        }

        // 停止所有电机
    }
}