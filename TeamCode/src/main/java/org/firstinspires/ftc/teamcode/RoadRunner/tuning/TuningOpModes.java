package org.firstinspires.ftc.teamcode.RoadRunner.tuning;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.reflection.ReflectionConfig;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.AngularRampLogger;
import com.acmerobotics.roadrunner.ftc.DeadWheelDirectionDebugger;
import com.acmerobotics.roadrunner.ftc.DriveType;
import com.acmerobotics.roadrunner.ftc.DriveView;
import com.acmerobotics.roadrunner.ftc.DriveViewFactory;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.EncoderGroup;
import com.acmerobotics.roadrunner.ftc.EncoderRef;
import com.acmerobotics.roadrunner.ftc.ForwardPushTest;
import com.acmerobotics.roadrunner.ftc.ForwardRampLogger;
import com.acmerobotics.roadrunner.ftc.LateralPushTest;
import com.acmerobotics.roadrunner.ftc.LateralRampLogger;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.acmerobotics.roadrunner.ftc.LynxQuadratureEncoderGroup;
import com.acmerobotics.roadrunner.ftc.ManualFeedforwardTuner;
import com.acmerobotics.roadrunner.ftc.MecanumMotorDirectionDebugger;
import com.acmerobotics.roadrunner.ftc.OTOSAngularScalarTuner;
import com.acmerobotics.roadrunner.ftc.OTOSEncoderGroup;
import com.acmerobotics.roadrunner.ftc.OTOSHeadingOffsetTuner;
import com.acmerobotics.roadrunner.ftc.OTOSIMU;
import com.acmerobotics.roadrunner.ftc.OTOSLinearScalarTuner;
import com.acmerobotics.roadrunner.ftc.OTOSPositionOffsetTuner;
import com.acmerobotics.roadrunner.ftc.PinpointEncoderGroup;
import com.acmerobotics.roadrunner.ftc.PinpointIMU;
import com.acmerobotics.roadrunner.ftc.PinpointView;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.OTOSLocalizer;
import org.firstinspires.ftc.teamcode.RoadRunner.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.RoadRunner.TankDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.ThreeDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.RoadRunner.TwoDeadWheelLocalizer;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * 调优OpMode注册器，用于注册和管理各种调优和测试相关的OpMode。
 * 
 * 此类定义了驱动类型配置，并注册了各种调优工具和测试OpMode，
 * 包括电机方向调试器、死轮方向调试器、前向/横向推测试验、
 * 前向/横向/角度斜坡记录器、手动前馈调优器等。
 */
public final class TuningOpModes {
    /**
     * 驱动类类型配置，默认使用MecanumDrive
     * 如果使用坦克驱动，需要修改为TankDrive.class
     */
    // TODO: change this to TankDrive.class if you're using tank
    public static final Class<?> DRIVE_CLASS = MecanumDrive.class;

    /**
     * OpMode组名
     */
    public static final String GROUP = "quickstart";
    
    /**
     * 是否禁用所有调优OpMode
     */
    public static final boolean DISABLED = false;

    /**
     * 私有构造函数，防止实例化此类
     */
    private TuningOpModes() {}

    /**
     * 为OpMode类创建元数据
     * 
     * @param cls OpMode类
     * @return OpMode元数据
     */
    private static OpModeMeta metaForClass(Class<? extends OpMode> cls) {
        return new OpModeMeta.Builder()
                .setName(cls.getSimpleName())
                .setGroup(GROUP)
                .setFlavor(OpModeMeta.Flavor.TELEOP)
                .build();
    }

    /**
     * 创建PinpointView实例，用于Pinpoint定位器的可视化和调试
     * 
     * @param pl PinpointLocalizer实例
     * @return PinpointView实例
     */
    private static PinpointView makePinpointView(PinpointLocalizer pl) {
        return new PinpointView() {
            /**
             * 平行编码器方向
             */
            GoBildaPinpointDriver.EncoderDirection parDirection = pl.initialParDirection;
            
            /**
             * 垂直编码器方向
             */
            GoBildaPinpointDriver.EncoderDirection perpDirection = pl.initialPerpDirection;

            /**
             * 更新Pinpoint驱动
             */
            @Override
            public void update() {
                pl.driver.update();
            }

            /**
             * 获取平行编码器位置
             * 
             * @return 平行编码器位置
             */
            @Override
            public int getParEncoderPosition() {
                return pl.driver.getEncoderX();
            }

            /**
             * 获取垂直编码器位置
             * 
             * @return 垂直编码器位置
             */
            @Override
            public int getPerpEncoderPosition() {
                return pl.driver.getEncoderY();
            }

            /**
             * 获取航向速度
             * 
             * @param unit 角度单位
             * @return 航向速度
             */
            @Override
            public float getHeadingVelocity(UnnormalizedAngleUnit unit) {
                return (float) pl.driver.getHeadingVelocity(unit);
            }

            /**
             * 设置平行编码器方向
             * 
             * @param direction 电机方向
             */
            @Override
            public void setParDirection(@NonNull DcMotorSimple.Direction direction) {
                parDirection = direction == DcMotorSimple.Direction.FORWARD ?
                        GoBildaPinpointDriver.EncoderDirection.FORWARD :
                        GoBildaPinpointDriver.EncoderDirection.REVERSED;
                pl.driver.setEncoderDirections(parDirection, perpDirection);
            }

            /**
             * 获取平行编码器方向
             * 
             * @return 平行编码器方向
             */
            @Override
            public DcMotorSimple.Direction getParDirection() {
                return parDirection == GoBildaPinpointDriver.EncoderDirection.FORWARD ?
                        DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE;
            }

            /**
             * 设置垂直编码器方向
             * 
             * @param direction 电机方向
             */
            @Override
            public void setPerpDirection(@NonNull DcMotorSimple.Direction direction) {
                perpDirection = direction == DcMotorSimple.Direction.FORWARD ?
                        GoBildaPinpointDriver.EncoderDirection.FORWARD :
                        GoBildaPinpointDriver.EncoderDirection.REVERSED;
                pl.driver.setEncoderDirections(parDirection, perpDirection);
            }

            /**
             * 获取垂直编码器方向
             * 
             * @return 垂直编码器方向
             */
            @Override
            public DcMotorSimple.Direction getPerpDirection() {
                return perpDirection == GoBildaPinpointDriver.EncoderDirection.FORWARD ?
                        DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE;
            }
        };
    }

    /**
     * 注册调优OpMode
     * 
     * 此方法根据配置的驱动类型创建相应的DriveViewFactory，
     * 然后注册各种调优和测试相关的OpMode。
     * 
     * @param manager OpMode管理器
     */
    @OpModeRegistrar
    public static void register(OpModeManager manager) {
        // 如果禁用，直接返回
        if (DISABLED) return;

        // 驱动视图工厂
        DriveViewFactory dvf;
        
        // 根据驱动类型创建相应的DriveViewFactory
        if (DRIVE_CLASS.equals(MecanumDrive.class)) {
            dvf = hardwareMap -> {
                // 创建MecanumDrive实例
                MecanumDrive md = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
                // 获取IMU
                LazyImu lazyImu = md.lazyImu;

                // 初始化编码器组和编码器引用列表
                List<EncoderGroup> encoderGroups = new ArrayList<>();
                List<EncoderRef> leftEncs = new ArrayList<>(), rightEncs = new ArrayList<>();
                List<EncoderRef> parEncs = new ArrayList<>(), perpEncs = new ArrayList<>();
                
                // 根据本地定位器类型设置编码器
                if (md.localizer instanceof MecanumDrive.DriveLocalizer) {
                    // 使用驱动轮编码器进行定位
                    MecanumDrive.DriveLocalizer dl = (MecanumDrive.DriveLocalizer) md.localizer;
                    encoderGroups.add(new LynxQuadratureEncoderGroup(
                            hardwareMap.getAll(LynxModule.class),
                            Arrays.asList(dl.leftFront, dl.leftBack, dl.rightFront, dl.rightBack)
                    ));
                    leftEncs.add(new EncoderRef(0, 0));
                    leftEncs.add(new EncoderRef(0, 1));
                    rightEncs.add(new EncoderRef(0, 2));
                    rightEncs.add(new EncoderRef(0, 3));
                } else if (md.localizer instanceof ThreeDeadWheelLocalizer) {
                    // 使用三个死轮编码器进行定位
                    ThreeDeadWheelLocalizer dl = (ThreeDeadWheelLocalizer) md.localizer;
                    encoderGroups.add(new LynxQuadratureEncoderGroup(
                            hardwareMap.getAll(LynxModule.class),
                            Arrays.asList(dl.par0, dl.par1, dl.perp)
                    ));
                    parEncs.add(new EncoderRef(0, 0));
                    parEncs.add(new EncoderRef(0, 1));
                    perpEncs.add(new EncoderRef(0, 2));
                } else if (md.localizer instanceof TwoDeadWheelLocalizer) {
                    // 使用两个死轮编码器进行定位
                    TwoDeadWheelLocalizer dl = (TwoDeadWheelLocalizer) md.localizer;
                    encoderGroups.add(new LynxQuadratureEncoderGroup(
                            hardwareMap.getAll(LynxModule.class),
                            Arrays.asList(dl.par, dl.perp)
                    ));
                    parEncs.add(new EncoderRef(0, 0));
                    perpEncs.add(new EncoderRef(0, 1));
                } else if (md.localizer instanceof OTOSLocalizer) {
                    // 使用OTOS进行定位
                    OTOSLocalizer ol = (OTOSLocalizer) md.localizer;
                    encoderGroups.add(new OTOSEncoderGroup(ol.otos));
                    parEncs.add(new EncoderRef(0, 0));
                    perpEncs.add(new EncoderRef(0, 1));
                    lazyImu = new OTOSIMU(ol.otos);
                }  else if (md.localizer instanceof PinpointLocalizer) {
                    // 使用Pinpoint进行定位
                    PinpointView pv = makePinpointView((PinpointLocalizer) md.localizer);
                    encoderGroups.add(new PinpointEncoderGroup(pv));
                    parEncs.add(new EncoderRef(0, 0));
                    perpEncs.add(new EncoderRef(0, 1));
                    lazyImu = new PinpointIMU(pv);
                } else {
                    // 未知定位器类型
                    throw new RuntimeException("unknown localizer: " + md.localizer.getClass().getName());
                }

                // 创建并返回DriveView
                return new DriveView(
                    DriveType.MECANUM,
                        MecanumDrive.PARAMS.inPerTick,
                        MecanumDrive.PARAMS.maxWheelVel,
                        MecanumDrive.PARAMS.minProfileAccel,
                        MecanumDrive.PARAMS.maxProfileAccel,
                        encoderGroups,
                        Arrays.asList(
                                md.leftFront,
                                md.leftBack
                        ),
                        Arrays.asList(
                                md.rightFront,
                                md.rightBack
                        ),
                        leftEncs,
                        rightEncs,
                        parEncs,
                        perpEncs,
                        lazyImu,
                        md.voltageSensor,
                        () -> new MotorFeedforward(MecanumDrive.PARAMS.kS,
                                MecanumDrive.PARAMS.kV / MecanumDrive.PARAMS.inPerTick,
                                MecanumDrive.PARAMS.kA / MecanumDrive.PARAMS.inPerTick),
                        0
                );
            };
        } else if (DRIVE_CLASS.equals(TankDrive.class)) {
            dvf = hardwareMap -> {
                // 创建TankDrive实例
                TankDrive td = new TankDrive(hardwareMap, new Pose2d(0, 0, 0));
                // 获取IMU
                LazyImu lazyImu = td.lazyImu;

                // 初始化编码器组和编码器引用列表
                List<EncoderGroup> encoderGroups = new ArrayList<>();
                List<EncoderRef> leftEncs = new ArrayList<>(), rightEncs = new ArrayList<>();
                List<EncoderRef> parEncs = new ArrayList<>(), perpEncs = new ArrayList<>();
                
                // 根据本地定位器类型设置编码器
                if (td.localizer instanceof TankDrive.DriveLocalizer) {
                    // 使用驱动轮编码器进行定位
                    TankDrive.DriveLocalizer dl = (TankDrive.DriveLocalizer) td.localizer;
                    List<Encoder> allEncoders = new ArrayList<>();
                    allEncoders.addAll(dl.leftEncs);
                    allEncoders.addAll(dl.rightEncs);
                    encoderGroups.add(new LynxQuadratureEncoderGroup(
                            hardwareMap.getAll(LynxModule.class),
                            allEncoders
                    ));
                    for (int i = 0; i < dl.leftEncs.size(); i++) {
                        leftEncs.add(new EncoderRef(0, i));
                    }
                    for (int i = 0; i < dl.rightEncs.size(); i++) {
                        rightEncs.add(new EncoderRef(0, dl.leftEncs.size() + i));
                    }
                } else if (td.localizer instanceof ThreeDeadWheelLocalizer) {
                    // 使用三个死轮编码器进行定位
                    ThreeDeadWheelLocalizer dl = (ThreeDeadWheelLocalizer) td.localizer;
                    encoderGroups.add(new LynxQuadratureEncoderGroup(
                            hardwareMap.getAll(LynxModule.class),
                            Arrays.asList(dl.par0, dl.par1, dl.perp)
                    ));
                    parEncs.add(new EncoderRef(0, 0));
                    parEncs.add(new EncoderRef(0, 1));
                    perpEncs.add(new EncoderRef(0, 2));
                } else if (td.localizer instanceof TwoDeadWheelLocalizer) {
                    // 使用两个死轮编码器进行定位
                    TwoDeadWheelLocalizer dl = (TwoDeadWheelLocalizer) td.localizer;
                    encoderGroups.add(new LynxQuadratureEncoderGroup(
                            hardwareMap.getAll(LynxModule.class),
                            Arrays.asList(dl.par, dl.perp)
                    ));
                    parEncs.add(new EncoderRef(0, 0));
                    perpEncs.add(new EncoderRef(0, 1));
                }  else if (td.localizer instanceof PinpointLocalizer) {
                    // 使用Pinpoint进行定位
                    PinpointView pv = makePinpointView((PinpointLocalizer) td.localizer);
                    encoderGroups.add(new PinpointEncoderGroup(pv));
                    parEncs.add(new EncoderRef(0, 0));
                    perpEncs.add(new EncoderRef(0, 1));
                    lazyImu = new PinpointIMU(pv);
                } else if (td.localizer instanceof OTOSLocalizer) {
                    // 使用OTOS进行定位
                    OTOSLocalizer ol = (OTOSLocalizer) td.localizer;
                    encoderGroups.add(new OTOSEncoderGroup(ol.otos));
                    parEncs.add(new EncoderRef(0, 0));
                    perpEncs.add(new EncoderRef(0, 1));
                    lazyImu = new OTOSIMU(ol.otos);
                } else {
                    // 未知定位器类型
                    throw new RuntimeException("unknown localizer: " + td.localizer.getClass().getName());
                }

                // 创建并返回DriveView
                return new DriveView(
                        DriveType.TANK,
                        TankDrive.PARAMS.inPerTick,
                        TankDrive.PARAMS.maxWheelVel,
                        TankDrive.PARAMS.minProfileAccel,
                        TankDrive.PARAMS.maxProfileAccel,
                        encoderGroups,
                        td.leftMotors,
                        td.rightMotors,
                        leftEncs,
                        rightEncs,
                        parEncs,
                        perpEncs,
                        lazyImu,
                        td.voltageSensor,
                        () -> new MotorFeedforward(TankDrive.PARAMS.kS,
                                TankDrive.PARAMS.kV / TankDrive.PARAMS.inPerTick,
                                TankDrive.PARAMS.kA / TankDrive.PARAMS.inPerTick),
                        0
                );
            };
        } else {
            // 未知驱动类型
            throw new RuntimeException();
        }

        // 注册各种调优和测试OpMode
        manager.register(metaForClass(AngularRampLogger.class), new AngularRampLogger(dvf));
        manager.register(metaForClass(ForwardPushTest.class), new ForwardPushTest(dvf));
        manager.register(metaForClass(ForwardRampLogger.class), new ForwardRampLogger(dvf));
        manager.register(metaForClass(LateralPushTest.class), new LateralPushTest(dvf));
        manager.register(metaForClass(LateralRampLogger.class), new LateralRampLogger(dvf));
        manager.register(metaForClass(ManualFeedforwardTuner.class), new ManualFeedforwardTuner(dvf));
        manager.register(metaForClass(MecanumMotorDirectionDebugger.class), new MecanumMotorDirectionDebugger(dvf));
        manager.register(metaForClass(DeadWheelDirectionDebugger.class), new DeadWheelDirectionDebugger(dvf));

        // 注册自定义调优OpMode
        manager.register(metaForClass(ManualFeedbackTuner.class), ManualFeedbackTuner.class);
        manager.register(metaForClass(SplineTest.class), SplineTest.class);
        manager.register(metaForClass(LocalizationTest.class), LocalizationTest.class);

        // 注册Y轴和航向调优OpMode
        manager.register(metaForClass(ManualFeedbackTuner_y.class), ManualFeedbackTuner_y.class);
        manager.register(metaForClass(ManualFeedbackTuner_heading.class), ManualFeedbackTuner_heading.class);

        // 注册OTOS调优OpMode
        manager.register(metaForClass(OTOSAngularScalarTuner.class), new OTOSAngularScalarTuner(dvf));
        manager.register(metaForClass(OTOSLinearScalarTuner.class), new OTOSLinearScalarTuner(dvf));
        manager.register(metaForClass(OTOSHeadingOffsetTuner.class), new OTOSHeadingOffsetTuner(dvf));
        manager.register(metaForClass(OTOSPositionOffsetTuner.class), new OTOSPositionOffsetTuner(dvf));

        // 配置FtcDashboard
        FtcDashboard.getInstance().withConfigRoot(configRoot -> {
            for (Class<?> c : Arrays.asList(
                    AngularRampLogger.class,
                    ForwardRampLogger.class,
                    LateralRampLogger.class,
                    ManualFeedforwardTuner.class,
                    MecanumMotorDirectionDebugger.class,
                    ManualFeedbackTuner.class
            )) {
                configRoot.putVariable(c.getSimpleName(), ReflectionConfig.createVariableFromClass(c));
            }
        });
    }
}
