package org.firstinspires.ftc.teamcode.utility.fakehardware;

import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.utility.InstanceTelemetry;

public class FakeDcMotor implements com.qualcomm.robotcore.hardware.DcMotor {
    MotorConfigurationType motorConfigurationType;
    /**
     * Returns the assigned type for this motor. If no particular motor type has been
     * configured, then {@link MotorConfigurationType#getUnspecifiedMotorType()} will be returned.
     * Note that the motor type for a given motor is initially assigned in the robot
     * configuration user interface, though it may subsequently be modified using methods herein.
     *
     * @return null
     */
    @Override
    public MotorConfigurationType getMotorType() {
        return motorConfigurationType;
    }

    /**
     * Sets the assigned type of this motor. Usage of this method is very rare.
     *
     * @param motorType the new assigned type for this motor
     * @see #getMotorType()
     */
    @Override
    public void setMotorType(MotorConfigurationType motorType) {
        motorConfigurationType = motorType;
    }

    /**
     * Returns the underlying motor controller on which this motor is situated.
     *
     * @return the underlying motor controller on which this motor is situated.
     * @see #getPortNumber()
     */
    @Override
    public DcMotorController getController() {
        return null;
    }

    /**
     * Returns the port number on the underlying motor controller on which this motor is situated.
     *
     * @return the port number on the underlying motor controller on which this motor is situated.
     * @see #getController()
     */
    @Override
    public int getPortNumber() {
        return -1;
    }
    ZeroPowerBehavior zeroPowerBehavior=ZeroPowerBehavior.UNKNOWN;
    /**
     * Sets the behavior of the motor when a power level of zero is applied.
     *
     * @param zeroPowerBehavior the new behavior of the motor when a power level of zero is applied.
     * @see ZeroPowerBehavior
     * @see #setPower(double)
     */
    @Override
    public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {
        this.zeroPowerBehavior = zeroPowerBehavior;
    }

    /**
     * Returns the current behavior of the motor were a power level of zero to be applied.
     *
     * @return the current behavior of the motor were a power level of zero to be applied.
     */
    @Override
    public ZeroPowerBehavior getZeroPowerBehavior() {
        return zeroPowerBehavior;
    }

    /**
     * Sets the zero power behavior of the motor to {@link ZeroPowerBehavior#FLOAT FLOAT}, then
     * applies zero power to that motor.
     *
     * <p>Note that the change of the zero power behavior to {@link ZeroPowerBehavior#FLOAT FLOAT}
     * remains in effect even following the return of this method. <STRONG>This is a breaking
     * change</STRONG> in behavior from previous releases of the SDK. Consider, for example, the
     * following code sequence:</p>
     *
     * <pre>
     *     motor.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE); // method not available in previous releases
     *     motor.setPowerFloat();
     *     motor.setPower(0.0);
     * </pre>
     *
     * <p>Starting from this release, this sequence of code will leave the motor floating. Previously,
     * the motor would have been left braked.</p>
     *
     * @see #setPower(double)
     * @see #getPowerFloat()
     * @see #setZeroPowerBehavior(ZeroPowerBehavior)
     * @deprecated This method is deprecated in favor of direct use of
     * {@link #setZeroPowerBehavior(ZeroPowerBehavior) setZeroPowerBehavior()} and
     * {@link #setPower(double) setPower()}.
     */
    @Override
    public void setPowerFloat() {
        zeroPowerBehavior = ZeroPowerBehavior.FLOAT;
    }

    /**
     * Returns whether the motor is currently in a float power level.
     *
     * @return whether the motor is currently in a float power level.
     * @see #setPowerFloat()
     */
    @Override
    public boolean getPowerFloat() {
        return zeroPowerBehavior==ZeroPowerBehavior.FLOAT;
    }
    Integer targetPosition=null;

    /**
     * Sets the desired encoder target position to which the motor should advance or retreat
     * and then actively hold thereat. This behavior is similar to the operation of a servo.
     * The maximum speed at which this advance or retreat occurs is governed by the power level
     * currently set on the motor. While the motor is advancing or retreating to the desired
     * taget position, {@link #isBusy()} will return true.
     *
     * <p>Note that adjustment to a target position is only effective when the motor is in
     * {@link RunMode#RUN_TO_POSITION RUN_TO_POSITION}
     * RunMode. Note further that, clearly, the motor must be equipped with an encoder in order
     * for this mode to function properly.</p>
     *
     * @param position the desired encoder target position
     * @see #getCurrentPosition()
     * @see #setMode(RunMode)
     * @see RunMode#RUN_TO_POSITION
     * @see #getTargetPosition()
     * @see #isBusy()
     */
    @Override
    public void setTargetPosition(int position) {
        targetPosition = position;
    }

    /**
     * Returns the current target encoder position for this motor.
     *
     * @return the current target encoder position for this motor.
     * @see #setTargetPosition(int)
     */
    @Override
    public int getTargetPosition() {
        return targetPosition;
    }

    /**
     * Returns true if the motor is currently advancing or retreating to a target position.
     *
     * @return true if the motor is currently advancing or retreating to a target position.
     * @see #setTargetPosition(int)
     */
    @Override
    public boolean isBusy() {
        return getTargetPosition()!=getCurrentPosition();
    }
    int currentPosition=0;
    /**
     * Returns the current reading of the encoder for this motor. The units for this reading,
     * that is, the number of ticks per revolution, are specific to the motor/encoder in question,
     * and thus are not specified here.
     *
     * @return the current reading of the encoder for this motor
     * @see #getTargetPosition()
     * @see RunMode#STOP_AND_RESET_ENCODER
     */
    @Override
    public int getCurrentPosition() {
        return currentPosition;
    }
    RunMode runMode = RunMode.RUN_WITHOUT_ENCODER;
    /**
     * Sets the current run mode for this motor
     *
     * @param mode the new current run mode for this motor
     * @see RunMode
     * @see #getMode()
     */
    @Override
    public void setMode(RunMode mode) {
        runMode = mode;
        switch (runMode) {
            case RUN_WITHOUT_ENCODER:
            case RUN_USING_ENCODER:
                break;
            case RUN_TO_POSITION:
                currentPosition = (int)(1.0*(targetPosition-currentPosition)/2 + currentPosition);
                InstanceTelemetry.getInstance().addLine(
                        "FakeDcMotor Power: "+power+
                                "\nRunMode: "+runMode.toString()+
                                "\nCurrentPosition: "+currentPosition+
                                "\nTargetPosition: "+targetPosition+
                                "\nDirection: "+direction.toString());
                break;
            case STOP_AND_RESET_ENCODER:
                currentPosition = 0;
                targetPosition = null;
                break;
        }
    }

    /**
     * Returns the current run mode for this motor
     *
     * @return the current run mode for this motor
     * @see RunMode
     * @see #setMode(RunMode)
     */
    @Override
    public RunMode getMode() {
        return runMode;
    }
    Direction direction=Direction.FORWARD;
    /**
     * Sets the logical direction in which this motor operates.
     *
     * @param direction the direction to set for this motor
     * @see #getDirection()
     */
    @Override
    public void setDirection(Direction direction) {
        this.direction = direction;
    }

    /**
     * Returns the current logical direction in which this motor is set as operating.
     *
     * @return the current logical direction in which this motor is set as operating.
     * @see #setDirection(Direction)
     */
    @Override
    public Direction getDirection() {
        return direction;
    }
    double power=0;
    long setPowerTime=0;
    /**
     * Sets the power level of the motor, expressed as a fraction of the maximum
     * possible power / speed supported according to the run mode in which the
     * motor is operating.
     *
     * <p>Setting a power level of zero will brake the motor</p>
     *
     * @param power the new power level of the motor, a value in the interval [-1.0, 1.0]
     * @see #getPower()
     * @see FakeDcMotor#setMode(RunMode)
     * @see FakeDcMotor#setPowerFloat()
     */
    @Override
    public void setPower(double power) {
        currentPosition = (int)(currentPosition+power*(System.currentTimeMillis()-setPowerTime));
        this.power=power;
        setPowerTime = System.currentTimeMillis();
        InstanceTelemetry.getInstance().addLine(
                "FakeDcMotor Power: "+power+
                        "\nRunMode: "+runMode.toString()+
                        "\nCurrentPosition: "+currentPosition+
                        "\nTargetPosition: "+targetPosition+
                        "\nDirection: "+direction.toString());

    }

    /**
     * Returns the current configured power level of the motor.
     *
     * @return the current level of the motor, a value in the interval [0.0, 1.0]
     * @see #setPower(double)
     */
    @Override
    public double getPower() {
        return power;
    }

    Manufacturer manufacturer = Manufacturer.GoBilda;
    /**
     * Returns an indication of the manufacturer of this device.
     *
     * @return the device's manufacturer
     */
    @Override
    public Manufacturer getManufacturer() {
        return manufacturer;
    }

    /**
     * Returns a string suitable for display to the user as to the type of device.
     * Note that this is a device-type-specific name; it has nothing to do with the
     * name by which a user might have configured the device in a robot configuration.
     *
     * @return device manufacturer and name
     */
    @Override
    public String getDeviceName() {
        return "FakeDcMotor";
    }

    /**
     * Get connection information about this device in a human readable format
     *
     * @return connection info
     */
    @Override
    public String getConnectionInfo() {
        return "Fake Connection Info";
    }

    /**
     * Version
     *
     * @return get the version of this device
     */
    @Override
    public int getVersion() {
        return 0;
    }

    /**
     * Resets the device's configuration to that which is expected at the beginning of an OpMode.
     * For example, motors will reset the their direction to 'forward'.
     */
    @Override
    public void resetDeviceConfigurationForOpMode() {
        this.targetPosition=null;
        this.runMode= RunMode.RUN_WITHOUT_ENCODER;
        this.direction= Direction.FORWARD;
        this.power=0;
        this.currentPosition=0;
    }

    /**
     * Closes this device
     */
    @Override
    public void close() {
    }
}
