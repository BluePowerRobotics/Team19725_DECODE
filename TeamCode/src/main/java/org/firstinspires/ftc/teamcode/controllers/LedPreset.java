package org.firstinspires.ftc.teamcode.controllers;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

/**
 * Enum wrapper around RevBlinkinLedDriver.BlinkinPattern providing a smaller set of presets
 * and helper methods (next/previous).
 */
public enum LedPreset {
    BLACK(RevBlinkinLedDriver.BlinkinPattern.BLACK),
    RED(RevBlinkinLedDriver.BlinkinPattern.RED),
    BLUE(RevBlinkinLedDriver.BlinkinPattern.BLUE),
    GREEN(RevBlinkinLedDriver.BlinkinPattern.GREEN),
    YELLOW(RevBlinkinLedDriver.BlinkinPattern.YELLOW),
    ORANGE(RevBlinkinLedDriver.BlinkinPattern.ORANGE),
    WHITE(RevBlinkinLedDriver.BlinkinPattern.WHITE),

    STROBE_RED(RevBlinkinLedDriver.BlinkinPattern.STROBE_RED),
    STROBE_BLUE(RevBlinkinLedDriver.BlinkinPattern.STROBE_BLUE),

    HEARTBEAT_RED(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED),
    HEARTBEAT_BLUE(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_BLUE),

    RAINBOW_RAINBOW_PALETTE(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE),
    RAINBOW_PARTY_PALETTE(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_PARTY_PALETTE);

    private final RevBlinkinLedDriver.BlinkinPattern pattern;

    LedPreset(RevBlinkinLedDriver.BlinkinPattern pattern) {
        this.pattern = pattern;
    }

    public RevBlinkinLedDriver.BlinkinPattern getPattern() {
        return pattern;
    }

    public LedPreset next() {
        LedPreset[] vals = values();
        int nextOrdinal = (this.ordinal() + 1) % vals.length;
        return vals[nextOrdinal];
    }

    public LedPreset previous() {
        LedPreset[] vals = values();
        int prevOrdinal = (this.ordinal() - 1 + vals.length) % vals.length;
        return vals[prevOrdinal];
    }
}

