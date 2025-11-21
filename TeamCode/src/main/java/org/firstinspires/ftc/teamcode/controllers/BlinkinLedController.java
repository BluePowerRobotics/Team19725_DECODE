package org.firstinspires.ftc.teamcode.controllers;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class BlinkinLedController {


    private RevBlinkinLedDriver blinkinLedDriver;
    private LedPreset currentPreset;

    public BlinkinLedController(HardwareMap hardwareMap) {
        this.blinkinLedDriver  = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        this.currentPreset = LedPreset.BLACK;
        blinkinLedDriver.setPattern(currentPreset.getPattern());
    }



    public void showRedTeam() {
        setPreset(LedPreset.STROBE_RED);


    }

    public void showBlueTeam() {
        setPreset(LedPreset.STROBE_BLUE);
    }

    public void showReady() {
        setPreset(LedPreset.GREEN);
    }

    public void showError() {
        setPreset(LedPreset.HEARTBEAT_RED);
    }

    /**
     * Compatibility method: set pattern using the raw RevBlinkinLedDriver.BlinkinPattern.
     * Also attempts to update the currentPreset if a matching LedPreset exists.
     */
    public void setColor(RevBlinkinLedDriver.BlinkinPattern COLOR) {
        blinkinLedDriver.setPattern(COLOR);
        // try to map back to a LedPreset when possible
        for (LedPreset p : LedPreset.values()) {
            if (p.getPattern() == COLOR) {
                currentPreset = p;
                return;
            }
        }
        // if no mapping found, clear currentPreset
        currentPreset = null;
    }

    /**
     * Set by enum preset
     */
    public void setPreset(LedPreset preset) {
        this.currentPreset = preset;
        blinkinLedDriver.setPattern(preset.getPattern());
    }

    public void setNextPreset(){
        if (currentPreset != null) {
            currentPreset = currentPreset.next();
            blinkinLedDriver.setPattern(currentPreset.getPattern());
        }
    }
    public void setPreviousPreset(){
        if (currentPreset != null) {
            currentPreset = currentPreset.previous();
            blinkinLedDriver.setPattern(currentPreset.getPattern());
        }
    }

    public LedPreset getCurrentPreset() {
        return currentPreset;
    }

    public RevBlinkinLedDriver.BlinkinPattern getCurrentPattern() {
        return currentPreset == null ? RevBlinkinLedDriver.BlinkinPattern.BLACK : currentPreset.getPattern();
    }

    public void turnOff() {
        setPreset(LedPreset.BLACK);
    }
    /*
    固定颜色：RED BLUE GREEN YELLOW ORANGE WHITE BLACK  // 关闭
    闪烁效果：STROBE_RED STROBE_BLUE STROBE_GOLD STROBE_WHITE
    彩虹效果：RAINBOW_RAINBOW_PALETTE RAINBOW_PARTY_PALETTE
            RAINBOW_OCEAN_PALETTE   RAINBOW_FOREST_PALETTE
    心跳效果：HEARTBEAT_RED HEARTBEAT_BLUE HEARTBEAT_WHITE
    扫描效果：SINELON_RAINBOW SINELON_PARTY SINELON_OCEAN SINELON_FOREST
    流动效果：LARSON_SCANNER_RED LARSON_SCANNER_BLUE LARSON_SCANNER_GOLD
            LIGHT_CHASE_RED LI_BLUEGHT_CHASE LIGHT_CHASE_GOLD
    其他效果：CONFETTI FIREWORKS CPMETTI_SHOT FIRE_LARGE
            FIRE_MEDIUM FIRE_SMALL CANDLE FILLER SHOT_RED SHOT_BLUE
    */
}