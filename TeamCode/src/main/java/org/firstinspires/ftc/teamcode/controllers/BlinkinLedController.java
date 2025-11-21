package org.firstinspires.ftc.teamcode.controllers;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class BlinkinLedController {


    private RevBlinkinLedDriver blinkinLedDriver;
    private RevBlinkinLedDriver.BlinkinPattern currentPattern;

    public BlinkinLedController(HardwareMap hardwareMap) {
        this.blinkinLedDriver  = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        this.currentPattern = RevBlinkinLedDriver.BlinkinPattern.BLACK;
        blinkinLedDriver.setPattern(currentPattern);
    }

    public void showRedTeam() {
        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_RED);
    }

    public void showBlueTeam() {
        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_BLUE);
    }

    public void showReady() {
        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
    }

    public void showError() {
        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED);
    }

    public void setColor(RevBlinkinLedDriver.BlinkinPattern COLOR) {
        blinkinLedDriver.setPattern(COLOR);
    }

    public void setColornext(){
        currentPattern = currentPattern.next();
        blinkinLedDriver.setPattern(currentPattern);
    }
    public void setColorprevious(){
        currentPattern = currentPattern.previous();
        blinkinLedDriver.setPattern(currentPattern);
    }

    public void setPattern(RevBlinkinLedDriver.BlinkinPattern pattern) {
        this.currentPattern = pattern;
        blinkinLedDriver.setPattern(pattern);
    }

    public RevBlinkinLedDriver.BlinkinPattern getCurrentPattern() {
        return currentPattern;
    }

    public void turnOff() {
        setColor(RevBlinkinLedDriver.BlinkinPattern.BLACK);
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