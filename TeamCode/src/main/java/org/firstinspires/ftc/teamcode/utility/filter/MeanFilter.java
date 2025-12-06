package org.firstinspires.ftc.teamcode.utility.filter;

public class MeanFilter {
    private final int windowSize;
    private final double[] buffer;
    private int index = 0;
    private int count = 0; // 已接收样本数 (<= windowSize)
    private double sum = 0.0;

    public MeanFilter(int windowSize) {
        if (windowSize <= 0) {
            throw new IllegalArgumentException("windowSize must be > 0");
        }
        this.windowSize = windowSize;
        this.buffer = new double[windowSize];
    }

    /**
     * 添加一个新样本并返回当前均值。
     * - 在缓冲未满时，返回 sum / count（实际已接收样本数）。
     * - 在缓冲填满后，作为滑动窗口：减去最旧值，加入新值，再返回 sum / windowSize。
     */
    public double filter(double newValue) {
        if(!Double.isNaN(newValue)&&Double.isFinite(newValue)) {
            if (count < windowSize) {
                // 缓冲未满：直接追加
                buffer[index] = newValue;
                sum += newValue;
                count++;
                index = (index + 1) % windowSize;
                return sum / count;
            } else {
                // 缓冲已满：减去最旧值，替换为新值
                sum -= buffer[index];
                buffer[index] = newValue;
                sum += newValue;
                index = (index + 1) % windowSize;
                return sum / windowSize;
            }
        }else{
            return getMean();
        }
    }

    /** 重置滤波器（清空历史样本） */
    public void reset() {
        for (int i = 0; i < windowSize; i++) buffer[i] = 0.0;
        index = 0;
        count = 0;
        sum = 0.0;
    }

    /** 不添加样本，直接返回当前均值（尚未有样本时返回 0.0） */
    public double getMean() {
        if (count == 0) return 0.0;
        return sum / count;
    }

    public int getCount() { return count; }
    public int getWindowSize() { return windowSize; }
}
