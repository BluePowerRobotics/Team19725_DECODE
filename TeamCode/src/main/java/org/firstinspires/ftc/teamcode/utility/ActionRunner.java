package org.firstinspires.ftc.teamcode.utility;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import java.util.LinkedList;
import java.util.Queue;

public class ActionRunner {
    // 待执行Action队列
    private final Queue<Action> pendingActions = new LinkedList<>();
    // 当前正在执行的Action
    private Action currentAction = null;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    /**
     * 添加一个要执行的 Action（串行队列）。
     */
    public void add(Action action) {
        if (action != null) {
            pendingActions.add(action);
        }
    }

    /**
     * 每帧更新当前Action，完成后自动取下一个。
     * 应当在 OpMode中循环调用。
     */
    public void update() {
        TelemetryPacket packet = new TelemetryPacket();

        // 如果没有当前Action，从队列取下一个
        if (currentAction == null && !pendingActions.isEmpty()) {
            currentAction = pendingActions.poll();
        }

        // 执行当前Action
        if (currentAction != null) {
            currentAction.preview(packet.fieldOverlay());
            boolean running = currentAction.run(packet);
            if (!running) {
                currentAction = null; // 执行完毕，准备下一个
            }
        }
        dashboard.sendTelemetryPacket(packet);
    }

    /**
     * 是否仍有动作在执行或队列中。
     */
    public boolean isBusy() {
        return currentAction != null || !pendingActions.isEmpty();
    }

    public Action getCurrentAction() {
        return currentAction;
    }

    /**
     * 停止所有动作（清空队列和当前Action）。
     */
    public void clear() {
        pendingActions.clear();
        currentAction = null;
    }
}
