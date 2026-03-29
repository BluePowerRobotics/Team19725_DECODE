package org.firstinspires.ftc.teamcode.controllers.shooter.DRL;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
// Android平台不支持java.awt和javax.swing，注释掉可视化相关的import
// import java.awt.Color;
// import java.awt.Font;
// import java.awt.Graphics;
// import javax.swing.*;
/*
simulator类，实现运动中的小车发射带有空气阻力的小球的模拟器{
    set_params函数，设置小车发射高度h，小球质量m，空气阻力系数k
    launch(v,d){
        输入小车当前速度v（二维向量），
        发射器参数d（三维向量，d=|v初|[cos(t)sin(a),cos(t)cos(a),sin(t)]），
        输出落点x（二维向量，从发射时的小车指向球的落点)
    }
    visualize(v,d)函数，生成一个窗口，实时渲染发射模拟
}
此程序不依赖任何其他程序
Environment.java依赖于此程序
 */

public class Simulator {
    private double h; // 小车发射高度
    private double m; // 小球质量
    private double k; // 空气阻力系数
    private double theta; // d与水平面的夹角（弧度）
    private final double g = 9.81; // 重力加速度
    private final double dt = 0.01; // 时间步长
    
    // 构造函数
    public Simulator() {
        // 默认参数
        this.h = 0.1;
        this.m = 0.1;
        this.k = 0.01;
    }
    
    // 设置参数
    public void set_params(double h, double m, double k) {
        this.h = h;
        this.m = m;
        this.k = k;
    }
    
    // 发射函数
    public double[] launch(double[] v, double[] d) {
        // v: 小车当前速度 (二维向量)
        // d: 发射器参数 (三维向量)
        
        // 计算d与水平面的夹角theta（弧度）
        double d_mag = Math.sqrt(d[0]*d[0] + d[1]*d[1] + d[2]*d[2]);
        if (d_mag > 0) {
            theta = Math.acos(d[2] / d_mag);
        } else {
            theta = 0;
        }
        
        // 初始化位置和速度
        double[] position = new double[3]; // [x, y, z]
        position[2] = h; // 初始高度
        
        double[] velocity = new double[3];
        velocity[0] = d[0] + v[0]; // 初速度x分量（考虑小车速度）
        velocity[1] = d[1] + v[1]; // 初速度y分量（考虑小车速度）
        velocity[2] = d[2]; // 初速度z分量
        
        // 模拟运动并计算时间
        double t = 0;
        while (position[2] >= 0) {
            // 计算速度大小
            double v_mag = Math.sqrt(velocity[0]*velocity[0] + velocity[1]*velocity[1] + velocity[2]*velocity[2]);
            
            // 计算空气阻力加速度
            double[] a_drag = new double[3];
            if (v_mag > 0) {
                double drag_mag = k * v_mag * v_mag / m;
                a_drag[0] = -drag_mag * velocity[0] / v_mag;
                a_drag[1] = -drag_mag * velocity[1] / v_mag;
                a_drag[2] = -drag_mag * velocity[2] / v_mag;
            }
            
            // 计算总加速度（重力 + 空气阻力）
            double[] acceleration = new double[3];
            acceleration[0] = a_drag[0];
            acceleration[1] = a_drag[1];
            acceleration[2] = a_drag[2] - g;
            
            // 更新速度
            velocity[0] += acceleration[0] * dt;
            velocity[1] += acceleration[1] * dt;
            velocity[2] += acceleration[2] * dt;
            
            // 更新位置
            position[0] += velocity[0] * dt;
            position[1] += velocity[1] * dt;
            position[2] += velocity[2] * dt;
            
            // 更新时间
            t += dt;
        }
        
        // 计算小车在时间t内的移动距离
        double car_move_x = v[0] * t;
        double car_move_y = v[1] * t;
        
        // 计算相对于发射时的小车位置的落点位置
        double[] x = new double[2];
        x[0] = position[0];
        x[1] = position[1];
        
        return x;
    }
    
    // 可视化函数 - Android平台不支持，已注释
    /*
    public void visualize(double[] v, double[] d) {
        JFrame frame = new JFrame("发射模拟");
        frame.setSize(800, 600);
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        
        SimulationPanel panel = new SimulationPanel(this, v, d);
        frame.add(panel);
        
        frame.setVisible(true);
        
        // 启动模拟线程
        Thread simulationThread = new Thread(() -> {
            while (true) {
                panel.update();
                try {
                    Thread.sleep((int)(dt * 1000));
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        });
        simulationThread.start();
    }
    */
    
    // 内部类：模拟面板 - Android平台不支持，已注释
    /*
    class SimulationPanel extends JPanel {
        private Simulator simulator;
        private double[] v;
        private double[] d;
        private double[] position;
        private double[] velocity;
        private double car_x;
        private double car_y;
        private boolean running;
        private List<double[]> trajectory; // 记录小球轨迹
        
        public SimulationPanel(Simulator simulator, double[] v, double[] d) {
            this.simulator = simulator;
            this.v = v;
            this.d = d;
            this.position = new double[3];
            this.position[2] = h;
            this.velocity = new double[3];
            this.velocity[0] = d[0] + v[0];
            this.velocity[1] = d[1] + v[1];
            this.velocity[2] = d[2];
            this.car_x = 100; // 初始位置
            this.car_y = 300; // 初始位置
            this.running = true;
            this.trajectory = new ArrayList<>(); // 初始化轨迹列表
        }
        
        public void update() {
            if (!running) return;
            
            // 计算速度大小
            double v_mag = Math.sqrt(velocity[0]*velocity[0] + velocity[1]*velocity[1] + velocity[2]*velocity[2]);
            
            // 计算空气阻力加速度
            double[] a_drag = new double[3];
            if (v_mag > 0) {
                double drag_mag = k * v_mag * v_mag / m;
                a_drag[0] = -drag_mag * velocity[0] / v_mag;
                a_drag[1] = -drag_mag * velocity[1] / v_mag;
                a_drag[2] = -drag_mag * velocity[2] / v_mag;
            }
            
            // 计算总加速度（重力 + 空气阻力）
            double[] acceleration = new double[3];
            acceleration[0] = a_drag[0];
            acceleration[1] = a_drag[1];
            acceleration[2] = a_drag[2] - g;
            
            // 更新速度
            velocity[0] += acceleration[0] * dt;
            velocity[1] += acceleration[1] * dt;
            velocity[2] += acceleration[2] * dt;
            
            // 更新位置
            position[0] += velocity[0] * dt;
            position[1] += velocity[1] * dt;
            position[2] += velocity[2] * dt;
            
            // 小车移动（y轴方向调整，使v[1]为正时向上移动）
            int scale = 20; // 缩放因子，1个单位对应20像素
            car_x += v[0] * dt * scale;
            car_y -= v[1] * dt * scale;
            
            // 记录小球轨迹
            if (running) {
                double[] point = {position[0], position[1], position[2]};
                trajectory.add(point);
            }
            
            // 检查是否落地
            if (position[2] < 0) {
                running = false;
            }
            
            repaint();
        }
        
        @Override
        protected void paintComponent(Graphics g) {
            super.paintComponent(g);
            
            // 缩放因子，1个单位对应20像素
            int scale = 20;
            
            // 绘制背景
            g.setColor(Color.WHITE);
            g.fillRect(0, 0, getWidth(), getHeight());
            
            // 绘制网格（俯视视角，全屏幕显示30个格子）
            g.setColor(Color.LIGHT_GRAY);
            // 计算网格间隔：窗口高度600像素，30个格子，每个格子20像素
            int grid_size = scale; // 每个格子20像素，对应1个单位
            
            for (int i = 0; i < getWidth(); i += grid_size) {
                g.drawLine(i, 0, i, getHeight());
                // 每5个单位添加一个刻度
                if (i % (5 * grid_size) == 0) {
                    g.setColor(Color.BLACK);
                    g.setFont(new Font("Arial", Font.PLAIN, 10));
                    int x_coord = (i - 100) / grid_size; // 100是小车初始x位置
                    g.drawString(String.valueOf(x_coord), i - 5, 30);
                    g.setColor(Color.LIGHT_GRAY);
                }
            }
            for (int i = 0; i < getHeight(); i += grid_size) {
                g.drawLine(0, i, getWidth(), i);
                // 每5个单位添加一个刻度
                if (i % (5 * grid_size) == 0) {
                    g.setColor(Color.BLACK);
                    g.setFont(new Font("Arial", Font.PLAIN, 10));
                    int y_coord = (300 - i) / grid_size; // 300是小车初始y位置
                    g.drawString(String.valueOf(y_coord), 10, i + 15);
                    g.setColor(Color.LIGHT_GRAY);
                }
            }
            
            // 绘制坐标轴
            g.setColor(Color.RED);
            g.drawLine(100, 0, 100, getHeight()); // y轴（小车初始x位置）
            g.drawLine(0, 300, getWidth(), 300); // x轴（小车初始y位置）
            
            // 绘制轨迹（俯视视角，相对于小车位置）
            g.setColor(Color.GRAY);
            for (int i = 0; i < trajectory.size() - 1; i++) {
                double[] point1 = trajectory.get(i);
                double[] point2 = trajectory.get(i + 1);
                // 计算相对于原点（100, 300）的位置，y轴方向调整
                int x1 = (int)(100 + point1[0] * scale);
                int y1 = (int)(300 - point1[1] * scale);
                int x2 = (int)(100 + point2[0] * scale);
                int y2 = (int)(300 - point2[1] * scale);
                g.drawLine(x1, y1, x2, y2);
            }
            
            // 绘制小车
            g.setColor(Color.BLUE);
            g.fillRect((int)(car_x - 10), (int)(car_y - 10), 20, 20);
            
            // 绘制小球（俯视视角：x和y坐标直接映射，相对于原点固定）
            if (running) {
                g.setColor(Color.RED);
                // 计算相对于原点（100, 300）的位置，y轴方向调整
                int ball_x = (int)(100 + position[0] * scale);
                int ball_y = (int)(300 - position[1] * scale);
                g.fillOval(ball_x - 5, ball_y - 5, 10, 10);
                
                // 在小球旁显示高度
                g.setColor(Color.BLACK);
                g.setFont(new Font("Arial", Font.PLAIN, 10));
                g.drawString("高度: " + String.format("%.2f", position[2]), ball_x + 10, ball_y - 10);
            }
            
            // 显示速度信息
            g.setColor(Color.BLACK);
            g.setFont(new Font("Arial", Font.PLAIN, 12));
            
            // 小车速度
            g.drawString("小车速度: v_x = " + String.format("%.2f", v[0]) + ", v_y = " + String.format("%.2f", v[1]), 20, 30);
            
            // 小球速度
            if (running) {
                double ball_speed_mag = Math.sqrt(velocity[0]*velocity[0] + velocity[1]*velocity[1] + velocity[2]*velocity[2]);
                g.drawString("小球速度: v_x = " + String.format("%.2f", velocity[0]) + ", v_y = " + String.format("%.2f", velocity[1]) + ", v_z = " + String.format("%.2f", velocity[2]), 20, 50);
                g.drawString("小球速度大小: " + String.format("%.2f", ball_speed_mag), 20, 70);
            }
        }
    }
    */
    
    // 主函数用于测试 - Android平台不支持可视化，已注释
    /*
    public static void main(String[] args) throws IOException, ClassNotFoundException {
        Simulator simulator = new Simulator();
        
        // 设置参数
        simulator.set_params(0.1, 0.1, 0.01);
        
        // 测试发射
        double[] v = {-2.0, -1.0}; // 小车速度
        double[] d = {7.549210641294196, 12.234507543968233, 14.135838137283258}; // 发射器参数
        
        double[] x = simulator.launch(v, d);
        System.out.println("落点位置: (" + x[0] + ", " + x[1] + ")");
        
        // 可视化
        simulator.visualize(v, d);
    }
    */
}