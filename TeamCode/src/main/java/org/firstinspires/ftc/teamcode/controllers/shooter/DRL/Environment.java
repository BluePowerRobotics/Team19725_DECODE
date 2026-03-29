package org.firstinspires.ftc.teamcode.controllers.shooter.DRL;
import java.io.*;
import java.util.*;
/*
Environment类，包括:
    参数m, k，二维矩阵database
    set_params函数，设定m,k
    get_data函数，从database中获取数据
    load_data函数，从.csv文件导入数据，存入database
    simulate函数，输入模拟次数n，最大小车速度v_max, 最大小球初速度d_max，噪声水平s，使用simulator类模拟n次，每次随机选取v,d(模长不超过v_max,d_max，方向随机），得到x，按（v, x, d)的顺序将数据存入database
    save_data函数，将database存入.csv文件
    record函数，用于重载实验
此程序依赖Simulator类
Agent.java依赖于此程序
 */
public class Environment {
    private int state_dim;
    private double h;
    private double m; // 小球质量
    private double k; // 空气阻力系数
    private double theta; // d与水平面的夹角（弧度）
    private double[] state = new double[state_dim]; // 7个状态变量：v[0], v[1], x[0], x[1], d[0], d[1], d[2]
    private List<double[]> database; // 二维矩阵，存储数据
    
    // 构造函数
    public Environment() {
        this.h=0.1;
        this.m = 0.1; // 默认值
        this.k = 0.01; // 默认值
        this.theta = Math.PI / 4; // 默认夹角为45度（π/4弧度）
        this.database = new ArrayList<>();
    }
    
    public List<double[]> get_data(){
        return this.database;
    }

    public double[] get_state(){
        return this.state;
    }

    public int get_state_dim(){
        return this.state_dim;
    }
    public void set_state(double[] state){
        this.state = state;
    }
    //可重载，根据action计算奖励并更新状态
    public double reward(double[] action){
        return 0;
    }

    public void reset(){
        // 随机选择一条数据作为初始状态
        if(state_dim == 0){
            System.out.println("环境状态维度为0，无法训练");
            return;
        }
        List<double[]> data = database;
        if (data.isEmpty()) {
            System.out.println("环境数据库为空，无法训练");
            return;
        }
        Random random = new Random();
        int startIndex = random.nextInt(data.size());
        this.state = Arrays.copyOf(data.get(startIndex), state_dim);
    }
    // 设置参数
    public void set_params(double h, double m, double k, double theta) {
        this.h = h;
        this.m = m;
        this.k = k;
        this.theta = theta;
    }

    public void set_params(double h, double m, double k, double theta, int state_dim, double[] state) {
        this.state_dim = state_dim;
        this.state = state;
        this.h = h;
        this.m = m;
        this.k = k;
        this.theta = theta;
    }
    
    public double get_h(){
        return this.h;
    }
    public double get_m(){
        return this.m;
    }
    public double get_k(){
        return this.k;
    }
    public double get_theta(){
        return this.theta;
    }
    // 重载set_params方法，保持向后兼容
    public void set_params(double h, double m, double k) {
        this.h = h;
        this.m = m;
        this.k = k;
    }
    
    // 从.csv文件导入数据
    public void load_data(String filename) throws IOException {
        database.clear(); // 清空原有数据
        
        try (BufferedReader br = new BufferedReader(new FileReader(filename))) {
            String line;
            br.readLine(); // 跳过表头
            while ((line = br.readLine()) != null) {
                String[] values = line.split(",");
                double[] data = new double[7]; // v[0], v[1], x[0], x[1], d[0], d[1], d[2]
                for (int i = 0; i < 7; i++) {
                    data[i] = Double.parseDouble(values[i]);
                }
                database.add(data);
            }
        }
        
        System.out.println("数据已从 " + filename + " 导入，共 " + database.size() + " 条记录");
    }
    
    // 模拟n次，生成数据，添加噪声
    public void simulate(int n, double v_max, double d_max, double s) {
        Simulator simulator = new Simulator();
        simulator.set_params(h, m, k); // 设置小车发射高度为0.1，质量和阻力系数使用环境参数
        for (int i = 0; i < n; i++) {
            record(simulator, v_max, d_max, s);
        }
        
        System.out.println("模拟完成，共生成 " + n + " 条记录，噪声方差: " + s);
    }
    
    // 模拟n次，生成数据（无噪声版本）
    public void simulate(int n, double v_max, double d_max) {
        simulate(n, v_max, d_max, 0);
    }
    //用于实验重载
    public void record(Simulator simulator, double v_max, double d_max, double s){
        Random random = new Random();
        // 随机生成小车速度v（模长不超过v_max，方向随机）
        double v_mag = random.nextDouble() * v_max;
        double v_theta = random.nextDouble() * 2 * Math.PI;
        double[] v = new double[2];
        v[0] = v_mag * Math.cos(v_theta);
        v[1] = v_mag * Math.sin(v_theta);
        
        // 随机生成小球初速度d（模长不超过d_max，方位角随机，仰角固定为theta）
        double d_mag = random.nextDouble() * d_max;
        double d_phi = random.nextDouble() * 2 * Math.PI; // 方位角，0到2π
        double[] d = new double[3];
        d[0] = d_mag * Math.sin(theta) * Math.cos(d_phi);
        d[1] = d_mag * Math.sin(theta) * Math.sin(d_phi);
        d[2] = d_mag * Math.cos(theta);
            
        // 模拟发射，得到落点x
        double[] x = simulator.launch(v, d);
            
        // 添加正态分布噪声，方差为s
        x[0] += random.nextGaussian() * Math.sqrt(s);
        x[1] += random.nextGaussian() * Math.sqrt(s);
            
        // 按（v, x, d)的顺序将数据存入database
        double[] data = new double[7];
        data[0] = v[0];
        data[1] = v[1];
        data[2] = x[0];
        data[3] = x[1];
        data[4] = d[0];
        data[5] = d[1];
        data[6] = d[2];
            
        database.add(data);
        return;
    }
    // 将database存入.csv文件
    public void save_data(String filename) throws IOException {
        try (BufferedWriter bw = new BufferedWriter(new FileWriter(filename))) {
            // 写入表头
            bw.write("v_x,v_y,x_x,x_y,d_x,d_y,d_z");
            bw.newLine();
            
            // 写入数据
            for (double[] data : database) {
                for (int i = 0; i < data.length; i++) {
                    bw.write(String.valueOf(data[i]));
                    if (i < data.length - 1) {
                        bw.write(",");
                    }
                }
                bw.newLine();
            }
        }
        
        System.out.println("数据已保存到 " + filename + "，共 " + database.size() + " 条记录");
    }
    
    // 主函数用于测试
    public static void main(String[] args) throws IOException {
        Environment env = new Environment();
        
        // 设置参数，包括theta（45度，即π/4弧度）
        env.set_params(0.1, 0.1, 0.01, Math.PI / 4);
        System.out.println("环境参数设置：");
        System.out.println("  发射高度h: 0.1");
        System.out.println("  小球质量m: 0.1");
        System.out.println("  空气阻力系数k: 0.01");
        System.out.println("  发射夹角theta: 45度 (π/4弧度)");
        System.out.println();
        
        // 模拟100次，最大小车速度5，最大小球初速度20，噪声方差0.1
        env.simulate(100, 5, 20, 0.1);
        
        // 保存数据
        env.save_data("simulation_data_with_noise.csv");
        
        // 导入数据
        env.load_data("simulation_data_with_noise.csv");
        List<double[]> data=env.get_data();
        System.out.println("环境数据 (v_x, v_y, x_x, x_y, d_x, d_y, d_z):");
        System.out.println("==================================================");
        
        for (int i = 0; i < data.size(); i++) {
            double[] record = data.get(i);
            System.out.printf("记录 %d: ", i+1);
            for (int j = 0; j < record.length; j++) {
                System.out.printf("%.4f", record[j]);
                if (j < record.length - 1) {
                    System.out.print(", ");
                }
            }
            System.out.println();
        }
        
        System.out.println("==================================================");
        System.out.println("共 " + data.size() + " 条记录");
    }
}