package org.firstinspires.ftc.teamcode.controllers.shooter.DRL;

import java.io.IOException;
import java.io.Serializable;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Random;
/**
 * Agent 类 - 实现强化学习智能体
 * 
 * 功能说明：
 * 1. 支持 DDPG 策略和高斯策略两种学习方法
 * 2. 提供集成学习能力，提高预测稳定性
 * 3. 支持模型的保存和加载
 * 4. 提供随机行动作为网络未初始化时的 fallback
 * 
 * 主要方法：
 * - Agent(int i, int o)：构造函数，初始化智能体
 * - set_action_range(double[][] action_range)：设置行动范围
 * - show_params()：显示 agent 的基本信息
 * - save(String filename)：保存策略网络到文件
 * - load(String filename)：从文件加载策略网络
 * - decide(double[] input)：根据输入做出决策
 * - ensembleDecide(double[] input)：使用集成学习进行决策
 * - randomAction()：生成随机行动
 * - clipAndDiscretize(double[] output)：裁剪输出到有效范围并离散化
 * - DDPG_learn(Environment env, int inputDim, int outputDim, int batchSize, boolean earlystopping, int n)：使用 DDPG 策略训练
 * - Gaussian_learn(Environment env, int inputDim, int outputDim, int batchSize, boolean earlystopping, int n)：使用高斯策略训练
 * - calculateGaussianValidationLoss(Net meanNet, Net varianceNet, double[][] input, double[][] output)：计算高斯策略的验证损失
 * - calculateValidationLoss(Net net, double[][] input, double[][] output)：计算验证损失
 * - accuracy(double[] output, double[] target)：计算准确率
 * - meanSquaredError(double[] pred, double[] target)：计算均方误差
 * - RF_learn(Environment environment, int o, int n)：使用带基线的REINFORCE算法训练agent
 * - A2C_learn(Environment environment, int o, int n)：使用A2C算法训练agent
 * - TD3_learn(Environment environment, int o, int n)：使用TD3算法训练agent
 * - ARSQ_learn(Environment environment, int o, int n)：使用ARSQ算法训练agent
 * - main(String[] args)：测试主函数
 * 
 * 此程序依赖Net类，Environment类
 * TestLoadModel.java依赖于此程序
 */
public class Agent implements Serializable {
    private static final long serialVersionUID = 5242672284402048342L;
    private int i; // 观察数据维度
    private int o; // 行动数据维度
    private boolean[] Is_discrete; // 行动数据每个量是否只能是0或1，true表示只能是0或1，false表示可以是任意实数
    private double[][] action_range; // 行动数据每个量的最大值和最小值
    private Net policy_net; // 策略网络
    private List<Net> ensemble_nets; // 集成学习的网络集合
    
    /**
     * 构造函数 - 初始化智能体
     * @param i 观察数据维度
     * @param o 行动数据维度
     */
    public Agent(int i, int o) {
        this.i = i;
        this.o = o;
        this.Is_discrete = new boolean[o];
        this.action_range = new double[o][2];
        this.ensemble_nets = new ArrayList<>();
    }
    
    /**
     * 设置行动范围
     * @param action_range 行动数据每个量的最大值和最小值，格式为 [行动维度][2]，其中第二个维度为 [最小值, 最大值]
     */
    public void set_action_range(double[][] action_range) {
        this.action_range = action_range;
    }
    
    /**
     * 保存整个Agent对象到文件（包括策略网络和集成网络）
     * @param filename 保存文件路径
     * @throws IOException 文件操作异常
     */
    public void save(String filename) throws IOException {
        try (java.io.ObjectOutputStream oos = new java.io.ObjectOutputStream(new java.io.FileOutputStream(filename))) {
            oos.writeObject(this);
        }
        System.out.println("Agent对象已保存到 " + filename);
    }
    
    /**
     * 从文件加载整个Agent对象
     * @param filename 加载文件路径
     * @return 加载的Agent对象
     * @throws IOException 文件操作异常
     * @throws ClassNotFoundException 类加载异常
     */
    public static Agent load(String filename) throws IOException, ClassNotFoundException {
        try (java.io.ObjectInputStream ois = new java.io.ObjectInputStream(new java.io.FileInputStream(filename))) {
            return (Agent) ois.readObject();
        }
    }
    
    /**
     * 显示 agent 的基本信息
     */
    public void show_params() {
        System.out.println("========== Agent 参数信息 ==========");
        System.out.println("观察维度: " + i);
        System.out.println("行动维度: " + o);
        System.out.println("行动范围:");
        for (int j = 0; j < o; j++) {
            System.out.printf("  行动 %d: [%.2f, %.2f]%s\n", 
                    j, action_range[j][0], action_range[j][1], 
                    Is_discrete[j] ? " (离散)" : " (连续)");
        }
        
        if (policy_net != null) {
            System.out.println("策略网络信息:");
            System.out.println("  网络状态: 已初始化");
            System.out.println("  学习率: " + policy_net.getLearningRate());
            System.out.println("  L2正则化系数: " + policy_net.getL2Regularization());
        } else {
            System.out.println("策略网络信息:");
            System.out.println("  网络状态: 未初始化");
        }
        
        if (ensemble_nets != null && !ensemble_nets.isEmpty()) {
            System.out.println("集成学习信息:");
            System.out.println("  集成模型数量: " + ensemble_nets.size());
            for (int j = 0; j < ensemble_nets.size(); j++) {
                Net net = ensemble_nets.get(j);
                System.out.printf("  模型 %d: 层数=%d, 神经元=%s\n", 
                        j + 1, net.getLayerCount(), Arrays.toString(net.getLayerSizes()));
            }
        } else {
            System.out.println("集成学习信息: 无集成模型");
        }
        System.out.println("===================================");
    }
    
    /**
     * 根据输入做出决策
     * @param input 输入观察数据
     * @return 行动向量
     */
    public double[] decide(double[] input) {
        if (policy_net == null) {
            System.out.println("策略网络未初始化，返回随机行动");
            return randomAction();
        }
        
        // 使用集成学习进行预测
        if (ensemble_nets != null && !ensemble_nets.isEmpty()) {
            return ensembleDecide(input);
        } else {
            double[] output = policy_net.predict(input);
            return clipAndDiscretize(output);
        }
    }
    
    /**
     * 使用集成学习进行决策
     * @param input 输入观察数据
     * @return 集成后的行动向量
     */
    private double[] ensembleDecide(double[] input) {
        double[] ensembleOutput = new double[o];
        
        // 对所有集成模型的预测结果取平均值
        for (Net net : ensemble_nets) {
            double[] output = net.predict(input);
            for (int j = 0; j < o; j++) {
                ensembleOutput[j] += output[j];
            }
        }
        
        // 计算平均值
        for (int j = 0; j < o; j++) {
            ensembleOutput[j] /= ensemble_nets.size();
        }
        
        return clipAndDiscretize(ensembleOutput);
    }
    
    /**
     * 生成随机行动
     * @return 随机行动向量
     */
    private double[] randomAction() {
        double[] action = new double[o];
        Random random = new Random();
        
        for (int j = 0; j < o; j++) {
            double min = action_range[j][0];
            double max = action_range[j][1];
            action[j] = min + random.nextDouble() * (max - min);
            if (Is_discrete[j]) {
                action[j] = Math.round(action[j]);
            }
        }
        return action;
    }
    
    /**
     * 裁剪输出到有效范围并离散化
     * @param output 原始输出向量
     * @return 裁剪和离散化后的行动向量
     */
    private double[] clipAndDiscretize(double[] output) {
        double[] clipped = new double[o];
        
        for (int j = 0; j < o; j++) {
            double min = action_range[j][0];
            double max = action_range[j][1];
            
            // 裁剪到范围
            clipped[j] = Math.max(min, Math.min(max, output[j]));
            
            // 离散化
            if (Is_discrete[j]) {
                clipped[j] = Math.round(clipped[j]);
            }
        }
        return clipped;
    }
    
    /**
     * 使用 DDPG 策略训练
     * @param env 环境对象
     * @param inputDim 输入维度
     * @param outputDim 输出维度
     * @param batchSize 批处理大小
     * @param earlystopping 是否使用早停
     * @param n 训练轮数
     */
    public void DDPG_learn(Environment env, int inputDim, int outputDim, int batchSize, boolean earlystopping, int n) {
        List<double[]> database = env.get_data();
        if (database.isEmpty()) {
            System.out.println("环境数据库为空，无法训练");
            return;
        }
        
        // 准备训练数据
        int dataSize = database.size();
        double[][] input = new double[dataSize][inputDim];
        double[][] output = new double[dataSize][outputDim];
        
        for (int j = 0; j < dataSize; j++) {
            double[] record = database.get(j);
            // 前inputDim行作为输入
            for (int k = 0; k < inputDim; k++) {
                input[j][k] = record[k];
            }
            // 后outputDim行作为输出
            for (int k = 0; k < outputDim; k++) {
                output[j][k] = record[inputDim + k];
            }
        }
        
        // 网格搜索超参数 - 缩小范围到表现较好的组合
        int[] layerSizesOptions = {2};
        int[] neuronsOptions = {64, 128};
        double[] learningRateOptions = {0.01};
        double[] l2Options = {0.00001};
        
        double bestValLoss = Double.MAX_VALUE;
        Net bestNet = null;
        List<Net> topNets = new ArrayList<>();
        
        for (int layers : layerSizesOptions) {
            for (int neurons : neuronsOptions) {
                for (double lr : learningRateOptions) {
                    for (double l2 : l2Options) {
                        // 构建网络结构
                        int[] layerSizes = new int[layers + 2];
                        layerSizes[0] = inputDim;
                        for (int l = 1; l <= layers; l++) {
                            layerSizes[l] = neurons;
                        }
                        layerSizes[layers + 1] = outputDim;
                        
                        Net net = new Net(layerSizes);
                        net.setLearningRate(lr);
                        net.setL2Regularization(l2);
                        
                        // 为每个网络模型创建输入输出数据的深拷贝
                        double[][] inputCopy = new double[input.length][input[0].length];
                        double[][] outputCopy = new double[output.length][output[0].length];
                        for (int j = 0; j < input.length; j++) {
                            inputCopy[j] = Arrays.copyOf(input[j], input[j].length);
                            outputCopy[j] = Arrays.copyOf(output[j], output[j].length);
                        }
                        
                        // 训练网络
                        net.fit(inputCopy, outputCopy, n, batchSize, 0.2, earlystopping);
                        
                        // 计算验证损失
                        double valLoss = calculateValidationLoss(net, input, output);
                        
                        if (valLoss < bestValLoss) {
                            bestValLoss = valLoss;
                            bestNet = net;
                            System.out.printf("找到更好的模型: 层数=%d, 神经元=%d, 学习率=%.4f, L2=%.5f, 验证损失=%.4f\n", 
                                    layers, neurons, lr, l2, valLoss);
                        }
                        
                        // 保存前5个最佳模型用于集成学习
                        topNets.add(net);
                        if (topNets.size() > 5) {
                            // 移除损失最大的模型
                            double maxLoss = Double.MIN_VALUE;
                            int maxIndex = 0;
                            for (int j = 0; j < topNets.size(); j++) {
                                double loss = calculateValidationLoss(topNets.get(j), input, output);
                                if (loss > maxLoss) {
                                    maxLoss = loss;
                                    maxIndex = j;
                                }
                            }
                            topNets.remove(maxIndex);
                        }
                    }
                }
            }
        }
        
        if (bestNet != null) {
            policy_net = bestNet;
            ensemble_nets = topNets;
            System.out.println("训练完成，最佳模型已设置为policy_net");
            System.out.println("集成学习模型已准备就绪，共" + ensemble_nets.size() + "个模型");
        }
    }
    
    /**
     * 使用高斯策略训练
     * @param env 环境对象
     * @param inputDim 输入维度
     * @param outputDim 输出维度
     * @param batchSize 批处理大小
     * @param earlystopping 是否使用早停
     * @param n 训练轮数
     */
    public void Gaussian_learn(Environment env, int inputDim, int outputDim, int batchSize, boolean earlystopping, int n) {
        List<double[]> database = env.get_data();
        if (database.isEmpty()) {
            System.out.println("环境数据库为空，无法训练");
            return;
        }
        
        // 准备训练数据
        int dataSize = database.size();
        double[][] input = new double[dataSize][inputDim];
        double[][] output = new double[dataSize][outputDim];
        
        for (int j = 0; j < dataSize; j++) {
            double[] record = database.get(j);
            // 前inputDim行作为输入
            for (int k = 0; k < inputDim; k++) {
                input[j][k] = record[k];
            }
            // 后outputDim行作为输出
            for (int k = 0; k < outputDim; k++) {
                output[j][k] = record[inputDim + k];
            }
        }
        
        // 网格搜索超参数
        int[] layerSizesOptions = {2, 3};
        int[] neuronsOptions = {64, 128};
        double[] learningRateOptions = {0.01, 0.005};
        double[] l2Options = {0.00001, 0.000001};
        
        double bestValLoss = Double.MAX_VALUE;
        Net bestMeanNet = null;
        Net bestVarianceNet = null;
        
        for (int layers : layerSizesOptions) {
            for (int neurons : neuronsOptions) {
                for (double lr : learningRateOptions) {
                    for (double l2 : l2Options) {
                        // 构建均值网络
                        int[] layerSizesMean = new int[layers + 2];
                        layerSizesMean[0] = inputDim;
                        for (int l = 1; l <= layers; l++) {
                            layerSizesMean[l] = neurons;
                        }
                        layerSizesMean[layers + 1] = outputDim;
                        
                        // 构建方差网络
                        int[] layerSizesVariance = new int[layers + 2];
                        layerSizesVariance[0] = inputDim;
                        for (int l = 1; l <= layers; l++) {
                            layerSizesVariance[l] = neurons;
                        }
                        layerSizesVariance[layers + 1] = outputDim;
                        
                        Net meanNet = new Net(layerSizesMean);
                        Net varianceNet = new Net(layerSizesVariance);
                        
                        meanNet.setLearningRate(lr);
                        meanNet.setL2Regularization(l2);
                        varianceNet.setLearningRate(lr);
                        varianceNet.setL2Regularization(l2);
                        
                        // 为每个网络模型创建输入输出数据的深拷贝
                        double[][] inputCopy = new double[input.length][input[0].length];
                        double[][] outputCopy = new double[output.length][output[0].length];
                        for (int j = 0; j < input.length; j++) {
                            inputCopy[j] = Arrays.copyOf(input[j], input[j].length);
                            outputCopy[j] = Arrays.copyOf(output[j], output[j].length);
                        }
                        
                        // 训练均值网络
                        meanNet.fit(inputCopy, outputCopy, n, batchSize, 0.2, earlystopping);
                        
                        // 计算方差网络的目标值（预测值与真实值的平方差）
                        double[][] varianceTarget = new double[dataSize][outputDim];
                        for (int j = 0; j < dataSize; j++) {
                            double[] pred = meanNet.predict(input[j]);
                            for (int k = 0; k < outputDim; k++) {
                                varianceTarget[j][k] = Math.pow(pred[k] - output[j][k], 2);
                            }
                        }
                        
                        // 训练方差网络
                        varianceNet.fit(inputCopy, varianceTarget, n, batchSize, 0.2, earlystopping);
                        
                        // 计算验证损失
                        double valLoss = calculateGaussianValidationLoss(meanNet, varianceNet, input, output);
                        
                        if (valLoss < bestValLoss) {
                            bestValLoss = valLoss;
                            bestMeanNet = meanNet;
                            bestVarianceNet = varianceNet;
                            System.out.printf("找到更好的高斯模型: 层数=%d, 神经元=%d, 学习率=%.4f, L2=%.5f, 验证损失=%.4f\n", 
                                    layers, neurons, lr, l2, valLoss);
                        }
                    }
                }
            }
        }
        
        if (bestMeanNet != null && bestVarianceNet != null) {
            policy_net = bestMeanNet;
            // 可以将方差网络存储在ensemble_nets中，或者添加新的成员变量
            ensemble_nets.clear();
            ensemble_nets.add(bestMeanNet);
            ensemble_nets.add(bestVarianceNet);
            System.out.println("高斯策略网络训练完成");
            System.out.println("均值网络已设置为policy_net");
            System.out.println("方差网络已添加到集成网络列表");
        }
    }
    
    /**
     * 计算高斯策略的验证损失
     * @param meanNet 均值网络
     * @param varianceNet 方差网络
     * @param input 输入数据
     * @param output 输出数据
     * @return 平均验证损失
     */
    private double calculateGaussianValidationLoss(Net meanNet, Net varianceNet, double[][] input, double[][] output) {
        int valSize = (int) (input.length * 0.2);
        double[][] valInput = new double[valSize][input[0].length];
        double[][] valOutput = new double[valSize][output[0].length];
        
        for (int j = 0; j < valSize; j++) {
            valInput[j] = input[input.length - valSize + j];
            valOutput[j] = output[output.length - valSize + j];
        }
        
        double totalLoss = 0;
        for (int j = 0; j < valSize; j++) {
            double[] meanPred = meanNet.predict(valInput[j]);
            double[] variancePred = varianceNet.predict(valInput[j]);
            
            // 计算高斯对数似然损失
            for (int k = 0; k < meanPred.length; k++) {
                double var = Math.max(variancePred[k], 1e-6); // 确保方差为正
                double diff = meanPred[k] - valOutput[j][k];
                totalLoss += 0.5 * (Math.log(2 * Math.PI * var) + (diff * diff) / var);
            }
        }
        
        double avgLoss = totalLoss / valSize;
        System.out.printf("高斯策略验证损失: %.4f\n", avgLoss);
        return avgLoss;
    }
    
    /**
     * 计算验证损失（使用裁剪前的原始网络输出）
     * @param net 网络模型
     * @param input 输入数据
     * @param output 输出数据
     * @return 平均验证损失
     */
    private double calculateValidationLoss(Net net, double[][] input, double[][] output) {
        int valSize = (int) (input.length * 0.2);
        double[][] valInput = new double[valSize][input[0].length];
        double[][] valOutput = new double[valSize][output[0].length];
        
        for (int j = 0; j < valSize; j++) {
            valInput[j] = input[input.length - valSize + j];
            valOutput[j] = output[output.length - valSize + j];
        }
        
        double totalLoss = 0;
        double totalAccuracy = 0;
        for (int j = 0; j < valSize; j++) {
            // 使用net.predict获取原始网络输出（裁剪前）
            double[] pred = net.predict(valInput[j]);
            // 直接使用原始输出计算损失，不进行裁剪
            totalLoss += meanSquaredError(pred, valOutput[j]);
            // 计算准确率
            totalAccuracy += accuracy(pred, valOutput[j]);
        }
        double avgLoss = totalLoss / valSize;
        double avgAccuracy = totalAccuracy / valSize;
        System.out.printf("验证损失: %.4f, 验证准确率: %.4f\n", avgLoss, avgAccuracy);
        return avgLoss;
    }
    
    /**
     * 计算准确率（当相对误差小于10%时视为正确）
     * @param output 网络输出
     * @param target 目标值
     * @return 准确率（0或1）
     */
    private double accuracy(double[] output, double[] target) {
        double errorSum = 0;
        double targetSum = 0;
        for (int i = 0; i < output.length; i++) {
            errorSum += Math.abs(output[i] - target[i]);
            targetSum += Math.abs(target[i]);
        }
        // 避免除零错误
        if (targetSum < 1e-8) {
            return 1.0;
        }
        // 相对误差小于10%时视为正确
        return errorSum / targetSum < 0.1 ? 1 : 0;
    }
    
    /**
     * 计算均方误差
     * @param pred 预测值
     * @param target 目标值
     * @return 均方误差
     */
    private double meanSquaredError(double[] pred, double[] target) {
        double sum = 0;
        for (int j = 0; j < pred.length; j++) {
            sum += Math.pow(pred[j] - target[j], 2);
        }
        return sum / pred.length;
    }
    
    /**
     * 使用带基线的REINFORCE算法训练agent
     * @param environment 环境对象
     * @param o 输出维度
     * @param n 训练轮数
     */
    public void RF_learn(Environment environment, int o, int n) {
        // 检查是否所有行动都是离散的
        for (boolean discrete : Is_discrete) {
            if (!discrete) {
                throw new IllegalArgumentException("REINFORCE算法只支持离散行动空间");
            }
        }
        
        // 初始化策略网络
        int state_dim = i; // 输入维度
        int[] layerSizes = {state_dim, 64, 64, o};
        policy_net = new Net(layerSizes);
        policy_net.setLearningRate(0.01);
        policy_net.setL2Regularization(0.00001);
        
        // 初始化基线网络
        Net baseline_net = new Net(new int[]{state_dim, 32, 1});
        baseline_net.setLearningRate(0.01);
        baseline_net.setL2Regularization(0.00001);
        
        
        
        for (int episode = 0; episode < n; episode++) {
            // 收集轨迹
            List<double[]> states = new ArrayList<>();
            List<Integer> actions = new ArrayList<>();
            List<Double> rewards = new ArrayList<>();
            
            environment.reset();
            
            for (int t = 0; t < 10; t++) { // 每个episode最多10步
                
                // 前state_dim个元素作为状态
                double[] state = environment.get_state();
                states.add(state);
                
                // 使用策略网络预测动作概率
                double[] actionProbs = policy_net.predict(state);
                
                // 根据概率选择动作
                int action = sampleAction(actionProbs);
                actions.add(action);
                
                // 获取奖励（使用环境的reward函数）
                double reward = environment.reward(new double[]{action});
                rewards.add(reward);
            }
            
            // 计算回报
            double[] returns = new double[rewards.size()];
            double G = 0;
            for (int t = rewards.size() - 1; t >= 0; t--) {
                G = rewards.get(t) + 0.99 * G; // 折扣因子0.99
                returns[t] = G;
            }
            
            // 标准化回报
            double meanReturn = 0;
            for (double r : returns) meanReturn += r;
            meanReturn /= returns.length;
            
            double stdReturn = 0;
            for (double r : returns) stdReturn += Math.pow(r - meanReturn, 2);
            stdReturn = Math.sqrt(stdReturn / returns.length);
            
            for (int t = 0; t < returns.length; t++) {
                returns[t] = (returns[t] - meanReturn) / (stdReturn + 1e-8);
            }
            
            // 训练策略网络和基线网络
            for (int t = 0; t < states.size(); t++) {
                double[] state_t = states.get(t);
                int action_t = actions.get(t);
                double return_t = returns[t];
                
                // 计算基线
                double baseline = baseline_net.predict(state_t)[0];
                double advantage = return_t - baseline;
                
                // 训练策略网络
                double[] target = new double[o];
                target[action_t] = advantage;
                policy_net.fit(new double[][]{state_t}, new double[][]{target}, 1, 1, 0, false);
                
                // 训练基线网络
                double[] baselineTarget = new double[1];
                baselineTarget[0] = return_t;
                baseline_net.fit(new double[][]{state_t}, new double[][]{baselineTarget}, 1, 1, 0, false);
            }
            
            if (episode % 100 == 0) {
                System.out.printf("Episode %d completed\n", episode);
            }
        }
        
        System.out.println("REINFORCE训练完成");
    }
    
    /**
     * 根据概率分布采样动作
     * @param probs 动作概率分布
     * @return 采样的动作索引
     */
    private int sampleAction(double[] probs) {
        Random random = new Random();
        double sum = 0;
        for (double prob : probs) sum += prob;
        
        double r = random.nextDouble() * sum;
        double cumulative = 0;
        for (int i = 0; i < probs.length; i++) {
            cumulative += probs[i];
            if (r <= cumulative) {
                return i;
            }
        }
        return probs.length - 1;
    }
    
    /**
     * 使用A2C算法训练agent
     * @param environment 环境对象
     * @param o 输出维度
     * @param n 训练轮数
     */
    public void A2C_learn(Environment environment, int o, int n) {
        // 检查是否所有行动都是离散的
        for (boolean discrete : Is_discrete) {
            if (!discrete) {
                throw new IllegalArgumentException("A2C算法只支持离散行动空间");
            }
        }
        
        // 初始化Actor网络（策略网络）
        int state_dim = i; // 输入维度
        int[] actorLayerSizes = {state_dim, 64, 64, o};
        Net actor_net = new Net(actorLayerSizes);
        actor_net.setLearningRate(0.01);
        actor_net.setL2Regularization(0.00001);
        
        // 初始化Critic网络（价值网络）
        int[] criticLayerSizes = {state_dim, 64, 64, 1};
        Net critic_net = new Net(criticLayerSizes);
        critic_net.setLearningRate(0.01);
        critic_net.setL2Regularization(0.00001);
        
        
        for (int episode = 0; episode < n; episode++) {
            // 收集轨迹
            List<double[]> states = new ArrayList<>();
            List<Integer> actions = new ArrayList<>();
            List<Double> rewards = new ArrayList<>();
            List<double[]> nextStates = new ArrayList<>();
            List<Boolean> dones = new ArrayList<>();
            
            environment.reset();

            
            for (int t = 0; t < 10; t++) { // 每个episode最多10步
                
                // 前state_dim个元素作为状态
                double[] state = environment.get_state();
                
                states.add(state);
                
                // 使用Actor网络预测动作概率
                double[] actionProbs = actor_net.predict(state);
                
                // 根据概率选择动作
                int action = sampleAction(actionProbs);
                actions.add(action);
                
                // 获取奖励（使用环境的reward函数）
                double reward = environment.reward(new double[]{action});
                rewards.add(reward);

                double[] nextState = environment.get_state();
                nextStates.add(nextState);
                // 标记是否结束
                dones.add(t == 9); // 10步后结束
            }
            
            // 计算优势函数和目标值
            double[] values = new double[states.size()];
            double[] nextValues = new double[states.size()];
            double[] advantages = new double[states.size()];
            double[] targets = new double[states.size()];
            
            // 计算状态价值
            for (int t = 0; t < states.size(); t++) {
                values[t] = critic_net.predict(states.get(t))[0];
                nextValues[t] = critic_net.predict(nextStates.get(t))[0];
            }
            
            // 计算优势函数和目标值
            for (int t = 0; t < states.size(); t++) {
                double tdTarget = rewards.get(t) + 0.99 * nextValues[t] * (dones.get(t) ? 0 : 1);
                advantages[t] = tdTarget - values[t];
                targets[t] = tdTarget;
            }
            
            // 训练Actor和Critic网络
            for (int t = 0; t < states.size(); t++) {
                double[] state_t = states.get(t);
                int action_t = actions.get(t);
                double advantage_t = advantages[t];
                double target_t = targets[t];
                
                // 训练Actor网络
                double[] actorTarget = new double[o];
                actorTarget[action_t] = advantage_t;
                actor_net.fit(new double[][]{state_t}, new double[][]{actorTarget}, 1, 1, 0, false);
                
                // 训练Critic网络
                double[] criticTarget = new double[1];
                criticTarget[0] = target_t;
                critic_net.fit(new double[][]{state_t}, new double[][]{criticTarget}, 1, 1, 0, false);
            }
            
            // 更新策略网络
            policy_net = actor_net;
            
            if (episode % 100 == 0) {
                System.out.printf("Episode %d completed\n", episode);
            }
        }
        
        System.out.println("A2C训练完成");
    }
    
    /**
     * 使用ARSQ算法训练agent
     * @param environment 环境对象
     * @param o 输出维度
     * @param n 训练轮数
     */
    public void ARSQ_learn(Environment environment, int o, int n) {
        // 初始化策略网络
        int state_dim = i; // 输入维度
        int[] layerSizes = {state_dim, 64, 64, o};
        policy_net = new Net(layerSizes);
        policy_net.setLearningRate(0.0001); // 进一步降低学习率
        policy_net.setL2Regularization(0.00001);
        
        // 检查是否所有行动都是离散的
        boolean isAllDiscrete = true;
        for (boolean discrete : Is_discrete) {
            if (!discrete) {
                isAllDiscrete = false;
                break;
            }
        }
        
        // 计算Q网络输入维度
        int qInputDim;
        if (isAllDiscrete) {
            // 离散动作：状态维度 + 动作维度（one-hot编码）
            qInputDim = state_dim + o;
        } else {
            // 连续动作：状态维度 + 动作维度（直接输入连续值）
            qInputDim = state_dim + o;
        }
        
        // 初始化Q网络
        int[] qLayerSizes = {qInputDim, 64, 64, 1};
        Net q_net = new Net(qLayerSizes);
        q_net.setLearningRate(0.0001); // 进一步降低学习率
        q_net.setL2Regularization(0.00001);
        
        // 重置环境
        environment.reset();
        
        Random random = new Random();
        double epsilon = 1.0; // 初始探索率
        double epsilonDecay = 0.995; // 探索率衰减
        double epsilonMin = 0.01; // 最小探索率
        double gamma = 0.99; // 折扣因子
        
        // 收集经验回放缓冲区
        List<double[]> qInputsBuffer = new ArrayList<>();
        List<double[]> qTargetsBuffer = new ArrayList<>();
        List<double[]> statesBuffer = new ArrayList<>();
        List<double[]> policyTargetsBuffer = new ArrayList<>();
        int batchSize = 32; // 批量大小
        
        for (int episode = 0; episode < n; episode++) {
            // 重置环境获取初始状态
            environment.reset();
            double[] state = environment.get_state();
            
            for (int t = 0; t < 10; t++) { // 每个episode最多10步
                // 选择动作
                double[] action;
                if (isAllDiscrete) {
                    // 离散动作：使用epsilon-greedy策略
                    int actionIndex;
                    if (random.nextDouble() < epsilon) {
                        // 随机探索
                        actionIndex = random.nextInt(o);
                    } else {
                        // 使用策略网络
                        double[] actionProbs = policy_net.predict(state);
                        actionIndex = sampleAction(actionProbs);
                    }
                    // 将离散动作转换为one-hot编码
                    action = new double[o];
                    action[actionIndex] = 1.0;
                } else {
                    // 连续动作：使用策略网络输出均值，添加噪声进行探索
                    double[] actionMean = policy_net.predict(state);
                    action = new double[o];
                    for (int j = 0; j < o; j++) {
                        // 添加高斯噪声进行探索
                        double noise = epsilon * random.nextGaussian();
                        action[j] = actionMean[j] + noise;
                        // 裁剪到行动范围内
                        action[j] = Math.max(action_range[j][0], Math.min(action_range[j][1], action[j]));
                    }
                }
                
                // 获取奖励并让环境更新状态
                double reward = environment.reward(action);
                
                // 获取下一个状态（从环境中获取，而不是从数据中）
                double[] nextState = environment.get_state();
                
                // 构建Q网络输入（状态+动作）
                double[] qInput = new double[qInputDim];
                System.arraycopy(state, 0, qInput, 0, state_dim);
                
                if (isAllDiscrete) {
                    // 离散动作：使用one-hot编码
                    System.arraycopy(action, 0, qInput, state_dim, o);
                } else {
                    // 连续动作：直接使用连续值
                    System.arraycopy(action, 0, qInput, state_dim, o);
                }
                
                // 计算目标Q值
                double targetQ = reward;
                if (t < 9) { // 如果不是最后一步
                    double maxNextQ = Double.NEGATIVE_INFINITY;
                    
                    if (isAllDiscrete) {
                        // 离散动作：遍历所有可能的动作
                        for (int a = 0; a < o; a++) {
                            double[] nextAction = new double[o];
                            nextAction[a] = 1.0;
                            double[] nextQInput = new double[qInputDim];
                            System.arraycopy(nextState, 0, nextQInput, 0, state_dim);
                            System.arraycopy(nextAction, 0, nextQInput, state_dim, o);
                            double qValue = q_net.predict(nextQInput)[0];
                            if (qValue > maxNextQ) {
                                maxNextQ = qValue;
                            }
                        }
                    } else {
                        // 连续动作：使用策略网络预测的动作
                        double[] nextAction = policy_net.predict(nextState);
                        double[] nextQInput = new double[qInputDim];
                        System.arraycopy(nextState, 0, nextQInput, 0, state_dim);
                        System.arraycopy(nextAction, 0, nextQInput, state_dim, o);
                        maxNextQ = q_net.predict(nextQInput)[0];
                    }
                    
                    // 确保maxNextQ不是NaN或无穷大
                    if (Double.isNaN(maxNextQ) || Double.isInfinite(maxNextQ)) {
                        maxNextQ = 0.0;
                    }
                    
                    targetQ += gamma * maxNextQ;
                }
                
                // 确保targetQ不是NaN或无穷大
                if (Double.isNaN(targetQ) || Double.isInfinite(targetQ)) {
                    targetQ = 0.0;
                }
                
                // 计算策略网络目标
                double currentQ = q_net.predict(qInput)[0];
                // 确保currentQ不是NaN或无穷大
                if (Double.isNaN(currentQ) || Double.isInfinite(currentQ)) {
                    currentQ = 0.0;
                }
                
                double[] policyTarget = new double[o];
                for (int j = 0; j < o; j++) {
                    policyTarget[j] = action[j] + 0.01 * currentQ; // 进一步减小学习率缩放因子
                }
                
                // 将样本添加到缓冲区
                qInputsBuffer.add(qInput);
                qTargetsBuffer.add(new double[]{targetQ});
                statesBuffer.add(state);
                policyTargetsBuffer.add(policyTarget);
                
                // 当缓冲区达到批量大小时进行训练
                if (qInputsBuffer.size() >= batchSize) {
                    // 转换为数组
                    double[][] qInputs = qInputsBuffer.toArray(new double[qInputsBuffer.size()][]);
                    double[][] qTargets = qTargetsBuffer.toArray(new double[qTargetsBuffer.size()][]);
                    double[][] states = statesBuffer.toArray(new double[statesBuffer.size()][]);
                    double[][] policyTargets = policyTargetsBuffer.toArray(new double[policyTargetsBuffer.size()][]);
                    
                    // 训练Q网络，减少训练轮数
                    q_net.fit(qInputs, qTargets, 5, batchSize, 0.2, true);
                    
                    // 训练策略网络，减少训练轮数
                    policy_net.fit(states, policyTargets, 5, batchSize, 0.2, true);
                    
                    // 清空缓冲区
                    qInputsBuffer.clear();
                    qTargetsBuffer.clear();
                    statesBuffer.clear();
                    policyTargetsBuffer.clear();
                }
                
                // 更新状态
                state = nextState;
            }
            
            // 衰减探索率
            epsilon = Math.max(epsilonMin, epsilon * epsilonDecay);
            
            if (episode % 100 == 0) {
                System.out.printf("Episode %d completed, epsilon: %.4f\n", episode, epsilon);
            }
        }
        
        System.out.println("ARSQ训练完成");
    }
    
    /**
     * 使用TD3算法训练agent
     * @param environment 环境对象
     * @param o 输出维度
     * @param n 训练轮数
     */
    public void TD3_learn(Environment environment, int o, int n) {
        // 初始化参数
        int state_dim = i; // 输入维度
        double gamma = 0.99; // 折扣因子
        double tau = 0.005; // 目标网络更新系数
        int policyDelay = 2; // 策略网络更新延迟
        double explorationNoise = 0.1; // 探索噪声
        double policyNoise = 0.2; // 策略噪声
        double noiseClip = 0.5; // 噪声裁剪
        int batchSize = 64; // 增加批量大小
        int replayBufferSize = 10000; // 经验回放缓冲区大小
        
        // 确保环境状态维度与Agent输入维度匹配
        if (environment.get_state_dim() != state_dim) {
            System.out.println("环境状态维度与Agent输入维度不匹配，正在调整...");
            double[] initialState = new double[state_dim];
            environment.set_params(environment.get_h(), environment.get_m(), environment.get_k(), environment.get_theta(), state_dim, initialState);
        }
        
        // 初始化策略网络
        int[] policyLayerSizes = {state_dim, 64, 64, o};
        Net policy_net = new Net(policyLayerSizes);
        policy_net.setLearningRate(0.001);
        policy_net.setL2Regularization(0.00001);
        
        // 初始化Q1网络
        int[] q1LayerSizes = {state_dim + o, 64, 64, 1};
        Net q1_net = new Net(q1LayerSizes);
        q1_net.setLearningRate(0.001);
        q1_net.setL2Regularization(0.00001);
        
        // 初始化Q2网络
        int[] q2LayerSizes = {state_dim + o, 64, 64, 1};
        Net q2_net = new Net(q2LayerSizes);
        q2_net.setLearningRate(0.001);
        q2_net.setL2Regularization(0.00001);
        
        // 初始化目标网络
        Net target_policy_net = new Net(policyLayerSizes);
        Net target_q1_net = new Net(q1LayerSizes);
        Net target_q2_net = new Net(q2LayerSizes);
        
        // 复制网络参数到目标网络
        target_policy_net.copyWeights(policy_net);
        target_q1_net.copyWeights(q1_net);
        target_q2_net.copyWeights(q2_net);
        
        // 经验回放缓冲区
        List<double[]> statesBuffer = new ArrayList<>();
        List<double[]> actionsBuffer = new ArrayList<>();
        List<Double> rewardsBuffer = new ArrayList<>();
        List<double[]> nextStatesBuffer = new ArrayList<>();
        List<Boolean> donesBuffer = new ArrayList<>();
        
        Random random = new Random();
        
        for (int episode = 0; episode < n; episode++) {
            // 重置环境获取初始状态
            environment.reset();
            double[] state = environment.get_state();
            double episodeReward = 0;
            
            for (int t = 0; t < 10; t++) { // 每个episode最多10步
                // 选择动作并添加探索噪声
                double[] action = policy_net.predict(state);
                for (int j = 0; j < o; j++) {
                    action[j] += explorationNoise * random.nextGaussian();
                    // 裁剪到行动范围内
                    action[j] = Math.max(action_range[j][0], Math.min(action_range[j][1], action[j]));
                }
                
                // 获取奖励并让环境更新状态
                double reward = environment.reward(action);
                episodeReward += reward;
                
                // 获取下一个状态
                double[] nextState = environment.get_state();
                boolean done = (t == 9); // 10步后结束
                
                // 将样本添加到缓冲区
                statesBuffer.add(state);
                actionsBuffer.add(action);
                rewardsBuffer.add(reward);
                nextStatesBuffer.add(nextState);
                donesBuffer.add(done);
                
                // 限制缓冲区大小
                if (statesBuffer.size() > replayBufferSize) {
                    statesBuffer.remove(0);
                    actionsBuffer.remove(0);
                    rewardsBuffer.remove(0);
                    nextStatesBuffer.remove(0);
                    donesBuffer.remove(0);
                }
                
                // 当缓冲区足够大时进行训练
                if (statesBuffer.size() >= batchSize) {
                    // 随机采样批次
                    int[] indices = new int[batchSize];
                    for (int j = 0; j < batchSize; j++) {
                        indices[j] = random.nextInt(statesBuffer.size());
                    }
                    
                    // 准备批次数据
                    double[][] batchStates = new double[batchSize][state_dim];
                    double[][] batchActions = new double[batchSize][o];
                    double[] batchRewards = new double[batchSize];
                    double[][] batchNextStates = new double[batchSize][state_dim];
                    boolean[] batchDones = new boolean[batchSize];
                    
                    for (int j = 0; j < batchSize; j++) {
                        int idx = indices[j];
                        batchStates[j] = statesBuffer.get(idx);
                        batchActions[j] = actionsBuffer.get(idx);
                        batchRewards[j] = rewardsBuffer.get(idx);
                        batchNextStates[j] = nextStatesBuffer.get(idx);
                        batchDones[j] = donesBuffer.get(idx);
                    }
                    
                    // 训练Q网络
                    for (int j = 0; j < batchSize; j++) {
                        // 为目标动作添加噪声
                        double[] targetAction = target_policy_net.predict(batchNextStates[j]);
                        for (int k = 0; k < o; k++) {
                            double noise = policyNoise * random.nextGaussian();
                            noise = Math.max(-noiseClip, Math.min(noiseClip, noise)); // 裁剪噪声
                            targetAction[k] = Math.max(action_range[k][0], Math.min(action_range[k][1], targetAction[k] + noise));
                        }
                        
                        // 构建Q网络输入
                        double[] q1Input = new double[state_dim + o];
                        double[] q2Input = new double[state_dim + o];
                        System.arraycopy(batchNextStates[j], 0, q1Input, 0, state_dim);
                        System.arraycopy(batchNextStates[j], 0, q2Input, 0, state_dim);
                        System.arraycopy(targetAction, 0, q1Input, state_dim, o);
                        System.arraycopy(targetAction, 0, q2Input, state_dim, o);
                        
                        // 计算目标Q值
                        double q1Target = target_q1_net.predict(q1Input)[0];
                        double q2Target = target_q2_net.predict(q2Input)[0];
                        double minQTarget = Math.min(q1Target, q2Target);
                        double target = batchRewards[j] + gamma * minQTarget * (batchDones[j] ? 0 : 1);
                        
                        // 构建当前Q网络输入
                        double[] currentQ1Input = new double[state_dim + o];
                        double[] currentQ2Input = new double[state_dim + o];
                        System.arraycopy(batchStates[j], 0, currentQ1Input, 0, state_dim);
                        System.arraycopy(batchStates[j], 0, currentQ2Input, 0, state_dim);
                        System.arraycopy(batchActions[j], 0, currentQ1Input, state_dim, o);
                        System.arraycopy(batchActions[j], 0, currentQ2Input, state_dim, o);
                        
                        // 训练Q1网络
                        q1_net.fit(new double[][]{currentQ1Input}, new double[][]{new double[]{target}}, 1, 1, 0, false);
                        
                        // 训练Q2网络
                        q2_net.fit(new double[][]{currentQ2Input}, new double[][]{new double[]{target}}, 1, 1, 0, false);
                    }
                    
                    // 延迟更新策略网络
                    if (episode % policyDelay == 0) {
                        for (int j = 0; j < batchSize; j++) {
                            // 计算策略梯度 - 使用更直接的方式
                            double[] currentAction = policy_net.predict(batchStates[j]);
                            // 训练策略网络，目标是最大化Q值
                            // 这里使用当前动作加上Q值的梯度方向
                            double[] qInput = new double[state_dim + o];
                            System.arraycopy(batchStates[j], 0, qInput, 0, state_dim);
                            System.arraycopy(currentAction, 0, qInput, state_dim, o);
                            double qValue = q1_net.predict(qInput)[0];
                            
                            // 训练策略网络，使用Q值作为奖励信号
                            // 对于连续动作空间，我们希望策略网络输出的动作能够最大化Q值
                            // 这里使用简单的梯度上升方法
                            double[] policyTarget = new double[o];
                            for (int k = 0; k < o; k++) {
                                // 策略梯度方向：增加Q值高的动作
                                policyTarget[k] = currentAction[k] + 0.01 * qValue;
                                // 确保目标动作在有效范围内
                                policyTarget[k] = Math.max(action_range[k][0], Math.min(action_range[k][1], policyTarget[k]));
                            }
                            policy_net.fit(new double[][]{batchStates[j]}, new double[][]{policyTarget}, 1, 1, 0, false);
                        }
                        
                        // 更新目标网络
                        target_policy_net.softUpdate(policy_net, tau);
                        target_q1_net.softUpdate(q1_net, tau);
                        target_q2_net.softUpdate(q2_net, tau);
                    }
                }
                
                // 更新状态
                state = nextState;
            }
            
            if (episode % 100 == 0) {
                System.out.printf("Episode %d completed, reward: %.2f\n", episode, episodeReward);
            }
        }
        
        // 设置最终的策略网络
        this.policy_net = policy_net;
        
        System.out.println("TD3训练完成");
    }
    
    // 主函数用于测试
    public static void main(String[] args) throws IOException, ClassNotFoundException {
        // 创建环境并生成更多数据
        Environment env = new Environment();
        env.set_params(0.1, 0.1, 0.01, Math.PI / 4);
        // 增加数据量到10000条
        env.simulate(10000, 5, 20, 0.05);
        // 添加更多数据增强
        env.simulate(2000, 5, 20, 0.1);  // 增加噪声
        env.simulate(2000, 10, 20, 0.05); // 增加小车速度范围
        env.simulate(2000, 5, 30, 0.05); // 增加小球初速度范围
        
        // 创建agent
        Agent agent = new Agent(4, 3); // 4维输入，3维输出
        
        // 设置行动范围
        double[][] actionRange = {
            {-15, 15},   // 第一个行动的范围
            {-15, 15},   // 第二个行动的范围
            {0, 20}    // 第三个行动的范围
        };
        agent.set_action_range(actionRange);
        
        // 显示agent参数信息
        agent.show_params();
        
        
        // 训练agent，增加训练轮数到300轮，批处理大小到64
        System.out.println("开始训练agent (DDPG策略)...");
        System.out.println("目标：验证损失小于0.005，验证集精确度0.9");
        agent.DDPG_learn(env, 4, 3, 64, true, 300);
        /* *
        // 测试高斯策略
        System.out.println("\n开始训练agent (高斯策略)...");
        System.out.println("目标：验证损失最小化");
        agent.Gaussian_learn(env, 4, 3, 64, true, 300);
        */
        
        
        // 显示训练后的agent参数信息
        System.out.println("\n训练完成后的agent参数信息:");
        agent.show_params();
        
        // 测试决策
        double[] testInput = {-2.0, -1.0, 12, 7};
        double[] action = agent.decide(testInput);
        System.out.println("测试输入: " + java.util.Arrays.toString(testInput));
        System.out.println("决策输出: " + java.util.Arrays.toString(action));
        
        // 保存模型
        agent.save("agent_DDPG.ser");
        System.out.println("模型已保存为 agent_DDPG.ser");
        
    }
}