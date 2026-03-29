package org.firstinspires.ftc.teamcode.controllers.shooter.DRL;

import java.io.*;
import java.util.*;
// Android平台不支持java.awt和javax.swing，注释掉可视化相关的import
// import java.awt.Color;
// import java.awt.Font;
// import java.awt.Graphics;
// import javax.swing.*;

/**
 * Net 类 - 实现深度神经网络，支持强化学习所需的功能
 * 
 * 功能说明：
 * 1. 支持任意多层的神经网络结构
 * 2. 使用 AdamW 优化器进行参数更新，支持 L2 正则化
 * 3. 提供数据归一化功能，提高训练稳定性
 * 4. 支持早停法，防止过拟合
 * 5. 支持模型的保存和加载
 * 6. 提供训练和预测功能
 * 7. 计算均方误差和准确率等评估指标
 * 
 * 主要方法：
 * 
 * 构造函数：Net(int... layerSizes)
 * - 功能：初始化神经网络，设置各层大小
 * - 输入：layerSizes - 各层神经元数量，例如new Net(4, 64, 3)表示输入层4个神经元，隐藏层64个，输出层3个
 * - 输出：无
 * 
 * setLearningRate(double learningRate)
 * - 功能：设置学习率
 * - 输入：learningRate - 学习率
 * - 输出：无
 * 
 * setL2Regularization(double l2Regularization)
 * - 功能：设置 L2 正则化系数
 * - 输入：l2Regularization - L2 正则化系数
 * - 输出：无
 * 
 * setAdamParams(double beta1, double beta2, double epsilon)
 * - 功能：设置 Adam 优化器参数
 * - 输入：beta1 - 一阶矩估计的指数衰减率，beta2 - 二阶矩估计的指数衰减率，epsilon - 防止除零的小常数
 * - 输出：无
 * 
 * forward(double[] input)
 * - 功能：前向传播计算
 * - 输入：input - 输入向量
 * - 输出：网络预测输出向量
 * 
 * backward(double[] input, double[] target)
 * - 功能：反向传播更新参数
 * - 输入：input - 输入向量，target - 目标输出向量
 * - 输出：无
 * 
 * standardize(double[][] input, double[][] output)
 * - 功能：归一化训练集
 * - 输入：input - 输入数据集，output - 输出数据集
 * - 输出：无（直接修改输入数据）
 * 
 * standardizeInput(double[] input)
 * - 功能：归一化单个输入
 * - 输入：input - 输入向量
 * - 输出：归一化后的输入向量
 * 
 * unstandardizeOutput(double[] output)
 * - 功能：反归一化输出
 * - 输入：output - 归一化的输出向量
 * - 输出：反归一化后的输出向量
 * 
 * meanSquaredError(double[] output, double[] target)
 * - 功能：计算均方误差
 * - 输入：output - 网络输出，target - 目标值
 * - 输出：均方误差
 * 
 * accuracy(double[] output, double[] target)
 * - 功能：计算准确率（当相对误差小于10%时视为正确）
 * - 输入：output - 网络输出，target - 目标值
 * - 输出：准确率（0或1）
 * 
 * fit(double[][] input, double[][] output, int epochs, int batchSize, double validationSplit, boolean EarlyStopping)
 * - 功能：训练模型
 * - 输入：input - 输入数据集，output - 输出数据集，epochs - 训练轮数，batchSize - 批处理大小，validationSplit - 验证集比例，EarlyStopping - 是否使用早停法
 * - 输出：无
 * 
 * predict(double[] input)
 * - 功能：预测输入的输出
 * - 输入：input - 输入向量
 * - 输出：预测输出向量
 * 
 * save(String filename)
 * - 功能：保存模型到文件
 * - 输入：filename - 保存路径
 * - 输出：无
 * 
 * load(String filename)
 * - 功能：从文件加载模型
 * - 输入：filename - 模型文件路径
 * - 输出：返回加载的Net实例
 */
public class Net implements Serializable {
    private static final long serialVersionUID = -5068306942584650477L;
    private List<double[][]> weights; // 权重
    private List<double[]> biases;    // 偏置
    private int[] layerSizes;         // 各层大小
    private double learningRate;      // 学习率
    private double l2Regularization;  // L2正则化系数
    private int inputSize;            // 输入维度
    private int outputSize;           // 输出维度
    
    // Adam优化器参数
    private double beta1;             // 一阶矩估计的指数衰减率
    private double beta2;             // 二阶矩估计的指数衰减率
    private double epsilon;           // 防止除零的小常数
    private int t;                    // 时间步
    
    // Adam一阶矩估计(m)和二阶矩估计(v)
    private List<double[][]> mWeights;    // 权重的一阶矩估计
    private List<double[]> mBiases;       // 偏置的一阶矩估计
    private List<double[][]> vWeights;    // 权重的二阶矩估计
    private List<double[]> vBiases;       // 偏置的二阶矩估计
    
    // 归一化参数
    private double[] inputMean;
    private double[] inputStd;
    private double[] outputMean;
    private double[] outputStd;
    
    // 构造函数
    public Net(int... layerSizes) {
        this.layerSizes = layerSizes;
        this.inputSize = layerSizes[0];
        this.outputSize = layerSizes[layerSizes.length - 1];
        this.learningRate = 0.001;
        this.l2Regularization = 0.0001;
        this.beta1 = 0.9;
        this.beta2 = 0.999;
        this.epsilon = 1e-8;
        this.t = 0;
        
        // 初始化权重和偏置
        initializeWeights();
        
        // 初始化Adam矩估计
        initializeAdamMoments();
    }
    
    // 设置学习率
    public void setLearningRate(double learningRate) {
        this.learningRate = learningRate;
    }
    
    // 设置L2权重衰减系数
    public void setL2Regularization(double l2Regularization) {
        this.l2Regularization = l2Regularization;
    }
    
    // 设置Adam参数
    public void setAdamParams(double beta1, double beta2, double epsilon) {
        this.beta1 = beta1;
        this.beta2 = beta2;
        this.epsilon = epsilon;
    }
    
    // 获取学习率
    public double getLearningRate() {
        return learningRate;
    }
    
    // 获取L2正则化系数
    public double getL2Regularization() {
        return l2Regularization;
    }
    
    // 获取网络层数
    public int getLayerCount() {
        return layerSizes.length;
    }
    
    // 获取各层神经元数量
    public int[] getLayerSizes() {
        return layerSizes;
    }
    
    // 使用AdamW优化器更新参数（带L2正则化）
    private void updateParamsWithAdamW(List<double[][]> weightGradients, List<double[]> biasGradients, int batchSizeActual) {
        // 更新时间步
        t++;
        
        // 计算Adam的偏差修正系数
        double mHatCorrection = 1.0 / (1.0 - Math.pow(beta1, t));
        double vHatCorrection = 1.0 / (1.0 - Math.pow(beta2, t));
        
        // 使用AdamW优化器更新参数
        for (int j = 0; j < weights.size(); j++) {
            double[][] weight = weights.get(j);
            double[] bias = biases.get(j);
            double[][] weightGradient = weightGradients.get(j);
            double[] biasGradient = biasGradients.get(j);
            double[][] mWeight = mWeights.get(j);
            double[] mBias = mBiases.get(j);
            double[][] vWeight = vWeights.get(j);
            double[] vBias = vBiases.get(j);
            
            for (int k = 0; k < weight.length; k++) {
                for (int l = 0; l < weight[k].length; l++) {
                    // 计算梯度（不包含L2正则化，因为AdamW会单独处理）
                    double gradient = weightGradient[k][l] / batchSizeActual;
                    
                    // 更新一阶矩估计：m = beta1 * m + (1 - beta1) * gradient
                    mWeight[k][l] = beta1 * mWeight[k][l] + (1 - beta1) * gradient;
                    // 更新二阶矩估计：v = beta2 * v + (1 - beta2) * gradient^2
                    vWeight[k][l] = beta2 * vWeight[k][l] + (1 - beta2) * gradient * gradient;
                    
                    // 偏差修正
                    double mHat = mWeight[k][l] * mHatCorrection;
                    double vHat = vWeight[k][l] * vHatCorrection;
                    
                    // AdamW：先应用权重衰减，再更新参数
                    weight[k][l] *= (1 - learningRate * l2Regularization);
                    // 更新权重：w = w - learningRate * mHat / (sqrt(vHat) + epsilon)
                    weight[k][l] -= learningRate * mHat / (Math.sqrt(vHat) + epsilon);
                }
            }
            
            for (int k = 0; k < bias.length; k++) {
                // 计算梯度
                double gradient = biasGradient[k] / batchSizeActual;
                
                // 更新一阶矩估计：m = beta1 * m + (1 - beta1) * gradient
                mBias[k] = beta1 * mBias[k] + (1 - beta1) * gradient;
                // 更新二阶矩估计：v = beta2 * v + (1 - beta2) * gradient^2
                vBias[k] = beta2 * vBias[k] + (1 - beta2) * gradient * gradient;
                
                // 偏差修正
                double mHat = mBias[k] * mHatCorrection;
                double vHat = vBias[k] * vHatCorrection;
                
                // 更新偏置：b = b - learningRate * mHat / (sqrt(vHat) + epsilon)
                bias[k] -= learningRate * mHat / (Math.sqrt(vHat) + epsilon);
            }
        }
    }
    
    // 初始化Adam矩估计
    private void initializeAdamMoments() {
        mWeights = new ArrayList<>();
        mBiases = new ArrayList<>();
        vWeights = new ArrayList<>();
        vBiases = new ArrayList<>();
        
        for (int i = 0; i < layerSizes.length - 1; i++) {
            int inSize = layerSizes[i];
            int outSize = layerSizes[i + 1];
            
            // 一阶矩估计初始化为0
            double[][] mWeight = new double[inSize][outSize];
            mWeights.add(mWeight);
            double[] mBias = new double[outSize];
            mBiases.add(mBias);
            
            // 二阶矩估计初始化为0
            double[][] vWeight = new double[inSize][outSize];
            vWeights.add(vWeight);
            double[] vBias = new double[outSize];
            vBiases.add(vBias);
        }
    }
    
    // 初始化权重和偏置
    private void initializeWeights() {
        weights = new ArrayList<>();
        biases = new ArrayList<>();
        
        for (int i = 0; i < layerSizes.length - 1; i++) {
            int inSize = layerSizes[i];
            int outSize = layerSizes[i + 1];
            
            // He初始化
            double[][] weight = new double[inSize][outSize];
            double std = Math.sqrt(2.0 / inSize);
            Random random = new Random();
            
            for (int j = 0; j < inSize; j++) {
                for (int k = 0; k < outSize; k++) {
                    weight[j][k] = random.nextGaussian() * std;
                }
            }
            weights.add(weight);
            
            // 偏置初始化为0
            double[] bias = new double[outSize];
            biases.add(bias);
        }
    }
    
    // ReLU激活函数
    private double[] relu(double[] x) {
        double[] result = new double[x.length];
        for (int i = 0; i < x.length; i++) {
            result[i] = Math.max(0, x[i]);
        }
        return result;
    }
    
    // ReLU导数
    private double[] reluDerivative(double[] x) {
        double[] result = new double[x.length];
        for (int i = 0; i < x.length; i++) {
            result[i] = x[i] > 0 ? 1 : 0;
        }
        return result;
    }
    
    // 前向传播
    public double[] forward(double[] input) {
        double[] current = input;
        
        for (int i = 0; i < weights.size(); i++) {
            double[][] weight = weights.get(i);
            double[] bias = biases.get(i);
            double[] next = new double[bias.length];
            
            // 矩阵乘法
            for (int j = 0; j < bias.length; j++) {
                next[j] = bias[j];
                for (int k = 0; k < current.length; k++) {
                    next[j] += current[k] * weight[k][j];
                }
            }
            
            // 激活函数（最后一层无激活）
            if (i < weights.size() - 1) {
                current = relu(next);
            } else {
                current = next;
            }
        }
        
        return current;
    }
    
    // 反向传播（Adam优化器）
    public void backward(double[] input, double[] target) {
        // 前向传播，保存中间结果
        List<double[]> activations = new ArrayList<>();
        List<double[]> preActivations = new ArrayList<>();
        double[] current = input;
        activations.add(current);
        
        for (int i = 0; i < weights.size(); i++) {
            double[][] weight = weights.get(i);
            double[] bias = biases.get(i);
            double[] next = new double[bias.length];
            
            // 矩阵乘法
            for (int j = 0; j < bias.length; j++) {
                next[j] = bias[j];
                for (int k = 0; k < current.length; k++) {
                    next[j] += current[k] * weight[k][j];
                }
            }
            
            preActivations.add(next);
            
            // 激活函数（最后一层无激活）
            if (i < weights.size() - 1) {
                current = relu(next);
            } else {
                current = next;
            }
            activations.add(current);
        }
        
        // 计算输出层误差
        double[] error = new double[outputSize];
        for (int i = 0; i < outputSize; i++) {
            error[i] = current[i] - target[i];
        }
        
        // 反向传播计算梯度
        List<double[][]> weightGradients = new ArrayList<>();
        List<double[]> biasGradients = new ArrayList<>();
        
        // 输出层梯度
        double[][] outputWeightGradient = new double[layerSizes[layerSizes.length - 2]][outputSize];
        double[] outputBiasGradient = new double[outputSize];
        
        for (int i = 0; i < layerSizes[layerSizes.length - 2]; i++) {
            for (int j = 0; j < outputSize; j++) {
                outputWeightGradient[i][j] = activations.get(activations.size() - 2)[i] * error[j];
            }
        }
        
        for (int i = 0; i < outputSize; i++) {
            outputBiasGradient[i] = error[i];
        }
        
        weightGradients.add(outputWeightGradient);
        biasGradients.add(outputBiasGradient);
        
        // 隐藏层梯度
        for (int i = weights.size() - 2; i >= 0; i--) {
            double[] nextError = new double[layerSizes[i + 1]];
            double[][] currentWeight = weights.get(i + 1);
            
            for (int j = 0; j < layerSizes[i + 1]; j++) {
                double sum = 0;
                for (int k = 0; k < layerSizes[i + 2]; k++) {
                    sum += error[k] * currentWeight[j][k];
                }
                nextError[j] = sum * reluDerivative(preActivations.get(i))[j];
            }
            
            double[][] weightGradient = new double[layerSizes[i]][layerSizes[i + 1]];
            double[] biasGradient = new double[layerSizes[i + 1]];
            
            for (int j = 0; j < layerSizes[i]; j++) {
                for (int k = 0; k < layerSizes[i + 1]; k++) {
                    weightGradient[j][k] = activations.get(i)[j] * nextError[k];
                }
            }
            
            for (int j = 0; j < layerSizes[i + 1]; j++) {
                biasGradient[j] = nextError[j];
            }
            
            weightGradients.add(0, weightGradient);
            biasGradients.add(0, biasGradient);
            error = nextError;
        }
        
        // 更新权重和偏置（带L2正则化）
        for (int i = 0; i < weights.size(); i++) {
            double[][] weight = weights.get(i);
            double[] bias = biases.get(i);
            double[][] weightGradient = weightGradients.get(i);
            double[] biasGradient = biasGradients.get(i);
            
            for (int j = 0; j < weight.length; j++) {
                for (int k = 0; k < weight[j].length; k++) {
                    // L2正则化
                    weight[j][k] -= learningRate * (weightGradient[j][k] + l2Regularization * weight[j][k]);
                }
            }
            
            for (int j = 0; j < bias.length; j++) {
                bias[j] -= learningRate * biasGradient[j];
            }
        }
    }
    
    // 归一化训练集
    public void standardize(double[][] input, double[][] output) {
        if (input.length == 0 || output.length == 0) {
            // 空输入，不进行归一化
            return;
        }
        
        int inputDim = input[0].length;
        int outputDim = output[0].length;
        
        // 计算输入均值和标准差
        inputMean = new double[inputDim];
        inputStd = new double[inputDim];
        
        for (int i = 0; i < inputDim; i++) {
            double sum = 0;
            for (double[] row : input) {
                sum += row[i];
            }
            inputMean[i] = sum / input.length;
            
            double squaredSum = 0;
            for (double[] row : input) {
                squaredSum += Math.pow(row[i] - inputMean[i], 2);
            }
            inputStd[i] = Math.sqrt(squaredSum / input.length);
            if (inputStd[i] < 1e-8) inputStd[i] = 1;
        }
        
        // 计算输出均值和标准差
        outputMean = new double[outputDim];
        outputStd = new double[outputDim];
        
        for (int i = 0; i < outputDim; i++) {
            double sum = 0;
            for (double[] row : output) {
                sum += row[i];
            }
            outputMean[i] = sum / output.length;
            
            double squaredSum = 0;
            for (double[] row : output) {
                squaredSum += Math.pow(row[i] - outputMean[i], 2);
            }
            outputStd[i] = Math.sqrt(squaredSum / output.length);
            if (outputStd[i] < 1e-8) outputStd[i] = 1;
        }
        
        // 归一化输入
        for (int i = 0; i < input.length; i++) {
            for (int j = 0; j < inputDim; j++) {
                input[i][j] = (input[i][j] - inputMean[j]) / inputStd[j];
            }
        }
        
        // 归一化输出
        for (int i = 0; i < output.length; i++) {
            for (int j = 0; j < outputDim; j++) {
                output[i][j] = (output[i][j] - outputMean[j]) / outputStd[j];
            }
        }
    }
    
    // 归一化单个输入
    public double[] standardizeInput(double[] input) {
        if (inputMean == null || inputStd == null) {
            // 归一化参数未初始化，直接返回原始输入
            return input;
        }
        
        double[] standardized = new double[input.length];
        for (int i = 0; i < input.length; i++) {
            standardized[i] = (input[i] - inputMean[i]) / inputStd[i];
        }
        return standardized;
    }
    
    // 反归一化输出
    public double[] unstandardizeOutput(double[] output) {
        if (outputMean == null || outputStd == null) {
            // 归一化参数未初始化，直接返回原始输出
            return output;
        }
        
        double[] unstandardized = new double[output.length];
        for (int i = 0; i < output.length; i++) {
            unstandardized[i] = output[i] * outputStd[i] + outputMean[i];
        }
        return unstandardized;
    }
    
    // 计算均方误差
    public double meanSquaredError(double[] output, double[] target) {
        double sum = 0;
        for (int i = 0; i < output.length; i++) {
            sum += Math.pow(output[i] - target[i], 2);
        }
        return sum / output.length;
    }
    
    // 计算准确率（对于YOLO v3输出格式的特殊处理）
    public double accuracy(double[] output, double[] target) {
        // 计算非零目标值的数量
        int nonZeroTargetCount = 0;
        int correctCount = 0;
        
        for (int i = 0; i < output.length; i++) {
            if (Math.abs(target[i]) > 1e-8) { // 目标值非零
                nonZeroTargetCount++;
                // 对于非零目标值，检查预测值是否在合理范围内
                if (Math.abs(output[i] - target[i]) / Math.abs(target[i]) < 0.3) { // 放宽阈值到30%
                    correctCount++;
                }
            } else {
                // 对于零目标值，检查预测值是否接近零
                if (Math.abs(output[i]) < 0.1) {
                    correctCount++;
                }
            }
        }
        
        // 避免除零错误
        if (nonZeroTargetCount == 0) {
            return 1.0;
        }
        
        // 返回准确率
        return (double) correctCount / output.length;
    }
    
    // 拟合数据
    public void fit(double[][] input, double[][] output, int epochs, int batchSize, double validationSplit, boolean EarlyStopping) {
        // 划分训练集和验证集
        int trainSize = (int) (input.length * (1 - validationSplit));
        double[][] trainInput = Arrays.copyOfRange(input, 0, trainSize);
        double[][] trainOutput = Arrays.copyOfRange(output, 0, trainSize);
        double[][] valInput = Arrays.copyOfRange(input, trainSize, input.length);
        double[][] valOutput = Arrays.copyOfRange(output, trainSize, output.length);
        
        // 归一化
        standardize(trainInput, trainOutput);
        
        // 归一化验证集
        for (int i = 0; i < valInput.length; i++) {
            valInput[i] = standardizeInput(valInput[i]);
            for (int j = 0; j < valOutput[i].length; j++) {
                valOutput[i][j] = (valOutput[i][j] - outputMean[j]) / outputStd[j];
            }
        }
        
        // 早停法参数
        int patience = 20;
        int patienceCounter = 0;
        double bestValLoss = Double.MAX_VALUE;
        List<double[][]> bestWeights = null;
        List<double[]> bestBiases = null;
        
        // 记录损失和准确率
        List<Double> trainLosses = new ArrayList<>();
        List<Double> valLosses = new ArrayList<>();
        List<Double> trainAccuracies = new ArrayList<>();
        List<Double> valAccuracies = new ArrayList<>();
        
        for (int epoch = 0; epoch < epochs; epoch++) {
            // 训练
            double trainLoss = 0;
            double trainAccuracy = 0;
            
            // 随机打乱训练数据
            int[] indices = new int[trainInput.length];
            for (int i = 0; i < indices.length; i++) indices[i] = i;
            shuffleArray(indices);
            
            // Mini-Batch Gradient Descent
            for (int i = 0; i < trainInput.length; i += batchSize) {
                int batchEnd = Math.min(i + batchSize, trainInput.length);
                int batchSizeActual = batchEnd - i;
                
                // 为每个batch计算累积梯度
                List<double[][]> batchWeightGradients = new ArrayList<>();
                List<double[]> batchBiasGradients = new ArrayList<>();
                
                // 初始化梯度为0
                for (int j = 0; j < weights.size(); j++) {
                    double[][] weight = weights.get(j);
                    double[] bias = biases.get(j);
                    double[][] weightGradient = new double[weight.length][weight[0].length];
                    double[] biasGradient = new double[bias.length];
                    batchWeightGradients.add(weightGradient);
                    batchBiasGradients.add(biasGradient);
                }
                
                double batchLoss = 0;
                double batchAccuracy = 0;
                
                for (int j = i; j < batchEnd; j++) {
                    int idx = indices[j];
                    double[] pred = forward(trainInput[idx]);
                    batchLoss += meanSquaredError(pred, trainOutput[idx]);
                    
                    // 使用反归一化后的数据计算准确率
                    double[] unstandardizedPred = unstandardizeOutput(pred);
                    double[] unstandardizedTarget = new double[outputSize];
                    for (int k = 0; k < outputSize; k++) {
                        unstandardizedTarget[k] = trainOutput[idx][k] * outputStd[k] + outputMean[k];
                    }
                    batchAccuracy += accuracy(unstandardizedPred, unstandardizedTarget);
                    
                    // 计算单个样本的梯度
                    List<double[]> activations = new ArrayList<>();
                    List<double[]> preActivations = new ArrayList<>();
                    double[] current = trainInput[idx];
                    activations.add(current);
                    
                    for (int k = 0; k < weights.size(); k++) {
                        double[][] weight = weights.get(k);
                        double[] bias = biases.get(k);
                        double[] next = new double[bias.length];
                        
                        for (int l = 0; l < bias.length; l++) {
                            next[l] = bias[l];
                            for (int m = 0; m < current.length; m++) {
                                next[l] += current[m] * weight[m][l];
                            }
                        }
                        
                        preActivations.add(next);
                        
                        if (k < weights.size() - 1) {
                            current = relu(next);
                        } else {
                            current = next;
                        }
                        activations.add(current);
                    }
                    
                    // 计算输出层误差
                    double[] error = new double[outputSize];
                    for (int k = 0; k < outputSize; k++) {
                        error[k] = current[k] - trainOutput[idx][k];
                    }
                    
                    // 反向传播计算梯度
                    List<double[][]> weightGradients = new ArrayList<>();
                    List<double[]> biasGradients = new ArrayList<>();
                    
                    // 输出层梯度
                    double[][] outputWeightGradient = new double[layerSizes[layerSizes.length - 2]][outputSize];
                    double[] outputBiasGradient = new double[outputSize];
                    
                    for (int k = 0; k < layerSizes[layerSizes.length - 2]; k++) {
                        for (int l = 0; l < outputSize; l++) {
                            outputWeightGradient[k][l] = activations.get(activations.size() - 2)[k] * error[l];
                        }
                    }
                    
                    for (int k = 0; k < outputSize; k++) {
                        outputBiasGradient[k] = error[k];
                    }
                    
                    weightGradients.add(outputWeightGradient);
                    biasGradients.add(outputBiasGradient);
                    
                    // 隐藏层梯度
                    for (int k = weights.size() - 2; k >= 0; k--) {
                        double[] nextError = new double[layerSizes[k + 1]];
                        double[][] currentWeight = weights.get(k + 1);
                        
                        for (int l = 0; l < layerSizes[k + 1]; l++) {
                            double sum = 0;
                            for (int m = 0; m < layerSizes[k + 2]; m++) {
                                sum += error[m] * currentWeight[l][m];
                            }
                            nextError[l] = sum * reluDerivative(preActivations.get(k))[l];
                        }
                        
                        double[][] weightGradient = new double[layerSizes[k]][layerSizes[k + 1]];
                        double[] biasGradient = new double[layerSizes[k + 1]];
                        
                        for (int l = 0; l < layerSizes[k]; l++) {
                            for (int m = 0; m < layerSizes[k + 1]; m++) {
                                weightGradient[l][m] = activations.get(k)[l] * nextError[m];
                            }
                        }
                        
                        for (int l = 0; l < layerSizes[k + 1]; l++) {
                            biasGradient[l] = nextError[l];
                        }
                        
                        weightGradients.add(0, weightGradient);
                        biasGradients.add(0, biasGradient);
                        error = nextError;
                    }
                    
                    // 累积梯度
                    for (int k = 0; k < weights.size(); k++) {
                        double[][] batchWeightGradient = batchWeightGradients.get(k);
                        double[] batchBiasGradient = batchBiasGradients.get(k);
                        double[][] sampleWeightGradient = weightGradients.get(k);
                        double[] sampleBiasGradient = biasGradients.get(k);
                        
                        for (int l = 0; l < batchWeightGradient.length; l++) {
                            for (int m = 0; m < batchWeightGradient[l].length; m++) {
                                batchWeightGradient[l][m] += sampleWeightGradient[l][m];
                            }
                        }
                        
                        for (int l = 0; l < batchBiasGradient.length; l++) {
                            batchBiasGradient[l] += sampleBiasGradient[l];
                        }
                    }
                }
                
                // 使用AdamW优化器更新参数
                updateParamsWithAdamW(batchWeightGradients, batchBiasGradients, batchSizeActual);
                
                trainLoss += batchLoss;
                trainAccuracy += batchAccuracy;
            }
            
            trainLoss /= trainInput.length;
            trainAccuracy /= trainInput.length;
            
            // 验证
            double valLoss = 0;
            double valAccuracy = 0;
            
            if (valInput.length > 0) {
                for (int i = 0; i < valInput.length; i++) {
                    double[] pred = forward(valInput[i]);
                    valLoss += meanSquaredError(pred, valOutput[i]);
                    
                    // 使用反归一化后的数据计算准确率
                    double[] unstandardizedPred = unstandardizeOutput(pred);
                    double[] unstandardizedTarget = new double[outputSize];
                    for (int j = 0; j < outputSize; j++) {
                        unstandardizedTarget[j] = valOutput[i][j] * outputStd[j] + outputMean[j];
                    }
                    valAccuracy += accuracy(unstandardizedPred, unstandardizedTarget);
                }
                
                valLoss /= valInput.length;
                valAccuracy /= valInput.length;
            } else {
                // 验证集为空，使用训练集损失作为验证损失
                valLoss = trainLoss;
                valAccuracy = trainAccuracy;
            }
            
            // 记录
            trainLosses.add(trainLoss);
            valLosses.add(valLoss);
            trainAccuracies.add(trainAccuracy);
            valAccuracies.add(valAccuracy);
            
            // 早停法
            if (EarlyStopping && valLoss < bestValLoss && patienceCounter < patience) {
                bestValLoss = valLoss;
                patienceCounter = 0;
                // 保存最佳模型
                bestWeights = deepCopyWeights(weights);
                bestBiases = deepCopyBiases(biases);
            } else {
                patienceCounter++;
                if (patienceCounter >= patience) {
                    System.out.println("早停：验证集损失连续" + patience + "轮不下降");
                    break;
                }
            }
            
            // 打印进度
            if ((epoch + 1) % 10 == 0) {
                System.out.printf("Epoch %d/%d, Train Loss: %.4f, Train Acc: %.4f, Val Loss: %.4f, Val Acc: %.4f%n",
                        epoch + 1, epochs, trainLoss, trainAccuracy, valLoss, valAccuracy);
            }
        }
        
        // 恢复最佳模型
        if (bestWeights != null) {
            weights = bestWeights;
            biases = bestBiases;
        }
    /*     
        // 展示loss和accuracy图表
        // 创建损失图表
        JFrame lossFrame = new JFrame("训练和验证损失");
        lossFrame.setSize(800, 600);
        lossFrame.setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE);
        lossFrame.add(new ChartPanel(trainLosses, valLosses, "损失", "轮次", "损失值"));
        lossFrame.setVisible(true);
        
        // 创建准确率图表
        JFrame accFrame = new JFrame("训练和验证准确率");
        accFrame.setSize(800, 600);
        accFrame.setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE);
        accFrame.add(new ChartPanel(trainAccuracies, valAccuracies, "准确率", "轮次", "准确率值"));
        accFrame.setVisible(true);
     */   
        System.out.println("训练完成，最佳验证集损失: " + bestValLoss);
    }
    
    // 预测
    public double[] predict(double[] input) {
        double[] standardizedInput;
        double[] standardizedOutput;
        double[] result;
        
        // 检查归一化参数是否已初始化
        if (inputMean != null && inputStd != null && outputMean != null && outputStd != null) {
            // 进行归一化和反归一化
            standardizedInput = standardizeInput(input);
            standardizedOutput = forward(standardizedInput);
            result = unstandardizeOutput(standardizedOutput);
        } else {
            // 直接使用原始输入和输出
            standardizedOutput = forward(input);
            result = standardizedOutput;
        }
        
        return result;
    }
    
    // 保存模型
    public void save(String filename) throws IOException {
        try (ObjectOutputStream oos = new ObjectOutputStream(new FileOutputStream(filename))) {
            oos.writeObject(layerSizes);
            oos.writeObject(weights);
            oos.writeObject(biases);
            oos.writeObject(learningRate);
            oos.writeObject(l2Regularization);
            oos.writeObject(inputMean);
            oos.writeObject(inputStd);
            oos.writeObject(outputMean);
            oos.writeObject(outputStd);
        }
        System.out.println("模型已保存到 " + filename);
    }
    
    // 加载模型
    public static Net load(String filename) throws IOException, ClassNotFoundException {
        try (ObjectInputStream ois = new ObjectInputStream(new FileInputStream(filename))) {
            int[] layerSizes = (int[]) ois.readObject();
            Net net = new Net(layerSizes);
            net.weights = (List<double[][]>) ois.readObject();
            net.biases = (List<double[]>) ois.readObject();
            net.learningRate = (double) ois.readObject();
            net.l2Regularization = (double) ois.readObject();
            net.inputMean = (double[]) ois.readObject();
            net.inputStd = (double[]) ois.readObject();
            net.outputMean = (double[]) ois.readObject();
            net.outputStd = (double[]) ois.readObject();
            System.out.println("模型已从 " + filename + " 加载");
            return net;
        }
    }
    
    // 辅助函数：打乱数组
    private void shuffleArray(int[] array) {
        Random random = new Random();
        for (int i = array.length - 1; i > 0; i--) {
            int j = random.nextInt(i + 1);
            int temp = array[i];
            array[i] = array[j];
            array[j] = temp;
        }
    }
    
    // 辅助函数：深拷贝权重
    private List<double[][]> deepCopyWeights(List<double[][]> weights) {
        List<double[][]> copy = new ArrayList<>();
        for (double[][] weight : weights) {
            double[][] weightCopy = new double[weight.length][];
            for (int i = 0; i < weight.length; i++) {
                weightCopy[i] = Arrays.copyOf(weight[i], weight[i].length);
            }
            copy.add(weightCopy);
        }
        return copy;
    }
    
    // 辅助函数：深拷贝偏置
    private List<double[]> deepCopyBiases(List<double[]> biases) {
        List<double[]> copy = new ArrayList<>();
        for (double[] bias : biases) {
            copy.add(Arrays.copyOf(bias, bias.length));
        }
        return copy;
    }
    
    // 复制权重和偏置到目标网络
    public void copyWeights(Net sourceNet) {
        this.weights = deepCopyWeights(sourceNet.weights);
        this.biases = deepCopyBiases(sourceNet.biases);
    }
    
    // 软更新网络参数
    public void softUpdate(Net sourceNet, double tau) {
        for (int i = 0; i < weights.size(); i++) {
            double[][] weight = weights.get(i);
            double[] bias = biases.get(i);
            double[][] sourceWeight = sourceNet.weights.get(i);
            double[] sourceBias = sourceNet.biases.get(i);
            
            // 权重软更新
            for (int j = 0; j < weight.length; j++) {
                for (int k = 0; k < weight[j].length; k++) {
                    weight[j][k] = (1 - tau) * weight[j][k] + tau * sourceWeight[j][k];
                }
            }
            
            // 偏置软更新
            for (int j = 0; j < bias.length; j++) {
                bias[j] = (1 - tau) * bias[j] + tau * sourceBias[j];
            }
        }
    }
    
    // 自定义图表面板类 - Android平台不支持，已注释
    /*
    private static class ChartPanel extends JPanel {
        private List<Double> data1;
        private List<Double> data2;
        private String title;
        private String xLabel;
        private String yLabel;
        
        public ChartPanel(List<Double> data1, List<Double> data2, String title, String xLabel, String yLabel) {
            this.data1 = data1;
            this.data2 = data2;
            this.title = title;
            this.xLabel = xLabel;
            this.yLabel = yLabel;
        }
        
        @Override
        protected void paintComponent(Graphics g) {
            super.paintComponent(g);
            
            int width = getWidth();
            int height = getHeight();
            int padding = 50;
            
            // 绘制标题
            g.setFont(new Font("Arial", Font.BOLD, 16));
            g.drawString(title, width / 2 - 50, 20);
            
            // 绘制坐标轴
            g.drawLine(padding, padding, padding, height - padding);
            g.drawLine(padding, height - padding, width - padding, height - padding);
            
            // 绘制标签
            g.setFont(new Font("Arial", Font.PLAIN, 12));
            g.drawString(xLabel, width / 2, height - 10);
            g.drawString(yLabel, 10, height / 2);
            
            // 计算数据范围
            double min1 = Collections.min(data1);
            double max1 = Collections.max(data1);
            double min2 = Collections.min(data2);
            double max2 = Collections.max(data2);
            double min = Math.min(min1, min2);
            double max = Math.max(max1, max2);
            double range = max - min;
            
            // 绘制x轴刻度和数字
            int xTicks = 10;
            for (int i = 0; i <= xTicks; i++) {
                int x = padding + (i * (width - 2 * padding)) / xTicks;
                g.drawLine(x, height - padding, x, height - padding + 5);
                int value = (int) (i * data1.size() / xTicks);
                g.drawString(String.valueOf(value), x - 10, height - padding + 20);
            }
            
            // 绘制y轴刻度和数字
            int yTicks = 10;
            for (int i = 0; i <= yTicks; i++) {
                int y = height - padding - (i * (height - 2 * padding)) / yTicks;
                g.drawLine(padding - 5, y, padding, y);
                double value = min + (i * range) / yTicks;
                g.drawString(String.format("%.2f", value), padding - 40, y + 5);
            }
            
            // 绘制数据点和线条
            g.setColor(Color.RED);
            for (int i = 0; i < data1.size(); i++) {
                int x = padding + (i * (width - 2 * padding)) / (data1.size() - 1);
                int y = height - padding - (int) ((data1.get(i) - min) * (height - 2 * padding) / range);
                if (i > 0) {
                    int prevX = padding + ((i - 1) * (width - 2 * padding)) / (data1.size() - 1);
                    int prevY = height - padding - (int) ((data1.get(i - 1) - min) * (height - 2 * padding) / range);
                    g.drawLine(prevX, prevY, x, y);
                }
                g.fillOval(x - 2, y - 2, 4, 4);
            }
            
            g.setColor(Color.BLUE);
            for (int i = 0; i < data2.size(); i++) {
                int x = padding + (i * (width - 2 * padding)) / (data2.size() - 1);
                int y = height - padding - (int) ((data2.get(i) - min) * (height - 2 * padding) / range);
                if (i > 0) {
                    int prevX = padding + ((i - 1) * (width - 2 * padding)) / (data2.size() - 1);
                    int prevY = height - padding - (int) ((data2.get(i - 1) - min) * (height - 2 * padding) / range);
                    g.drawLine(prevX, prevY, x, y);
                }
                g.fillOval(x - 2, y - 2, 4, 4);
            }
            
            // 绘制图例
            g.setColor(Color.RED);
            g.fillRect(width - padding + 10, padding + 10, 10, 10);
            g.setColor(Color.BLACK);
            g.drawString("训练", width - padding + 25, padding + 20);
            
            g.setColor(Color.BLUE);
            g.fillRect(width - padding + 10, padding + 30, 10, 10);
            g.setColor(Color.BLACK);
            g.drawString("验证", width - padding + 25, padding + 40);
        }
    }
    */
    
    // 主函数用于测试 - Android平台不支持，已注释
    /*
    public static void main(String[] args) throws IOException, ClassNotFoundException {
        // 创建一个简单的神经网络：输入4维，隐藏层64维，输出3维
        Net net = new Net(4, 64, 3);
        
        // 设置Adam优化器参数
        net.setLearningRate(0.001);       // 设置学习率为0.001
        net.setL2Regularization(0.0001);  // 设置L2权重衰减系数为0.0001
        net.setAdamParams(0.9, 0.999, 1e-8);  // 设置Adam参数：beta1=0.9, beta2=0.999, epsilon=1e-8
        
        System.out.println("Adam优化器参数设置：");
        System.out.println("  学习率: 0.001");
        System.out.println("  L2权重衰减系数: 0.0001");
        System.out.println("  beta1: 0.9");
        System.out.println("  beta2: 0.999");
        System.out.println("  epsilon: 1e-8");
        System.out.println();
        
        // 生成随机训练数据
        int sampleSize = 1000;
        double[][] input = new double[sampleSize][4];
        double[][] output = new double[sampleSize][3];
        Random random = new Random();
        
        for (int i = 0; i < sampleSize; i++) {
            for (int j = 0; j < 4; j++) {
                input[i][j] = random.nextDouble() * 10 - 5;
            }
            for (int j = 0; j < 3; j++) {
                output[i][j] = random.nextDouble() * 5;
            }
        }
        
        // 拟合数据
        net.fit(input, output, 100, 32, 0.2, false);
        
        // 保存模型
        net.save("model.ser");
        
        // 加载模型
        Net loadedNet = Net.load("model.ser");
        
        // 测试预测
        double[] testInput = {1.0, 2.0, 3.0, 4.0};
        double[] prediction = loadedNet.predict(testInput);
        System.out.println("测试输入: " + Arrays.toString(testInput));
        System.out.println("预测输出: " + Arrays.toString(prediction));
    }
    */
}