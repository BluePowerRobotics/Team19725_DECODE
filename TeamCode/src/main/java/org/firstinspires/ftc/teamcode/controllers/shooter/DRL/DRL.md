# DRL包使用说明
DRL实现了深度学习和强化学习的基本库，并内置若干训练高级算法和一个发射模拟器，其代码主要架构为：

Net—支持—>**Agent**<—reward(action)—>**Environment**<—模拟—Simulator

## Environment
用于训练Agent的类，主要由reward函数定义,内置了经验回放所需的database
### reward(action)
reward函数直接决定了一个环境的性质，默认为空，必须在训练前**手动重载**(DDPG，Gaussian除外)，否则无法训练agent。重载时必须**以set_state函数更新状态**，并**以double形式返回计算出的奖励**，具体的实现则需要视待解决的问题而定。
### record(simulator, v_max, d_max, s)
record函数用于实验收集数据，可以重载，重载时必须将得到的数据**以（state,action）形式存入database**，重载后可用simulate函数直接自动多次实验。输入的s表示噪声强度，simulator类用于物理模拟，可以不使用。
## Agent
Agent类实现了强化学习所需的智能体，可学习Environment类，可用save函数保存至文件,内置了高级训练算法：

### DDPG_learn(env,input_dim,output_dim,batchSize,earlystopping,n)
使用简化的DDPG训练，实质是行为克隆，直接模仿env.database的记录，因此可以不重载reward。需要大量优质数据，结果稳定。

### TD3_learn(env,input_dim,output_dim,batchSize,earlystopping,n)
使用带有TD3的DDPG算法训练，数据量需求较DDPG更少，结果稳定。

### Gaussian_learn(...)
使用简化的随机高斯策略梯度,本质是添加了随机正态分布输出的DDPG，相较DDPG更不稳定，但结果随机性更强，适用于较为混沌的环境

### RF_learn(...)
使用带基线的REINFORCE算法，**只适用于离散动作**，适合reward随机性低的环境

### A2C_learn(...)
使用带基线的Actor-Critic算法，**只适用于离散动作**，相较RF更不稳定，适合reward定义复杂或随机性强的环境

### ARSQ_learn(...)
使用ARSQ算法，需求数据较DDPG更少，但当前仍未完全实现。

## Net
Net提供了对Agent中各个算法的支持，也可直接单独用于DL算法
## fit(input,output,epoches,batchSize,validationSplit,Earlystopping)
直接拟合数据，单独表现不稳定，建议从Agent类间接调用

## 函数列表

本列表详细说明 `Agent.java`、`Net.java` 和 `Environment.java` 三个文件中各个函数的格式和作用

## 1. Agent.java

### 1.1 构造函数

**函数签名：**
```java
public Agent(int i, int o)
```

**参数：**
- `i`：观察数据维度
- `o`：行动数据维度

**作用：**
初始化智能体，设置观察维度和行动维度，创建行动范围数组和集成学习网络列表。

### 1.2 设置行动范围

**函数签名：**
```java
public void set_action_range(double[][] action_range)
```

**参数：**
- `action_range`：行动数据每个量的最大值和最小值，格式为 [行动维度][2]，其中第二个维度为 [最小值, 最大值]

**作用：**
设置智能体的行动范围，用于后续的动作裁剪。

### 1.3 保存Agent对象

**函数签名：**
```java
public void save(String filename) throws IOException
```

**参数：**
- `filename`：保存文件路径

**作用：**
将整个Agent对象保存到文件，包括策略网络和集成网络。

### 1.4 加载Agent对象

**函数签名：**
```java
public static Agent load(String filename) throws IOException, ClassNotFoundException
```

**参数：**
- `filename`：加载文件路径

**返回值：**
- 加载的Agent对象

**作用：**
从文件加载整个Agent对象。

### 1.5 显示参数信息

**函数签名：**
```java
public void show_params()
```

**作用：**
显示智能体的基本信息，包括观察维度、行动维度、行动范围、策略网络信息和集成学习信息。

### 1.6 决策函数

**函数签名：**
```java
public double[] decide(double[] input)
```

**参数：**
- `input`：输入观察数据

**返回值：**
- 行动向量

**作用：**
根据输入观察数据做出决策，返回行动向量。如果策略网络未初始化，返回随机行动；如果有集成网络，使用集成学习进行决策。

### 1.7 集成学习决策

**函数签名：**
```java
private double[] ensembleDecide(double[] input)
```

**参数：**
- `input`：输入观察数据

**返回值：**
- 集成后的行动向量

**作用：**
使用集成学习方法，对所有集成模型的预测结果取平均值，然后进行裁剪和离散化。

### 1.8 生成随机行动

**函数签名：**
```java
private double[] randomAction()
```

**返回值：**
- 随机行动向量

**作用：**
生成随机行动，用于策略网络未初始化时的 fallback。

### 1.9 裁剪和离散化

**函数签名：**
```java
private double[] clipAndDiscretize(double[] output)
```

**参数：**
- `output`：原始输出向量

**返回值：**
- 裁剪和离散化后的行动向量

**作用：**
将网络输出裁剪到有效范围，并对离散行动进行四舍五入。

### 1.10 DDPG策略训练

**函数签名：**
```java
public void DDPG_learn(Environment env, int inputDim, int outputDim, int batchSize, boolean earlystopping, int n)
```

**参数：**
- `env`：环境对象
- `inputDim`：输入维度
- `outputDim`：输出维度
- `batchSize`：批处理大小
- `earlystopping`：是否使用早停
- `n`：训练轮数

**作用：**
使用DDPG策略训练智能体，包括数据准备、超参数网格搜索、模型训练和集成学习模型选择。

### 1.11 高斯策略训练

**函数签名：**
```java
public void Gaussian_learn(Environment env, int inputDim, int outputDim, int batchSize, boolean earlystopping, int n)
```

**参数：**
- `env`：环境对象
- `inputDim`：输入维度
- `outputDim`：输出维度
- `batchSize`：批处理大小
- `earlystopping`：是否使用早停
- `n`：训练轮数

**作用：**
使用高斯策略训练智能体，训练均值网络和方差网络。

### 1.12 计算高斯策略验证损失

**函数签名：**
```java
private double calculateGaussianValidationLoss(Net meanNet, Net varianceNet, double[][] input, double[][] output)
```

**参数：**
- `meanNet`：均值网络
- `varianceNet`：方差网络
- `input`：输入数据
- `output`：输出数据

**返回值：**
- 平均验证损失

**作用：**
计算高斯策略的验证损失，使用高斯对数似然损失函数。

### 1.13 计算验证损失

**函数签名：**
```java
private double calculateValidationLoss(Net net, double[][] input, double[][] output)
```

**参数：**
- `net`：网络模型
- `input`：输入数据
- `output`：输出数据

**返回值：**
- 平均验证损失

**作用：**
计算验证损失和准确率，使用均方误差作为损失函数。

### 1.14 计算准确率

**函数签名：**
```java
private double accuracy(double[] output, double[] target)
```

**参数：**
- `output`：网络输出
- `target`：目标值

**返回值：**
- 准确率（0或1）

**作用：**
计算准确率，当相对误差小于10%时视为正确。

### 1.15 计算均方误差

**函数签名：**
```java
private double meanSquaredError(double[] pred, double[] target)
```

**参数：**
- `pred`：预测值
- `target`：目标值

**返回值：**
- 均方误差

**作用：**
计算预测值与目标值之间的均方误差。

### 1.16 REINFORCE算法训练

**函数签名：**
```java
public void RF_learn(Environment environment, int o, int n)
```

**参数：**
- `environment`：环境对象
- `o`：输出维度
- `n`：训练轮数

**作用：**
使用带基线的REINFORCE算法训练智能体，只支持离散行动空间。

### 1.17 A2C算法训练

**函数签名：**
```java
public void A2C_learn(Environment environment, int o, int n)
```

**参数：**
- `environment`：环境对象
- `o`：输出维度
- `n`：训练轮数

**作用：**
使用A2C算法训练智能体，只支持离散行动空间。

### 1.18 ARSQ算法训练

**函数签名：**
```java
public void ARSQ_learn(Environment environment, int o, int n)
```

**参数：**
- `environment`：环境对象
- `o`：输出维度
- `n`：训练轮数

**作用：**
使用ARSQ算法训练智能体，支持连续和离散行动空间，包括经验回放、Q网络训练和策略网络训练。**（仍是半成品）**

### 1.19 采样动作

**函数签名：**
```java
private int sampleAction(double[] probs)
```

**参数：**
- `probs`：动作概率分布

**返回值：**
- 采样的动作索引

**作用：**
根据概率分布采样动作，用于离散行动空间。


**作用：**
测试智能体的训练和预测功能，包括环境创建、数据生成、智能体训练和模型保存。

## 2. Net.java

### 2.1 构造函数

**函数签名：**
```java
public Net(int... layerSizes)
```

**参数：**
- `layerSizes`：各层神经元数量，例如new Net(4, 64, 3)表示输入层4个神经元，隐藏层64个，输出层3个

**作用：**
初始化神经网络，设置各层大小，初始化权重和偏置，设置优化器参数。

### 2.2 设置学习率

**函数签名：**
```java
public void setLearningRate(double learningRate)
```

**参数：**
- `learningRate`：学习率

**作用：**
设置神经网络的学习率。

### 2.3 设置L2正则化系数

**函数签名：**
```java
public void setL2Regularization(double l2Regularization)
```

**参数：**
- `l2Regularization`：L2正则化系数

**作用：**
设置神经网络的L2正则化系数。

### 2.4 设置Adam优化器参数

**函数签名：**
```java
public void setAdamParams(double beta1, double beta2, double epsilon)
```

**参数：**
- `beta1`：一阶矩估计的指数衰减率
- `beta2`：二阶矩估计的指数衰减率
- `epsilon`：防止除零的小常数

**作用：**
设置Adam优化器的参数。

### 2.5 获取学习率

**函数签名：**
```java
public double getLearningRate()
```

**返回值：**
- 学习率

**作用：**
获取神经网络的学习率。

### 2.6 获取L2正则化系数

**函数签名：**
```java
public double getL2Regularization()
```

**返回值：**
- L2正则化系数

**作用：**
获取神经网络的L2正则化系数。

### 2.7 获取网络层数

**函数签名：**
```java
public int getLayerCount()
```

**返回值：**
- 网络层数

**作用：**
获取神经网络的层数。

### 2.8 获取各层神经元数量

**函数签名：**
```java
public int[] getLayerSizes()
```

**返回值：**
- 各层神经元数量数组

**作用：**
获取神经网络各层的神经元数量。

### 2.9 AdamW优化器更新参数

**函数签名：**
```java
private void updateParamsWithAdamW(List<double[][]> weightGradients, List<double[]> biasGradients, int batchSizeActual)
```

**参数：**
- `weightGradients`：权重梯度
- `biasGradients`：偏置梯度
- `batchSizeActual`：实际批量大小

**作用：**
使用AdamW优化器更新神经网络参数，包括权重衰减和偏差修正。

### 2.10 初始化Adam矩估计

**函数签名：**
```java
private void initializeAdamMoments()
```

**作用：**
初始化Adam优化器的一阶矩估计和二阶矩估计。

### 2.11 初始化权重和偏置

**函数签名：**
```java
private void initializeWeights()
```

**作用：**
初始化神经网络的权重和偏置，使用He初始化方法。

### 2.12 ReLU激活函数

**函数签名：**
```java
private double[] relu(double[] x)
```

**参数：**
- `x`：输入向量

**返回值：**
- 激活后的向量

**作用：**
对输入向量应用ReLU激活函数。

### 2.13 ReLU导数

**函数签名：**
```java
private double[] reluDerivative(double[] x)
```

**参数：**
- `x`：输入向量

**返回值：**
- ReLU导数向量

**作用：**
计算ReLU激活函数的导数。

### 2.14 前向传播

**函数签名：**
```java
public double[] forward(double[] input)
```

**参数：**
- `input`：输入向量

**返回值：**
- 网络预测输出向量

**作用：**
执行神经网络的前向传播计算。

### 2.15 反向传播

**函数签名：**
```java
public void backward(double[] input, double[] target)
```

**参数：**
- `input`：输入向量
- `target`：目标输出向量

**作用：**
执行神经网络的反向传播，计算梯度并更新参数。

### 2.16 归一化训练集

**函数签名：**
```java
public void standardize(double[][] input, double[][] output)
```

**参数：**
- `input`：输入数据集
- `output`：输出数据集

**作用：**
归一化训练集，计算均值和标准差，并对数据进行归一化处理。

### 2.17 归一化单个输入

**函数签名：**
```java
public double[] standardizeInput(double[] input)
```

**参数：**
- `input`：输入向量

**返回值：**
- 归一化后的输入向量

**作用：**
对单个输入向量进行归一化处理。

### 2.18 反归一化输出

**函数签名：**
```java
public double[] unstandardizeOutput(double[] output)
```

**参数：**
- `output`：归一化的输出向量

**返回值：**
- 反归一化后的输出向量

**作用：**
对归一化的输出向量进行反归一化处理。

### 2.19 计算均方误差

**函数签名：**
```java
public double meanSquaredError(double[] output, double[] target)
```

**参数：**
- `output`：网络输出
- `target`：目标值

**返回值：**
- 均方误差

**作用：**
计算网络输出与目标值之间的均方误差。

### 2.20 计算准确率

**函数签名：**
```java
public double accuracy(double[] output, double[] target)
```

**参数：**
- `output`：网络输出
- `target`：目标值

**返回值：**
- 准确率（0或1）

**作用：**
计算准确率，当相对误差小于10%时视为正确。

### 2.21 拟合数据

**函数签名：**
```java
public void fit(double[][] input, double[][] output, int epochs, int batchSize, double validationSplit, boolean EarlyStopping)
```

**参数：**
- `input`：输入数据集
- `output`：输出数据集
- `epochs`：训练轮数
- `batchSize`：批处理大小
- `validationSplit`：验证集比例
- `EarlyStopping`：是否使用早停法

**作用：**
训练神经网络，包括数据划分、归一化、批量训练、验证和早停。

### 2.22 预测

**函数签名：**
```java
public double[] predict(double[] input)
```

**参数：**
- `input`：输入向量

**返回值：**
- 预测输出向量

**作用：**
使用神经网络进行预测，包括归一化和反归一化处理。

### 2.23 保存模型

**函数签名：**
```java
public void save(String filename) throws IOException
```

**参数：**
- `filename`：保存路径

**作用：**
将神经网络模型保存到文件。

### 2.24 加载模型

**函数签名：**
```java
public static Net load(String filename) throws IOException, ClassNotFoundException
```

**参数：**
- `filename`：模型文件路径

**返回值：**
- 加载的Net实例

**作用：**
从文件加载神经网络模型。

### 2.25 辅助函数：打乱数组

**函数签名：**
```java
private void shuffleArray(int[] array)
```

**参数：**
- `array`：要打乱的数组

**作用：**
打乱数组元素的顺序，用于训练数据的随机化。

### 2.26 辅助函数：深拷贝权重

**函数签名：**
```java
private List<double[][]> deepCopyWeights(List<double[][]> weights)
```

**参数：**
- `weights`：权重列表

**返回值：**
- 权重的深拷贝

**作用：**
深拷贝神经网络的权重，用于保存最佳模型。

### 2.27 辅助函数：深拷贝偏置

**函数签名：**
```java
private List<double[]> deepCopyBiases(List<double[]> biases)
```

**参数：**
- `biases`：偏置列表

**返回值：**
- 偏置的深拷贝

**作用：**
深拷贝神经网络的偏置，用于保存最佳模型。

### 2.28 自定义图表面板类

**类名：**
```java
private static class ChartPanel extends JPanel
```

**作用：**
用于绘制训练和验证损失、准确率的图表。

## 3. Environment.java

### 3.1 构造函数

**函数签名：**
```java
public Environment()
```

**作用：**
初始化环境，设置默认参数，创建数据库列表。

### 3.2 获取数据

**函数签名：**
```java
public List<double[]> get_data()
```

**返回值：**
- 数据库列表

**作用：**
获取环境的数据库。

### 3.3 获取状态

**函数签名：**
```java
public double[] get_state()
```

**返回值：**
- 环境状态

**作用：**
获取环境的当前状态。

### 3.4 获取状态维度

**函数签名：**
```java
public int get_state_dim()
```

**返回值：**
- 状态维度

**作用：**
获取环境状态的维度。

### 3.5 设置状态

**函数签名：**
```java
public void set_state(double[] state)
```

**参数：**
- `state`：新的状态

**作用：**
设置环境的当前状态。

### 3.6 奖励函数

**函数签名：**
```java
public double reward(double[] action)
```

**参数：**
- `action`：行动向量

**返回值：**
- 奖励值

**作用：**
计算行动的奖励，可被子类重载。

### 3.7 重置环境

**函数签名：**
```java
public void reset()
```

**作用：**
重置环境状态，随机选择一条数据作为初始状态。

### 3.8 设置参数（1）

**函数签名：**
```java
public void set_params(double h, double m, double k, double theta)
```

**参数：**
- `h`：发射高度
- `m`：小球质量
- `k`：空气阻力系数
- `theta`：发射夹角

**作用：**
设置环境参数。

### 3.9 设置参数（2）

**函数签名：**
```java
public void set_params(double h, double m, double k, double theta, int state_dim, double[] state)
```

**参数：**
- `h`：发射高度
- `m`：小球质量
- `k`：空气阻力系数
- `theta`：发射夹角
- `state_dim`：状态维度
- `state`：初始状态

**作用：**
设置环境参数，包括状态维度和初始状态。

### 3.10 获取发射高度

**函数签名：**
```java
public double get_h()
```

**返回值：**
- 发射高度

**作用：**
获取环境的发射高度参数。

### 3.11 获取小球质量

**函数签名：**
```java
public double get_m()
```

**返回值：**
- 小球质量

**作用：**
获取环境的小球质量参数。

### 3.12 获取空气阻力系数

**函数签名：**
```java
public double get_k()
```

**返回值：**
- 空气阻力系数

**作用：**
获取环境的空气阻力系数参数。

### 3.13 获取发射夹角

**函数签名：**
```java
public double get_theta()
```

**返回值：**
- 发射夹角

**作用：**
获取环境的发射夹角参数。

### 3.14 设置参数（3）

**函数签名：**
```java
public void set_params(double h, double m, double k)
```

**参数：**
- `h`：发射高度
- `m`：小球质量
- `k`：空气阻力系数

**作用：**
设置环境参数，保持向后兼容。

### 3.15 从文件导入数据

**函数签名：**
```java
public void load_data(String filename) throws IOException
```

**参数：**
- `filename`：数据文件路径

**作用：**
从CSV文件导入数据到环境数据库。

### 3.16 模拟数据生成（带噪声）

**函数签名：**
```java
public void simulate(int n, double v_max, double d_max, double s)
```

**参数：**
- `n`：模拟次数
- `v_max`：最大小车速度
- `d_max`：最大小球初速度
- `s`：噪声水平

**作用：**
模拟发射过程，生成带噪声的数据。

### 3.17 模拟数据生成（无噪声）

**函数签名：**
```java
public void simulate(int n, double v_max, double d_max)
```

**参数：**
- `n`：模拟次数
- `v_max`：最大小车速度
- `d_max`：最大小球初速度

**作用：**
模拟发射过程，生成无噪声的数据。

### 3.18 记录实验数据

**函数签名：**
```java
public void record(Simulator simulator, double v_max, double d_max, double s)
```

**参数：**
- `simulator`：模拟器对象
- `v_max`：最大小车速度
- `d_max`：最大小球初速度
- `s`：噪声水平

**作用：**
记录单次实验数据，包括小车速度、小球初速度、落点位置等。

### 3.19 保存数据到文件

**函数签名：**
```java
public void save_data(String filename) throws IOException
```

**参数：**
- `filename`：保存文件路径

**作用：**
将环境数据库保存到CSV文件。
