# STA 显式与隐式 Euler 离散化仿真

使用 MATLAB 对比超螺旋算法（Super-Twisting Algorithm，STA）的显式 Euler（ESTA）与隐式 Euler（ISTA）实现，包含无扰动和有扰动两组实验。

参考 Brogliato、Polyakov 和 Efimov 的论文 *The implicit discretization of the super-twisting sliding-mode control algorithm*（IEEE TAC，2020）。论文见 [doc/BBSTA.pdf](doc/BBSTA.pdf)，作者提供的仿真公式见 [doc/code1.png](doc/code1.png)。

## 文件说明

| 文件 | 用途 |
|---|---|
| `ESTA.m` | 显式 Euler 控制器 |
| `ISTA.m` | 原论文的隐式 Euler 控制器 |
| `ISTA_author_step.m` | 按作者截图实现的有扰动闭环更新 |
| `fig4_fig5_comparison.m` | 无扰动对比，步长为 0.001 s 和 0.1 s |
| `fig6_fig7_comparison.m` | 按作者截图进行有扰动对比，步长为 0.001 s |
| `fig6_fig7_comparison_before_author_fix.m` | 修改前的有扰动仿真，保留供对照 |

## 运行方法

使用支持脚本内局部函数的 MATLAB，无需额外工具箱。将 MATLAB 当前文件夹设为本仓库目录，按需分别运行：

```matlab
% 无扰动：Fig. 4、Fig. 5
fig4_fig5_comparison

% 有扰动：Fig. 6、Fig. 7
fig6_fig7_comparison
```

默认增益为 `lambda_1 = 10`、`lambda_2 = 6`，仿真时长为 5 s，第一状态与控制器积分状态初值均为 10，扰动初值为 0。有扰动实验采用

$$
\dot\varphi(t)=\Delta(t)=2\bigl[1+\sin(2t^2)\bigr].
$$

参数可直接在对应仿真脚本开头修改。

无扰动脚本直接显示图形；有扰动脚本另外生成绘图量与步进量的对照图，并将 PNG、MATLAB FIG、CSV 和 MAT 文件保存到仓库目录的上一级：`../output/author_reproduction/matlab/`。

## 有扰动实验的变量含义

原 `ISTA.m` 中的 `nu` 是控制器积分状态。`ISTA_author_step.m` 中的 `nu` 则表示闭环第二状态：

$$
\nu_k=\nu_{\mathrm{ctrl},k}+\varphi_k.
$$

按照作者截图，先在第二状态更新中加入 `h*Delta_k`，再计算本步速率 `u_k`：

```matlab
[u_k, nu_next] = ISTA_author_step(x_k, nu_k, lambda_1, lambda_2, h, Delta_k);
x_next = x_k + h * u_k;
```

Fig. 6 使用同一时刻的状态计算误差；Fig. 7 使用存储状态计算后处理量：

$$
e_k=|x_k|+\nu_k^2,\qquad
u_{\mathrm{plot},k}=\nu_k-\lambda_1\sqrt{|x_k|}\operatorname{sign}(x_k).
$$

Fig. 7 绘制 $|u_{\mathrm{plot},k}|$。隐式方法中，`u_plot` 一般不等于步进量 `u_k`。这组实验复现作者截图的闭环数值流程，不能直接把这两个量当作原被控对象的物理控制输入。
