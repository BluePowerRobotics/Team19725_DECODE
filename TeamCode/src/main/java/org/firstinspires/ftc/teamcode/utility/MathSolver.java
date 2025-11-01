package org.firstinspires.ftc.teamcode.utility;

public class MathSolver {
    private static final double EPSILON = 1e-10; // 误差容限
    public static double sgn(double n){return Math.abs(n)/n;}

    // 求解四次方程 ax^4 + bx^3 + cx^2 + dx + e = 0 的实根
    public static double[] solve4(double a, double b, double c, double d, double e) {
        if (Math.abs(a) < EPSILON) {
            return solve3(b, c, d, e);
        }


        /*
        四次方程天珩求根公式
ax^4+bx^3+cx^2+dx+e=0\left(a,b,c,d,e\in R,a\ne0\right)
重根判别式：D = 3 * b ^ 2 - 8 * a * c
E = - b ^ 3 + 4 * a * b * c - 8 * a ^ 2 * d
F = 3 * b ^ 4 + 16 * a ^ 2 * c ^ 2 -16*a*b^2*c+16a^2*b*d-64*a^3*e
A=D^2-3F
B=D*F-9*E^2
C = F ^ 2-3*D*E^2
判别式=B^2-4*A*C

!!!(1)当D=E=F=0时，方程有一个四重实根。
x_1=x_2=x_3=x_4=-\frac{b}{4a}=-\frac{2c}{3b}=-\frac{3d}{2c}=-\frac{4e}{d}
!!!(2)当DEF≠0，A=B=C=0时，方程有四个实根，其中有一个三重根。
x_1=\frac{-bD+9E}{4aD},x_2=x_3=x_4=\frac{-bD-3E}{4aD}
!!!(3)当E=F=0，D≠0时，方程有两对二重根；若D＞0，根为实数；若D＜0，根为虚数。
x_{1,2}=x_{3,4}=\frac{-b\pm\sqrt{D}}{4a}
!!!(4)当ABC≠0，Δ=0时，方程有一对二重实根；若AB＞0，则其余两根为不等实根；若AB＜0，则其余两根为共轭虚根。
x_{1,2}=\frac{-b+\frac{2AE}{B}\pm\sqrt{\frac{2B}{A}}}{4a},x_3=x_4=\frac{-b-\frac{2AE}{B}}{4a}
!!!(5)当Δ>0时，方程有两个不等实根和一对共轭虚根。
令
z_{1,2}=AD+3\left(\frac{-B\pm\sqrt{B^2-4AC}}{2}\right),z=D^2-D\left(\sqrt[3]{z_1}+\sqrt[3]{z_2}\right)+\left(\sqrt[3]{z_1}+\sqrt[3]{z_2}\right)^2-3A
则有：
x_{1,2}=\frac{-b+sgn\left(E\right)\sqrt{\frac{D+\sqrt[3]{z_1}+\sqrt[3]{z_2}}{3}}\pm\sqrt{\frac{2D-\left(\sqrt[3]{z_1}+\sqrt[3]{z_2}\right)+2\sqrt{z}}{3}}}{4a}
x_{3,4}=\frac{-b-sgn\left(E\right)\sqrt{\frac{D+\sqrt[3]{z_1}+\sqrt[3]{z_2}}{3}}}{4a}\pm\frac{\sqrt{\frac{-2D+\sqrt[3]{z_1}+\sqrt[3]{z_2}+2\sqrt{z}}{3}}}{4a}i
其中，sgn表示符号因子。计算方法如下：
<1>当n=0时，sgn(n)=0。
<2>当n≠0时，sgn(n)=abs(n)/n，即 (下同)
(6)当Δ＜0时，若D与F均为正数，则方程有四个不等实根；否则方程有两对不等共轭虚根。
令 \theta=\arccos\frac{3B-2AD}{2A\sqrt{A}},y_1=\frac{D-2\sqrt{A}\cos\frac{\theta}{3}}{3},y_{2,3}=\frac{D+\sqrt{A}\left(\cos\frac{\theta}{3}\pm\sqrt{3}\sin\frac{\theta}{3}\right)}{3}

<1>若E=0,D＞0,F＞0，
x_{1,2}=\frac{-b\pm\sqrt{D+2\sqrt{F}}}{4a},x_{3,4}=\frac{-b\pm\sqrt{D-2\sqrt{F}}}{4a}
<2>若E=0,D<0,F>0，
x_{1,2}=-\frac{b}{4a}\pm\frac{\sqrt{-D+2\sqrt{F}}}{4a}i,x_{3,4}=-\frac{b}{4a}\pm\frac{\sqrt{-D-2\sqrt{F}}}{4a}i
<3>若E=0,F＜0，
x_{1,2}=\frac{-2b+\sqrt{2D+2\sqrt{A-F}}}{8a}\pm\frac{\sqrt{-2D+2\sqrt{A-F}}}{8a}i
x_{3,4}=\frac{-2b-\sqrt{2D+2\sqrt{A-F}}}{8a}\pm\frac{\sqrt{-2D+2\sqrt{A-F}}}{8a}i
<4>若E≠0，一定存在\max\left(y_1,y_2,y_3\right)=y_2；故若D或F中有非正值即方程无实数解时，\sqrt{y_1}=\sqrt{-y_1}i,\sqrt{y_3}=\sqrt{-y_3}i，而y_2始终为正。
此时有：
当D与F均为正时，四实根为：
x_{1,2}=\frac{-b+sgn\left(E\right)\sqrt{y_1}\pm\left(\sqrt{y_2}+\sqrt{y_3}\right)}{4a},x_{3,4}=\frac{-b-sgn\left(E\right)\sqrt{y_1}\pm\left(\sqrt{y_2}-\sqrt{y_3}\right)}{4a}
当D或F中有非正值时，四虚根为：
x_{1,2}=\frac{-b-\sqrt{y_2}}{4a}\pm\frac{sgn\left(E\right)\sqrt{-y_1}+\sqrt{-y_3}}{4a}i,x_{3,4}=\frac{-b+\sqrt{y_2}}{4a}\pm\frac{sgn\left(E\right)\sqrt{-y_1}-\sqrt{-y_3}}{4a}i
公式中的总判别式
B^2-4AC
与三次方程盛金公式中的
B^2-4AC
以及二次方程求根公式中的
b^2-4ac
极为相似，体现了数学中的有序、对称、和谐与简洁美。
对于方程什么时候有实数根，什么时候有重根，天珩定理给出以下证明：
定理1：当D=E=F=0时，若b=0，则c=d=e=0；（此时方程有一个四重零根，天珩公式仍成立）（适用公式1）
定理2：当D=E=F=0时，若b≠0，则c，d，e≠0；（适用公式1）
定理3：当A=B=C=0时，若D，E，F中有0值时，则D=E=F=0；（适用公式1）
定理4：当Δ=0，A或C中有0值时，则A=B=C=0；（适用公式2）
定理5：当ABC≠0，Δ=0时，A＞0恒成立；（适用公式4）
定理6：当A<0，Δ>0恒成立；（适用公式5）
定理7：当E=0，D或F中有非正值时，方程无实数解；（适用公式6的<2><3>）
定理8：当Δ<0时，若E≠0，\max\left(y_1,y_2,y_3\right)=y_2恒成立；故若D或F中有非正值即方程无实数解时，
\sqrt{y_1}=\sqrt{-y_1}i,\sqrt{y_3}=\sqrt{-y_3}i
，而
y_2
>0恒成立；（适用公式6的<4>）
定理9：当Δ<0时，若E≠0，A>0恒成立；（适用公式6的<4>）
定理10：当Δ<0时，若E≠0，-1＜\frac{3B-2AD}{2A\sqrt{A}}＜1恒成立；（适用公式6的<4>）
         */

        // 计算判别式
        double D = 3 * b * b - 8 * a * c;
        double E = -b * b * b + 4 * a * b * c - 8 * a * a * d;
        double F = 3 * Math.pow(b, 4) + 16 * a * a * c * c - 16 * a * b * b * c + 16 * a * a * b * d - 64 * Math.pow(a, 3) * e;
        double A = D * D - 3 * F;
        double B = D * F - 9 * E * E;
        double C = F * F - 3 * D * E * E;
        double delta = B * B - 4 * A * C;


        // 情况1: 四重根
        //D=E=F=0
        if (Math.abs(D) < EPSILON && Math.abs(E) < EPSILON && Math.abs(F) < EPSILON) {
            return new double[]{-b / (4 * a)};
        }

        // 情况2: 三重根+单根
        //A=B=C=0,D*E*F!=0
        if (Math.abs(A) < EPSILON && Math.abs(B) < EPSILON && Math.abs(C) < EPSILON && (Math.abs(D) >= EPSILON || Math.abs(E) >= EPSILON || Math.abs(F) >= EPSILON)) {
            double x1 = (-b * D + 9 * E) / (4 * a * D);
            double x2 = (-b * D - 3 * E) / (4 * a * D);
            return new double[]{x1, x2};
        }

        // 情况3: 两对二重根
        if (Math.abs(E) < EPSILON && Math.abs(F) < EPSILON && Math.abs(D) >= EPSILON) {
            if (D > 0) {
                double sqrtD = Math.sqrt(D);
                return new double[]{(-b + sqrtD) / (4 * a), (-b - sqrtD) / (4 * a)};
            }
            return new double[]{}; // D<0时为虚根
        }

        // 情况4: 一对二重根+两个不等实根
        if (Math.abs(delta) < EPSILON && A * B > 0) {
            double x1 = (-b + 2 * A * E / B + Math.sqrt(2 * B / A)) / (4 * a);
            double x2 = (-b + 2 * A * E / B - Math.sqrt(2 * B / A)) / (4 * a);
            double x3 = (-b - 2 * A * E / B) / (4 * a);
            return new double[]{x1, x2, x3};
        }

        // 情况5: 两个不等实根+一对共轭虚根
        if (delta > 0) {
            double sqrtDelta = Math.sqrt(delta);
            double z1 = A * D + 3 * ((-B + sqrtDelta) / 2);
            double z2 = A * D + 3 * ((-B - sqrtDelta) / 2);
            double cbrtZ1 = Math.cbrt(z1);
            double cbrtZ2 = Math.cbrt(z2);
            double z = D * D - D * (cbrtZ1 + cbrtZ2) + (cbrtZ1 + cbrtZ2) * (cbrtZ1 + cbrtZ2) - 3 * A;

            double signE = sgn(E); // E的符号因子
            double realPart = (-b + signE * Math.sqrt((D + cbrtZ1 + cbrtZ2) / 3)) / (4 * a);
            double imagPart = Math.sqrt((2 * D - (cbrtZ1 + cbrtZ2) + 2 * Math.sqrt(z)) / 3) / (4 * a);

            double x1 = realPart + imagPart;
            double x2 = realPart - imagPart;
            return new double[]{x1, x2};
        }

        // 情况6: 四个不等实根 (仅当D>0且F>0时)
        if (delta < 0 && D > 0 && F > 0) {
            double theta = Math.acos((3 * B - 2 * A * D) / (2 * A * Math.sqrt(A)));
            double y1 = (D - 2 * Math.sqrt(A) * Math.cos(theta / 3)) / 3;
            double y2 = (D + Math.sqrt(A) * (Math.cos(theta / 3) + Math.sqrt(3) * Math.sin(theta / 3))) / 3;
            double y3 = (D + Math.sqrt(A) * (Math.cos(theta / 3) - Math.sqrt(3) * Math.sin(theta / 3))) / 3;

            if (Math.abs(E) < EPSILON) {
                return new double[]{
                        (-b + Math.sqrt(D + 2 * Math.sqrt(F))) / (4 * a),
                        (-b - Math.sqrt(D + 2 * Math.sqrt(F))) / (4 * a),
                        (-b + Math.sqrt(D - 2 * Math.sqrt(F))) / (4 * a),
                        (-b - Math.sqrt(D - 2 * Math.sqrt(F))) / (4 * a)
                };
            } else {
                double signE = sgn(E);
                return new double[]{
                        (-b + signE * Math.sqrt(y1) + Math.sqrt(y2) + Math.sqrt(y3)) / (4 * a),
                        (-b + signE * Math.sqrt(y1) - Math.sqrt(y2) - Math.sqrt(y3)) / (4 * a),
                        (-b - signE * Math.sqrt(y1) + Math.sqrt(y2) - Math.sqrt(y3)) / (4 * a),
                        (-b - signE * Math.sqrt(y1) - Math.sqrt(y2) + Math.sqrt(y3)) / (4 * a)
                };
            }
        }
        return new double[]{}; // 其他情况都是虚根
    }
    public static double[] solve3(double a, double b, double c, double d) {
        // 使用Cardano公式求解三次方程 ax^3 + bx^2 + cx + d = 0 的实根
        if (Math.abs(a) < EPSILON) {
            return solve2(b, c, d);
        }
        // 将方程化为标准形式 x^3 + px + q = 0
        double p = (3 * a * c - b * b) / (3 * a * a);
        double q = (2 * b * b * b - 9 * a * b * c + 27 * a * a * d) / (27 * a * a * a);

        double discriminant = (q * q) / 4 + (p * p * p) / 27;
        if (discriminant > 0) {
            // 一个实根
            double u = Math.cbrt(-q / 2 + Math.sqrt(discriminant));
            double v = Math.cbrt(-q / 2 - Math.sqrt(discriminant));
            double root = u + v - b / (3 * a);
            return new double[]{root};
        } else if (Math.abs(discriminant) < EPSILON) {
            // 三重根或一个单根和一个双根
            if(Math.abs(q)<EPSILON){
                double root = -b / (3 * a);
                return new double[]{root};//三重根
            }else{
                double root1 = -2 * Math.cbrt(q / 2) - b / (3 * a);
                double root2 = Math.cbrt(q / 2) - b / (3 * a);
                return new double[]{root1, root2};//一个单根和一个双根
            }
        } else {
            // 三个实根
            double r = Math.sqrt(-p / 3);
            double theta = Math.acos(Math.max(-1, Math.min(1,-q / (2 * r * r * r))));
            double root1 = 2 * r * Math.cos(theta / 3) - b / (3 * a);
            double root2 = 2 * r * Math.cos((theta + 2 * Math.PI) / 3) - b / (3 * a);
            double root3 = 2 * r * Math.cos((theta + 4 * Math.PI) / 3) - b / (3 * a);
            return new double[]{root1, root2, root3};
        }
    }
    public static double[] solve2(double a, double b, double c) {
        // 使用求根公式求解二次方程 ax^2 + bx + c = 0 的实根
        if (Math.abs(a) < EPSILON) {
            return solve1(b, c);
        }
        double discriminant = b * b - 4 * a * c;
        if (discriminant > 0) {
            double root1 = (-b + Math.sqrt(discriminant)) / (2 * a);
            double root2 = (-b - Math.sqrt(discriminant)) / (2 * a);
            return new double[]{root1, root2};
        } else if (Math.abs(discriminant) < EPSILON) {
            double root = -b / (2 * a);
            return new double[]{root};
        } else {
            return new double[]{}; // 无实根
        }
    }
    public static double[] solve1(double a, double b) {
        // 求解一元一次方程 ax + b = 0 的实根
        if (Math.abs(a) < EPSILON) {
            throw new IllegalArgumentException("系数不能为0");
        }
        double root = -b / a;
        return new double[]{root};
    }
    public static double toInch(double MM){
        return MM*0.0394;
    }
    public static double toMM(double Inch){
        return Inch*25.4;
    }

    public static double hypot(double a, double b) {
        double r;
        if (Math.abs(a) > Math.abs(b)) {
            r = b/a;
            r = Math.abs(a)*Math.sqrt(1+r*r);
        } else if (b != 0) {
            r = a/b;
            r = Math.abs(b)*Math.sqrt(1+r*r);
        } else {
            r = 0.0;
        }
        return r;
    }
    public static double normalizeAngle(double angle) {
        if(Double.isNaN(angle)){
            return Double.NaN;
        }
        while (angle > Math.PI) {
            angle -= 2 * Math.PI;
        }
        while (angle <= -Math.PI) {
            angle += 2 * Math.PI;
        }
        return angle;
    }
}
