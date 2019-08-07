clear;
clc;
close all hidden;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fno = 'output.txt';
fid = fopen('1.txt','r');
x = fscanf(fid,'%f',[1,inf]);
status = fclose(fid);
 
fs = 2;            %采样频率
fmin = 1;           %最小截止频率
fmax = 200;           %最大截止频率
c = 9810.5;           %单位变换系数
it = 2;               %积分次数
 
sx = '时间';          %横坐标标注
sy1 = 'y1';           %纵坐标标注
sy2 = 'y2';           %纵坐标标注
 
n=length(x);                    %待分析数组长度
t=0:1/sf:(n-1)/sf;              %建立时间向量
nfft=2^nextpow2(n);             %大于并接近n的2的幂次方的FFT长度
y=fft(x,nfft);                  %FFT变换

df=sf/nfft;                     %计算频率间隔（Hz/s）
ni=round(fmin/df+0.5);          %高通频率截止点=最小截止频率/时间步长t+1
na=round(fmax/df+0.5);          %低通频率截止点=最大截止频率/时间步长t+1
dw=2*pi*df;                     %圆频率间隔（rad/s）
w1=0:dw:2*pi*(0.5*sf-df);       %正离散圆频率向量
w2=-2*pi*(0.5*sf-df):dw:-dw;    %负离散圆频率向量
w=[w1,w2];                      %连接w1,w2
w=w.^it;                        %以积分次数为指数，建立圆频率变量向量
subplot(2,2,2);
plot(abs(y));
 
a=zeros(1,nfft);                %进行积分的频域变换
a(2:nfft-1)=y(2:nfft-1)./w(2:nfft-1);
subplot(2,2,4);                 %绘制积分前的时程曲线图形
plot(abs(a));
 
if it==2
    y=-a;                       %进行二次积分的相位变换
else
    y = imag(a) - 1i*real(a);   %进行一次积分的相位变换
end
 
a=zeros(1,na);                  
a(ni:na)=y(ni:na);              %消除指定正频带外的频率成分
a(nfft-na+1:nfft-ni+1)=y(nfft-na+1:nfft-ni+1);%消除指定负频带外的频率成分
 
y=ifft(a,nfft);                 %IFFT变换
y=real(y(1:n))*c;               %取逆变换的实部n个元素并乘以单位变换系数为积分结果
subplot(2,2,1);                 %绘制积分前的时程曲线图形
plot(t,x);
xlabel('时间(s)');
ylabel('加速度(g)');
grid on;
subplot(2,2,3);                 %绘制积分前的时程曲线图形
plot(t,y);
xlabel('时间(s)');
if it==2
    ylabel('距离(m)');
else
    ylabel('速度(m/s)');
end
grid on;
fid=fopen(fno,'w');             %输出积分后的数据
for k=1:n
	fprintf(fid,'%f/n',y(k));
end
status=fclose(fid);