clear;clc;close all;
filename = '0725_bigC_200ms_1.txt';
[accx,accy,accz,angx,angy,angz]=...
    textread(filename,'ACC:%n%n%n\r\nAngle:%n%n%n','delimiter', ',');
result=zeros(3,111);


for i =1:length(accx)
    
    roll = angx(i);
    pitch = angy(i);
    yaw = angz(i);
    
    Rx=[1,0,0;0,cosd(roll),-sind(roll);0,sind(roll),cosd(roll)];
    Ry=[cosd(pitch),0,sind(pitch);0,1,0;-sind(pitch),0,cosd(pitch)];
    Rz=[cosd(yaw),-sind(yaw),0;sind(yaw),cosd(yaw),0;0,0,1];
    
    result(:,i)=Rz*Ry*Rx*[accx(i);accy(i);accz(i)];
    
end

z_acc_g = result(3,:);
z_acc = (z_acc_g - 1)*9.8;
% z_acc = smooth(z_acc);



fno = 'output.txt';

fs = 5;              %采样频率
fmin = 0.15;            %最小截止频率
fmax = 0.23;           %最大截止频率

% fmin = 0.1;            %最小截止频率
% fmax = 0.7;           %最大截止频率
% c = 9810.5;           %单位变换系数
it = 2;               %积分次数

sx = '时间';          %横坐标标注
sy1 = 'y1';           %纵坐标标注
sy2 = 'y2';           %纵坐标标注

n=length(z_acc);                    %待分析数组长度
t=0:1/fs:(n-1)/fs;              %建立时间向量
nfft=2^nextpow2(n);             %大于并接近n的2的幂次方的FFT长度
% fft_res=fft(z_acc,nfft);                  %FFT变换

df=fs/nfft;                     %计算频率间隔（Hz/s）
ni=round(fmin/df+0.5);          %高通频率截止点=最小截止频率/时间步长t+1
na=round(fmax/df+0.5);          %低通频率截止点=最大截止频率/时间步长t+1
dw=2*pi*df;                     %圆频率间隔（rad/s）
w1=0:dw:2*pi*(0.5*fs-df);       %正离散圆频率向量
w2=-2*pi*(0.5*fs-df):dw:-dw;    %负离散圆频率向量
w=[w1,w2];                      %连接w1,w2

w=w.^it;                        %以积分次数为指数，建立圆频率变量向量


window_len = 51;
hamming_window = hamming(window_len);
box_window = boxcar(nfft+1);

% window_fft = fft(hamming_window,64);



a_filter = fir1(window_len-1, [fmin fmax]/(fs/2), hamming_window);
f_filter = fft(a_filter,nfft);

% [h,f]=freqz(a_filter,1,nfft);
% figure(4)
% plot(f,h);


% h2 = zeros(1,nfft/2);
% h = [h', h2];

acc_z_fft = fft(z_acc,nfft);




% figure(4)
% plot(abs(a_filter))

% Hd = dfilt.dffir(a_filter);

% filter_res = filter(Hd, z_acc);

% fft_res = fft(filter_res,nfft);
% fft_res1 = fft(z_acc,nfft);





fft_res_frequency = f_filter.*acc_z_fft;

figure(4)
subplot(2,2,1)
plot(abs(f_filter))
xlabel('点数N');
ylabel('滤波器频谱');

subplot(2,2,2)
plot(abs(acc_z_fft))
xlabel('点数N');
ylabel('加速度频谱');

subplot(2,2,3)
plot(abs(fft_res_frequency))
xlabel('点数N');
ylabel('滤波结果');

ifft_a_z = ifft(fft_res_frequency,nfft);

figure(7)
plot(t,ifft_a_z)

a_filter=zeros(1,nfft);                  
a_filter(ni:na)=fft_res_frequency(ni:na); 
a_filter(nfft-na+1:nfft-ni+1)=fft_res_frequency(nfft-na+1:nfft-ni+1);%消除指定负频带外的频率成分





int_res=zeros(1,nfft);                %进行积分的频域变换
int_res(2:nfft-1)=a_filter(2:nfft-1)./w(2:nfft-1);
% int_res(2:nfft-1)=fft_res(2:nfft-1)./w(2:nfft-1);
% res1和res一个是经过滤波，一个未经过滤波


if it==2
    f_w=-int_res;                       %进行二次积分的相位变换
else
    f_w = imag(int_res) - 1i*real(int_res);   %进行一次积分的相位变换
end
             %消除指定正频带外的频率成分


% a = f_w;
s_filter=zeros(1,nfft);                  
s_filter(ni:na)=f_w(ni:na); 
s_filter(nfft-na+1:nfft-ni+1)=f_w(nfft-na+1:nfft-ni+1);%消除指定负频带外的频率成分

figure(2)
subplot(2,2,2);
plot(abs(fft_res_frequency));
subplot(2,2,4);                 %绘制积分前的时程曲线图形
plot(abs(s_filter));

ifft_res=ifft(s_filter,nfft);                 %IFFT变换
% y=real(y(1:n));               %取逆变换的实部n个元素并乘以单位变换系数为积分结果

ifft_res = real(ifft_res(1:n));

subplot(2,2,1);                 %绘制积分前的时程曲线图形
plot(t,z_acc);
xlabel('时间(s)');
ylabel('加速度(g)');
grid on;

subplot(2,2,3);                 %绘制积分前的时程曲线图形
plot(t,ifft_res);
xlabel('时间(s)');



if it==2
    ylabel('距离(m)');
else
    ylabel('速度(m/s)');
end
grid on;
fid=fopen(fno,'w');             %输出积分后的数据
for k=1:n
	fprintf(fid,'%f/n',ifft_res(k));
end
status=fclose(fid);

figure(5)                %绘制积分前的时程曲线图形
plot(t,ifft_res);
xlabel('时间(s)');
ylabel('距离(m)');