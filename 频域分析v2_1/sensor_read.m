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

fs = 5;              %����Ƶ��
fmin = 0.15;            %��С��ֹƵ��
fmax = 0.23;           %����ֹƵ��

% fmin = 0.1;            %��С��ֹƵ��
% fmax = 0.7;           %����ֹƵ��
% c = 9810.5;           %��λ�任ϵ��
it = 2;               %���ִ���

sx = 'ʱ��';          %�������ע
sy1 = 'y1';           %�������ע
sy2 = 'y2';           %�������ע

n=length(z_acc);                    %���������鳤��
t=0:1/fs:(n-1)/fs;              %����ʱ������
nfft=2^nextpow2(n);             %���ڲ��ӽ�n��2���ݴη���FFT����
% fft_res=fft(z_acc,nfft);                  %FFT�任

df=fs/nfft;                     %����Ƶ�ʼ����Hz/s��
ni=round(fmin/df+0.5);          %��ͨƵ�ʽ�ֹ��=��С��ֹƵ��/ʱ�䲽��t+1
na=round(fmax/df+0.5);          %��ͨƵ�ʽ�ֹ��=����ֹƵ��/ʱ�䲽��t+1
dw=2*pi*df;                     %ԲƵ�ʼ����rad/s��
w1=0:dw:2*pi*(0.5*fs-df);       %����ɢԲƵ������
w2=-2*pi*(0.5*fs-df):dw:-dw;    %����ɢԲƵ������
w=[w1,w2];                      %����w1,w2

w=w.^it;                        %�Ի��ִ���Ϊָ��������ԲƵ�ʱ�������


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
xlabel('����N');
ylabel('�˲���Ƶ��');

subplot(2,2,2)
plot(abs(acc_z_fft))
xlabel('����N');
ylabel('���ٶ�Ƶ��');

subplot(2,2,3)
plot(abs(fft_res_frequency))
xlabel('����N');
ylabel('�˲����');

ifft_a_z = ifft(fft_res_frequency,nfft);

figure(7)
plot(t,ifft_a_z)

a_filter=zeros(1,nfft);                  
a_filter(ni:na)=fft_res_frequency(ni:na); 
a_filter(nfft-na+1:nfft-ni+1)=fft_res_frequency(nfft-na+1:nfft-ni+1);%����ָ����Ƶ�����Ƶ�ʳɷ�





int_res=zeros(1,nfft);                %���л��ֵ�Ƶ��任
int_res(2:nfft-1)=a_filter(2:nfft-1)./w(2:nfft-1);
% int_res(2:nfft-1)=fft_res(2:nfft-1)./w(2:nfft-1);
% res1��resһ���Ǿ����˲���һ��δ�����˲�


if it==2
    f_w=-int_res;                       %���ж��λ��ֵ���λ�任
else
    f_w = imag(int_res) - 1i*real(int_res);   %����һ�λ��ֵ���λ�任
end
             %����ָ����Ƶ�����Ƶ�ʳɷ�


% a = f_w;
s_filter=zeros(1,nfft);                  
s_filter(ni:na)=f_w(ni:na); 
s_filter(nfft-na+1:nfft-ni+1)=f_w(nfft-na+1:nfft-ni+1);%����ָ����Ƶ�����Ƶ�ʳɷ�

figure(2)
subplot(2,2,2);
plot(abs(fft_res_frequency));
subplot(2,2,4);                 %���ƻ���ǰ��ʱ������ͼ��
plot(abs(s_filter));

ifft_res=ifft(s_filter,nfft);                 %IFFT�任
% y=real(y(1:n));               %ȡ��任��ʵ��n��Ԫ�ز����Ե�λ�任ϵ��Ϊ���ֽ��

ifft_res = real(ifft_res(1:n));

subplot(2,2,1);                 %���ƻ���ǰ��ʱ������ͼ��
plot(t,z_acc);
xlabel('ʱ��(s)');
ylabel('���ٶ�(g)');
grid on;

subplot(2,2,3);                 %���ƻ���ǰ��ʱ������ͼ��
plot(t,ifft_res);
xlabel('ʱ��(s)');



if it==2
    ylabel('����(m)');
else
    ylabel('�ٶ�(m/s)');
end
grid on;
fid=fopen(fno,'w');             %������ֺ������
for k=1:n
	fprintf(fid,'%f/n',ifft_res(k));
end
status=fclose(fid);

figure(5)                %���ƻ���ǰ��ʱ������ͼ��
plot(t,ifft_res);
xlabel('ʱ��(s)');
ylabel('����(m)');