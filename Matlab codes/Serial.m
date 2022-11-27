function Serial()   %创建函数
delete(instrfindall);   %先关闭串口，否则可能导致出错

global x            %全局变量，供串口中断函数使用

global t;			%全局变量，这里根据需要绘制图形的个人需要而设
global m;
global i;

t = [0];      %时间轴
m = [0];      %数据轴
i = 0;        %用于计数

 p = plot(t,m);
 axis([i-50 i+50 -100 100]);
 grid on;

x = serial('com3');
set(x,'BaudRate',115200);
set(x,'BytesAvailableFcnMode','Terminator') %ASCII触发，字符触发
set(x,'Terminator','CR/LF')         %接收到\r\n后触发中断
x.BytesAvailableFcn = @Callback  %定义中断响应函数对象，类似于中断函数名
fopen(x);

pause

fclose(x);
end

