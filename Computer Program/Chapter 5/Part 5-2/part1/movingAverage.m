clc
clear
close all
mypi = raspi('192.168.43.77','pi','raspberry');
add = scanI2CBus(mypi,'i2c-1');
sensor = mpu6050(mypi,'Bus','i2c-1','I2CAddress','0x68');
ax=0; ay=0; az=0; ax0=0; ay0=0; az0=0;
vx=0; vx0=0; vy=0; vy0=0; vz=0; vz0=0;
x=0; y=0; z=0;
qx=0; qy=0; qz=0;
qx0=0; qy0=0; qz0=0;
roll=0; pitch=0; yaw=0;
k=1;
xx=0; yy=0; zz=0; qxx=0; qyy=0; qzz=0;
axx=[0 0]; ayy=0; azz=0; vxx=[0 0]; vyy=[0 0]; vzz=[0 0];
rolll=0; pitchh=0; yaww=0;
c1=0;c2=0;c3=0;
b1=0;b2=0;b3=0;
t=0; t1=0;
for j=1:3000
    [accelReadings] = readAcceleration(sensor);
     b1=[b1 accelReadings(1)];
     b2=[b2 accelReadings(2)];
     b3=[b3 accelReadings(3)];
     [gyroReadings] = readAngularVelocity(sensor)*(180/pi);
     c1=[c1 gyroReadings(1)];
     c2=[c2 gyroReadings(2)];
     c3=[c3 gyroReadings(3)];
%      pause(T-t)
end
b1=b1(2:end); b2=b2(2:end); b3=b3(2:end);
g0=[mean(b1);mean(b2);mean(b3)]; g=g0;
c1=c1(2:end); c2=c2(2:end); c3=c3(2:end);
c0=[mean(c1) mean(c2) mean(c3)];
qxx0=[0 0]; qyy0=[0 0]; qzz0=[0 0];
axx0=[0 0]; ayy0=[0 0]; azz0=[0 0];
alpha=0.1; e_x=0;
while 1
    tic   
    [accelReadings] = readAcceleration(sensor); %m/s^2
    [gyroReadings] = readAngularVelocity(sensor)*(180/pi); %deg/s
    k_r=1; k_p=1; k_y=1;
    if abs(qx)<=0.5 && abs(roll)<=5
        roll=0;
        c1 = [c1 gyroReadings(1)];
        c0(1) = mean(c1);
        k_r=0;
    end
    if abs(qy)<=0.5 && abs(pitch)<=5
    	c2 = [c2 gyroReadings(2)];
        c0(2) = mean(c2);
        k_p=0;
        pitch=0;
    end
    if abs(qz)<=0.5 && abs(yaw)<=5
    	c3 = [c3 gyroReadings(3)];
        c0(3) = mean(c3);
        k_y=0;
        yaw=0;
    end
    
    gyroReadings = gyroReadings - c0;
    
    qx = gyroReadings(1);
    if k>=50
        qx0 = 0;
        for l=k-49:k
            qx0 = qx0 + qxx(l);
        end
        qx0 = qx0 / 50;
        qxx0 = [qxx0 qx0];
%         roll = roll + (qxx0(k-3)+4*qxx0(k-2)+qxx0(k-1))*(2*t/6);
        roll = roll + k_r*(qxx0(k-49)+qxx0(k-48))*(t/2)
        rolll = [rolll roll];
    end
    qxx = [qxx qx];
%     plot(qxx,'r'); drawnow
%     hold on
%     plot(qxx0,'b'); drawnow
%     hold on
%     plot(rolll,'g'); drawnow
    
    qy = gyroReadings(2);
    if k>=50
        qy0 = 0;
        for l=k-49:k
            qy0 = qy0 + qyy(l);
        end
        qy0 = qy0 / 50;
        qyy0 = [qyy0 qy0];
        pitch = pitch + k_p*(qyy0(k-49)+qyy0(k-48))*(t/2)
        pitchh = [pitchh pitch];
    end
    qyy = [qyy qy];
%     plot(pitchh); drawnow
    
    qz = gyroReadings(3);
    if k>=50
        qz0 = 0;
        for l=k-49:k
            qz0 = qz0 + qzz(l);
        end
        qz0 = qz0 / 50;
        qzz0 = [qzz0 qz0];
        yaw = yaw + k_y*(qzz0(k-49)+qzz0(k-48))*(t/2);
        yaww = [yaww yaw];
    end
    qzz = [qzz qz];
%     plot(yaww); drawnow
    
    Rx = [1 0 0;0 cosd(roll) sind(roll);0 -sind(roll) cosd(roll)];
    Ry = [cosd(pitch) 0 -sind(pitch);0 1 0;sind(pitch) 0 cosd(pitch)];
    Rz = [cosd(yaw) sind(yaw) 0;-sind(yaw) cosd(yaw) 0;0 0 1];
    R = Rz*Ry*Rx;
%     g = R*g0;
    accelReadings = (inv(R)*accelReadings')';
    
    ax = accelReadings(1);
    ax = ax - g(1);
    if k>=10
        ax0 = 0;
        for l=k-9:k
            ax0 = ax0 + axx(l);
        end
        ax0 = ax0 / 10;
        alpha=0.9; beta=1;
        if abs(ax0)<=0.1
            beta=0;
            if k>=20
                e_x = mean(axx0(end-10:end));
            end
        end
        ax0 = ax0 - e_x;
        axx0 = [axx0 ax0];
        vx = alpha*vx + beta*(axx0(k-9)+axx0(k-8))*(t/2);
        beta=1;
        if abs(vx)<=0.02
            beta=0;
        end
        x = x + beta*(vxx(k-9)+vxx(k-8))*(t/2);
        xx = [xx x];
        vxx = [vxx vx];
    end
%     uax = (1-alpha)*uax + alpha*ax;
% 	ax1 = ax - uax;
% 	vx = vx + (axx(k)+axx(k+1))*(t/2);
% 	uvx = (1-alpha)*uvx + alpha*vx;
% 	vx1 = vx - uvx;
% 	x = x + (vxx(k)+vxx(k+1))*(t/2);
% 	ux = (1-alpha)*ux + alpha*x;
% 	x1 = x - ux;
% 	xx = [xx x];
% 	vxx = [vxx vx1];
    axx = [axx ax];
%     plot(xx); drawnow
    
    ay = accelReadings(2);
    ay = ay - g(2);
    if k>=50
        ay0 = 0;
        for l=k-49:k
            ay0 = ay0 + ayy(l);
        end
        ay0 = ay0 / 50;
        ayy0 = [ayy0 ay0];
        vy = vy + (ayy0(k-49)+ayy0(k-48))*(t/2);
        y = y + (vyy(k-49)+vyy(k-48))*(t/2);
        yy = [yy y];
        vyy = [vyy vy];
    end
    ayy = [ayy ay];
%     plot(ayy); drawnow
    
    az = accelReadings(3);
    az = az - g(3);
    if k>=50
        az0 = 0;
        for l=k-49:k
            az0 = az0 + azz(l);
        end
        az0 = az0 / 50;
        azz0 = [azz0 az0];
        vz = vz + (azz0(k-49)+azz0(k-48))*(t/2);
        z = z + (vzz(k-49)+vzz(k-48))*(t/2);
        zz = [zz z];
        vzz = [vzz vz];
    end
    azz = [azz az];
%     plot(zz); drawnow
    
    k = k + 1;
    t1=t1+t;
    t=toc;
end
