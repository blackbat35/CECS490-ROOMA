close all, clc;
s=tcpip('192.168.4.1',8880,'NetworkRole','client')
fopen(s);

while 1
    r=input('press a key ','s')  
switch r
    case 'o'
        fwrite(s,'O')
        CM(1)=0;
        pos(1)=0;
        i=1;
        while (pos(1) < 180)
            pos(2) = fscanf(s,'%e');
            CM(2) = fscanf(s,'%e')
            pos(1)= pos(2)
            CM(1)=CM(2)
            table(i,1) = pos(1);
            table(i,2) = CM(1);
            i=i+1;
        end
        polar(table(:,1)*pi/180, table(:,2));
        title('Map of the Environment');
        grid on;
    case 'p'
        fwrite(s,'P')
        CM(1)=0;
        pos(1)=180;
        i=1;
        while (pos(1) > 0)
            pos(2) = fscanf(s,'%e');
            CM(2)  = fscanf(s,'%e');
            pos(1) = pos(2)
            CM(1)  = CM(2)
            table(i,1) = pos(1);
            table(i,2) = CM(1);
            i=i+1;
        end
        polar(table(:,1)*pi/180, table(:,2));
        title('Map of the Environment');
        grid on;
     case 'l' %Forward
        fwrite(s,'L')
     case 'r' %Reverse
        fwrite(s,'R') 
    %Control L298N Motor Driver
     case 'w' %Forward
        fwrite(s,'W')
     case 's' %Reverse
        fwrite(s,'S')
     case 'a' %Left
        fwrite(s,'A')
     case 'd' %Right
        fwrite(s,'D')
     case 'z' %Stop
        fwrite(s,'Z')
    %Flash
    case 'q' %turn off
        fwrite(s,'Q')
    case 'e' %turn on
        fwrite(s,'E')
    %Servo
    case 'f'
        disp('g=0,h=45,j=90,k=135,l=180');
    case 'g'
       fwrite(s,'G');
        %fwrite(s, '4');
        %fprintf(s,'%s', char(4));       
    case 'h'
        fwrite(s,'H');
    case 'j'
        fwrite(s,'J');
    case 'k'
        fwrite(s,'K');
    case 'l'
        fwrite(s,'L');    
    case 'm'
       fclose(s)
        
end
end