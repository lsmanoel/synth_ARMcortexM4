clear all
fileName = 'outputF32Buffer_MONO_File.txt';
fileID = fopen(fileName, 'r');
n_part = 3;

sizeOfBuffer = 1024;
inputArray = zeros(1, fix(n_part*sizeOfBuffer));

for i=1:fix(n_part*sizeOfBuffer)
    inputArray(i)=fscanf(fileID,'%f/n');
end
fclose(fileID);

%% Resultados
figura_n=0;

% -------------------------------------------------------------------------
% y1
figura_n=figura_n+1;

figura(figura_n) = figure('Color', [1 1 1], 'unit', 'centimeters', 'position', [2*figura_n figura_n 15 10]);
eixo(figura_n) = axes('FontName', 'Arial', 'FontSize', 9); hold(eixo(figura_n),'on');

plot(inputArray);

ylabel('y1', 'FontName', 'Times', 'FontSize', 10);
xlabel('amostra', 'FontName', 'Times', 'FontSize', 10);
box(eixo(figura_n),'on');


