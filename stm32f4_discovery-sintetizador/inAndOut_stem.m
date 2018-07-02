clear all

f0=48000;
% -------------------------------------------------------------------------
fileName = 'outputF32Buffer_MONO_File.txt';
% fileName = 'outputF32Buffer_MONO_File_save1.txt';
fileID = fopen(fileName, 'r');
n_part = 4;

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

%% Transformada discreta de Fourier
N0=1024;
xFFT = fft(inputArray, N0);
yFFT = xFFT(1:floor(N0/2));
% f = [0:floor(N0/2)-1];
% f = [0:floor(N0/2)-1]*f0;
f = linspace(0, 1, N0/2)*f0/2;

% Transformada discreta de Fourier
figura_n=figura_n+1;
figura(figura_n) = figure('Color', [1 1 1], 'unit', 'centimeters', 'position', [5 5 15 10]);
eixo(figura_n) = axes('FontName', 'Arial', 'FontSize', 9); hold(eixo(figura_n),'on');
plot(f, 10*log10(abs(yFFT)));
% plot(f, abs(yFFT));

ylabel('DFT', 'FontName', 'Times', 'FontSize', 10);
xlabel('Frequência (Hz)', 'FontName', 'Times', 'FontSize', 10);
xlim([f(1), f(end)]);
box(eixo(figura_n),'on');


