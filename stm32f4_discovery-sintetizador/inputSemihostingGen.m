clear
fileName = 'inputArray32_File.txt';
fileId = fopen(fileName, 'w');

sizeOfSig = 32;
Omega = linspace(0, 2*pi, sizeOfSig);
sig = zeros(0, sizeOfSig);

for i=1:sizeOfSig
%     sig(i)=i;
    sig(i)= sin(8*Omega(i));
    fprintf(fileId, '%f\t', sig(i));
    disp(sig(i));
end

fclose(fileId);

%% Resultados
figura_n=0;

% -------------------------------------------------------------------------
figura_n=figura_n+1;

figura(figura_n) = figure('Color', [1 1 1], 'unit', 'centimeters', 'position', [2*figura_n figura_n 25 10]);
eixo(figura_n) = axes('FontName', 'Arial', 'FontSize', 9); hold(eixo(figura_n),'on');
    subplot(2,1,1);
        stem(sig);
        ylabel('sinal', 'FontName', 'Times', 'FontSize', 10);
        xlabel('amostra', 'FontName', 'Times', 'FontSize', 10);
    subplot(2,1,2);
        bufferFFT = abs(fft(sig,sizeOfSig));
        stem(bufferFFT(1:sizeOfSig/2));
        ylabel('FFT', 'FontName', 'Times', 'FontSize', 10);
        xlabel('frequência digital de 0 a \pi', 'FontName', 'Times', 'FontSize', 10);

