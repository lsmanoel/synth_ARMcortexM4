clear all
fileInputName = 'inputArray32_File.txt';
fileInputID = fopen(fileInputName, 'r');
fileOutputName = 'outputArray32_File.txt';
fileOutputID = fopen(fileOutputName, 'r');

sizeOfSig = 32;

inputArray = zeros(1 ,sizeOfSig/2);
outputArray = inputArray;

for i=1:sizeOfSig
    inputArray(i)=fscanf(fileInputID,'%f/t');
end

for i=1:sizeOfSig/2
    outputArray(i)=fscanf(fileOutputID,'%f/t'); 
end
% fclose(fileInputName);
% fclose(fileOutputName);
fclose('all');

figura_n = 1;
rfft_input = abs(fft(inputArray, sizeOfSig));
figura(figura_n) = figure('Color', [1 1 1], 'unit', 'centimeters', 'position', [2*figura_n figura_n 15 10]);
eixo(figura_n) = axes('FontName', 'Arial', 'FontSize', 9); hold(eixo(figura_n),'on');

stem(rfft_input(1:sizeOfSig/2));
hold on
stem(outputArray);
hold off

ylabel('magnitude FFT', 'FontName', 'Times', 'FontSize', 10);
xlabel('amostra', 'FontName', 'Times', 'FontSize', 10);
box(eixo(figura_n),'on');

