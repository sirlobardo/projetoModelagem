% Controle PWM (RESISTOR) - TEMP

clear all;
clc;
close all;

s = tf('s');

% Especificações do filtro
Fs = 1;  % Taxa de amostragem (Hz)
Fpass = 0.45;  % Frequência de passagem (Hz)
Fstop = 0.5;  % Frequência de rejeição (Hz)
Rp = 1;  % Ondulação na banda de passagem (dB)
Ast = 60;  % Atenuação na banda de rejeição (dB)

% Especificações do filtro PWM x RPM
Fs_1 = 1;  % Taxa de amostragem (Hz)    
Fpass_1 = 0.1;  % Frequência de passagem (Hz)
Fstop_1 = 0.2;  % Frequência de rejeição (Hz)
Rp_1 = 1;  % Ondulação na banda de passagem (dB)
Ast_1 = 60;  % Atenuação na banda de rejeição (dB)

% Design do filtro Butterworth IIR RPM x Temperatura
Hlp = designfilt('lowpassiir', 'PassbandFrequency', Fpass, ...
               'StopbandFrequency', Fstop, 'PassbandRipple', Rp, ...
               'StopbandAttenuation', Ast, 'SampleRate', Fs, ...
               'DesignMethod', 'butter', 'MatchExactly', 'stopband');

% Design do filtro Butterworth IIR PWM x RPM
Hlp_1 = designfilt('lowpassiir', 'PassbandFrequency', Fpass_1, ...
               'StopbandFrequency', Fstop_1, 'PassbandRipple', Rp_1, ...
               'StopbandAttenuation', Ast_1, 'SampleRate', Fs_1, ...
               'DesignMethod', 'butter', 'MatchExactly', 'stopband');

% Carregando os dados
raw_data = readmatrix('pwm_temperatura.csv');
PWM = raw_data(:,1);
PWM = PWM - mean(PWM);
PWM = filter(Hlp, PWM);
PWM = (PWM - min(PWM)) / (max(PWM) - min(PWM));

TEMP = raw_data(:,2);
TEMP = TEMP - mean(TEMP);
TEMP = filter(Hlp, TEMP);
TEMP = (TEMP - min(TEMP)) / (max(TEMP) - min(TEMP));

% Tratamento de valores NaN
PWM = fillmissing(PWM, 'linear'); % Substitui NaNs por interpolação linear
TEMP = fillmissing(TEMP, 'linear'); % Substitui NaNs por interpolação linear

% PWM X TEMPERATURA
% Definição do período de amostragem
Ts = 1;

% Transformação dos dados no formato de identificação
data = iddata(PWM, TEMP, Ts);

% Conjunto de dados de estimação
datae = data(1:2000);

% Conjunto de dados de validação
datav = data(2001:7000);

% PWM X TEMPERATURA
M_arx = armax(datae, [2 2 1 1]);

% Obtendo a FT discreta
y_discrete = tf(M_arx);
y_discrete = minreal(y_discrete);

% Convertendo para a FT contínua
y_continuous = d2c(y_discrete, 'tustin');

planta = y_continuous;

% Visualização da planta
figure
margin(planta);
figure
title('Diagrama de Bode da Planta');
step(feedback(planta,1));
stepinfo((feedback(planta,1)))
