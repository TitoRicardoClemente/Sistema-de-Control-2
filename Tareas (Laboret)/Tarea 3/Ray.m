clear all; close all; clc;
syms s t real
% Tito Ricardo Clemente
% Ingeniería Electronica
% Sistema de Control II - 2023
% 1. Tarea Nº3 
% ===============================================================
% CASO 2: Sistema no lineal de cuatro variables de estado 
% -------------------------------------------------------

% Datos:
% ------

% Polos:
p1=-3;p2=4;
% Ganancia:
K=10;
% Tiempo de Establecimiento (2%)
tss=3;

% Sistema:
G=zpk([],[p1 p2],K);  
sisotool(G)