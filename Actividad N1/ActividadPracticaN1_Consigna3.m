clear all; close all; clc;
syms s t real
% Tito Ricardo Clemente
% Ingeniería Electronica
% Sistema de Control II - 2023
% 1. Actividad Práctica Nº1 Representación de sistemas y control PID 
% ===================================================================
% Circuito RLC - CASO 3
% Ve: tensión de entrada [Volts]
% L: inductancia [Henrios]
% C: capacidad [Faradios]
% R: resistencia [Ohms]
% Vr: tensión de salida (en la resistencia) [Volts]
% i: corriente

%Cargar Datos
datos_ex= xlsread('Curvas_Medidas_RLC.xls','Hoja1');
% guardo los datos del excel en una variable 
tiempo = xlsread('Curvas_Medidas_RLC.xls','Hoja1','A:A');
corriente_L=xlsread('Curvas_Medidas_RLC.xls','Hoja1','B:B');
tension_C=xlsread('Curvas_Medidas_RLC.xls','Hoja1','C:C');




