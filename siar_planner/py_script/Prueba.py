# -*- coding: utf-8 -*-
"""
Apertura de archivos CSV y gestion de datos
"""

'''Para probar como son los archivos, en mis pruebas me separaba las lineas por ',' y las columnas por ';'
file = open('/Users/domi/Desktop/Pruebas_python/resultados.csv', 'r')
lines = file.read() 
'''
import os.path
'''Tomar estos valores por parametros'''
from sys import argv
#File path
#file_name = argv1
#file_name = 'resultados'
file_name = 'test_4_a_rrt_in_complex_scenario_turn_L_1'
file_path = '/home/domi/siar_ws/src/siar_navigation/siar_planner/py_script/' + file_name + '.txt'
#number of tests
total_rows = 100
success_rows = 0 
#column numbers
test_time_col = 0
path_time_col = 1
path_cost_col = 5

#output values
test_time_average = 0.0 #cambiar esto para que sean NAN??
path_time_average = 0.0
path_cost_average = 0.0

file = open(file_path, 'r')
lines = file.read().splitlines() #crea una lista
lines.pop(0) #quitamos primera linea en caso de tener titulos 
print(lines)

for l in lines:
    line = l.split(';') #devuelve igualmente una lista
    print line
    
    #Calculos en todos los tests
    print float(line[test_time_col])
    '''
    test_time_average = test_time_average + float(line[test_time_col])
    
    #Calculos en test con solucion
    if float(line[path_time_col]) != 0:
        #print float(line[path_time_col])
        success_rows = success_rows + 1
        path_time_average = path_time_average + float(line[path_time_col])
        path_cost_average = path_cost_average + float(line[path_cost_col])
'''
