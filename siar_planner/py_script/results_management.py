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
#file_name = argv
file_name = 'resultados'
file_path = '/Users/domi/Desktop/Pruebas_python/' + file_name + '.csv'
#number of tests
total_rows = 5
success_rows = 0 
#column numbers
test_time_col = 0
path_time_col = 1
path_cost_col = 2

#output values
test_time_average = 0.0
path_time_average = 0.0
path_cost_average = 0.0

file = open(file_path, 'r')
lines = file.read().splitlines() #crea una lista
lines.pop(0) #quitamos primera linea en caso de tener titulos 
#print(lines)

for l in lines:
    line = l.split(';') #devuelve igualmente una lista
    #print line
    
    #Calculos en todos los tests
    #print float(line[test_time_col])
    test_time_average = test_time_average + float(line[test_time_col])
    
    #Calculos en test con solucion
    if float(line[path_time_col]) != 0:
        #print float(line[path_time_col])
        success_rows = success_rows + 1
        path_time_average = path_time_average + float(line[path_time_col])
        path_cost_average = path_cost_average + float(line[path_cost_col])


test_time_average = test_time_average/total_rows
if success_rows != 0:
    path_time_average = path_time_average/success_rows
    path_cost_average = path_cost_average/success_rows


print 'Solutions are:'
print test_time_average
print path_time_average
print path_cost_average

#Generar archivo de solucion, comprobar si existe, si no crearlo poniendole los titulos
if os.path.isfile('/Users/domi/Desktop/Pruebas_python/tabla_resultados.csv')==False: #comprueba si existe el archivo
    output_file = open('tabla_resultados.csv', 'w') #Si no existía el archivo, lo crea 
    #Añadimos titulos de columnas
    output_file.writelines('Test name;Test time average;Trajectory time average;Path cost average\n')
    output_file.close()


output_file = open('tabla_resultados.csv', 'a') #Si no existía el archivo, lo crea y escribe al final
#Añadir resultados, segun columnas: nombre de test, tiempo medio de test, tiempo medio de path obtenido, coste medio de path obtenido
output_file.writelines(file_name + ';' + str(test_time_average) + ';' + str(path_time_average) + ';' + str(path_cost_average) + '\n')
output_file.close()
