# -*- coding: utf-8 -*-
import vrep
from numpy import radians, degrees

#===================================================================================
# Funções de Configuração do VREP
#===================================================================================

#Realiza conexão com o software Vrep
def connectVREP():
	#Fecha conexoes existentes
	vrep.simxFinish(-1)		

	#Define objeto de conexão ao Vrep												
	clientID = vrep.simxStart('127.0.0.1', 19997 , True, True, 5000, 5) 

	#Verifica status da conexão
	if clientID != -1:
		print "Conectado ao Vrep.\n"

		#Inicia conexão com a simulação do Vrep
		vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot_wait)

		#Define modo de conexão como sincrono
		vrep.simxSynchronous(clientID, False)

		#Retorna objeto de conexão
		return clientID
	else:
		print "Erro ao conectar ao Vrep!\n"
		sys.exit(0)

#Inicia simulação no vrep
def startSim(clientID):
	vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot_wait)

#Para simulação no vrep
def stopSim(clientID):
	vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot_wait)

#Disconecta do client Vrep
def disconnectVREP(clientID):
	vrep.simxFinish(clientID)
	print u"Conexão finalizada"

#===================================================================================
# Funções de Escrita de valores
#===================================================================================

#Escreve angulo de posicao da Junta
def write_angle(clientID, handler, ang):
	vrep.simxSetJointPosition(clientID,handler,radians(ang),vrep.simx_opmode_oneshot)  

#Escreve posição na junta
def write_pos(clientID, handler, m):
	vrep.simxSetJointPosition(clientID,handler, m,vrep.simx_opmode_oneshot)  

#===================================================================================
# Funções de Leitura de Valores
#===================================================================================

#Executa leitura de angulo da junta
def read_angle(clientID, handler):
	ret, joint = vrep.simxGetJointPosition(clientID,handler,vrep.simx_opmode_oneshot_wait)
	joint = round(degrees(joint), 1)
	return joint

#Executa leitura de posição da junta
def read_position(clientID, handler):
	ret, joint = vrep.simxGetJointPosition(clientID,handler,vrep.simx_opmode_oneshot_wait)
	return joint

#Executa leitura de sensor para operação XY do vrep
def read_sensor_xy(clientID, handler1, handler2):

	#Recebe valores de sensor do vrep
	sens1 = read_angle(clientID, handler1)
	sens2 = read_angle(clientID, handler2)

	#Constroi e retorna vetor com angulos de sensor
	sensor = [sens1, sens2]

	return sensor

#Executa leitura de sensor para operação Z do vrep
def read_sensor_z(clientID, handler):
	sens = read_position(clientID, handler)
	return [sens]

#Executa leitura de sensor para operação R do vrep
def read_sensor_r(clientID, handler):
	sens = read_angle(clientID, handler)
	return [sens]