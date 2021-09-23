#include <arduino.h>

//Vetor de Posição
//Armazena ponto de coordenadas no espaço (x, y)
typedef struct
{
	float x = 0, y = 0;
} vector_position;

//Vetor de Ângulos
//Armazena configuração de angulos nas juntas (t1, t2)
typedef struct
{
	float t1 = 0, t2 = 0;
} vector_angle;

//Vetor de Ângulos - Dois pares de ângulos
//Armazena ângulos nas juntas em dois pares (t1, t2) e (t3, t4)
typedef struct
{
	float t1 = 0, t2 = 0, t3 = 0, t4 = 0; 
} vector4_angle;

//Função para arredondar valores em floats
//Arredonda com precisão de n casas decimais
float roundx(float value, byte dec)
{
	return round(value * int(pow(10, dec))) / pow(10, dec);
}

//Função para calcular diferença absoluta entre dois valores
float abs_dif(float value1, float value2)
{
	return max(value1, value2) - min(value1, value2);
}

//Função para calcular sinal de um valor
// -1 para valores menores do que zero
//  0 para valores iguais a zero
//  1 para valores maiores do que zero
int sign(float a)
{
	if(a > 0) return 1;
	if(a < 0) return -1;
	if(a == 0) return 0;
}