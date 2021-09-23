#include "util.h"

// =================================================================================================
// Definições Fixas (devem ser definidas antes da inicialização do robô)
// =================================================================================================
#define elo1 0.4670f			//Comprimento do Elo 1 (m)
#define elo2 0.4005f			//Comprimento do Elo 2 (m)
#define elo1_min -360			//Ângulo mínimo para a Junta 1 (ang°)
#define elo1_max 360			//Ângulo máximo para a Junta 1 (ang°)
#define elo2_min -360			//Ângulo mínimo para a Junta 2 (ang°)
#define elo2_max 360			//Ângulo máximo para a Junta 2 (ang°)
#define home_z 0.132f			//Posição de repouso do eixo Z
#define delay_z 250				//Delay para V-Rep interpretar captura de peça

// =================================================================================================
// Definições de Configurações (podem ser alteradas durante o funcionamento do robô)
// =================================================================================================
float 	velocity_joint1;		//Velocidade de movimentação da Junta 1
float 	velocity_joint2;		//Velocidade de movimentação da Junta 2
byte 	approach;				//Tipo de aproximação das juntas
byte 	angular_correction;		//Configuração de correção angular	

//Configurações de Aproximação Suave
float 	min_acceptable_step = 0.3f;		//Menor step aceitável para aproximação suave
float 	min_distance 		= 0.9f;		//Menor distancia aceitável p/ de iniciar aproximação suave
float 	fine_approach 		= 0.3f;		//Step durante aproximação suave

// =================================================================================================
// Definições de Comandos (alguns são de uso interno do programa, verificar documentação)
// =================================================================================================
byte scara_command 				= 0;
#define halt(); 				while (true){}
#define command_com_cycle(); 	Serial.write(scara_command);
#define cmd_vrep();				{scara_command = 1; command_com_cycle();}
#define plot_start();			{cmd_vrep(); scara_command = 1; command_com_cycle();}
#define plot_end();				{cmd_vrep(); scara_command = 2; command_com_cycle();}
#define cmd_end();				{scara_command = 3; command_com_cycle();}
#define cmd_debug();			{scara_command = 4; command_com_cycle();}
#define suction_on();			{cmd_vrep(); scara_command = 3; command_com_cycle();}
#define suction_off();			{cmd_vrep(); scara_command = 4; command_com_cycle();}
#define constant 				0
#define fast 					1
#define disabled 				0
#define enabled 				1

// =================================================================================================
// Variaveis Globais
// =================================================================================================
vector_angle 		setpoint;			//Conjunto de ângulos de setpoint p/ enviar ao V-Rep
vector_angle 		sensor;				//Conjunto de ângulos de sensoriamento recebidos do V-Rep
vector_angle 		last_position;		//Conjunto de ângulos para guardar a ultima posição do robô
vector_angle 		last_sensor;		//Conjunto de ângulos para guardar o ultimo sensor do robô
vector4_angle 		inv;				//Conjunto de "hold" para transformação inversa
vector_position 	dir;				//Conjunto de "hold" para transformação direta
float 				setpointz;			//Setpoint para o eixo Z
float 				sensorz;			//Sensor para o eixo Z
float 				setpointr;			//Setpoint para o eixo R
float 				sensorr;			//Sensor para o eixo R

// =================================================================================================
// Funções de Cinemática
// =================================================================================================
vector_position dir_transform(float t1, float t2, float l1, float l2)
{
	//Realiza Transformação Direta
	//Passagem do espaço das juntas para o espaço cartesiano

	//Define vetores de posição
	vector_position r1, r2, r3;

	//Calcula vetores individuais para Elo 1
	r1.x = l1 * cos(radians(t1));
	r1.y = l1 * sin(radians(t1));

	//Calcula vetores individuais para Elo 2
	r2.x = l2 * cos(radians(t1 + t2));
	r2.y = l2 * sin(radians(t1 + t2));

	//Realiza soma vetorial para posição final
	r3.x = r1.x + r2.x;
	r3.y = r1.y + r2.y;

	//Retorna vetor com coordenadas finais
	return r3;
}

vector4_angle inv_transform(float x, float y, float l1, float l2)
{
	//Realiza Transformação Inversa
	//Passagem do espaço cartesiano para o espaço das juntas

	//Define vetor de resposta e variaveis para cálculo
	vector4_angle res;
	float t1, t2, num, den;

	//Calcula t2 para situação com t2 positivo
	t2 = acos((pow(x, 2) + pow(y, 2) - pow(l1, 2) - pow(l2, 2)) / (2 * l1 * l2));

	//Verifica exceção na função acos
	if (isnan(t2)) t2 = 0;

	//Calcula t1 para situação com t2 positivo
	num = y * (l1 + l2 * cos(t2)) - x * l2 * sin(t2);
	den = x * (l1 + l2 * cos(t2)) + y * l2 * sin(t2);
	t1 = atan2(num, den);

	//Realiza conversão de radianos para graus
	t1 = degrees(t1);
	t2 = degrees(t2);

	//Adiciona primeiro conjunto de ângulos ao vetor final
	res.t1 = t1;
	res.t2 = t2;

	//Calcula t2 para situação com t2 negativo
	t2 = -acos((pow(x, 2) + pow(y, 2) - pow(l1, 2) - pow(l2, 2)) / (2 * l1 * l2));

	// Verifica exceção na funçao acos
	if (isnan(t2)) t2 = 0;

	//Calcula t1 para situação com t2 negativo
	num = y * (l1 + l2 * cos(t2)) - x * l2 * sin(t2);
	den = x * (l1 + l2 * cos(t2)) + y * l2 * sin(t2);
	t1 = atan2(num, den);

	//Realiza conversão de radianos para graus
	t1 = degrees(t1);
	t2 = degrees(t2);

	//Adiciona segundo conjunto de ângulos ao vetor final
	res.t3 = t1;
	res.t4 = t2;

	//Retorna vetor final com dois parês de ângulos
	return res;
}

// =================================================================================================
// Funções de Funcionalidade do Sistema
// =================================================================================================
void com_cycle() //Comunicação para movimentação em X Y
{
	//Comunicação Normal, movimentação no eixo X e Y

	// Envia byte de comando
	scara_command = 2; // Move XY Normal
	Serial.write(scara_command);

	// Trata dados a enviar
	int send_setpoint1 = roundx(setpoint.t1, 1) * 10;
	int send_setpoint2 = roundx(setpoint.t2, 1) * 10;
	// Envia angulos de sepoint1
	Serial.write((const char *) & send_setpoint2, sizeof(int));
	Serial.write((const char *) & send_setpoint1, sizeof(int));
	// Cria variaveis responsáveis pela leitura
	int incoming_value1, incoming_value2;
	unsigned char buffer[2];
	// Executa leitura do sensor 1
	Serial.readBytes(buffer, sizeof(int));
	memcpy(& incoming_value1, buffer, sizeof(int));
	// Executa leitura do sensor 2
	Serial.readBytes(buffer, sizeof(int));
	memcpy(& incoming_value2, buffer, sizeof(int));
	// Executa adaptação dos valores lidos
	sensor.t1 = roundx(incoming_value1 / 10.0f, 1);
	sensor.t2 = roundx(incoming_value2 / 10.0f, 1);
}

void com_cycle1() //Comunicação para movimentação em Z
{
	//Movimentação do eixo Z

	//Envia byte de comando
	scara_command = 5; //Move eixo Z
	Serial.write(scara_command);

	//Trata dados a enviar
	int send_setpoint = roundx(setpointz, 3) * 1000;
	//Envia valor de setpoint z
	Serial.write((const char *) & send_setpoint, sizeof(int));
	//Cria variaveis responsáveis pela leitura
	int incoming_value;
	unsigned char buffer[2];
	//Executa leitura do sensor de eixo z
	Serial.readBytes(buffer, sizeof(int));
	memcpy(& incoming_value, buffer, sizeof(int));
	//executa adaptação dos valores lidos
	sensorz = roundx(incoming_value / 1000.0f, 3);
}

void com_cycle2() //Comunicação para movimentação em R
{
	//Rotação da ferramenta, eixo R

	//Envia byte de comando
	scara_command = 6; //Move eixo R
	Serial.write(scara_command);

	//Trata dados a enviar
	int send_setpoint = roundx(setpointr, 1) * 10;
	//Envia valor de setpoint R
	Serial.write((const char *) & send_setpoint, sizeof(int));
	//Cria variaveis responsáveis pela leitura
	int incoming_value;
	unsigned char buffer[2];
	//Executa leitura do sensor de eixo z
	Serial.readBytes(buffer, sizeof(int));
	memcpy(& incoming_value, buffer, sizeof(int));
	//executa adaptação dos valores lidos
	sensorr = roundx(incoming_value / 10.0f, 1);
}

void start_setup() //Procimento de inicialização
{
	//Procedimento Comum

	// Cria variaveis responsáveis pela leitura
	int incoming_value1, incoming_value2, incoming_value3, incoming_value4;
	unsigned char buffer[2];

	//Procedimento para aquisição dos valores XY
	// Executa leitura do sensor 1
	Serial.readBytes(buffer, sizeof(int));
	memcpy(& incoming_value1, buffer, sizeof(int));

	// Executa leitura do sensor 2
	Serial.readBytes(buffer, sizeof(int));
	memcpy(& incoming_value2, buffer, sizeof(int));

	// Executa leitura do sensor Z
	Serial.readBytes(buffer, sizeof(int));
	memcpy(& incoming_value3, buffer, sizeof(int));

	// Executa leitura do sensor Z
	Serial.readBytes(buffer, sizeof(int));
	memcpy(& incoming_value4, buffer, sizeof(int));

	// Executa adaptação dos valores lidos
	sensor.t1 = roundx(incoming_value1 / 10.0f, 1);
	sensor.t2 = roundx(incoming_value2 / 10.0f, 1);
	sensorz   = roundx(incoming_value3 / 1000.0f, 3);
	sensorr   = roundx(incoming_value4 / 10.0f, 1);

	// Define setpoint inicial para posição atual
	setpoint.t1 = sensor.t1;
	setpoint.t2 = sensor.t2;

	plot_end();
}

// =================================================================================================
// Funções Auxiliares
// =================================================================================================
void debug(String text, float value, int dec)
{
	//Seleciona comando de modo debug
	cmd_debug();
	//Envia precisão decimal
	Serial.write(dec);
	//Envia texto de debug
	Serial.println(text);
	//Trata valor de debug a ser enviado
	int send_value = roundx(value, dec) * pow(10, dec);
	//Envia valor de debug
	Serial.write((const char *) & send_value, sizeof(int));
}

vector_angle closest_group(vector4_angle group)
{
	//Realiza escolha de angulos mais próximos dos atuais
	vector_angle response;
	//Computa diferenças para conjunto 1
	float dif1_elo1 = abs_dif(sensor.t1, group.t1); // Diferença do t1 atual para t1 do conjunto 1
	float dif1_elo2 = abs_dif(sensor.t2, group.t2); // Diferença do t2 atual para t2 do conjunto 1
	float dif_mean1 = (dif1_elo1 + dif1_elo2) / 2.0f;
	//Computa diferenças para conjunto 2
	float dif2_elo1 = abs_dif(sensor.t1, group.t3); // Diferença do t1 atual para t1 do conjunto 2
	float dif2_elo2 = abs_dif(sensor.t2, group.t4); // Diferença do t2 atual para t2 do conjunto 2
	float dif_mean2 = (dif2_elo1 + dif2_elo2) / 2.0f;
	//Realiza escolha de conjuntos
	response.t1 = (dif_mean1 <= dif_mean2)? group.t1:group.t3;
	response.t2 = (dif_mean1 <= dif_mean2)? group.t2:group.t4;
	return response;
}

vector_position create_point(float x, float y)
{
	//Cria ponto cartesiano e retorna vetor correspondente
	vector_position hold;
	hold.x = x;
	hold.y = y;
	return hold;
}

// =================================================================================================
// Funções de Movimento
// =================================================================================================

//Movimenta para posição no espaço das juntas
void moveJ(float t1, float t2, bool interpolation)
{
	//Recebe posição de juntas em t1, t2
	//Move para um ponto no espaço das juntas

	//Computa diferenças de angulos
	float dif_elo1 = abs_dif(sensor.t1, t1); // Diferença do t1 atual para t1 do alvo
	float dif_elo2 = abs_dif(sensor.t2, t2); // Diferença do t2 atual para t2 do alvo
	
	//Computa sentido de movimento (crescente ou decrescente)
	float sign_t1 = (sensor.t1 > t1)? -1:1;
	float sign_t2 = (sensor.t2 > t2)? -1:1;

	//Flags
	bool 	moved_j1 	= false	, moved_j2 		= false; 	//Movimento nas juntas finalizado
	float 	step_t1 	= 0		, step_t2 		= 0; 		//Ângulo de incremento a cada ciclo de movimento
	int 	amount_t1 	= 0		, amount_t2 	= 0; 		//Quantidade de ciclos de movimento
	int 	count_t1 	= 0		, count_t2 		= 0;		//Contagem de ciclos de movimento
	bool 	flag_min_t1 = 0		, flag_min_t2 	= 0;		//Distância mínima atingida nas juntas
	float 	soma 		= 0;								//Variável para correção angular

	//Approach 0 - Modo Constante

	if(approach == constant)
	{
		//Linear Mode Constante
		if (interpolation)
		{
			// Movimento com interpolação de juntas
			if (dif_elo1 > dif_elo2)
			{
				// Diferença no elo 1 é maior, serve de unidade para o movimento
				step_t1 = velocity_joint1;
				amount_t1 = round(dif_elo1 / step_t1) + 1;
				step_t2 = dif_elo2 / amount_t1;
				amount_t2 = round(dif_elo2 / step_t2) + 1;
			}
			else
			{
				// Diferença no elo 2 é maior, serve de unidade para o movimento
				step_t2 = velocity_joint2;
				amount_t2 = round(dif_elo2 / step_t2) + 1;
				step_t1 = dif_elo1 / amount_t2;
				amount_t1 = round(dif_elo1 / step_t1) + 1;
			}
		}
		else
		{
			// Movimento sem interpolação de Juntas
			// Define quantidade de pontos de movimento nas juntas
			amount_t1 = round(dif_elo1 / velocity_joint1);
			amount_t2 = round(dif_elo2 / velocity_joint2);
			// Define steps como sendo a resoluçaõ da junta
			step_t1 = velocity_joint1;
			step_t2 = velocity_joint2;
		}

		// Realiza movimento
		while (!moved_j1 || !moved_j2)
		{

			// Movimento para junta 1
			if (count_t1++ < amount_t1)
			{
				setpoint.t1 += (step_t1 * sign_t1);
			}else
				moved_j1 = true;

			// Movimento para junta 2
			if (count_t2++ < amount_t2)
			{
				setpoint.t2 += (step_t2 * sign_t2);
			}else
				moved_j2 = true;

			last_sensor = sensor;
			com_cycle();

			if(angular_correction)
			{
				soma = 0;
				soma += abs_dif(last_sensor.t1, sensor.t1) * sign_t1;
				soma += abs_dif(last_sensor.t2, sensor.t2) * sign_t2;
				setpointr = sensorr - soma;
				com_cycle2();
			}
		}
	} else 
	{
		//Linear Mode Fast
		if(interpolation)
		{
			// Movimento com interpolação de juntas
			if (dif_elo1 > dif_elo2)
			{
				// Diferença no elo 1 é maior, serve de unidade para o movimento
				step_t1 = velocity_joint1;
				amount_t1 = round(dif_elo1 / step_t1) + 1;
				step_t2 = dif_elo2 / amount_t1;
				step_t2 = (step_t2 <= min_acceptable_step)? 0.1f : step_t2;
			}
			else
			{
				// Diferença no elo 2 é maior, serve de unidade para o movimento
				step_t2 = velocity_joint2;
				amount_t2 = round(dif_elo2 / step_t2) + 1;
				step_t1 = dif_elo1 / amount_t2;
				step_t1 = (step_t1 <= min_acceptable_step)? 0.1f : step_t1;
			}
		}

		while(!moved_j1 || !moved_j2)
		{
			//Movimento da Junta 1
			if(!moved_j1)
			{
				//Verifica se pode mover o valor inteiro de step
				if(dif_elo1 >= min_distance + step_t1)
				{
					//Mover o valor de step não ultrapassará a menor distância
					setpoint.t1 = sensor.t1 + (step_t1 * sign_t1);
				} else 
				{
					//Mover o valor de step irá ultrapassar a menor distância
					if(!flag_min_t1)
					{
						//Se mover para a menor distância
						setpoint.t1 = t1 - (min_distance * sign_t1);
						flag_min_t1 = true;
					} else 
					{	
						//Verifica se é permitido iniciar a aproximação suave
						if(dif_elo1 >= fine_approach)
						{
							//Iniciar aproximação suave
							setpoint.t1 = sensor.t1 + (fine_approach * sign_t1);
						} else 
						{
							//Movimento em T1 encerrado
							moved_j1 = true;
						}
					}
				}
			}

			//Movimento da Junta 2
			if(!moved_j2)
			{
				//Verifica se pode mover o valor inteiro de step
				if(dif_elo2 >= min_distance + step_t2)
				{
					//Mover o valor de step não ultrapassará a menor distância
					setpoint.t2 = sensor.t2 + (step_t2 * sign_t2);
				} else 
				{
					//Mover o valor de step irá ultrapassar a menor distância
					if(!flag_min_t2)
					{
						//Se mover para a menor distância
						setpoint.t2 = t2 - (min_distance * sign_t2);
						flag_min_t2 = true;
					} else 
					{	
						//Verifica se é permitido iniciar a aproximação suave
						if(dif_elo2 >= fine_approach)
						{
							//Iniciar aproximação suave
							setpoint.t2 = sensor.t2 + (fine_approach * sign_t2);
						} else 
						{
							//Movimento em T1 encerrado
							moved_j2 = true;
						}
					}
				}
			}

			last_sensor = sensor;
			com_cycle();

			//Realiza correção angular
			if(angular_correction)
			{
				soma = 0;
				soma += abs_dif(last_sensor.t1, sensor.t1) * sign_t1;
				soma += abs_dif(last_sensor.t2, sensor.t2) * sign_t2;
				setpointr = sensorr - soma;
				com_cycle2();
			}
			
			dif_elo1 = abs_dif(sensor.t1, t1);
			dif_elo2 = abs_dif(sensor.t2, t2);

		}
	}
	
	debug("Tgt em   1 ", t1,        1);
	debug("Tgt em   2 ", t2,        1);
	debug("Final em 1 ", sensor.t1, 1);
	debug("Final em 2 ", sensor.t2, 1);
}

//Movimenta para posição no espaço cartesiano
void moveP(float x, float y, bool interpolation)
{
	//Recebe posição de espaço em X, Y
	//Move para um ponto no espaço cartesiano

	//Calcula cinemática inversa
	vector4_angle envelope = inv_transform(x, y, elo1, elo2);

	//Computa diferenças para conjunto 1
	float dif1_elo1 = abs_dif(sensor.t1, envelope.t1); // Diferença do t1 atual para t1 do conjunto 1
	float dif1_elo2 = abs_dif(sensor.t2, envelope.t2); // Diferença do t2 atual para t2 do conjunto 1
	float dif_mean1 = (dif1_elo1 + dif1_elo2) / 2.0f;

	//Computa diferenças para conjunto 2
	float dif2_elo1 = abs_dif(sensor.t1, envelope.t3); // Diferença do t1 atual para t1 do conjunto 2
	float dif2_elo2 = abs_dif(sensor.t2, envelope.t4); // Diferença do t2 atual para t2 do conjunto 2
	float dif_mean2 = (dif2_elo1 + dif2_elo2) / 2.0f;

	//Realiza escolha de conjuntos
	vector_angle output;
	output.t1 = (dif_mean1 <= dif_mean2)? envelope.t1:envelope.t3;
	output.t2 = (dif_mean1 <= dif_mean2)? envelope.t2:envelope.t4;

	//Realiza movimento
	moveJ(output.t1, output.t2, interpolation);
}

//Movimento linear em direção a um ponto no espaço cartesiano
void moveL(float x, float y, int points)
{
	//Recebe posição de espaço em X, Y
	//Move linearmente para um ponto no espaço cartesiano

	//Transformação da posição atual do robô
	vector_position cur_position = dir_transform(sensor.t1, sensor.t2, elo1, elo2);

	float dif_x = abs_dif(x, cur_position.x);	//Diferença a ser percorrida no eixo x
	float dif_y = abs_dif(y, cur_position.y);	//Diferença a ser percorrida no eixo y
	char sign_x = (cur_position.x > x)? -1:1;	//Sentido da diferença no eixo x
	char sign_y = (cur_position.y > y)? -1:1;	//Sentido da diferença no eixo y

	// Flags
	int count = 0;		//Contagem de pontos realizados
	float next_x = 0;	//Próxima coordenada em x
	float next_y = 0;	//Próxima coordenada em y
	float step_x = 0;	//Step por ciclo em x
	float step_y = 0;	//Step por ciclo em y

	//Define steps para eixos x e y
	step_x = dif_x / points;
	step_y = dif_y / points;

	//Inicia interpolação
	while(count++ < points)
	{
		//Computa próxima coordenada da interpolação
		next_x = cur_position.x + (sign_x * step_x * count);
		next_y = cur_position.y + (sign_y * step_y * count);

		//Realiza transformação e seleciona grupo mais próximo
		vector4_angle group = inv_transform(next_x, next_y, elo1, elo2);
		vector_angle output = closest_group(group);

		//Define setpoint
		setpoint.t1 = output.t1;
		setpoint.t2 = output.t2;

		//Realiza movimentação
		com_cycle();
	}
}

//Movimento para posição no espaço cartesiano a partir de posição de offset
void moveP_Offs(vector_position point, float x, float y, bool interpolation)
{
	//Realiza movimento ponto a ponto com offset
	moveP(point.x + x, point.y + y, interpolation);
}

//Movimento linear a partir de posição de offset
void moveL_Offs(vector_position point, float x, float y, byte points)
{
	//Realiza movimento linear com offset
	moveL(point.x + x, point.y + y, points);
}

//Movimento do eixo Z
void moveZ(float z, float speed)
{
	//Tempo para o V-rep processar o comando do eixo z
	delay(delay_z);

	//Computa valor real de Z
	float tgt_z = home_z - z;
	//Computa diferença no valor do eixo Z
	float dif_z = abs_dif(sensorz, tgt_z);
	// Computa sentido de movimento (crescente ou decrescente)
	char sign_z = (sensorz > tgt_z)? -1 : 1;

	//Flags
	bool 	moved_z 		= false;	//Movimento no eixo z realizado
	bool 	flag_min_z 		= false;	//Distancia mínima atingida
	float 	min_distance 	= 0.01f;	//Distancia para iniciar aproximação suave
	float 	fine_approach 	= 0.001f;	//Step para aproximação suave

	while(!moved_z)
	{
		//Verifica se pode mover o valor inteiro do step
		if(dif_z >= min_distance + speed)
		{
			//Mover o valor de step não ultrapassará a menor distância
			setpointz = sensorz + (speed * sign_z);
		} else
		{
			if(!flag_min_z)
			{
				//Mover para distância mínima
				setpointz = tgt_z - (min_distance * sign_z);
				flag_min_z = true;
			} else 
			{
				if(roundx(dif_z, 3) >= fine_approach)
				{
					//Verifica se é possível iniciar a aproximação suave
					setpointz = sensorz + (fine_approach * sign_z);
				} else 
				{
					moved_z	= true;
				}
			}
		}

		//Realiza Movimento
		com_cycle1();

		//Recalcula Diferença
		dif_z = abs_dif(sensorz, tgt_z);
	}

	debug("Tgt em   Z ", tgt_z,   3);
	debug("Final em Z ", sensorz, 3);
}

//Movimento da rotação da ferramenta
void moveR(float ang, float step)
{
	//Converter valor para quadrante do vrep
	//Classificar qual sentido oferece menor distancia
	//Se mover em direção ao sentido
	//Se mover até ser maior/menor ou igual a posição desejada

	//Converter valor desejado para quadrante do Vrep
	//float tgt_angle = (ang < 180)? ang : -(- abs(ang - 180) + 180);

	//Converter quadrante do vrep para quadrante real
	float sensor_real = (sensorr >= 0)? sensorr : 360 + sensorr; 

	//Classificar qual sentido oferece menor distancia
	int dir = ((sensorr - ang) > (360 - sensorr + ang))? -1 : 1;   

	//Calcula a distância
	float delta = degrees(atan2(sin(radians(ang - sensorr)), cos(radians(ang - sensorr))));

	float steps = 0;
	while(steps < abs(delta))
	{
		setpointr += sign(delta) * step;
		com_cycle2();
		steps += step;
	}
}

void addR(float ang, float step)
{
	//Adiciona rotação "ang" ao angulo atual da ferramenta do V-Rep
	float steps = 0;
	while(steps < abs(ang))
	{
		setpointr += step;
		com_cycle2();
		steps += step;
	}
}

// =================================================================================================
// Funções de Desenho
// =================================================================================================

void quadrado()
{
	// Quadrado com Interpolação Linear

	moveP(0.3f, 0.6f, true); 	//Movimenta para o ponto inicial
	plot_start(); 				//Inicia coleta de trajetória
	moveL(0.6f, 0.6f, 10);		//Movimento Linear
	moveL(0.6f, 0.3f, 10);		//Movimento Linear
	moveL(0.3f, 0.3f, 10);		//Movimento Linear
	moveL(0.3f, 0.6f, 10);		//Movimento Linear
	plot_end(); 				//Finaliza coleta de trajetória
	moveJ(0.0f, 0.0f, true); 	//Movimenta para ponto de repouso
}

void ifsp()
{
	//Escrita da palavra "IFSP" 

	//Letra "I"
	moveP(-0.35f, 0.70f, true);
	plot_start();
	moveL(-0.25f, 0.70f, 60);
	plot_end();
	moveP(-0.3f, 0.70f, true);
	plot_start();
	moveL(-0.3f, 0.50f, 120);
	plot_end();
	moveP(-0.35f, 0.50, true);
	plot_start();
	moveL(-0.25f, 0.50f, 60);
	plot_end();
	
	//Letra "F"
	moveP(-0.15f, 0.50f, true);
	plot_start();
	moveL(-0.15f, 0.70f, 120);
	moveL(-0.05f, 0.70f, 60);
	plot_end();
	moveP(-0.15f, 0.60f, true);
	plot_start();
	moveL(-0.05f, 0.60f, 60);
	plot_end();
	
	//Letra "S"
	moveP(0.05f, 0.50f, true);
	plot_start();
	moveL(0.15f, 0.50f, 60);
	moveL(0.15f, 0.60f, 60);
	moveL(0.05f, 0.60f, 60);
	moveL(0.05f, 0.70f, 60);
	moveL(0.15f, 0.70f, 60);
	plot_end();

	//Letra "P"
	moveP(0.25f, 0.60f, true);
	plot_start();
	moveL(0.35f, 0.60f, 120);
	moveL(0.35f, 0.70f, 60);
	moveL(0.25f, 0.70f, 60);
	moveL(0.25f, 0.50f, 60);
	plot_end();
}

void cubo()
{
	//Desenho de Figura Tridimensional

	vector_position home = create_point(0.20f, 0.7f);

	float side = 0.24f; 
	float deep = 0.012f;

	moveP_Offs(home, 0, 0, true);

	plot_start();
	moveL_Offs(home, side,     0, 120);
	moveL_Offs(home, side, -side, 120);
	moveL_Offs(home,    0, -side, 120);
	moveL_Offs(home,    0,     0, 120);

	moveZ(deep, 0.001f);

	moveL_Offs(home, side,     0, 120);
	moveL_Offs(home, side, -side, 120);
	moveL_Offs(home,    0, -side, 120);
	moveL_Offs(home,    0,     0, 120);

	plot_end();
	
	moveZ(home_z, 0.001f);

	moveP_Offs(home, side, 0, true);
	plot_start();
	moveZ(deep, 0.001f);
	plot_end();
	moveZ(home_z, 0.001f);
	plot_end();
	
	moveP_Offs(home, side, -side, true);

	plot_start();
	moveZ(deep, 0.001f);
	plot_end();
	moveZ(home_z, 0.001f);
	plot_end();
	
	moveP_Offs(home, 0, -side, true);

	plot_start();
	moveZ(deep, 0.001f);
	plot_end();
	moveZ(home_z, 0.001f);
	plot_end();	
}

void logo()
{
	//Montagem do Logo IFSP

	vector_position piece1  = create_point(0.15, 0.50);
	vector_position piece2  = create_point(0.35, 0.60);
	vector_position piece3  = create_point(0.35, 0.60);
	vector_position piece4  = create_point(0.55, 0.60);
	vector_position piece5  = create_point(0.55, 0.60);
	vector_position piece6  = create_point(0.35, 0.40);
	vector_position piece7  = create_point(0.35, 0.40);
	vector_position piece8  = create_point(0.55, 0.40);
	vector_position piece9  = create_point(0.55, 0.40);
	vector_position piece10 = create_point(0.55, 0.40);

	vector_position tgt_piece1  = create_point(-0.80,  0.30);
	vector_position tgt_piece2  = create_point(-0.80,  0.15);
	vector_position tgt_piece3  = create_point(-0.80,  0.00);
	vector_position tgt_piece4  = create_point(-0.80, -0.15);
	vector_position tgt_piece5  = create_point(-0.65,  0.30);
	vector_position tgt_piece6  = create_point(-0.65,  0.15);
	vector_position tgt_piece7  = create_point(-0.65,  0.00);
	vector_position tgt_piece8  = create_point(-0.65, -0.15);
	vector_position tgt_piece9  = create_point(-0.50,  0.30);
	vector_position tgt_piece10 = create_point(-0.50,  0.00);

	float alt1        = 0.030;
	float alt2        = 0.060;
	float alt3        = 0.090;
	float alt_solta   = 0.030;

	float zspeed = 0.01f;

	//Captura Peça 2
	moveP(piece2.x, piece2.y, true);
	moveZ(alt2, zspeed);
	suction_on();
	moveZ(home_z, zspeed);
	//Entrega Peça 2
	moveP(tgt_piece2.x, tgt_piece2.y, true);
	moveZ(alt_solta, zspeed);
	suction_off();
	moveZ(home_z, zspeed);

	//Captura Peça 3
	moveP(piece3.x, piece3.y, true);
	moveZ(alt1, zspeed);
	suction_on();
	moveZ(home_z, zspeed);
	//Entrega Peça 3
	moveP(tgt_piece3.x, tgt_piece3.y, true);
	moveZ(alt_solta, zspeed);
	suction_off();
	moveZ(home_z, zspeed);

	//Captura Peça 4
	moveP(piece4.x, piece4.y, true);
	moveZ(alt2, zspeed);
	suction_on();
	moveZ(home_z, zspeed);
	//Corrige Rotação
	addR(45, 1);
	//Entrega Peça 4
	moveP(tgt_piece4.x, tgt_piece4.y, true);
	moveZ(alt_solta, zspeed);
	suction_off();
	moveZ(home_z, zspeed);

	//Captura Peça 5
	moveP(piece5.x, piece5.y, true);
	moveZ(alt1, zspeed);
	suction_on();
	moveZ(home_z, zspeed);
	//Corrige Rotação
	addR(45, 1);
	//Entrega Peça 4
	moveP(tgt_piece5.x, tgt_piece5.y, true);
	moveZ(alt_solta, zspeed);
	suction_off();
	moveZ(home_z, zspeed);

	//Captura Peça 6
	moveP(piece6.x, piece6.y, true);
	moveZ(alt2, zspeed);
	suction_on();
	moveZ(home_z, zspeed);
	//Corrige Rotação
	addR(45, 1);
	//Entrega Peça 4
	moveP(tgt_piece6.x, tgt_piece6.y, true);
	moveZ(alt_solta, zspeed);
	suction_off();
	moveZ(home_z, zspeed);

	//Captura Peça 7
	moveP(piece7.x, piece7.y, true);
	moveZ(alt1, zspeed);
	suction_on();
	moveZ(home_z, zspeed);
	//Corrige Rotação
	addR(45, 1);
	//Entrega Peça 7
	moveP(tgt_piece7.x, tgt_piece7.y, true);
	moveZ(alt_solta, zspeed);
	suction_off();
	moveZ(home_z, zspeed);

	//Captura Peça 8
	moveP(piece8.x, piece8.y, true);
	moveZ(alt3, zspeed);
	suction_on();
	moveZ(home_z, zspeed);
	//Entrega Peça 2
	moveP(tgt_piece8.x, tgt_piece8.y, true);
	moveZ(alt_solta, zspeed);
	suction_off();
	moveZ(home_z, zspeed);

	//Captura Peça 9
	moveP(piece9.x, piece9.y, true);
	moveZ(alt2, zspeed);
	suction_on();
	moveZ(home_z, zspeed);
	//Entrega Peça 2
	moveP(tgt_piece9.x, tgt_piece9.y, true);
	moveZ(alt_solta, zspeed);
	suction_off();
	moveZ(home_z, zspeed);

	//Captura Peça 2
	moveP(piece10.x, piece10.y, true);
	moveZ(alt1, zspeed);
	suction_on();
	moveZ(home_z, zspeed);
	//Entrega Peça 2
	moveP(tgt_piece10.x, tgt_piece10.y, true);
	moveZ(alt_solta, zspeed);
	suction_off();
	moveZ(home_z, zspeed);

	//Captura Peça 1
	moveP(piece1.x, piece1.y, true);
	moveZ(alt1, zspeed);
	suction_on();
	moveZ(home_z, zspeed);
	//Entrega Peça 2
	moveP(tgt_piece1.x, tgt_piece1.y, true);
	moveZ(alt_solta, zspeed);
	suction_off();
	moveZ(home_z, zspeed);
}

void teste_movej()
{
	moveJ(90, 0, true);		//Movimenta para ponto inicial
	plot_start();			//Inicia coleta da trajetória
	moveJ(0, 70, false);	//Realiza movimento no espaço das juntas
	plot_end();				//Finaliza coleta da trajetória
	moveJ(0, 0, true);		//Movimenta para posição de repouso
}

void teste_movep()
{
	vector_position home = create_point(0.20f, 0.7f);
	moveP_Offs(home, 0, 0, true);

	float side = 0.24f;

	plot_start();
	moveP_Offs(home,  side,     0, true);
	moveP_Offs(home,  side, -side, true);
	moveP_Offs(home,     0, -side, true);
	moveP_Offs(home,     0,     0, true);
	plot_end();
}

void teste_movel()
{
	vector_position home = create_point(0.20f, 0.7f);
	moveP_Offs(home, 0, 0, true);

	float side = 0.24f;
	int points = 120;

	plot_start();
	moveL_Offs(home,  side,     0, points);
	moveL_Offs(home,  side, -side, points);
	moveL_Offs(home,     0, -side, points);
	moveL_Offs(home,     0,     0, points);
	plot_end();
}

void move_rot()
{
	delay(2000);

	moveJ(130, -45, true);

	vector_position pos1  = create_point(0.0f, 0.7f);
	vector_position pos2  = create_point(0.6f, 0.4f);

	float alt1 = 0.030;	
	float zspeed = 0.01f;

	//Captura Peça
	moveP(pos1.x, pos1.y, true);
	moveZ(alt1, zspeed);
	suction_on();
	moveZ(home_z, zspeed);

	//Entrega Peça
	moveP(pos2.x, pos2.y, true);
	moveZ(alt1, zspeed);
	suction_off();
	moveZ(home_z, zspeed);

	moveJ(0, 0, true);

	delay(100000);
	//cmd_end();
	halt();
}

void workspace()
{
	moveJ(90, 0, false);

	moveJ(-25, -115, false);
	plot_start();
	moveJ(-25, 0, false);
	moveJ(205, 0, false);
	moveJ(205, 115, false);
	moveJ(25, 115, false);
	plot_end();
	moveJ(-25, -115, false);
	plot_start();
	moveJ(125, -115, false);
	plot_end();
	moveJ(90, 0, false);
}

void triangle()
{
	moveP(0.15f, 0.55f, true);
	moveZ(0.03, 0.01);
	plot_start();
	moveL(-0.15f, 0.55f, 150);
	moveL(0.0f, 0.8f, 150);
	moveL(0.15f, 0.55f, 150);
	plot_end();
	moveZ(home_z, 0.01);
}

void square()
{
	moveP(0.25, 0.65, true);
	moveZ(0.03, 0.01);
	plot_start();
	moveL(0.45, 0.65, 150);
	moveL(0.45, 0.45, 150);
	moveL(0.25, 0.45, 150);
	moveL(0.25, 0.65, 150);
	plot_end();
	moveZ(home_z, 0.01);
}

void octo()
{
	moveP(0.55, 0.40, true);
	moveZ(0.03, 0.01);
	plot_start();
	moveL(0.70, 0.40, 150);
	moveL(0.80, 0.30, 150);
	moveL(0.80, 0.15, 150);
	moveL(0.70, 0.05, 150);
	moveL(0.55, 0.05, 150);
	moveL(0.45, 0.15, 150);
	moveL(0.45, 0.30, 150);
	moveL(0.55, 0.40, 150);
	plot_end();
	moveZ(home_z, 0.01);
}

void figures()
{
	triangle();
	square();
	octo();
}

void circle(float j, float k, float radius, float sides)
{
	float x, y;
	float step = 360 / sides;

	//Movimenta para ponto inicial
	x = radius * cos(radians(0)) + j;
	y = radius * sin(radians(0)) + k;
	moveP(x, y, true);

	moveZ(0.03, 0.1);
	plot_start();

	//Movimenta pelo circulo
	for(int i = 1; i < sides; i++)
	{
		x = radius * cos(radians(i * step)) + j;
		y = radius * sin(radians(i * step)) + k;
		moveL(x, y, 10);
	}

	//Movimenta para ponto inicial (segmento final de reta)
	x = radius * cos(radians(0)) + j;
	y = radius * sin(radians(0)) + k;
	moveL(x, y, 10);

	plot_end();
	moveZ(home_z, 0.01);
}

//circle(0.00f, 0.65f, 0.15f, 32);

// =================================================================================================
// Funções do Arduino
// =================================================================================================
void setup()
{
	//Inicia porta de comunicação serial
	Serial.begin(230400);

	//Adquire angulos de inicio do robô
	start_setup();
}

void loop()
{
	//Configuração
	velocity_joint1 	= 0.3f;
	velocity_joint2 	= 0.3f;
	approach 			= constant;
	angular_correction 	= enabled;

	//Rotina de Movimentação
	circle(0.00f, 0.65f, 0.15f, 64);

	//Finalização
	moveJ(0, 0, true);
	cmd_end();
	halt();
}
