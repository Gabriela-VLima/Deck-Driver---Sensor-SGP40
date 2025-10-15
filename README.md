## SGP40 - Deck Driver para Crazyflie
Este repositório contém o deck driver para o sensor de Compostos Orgânicos Voláteis (VOC) SGP40, fabricado pela Sensirion. Este driver foi desenvolvido para ser integrado ao firmware da aeronave Crazyflie 2.1 da Bitcraze, permitindo o monitoramento da qualidade do ar em voo.

## 1. Visão Geral e Aplicação
O sensor SGP40 é um sensor de qualidade do ar interno (IAQ) que mede a concentração de compostos orgânicos voláteis (VOCs). Sua integração à plataforma Crazyflie 2.1 possibilita que o drone colete dados ambientais em tempo real, durante o voo.

Componentes Chave:
- Sensor SGP40. Medição de VOCs. Comunicação I²C
- Crazyflie 2.1. Plataforma de voo e processamento. Firmware em C/C++

<img width="165" height="220" alt="Sensor" src="https://github.com/user-attachments/assets/d7e45ac0-79fe-4539-9a97-7cf503aa53ff" />

## 2. Instalação e Conexão Física

O sensor SGP40 é integrado ao Crazyflie através da interface I²C.

Endereço I²C do Sensor: 0x59 (hexadecimal).

A placa do sensor SGP40 possui cinco pinos, dos quais quatro foram utilizados neste projeto. O pino Vin foi conectado à saída de 3 V do Crazyflie, enquanto o pino GND foi ligado ao terra (GND) da aeronave. Os pinos de comunicação I2C, SDA e SCL, foram conectados aos respectivos pinos da mesma interface no Crazyflie.

<img width="975" height="318" alt="image" src="https://github.com/user-attachments/assets/56c161d3-9636-43be-a92a-c15f1c5ef138" />

## 3. Implementação do Firmware (Linguagem C)
O core deste projeto é um programa em linguagem C desenvolvido para gerenciar a operação do sensor SGP40 dentro do ambiente do firmware da Crazyflie.

A implementação segue rigorosamente as instruções de comunicação especificadas no Datasheet do SGP40, conforme ilustrado no fluxograma de operação (Figura 4).

Fluxo de Operação e Rotinas:
- Inicialização: O driver realiza a inicialização do sensor.
- Leitura Periódica: A leitura dos dados é realizada com uma frequência fixa de um segundo.
- Verificação de Integridade: Antes de registrar ou utilizar os dados, o firmware realiza uma verificação da integridade da informação lida por meio da soma de verificação CRC-8.
- Registro: Após a verificação do CRC-8, os dados são registrados no sistema de log do Crazyflie.

<img width="603" height="245" alt="image" src="https://github.com/user-attachments/assets/7006ffdc-00fd-489a-8dd9-235c77c163f7" />

## 4. Análise do Sinal SRAW 
O sinal de saída do sensor SGP40 é denominado SRAW.

O SRAW é um valor de 16 bits expresso em 'ticks', que varia de 0 a 65.535 unidades.

Obervse que este sinal é uma estimativa da concentração de compostos orgânicos voláteis no ambiente. Uma correlação quantitativa precisa entre os 'ticks' e as concentrações reais de VOC só pode ser estabelecida por meio de um processo de calibração com gases de referência de concentrações conhecidas, um procedimento que não foi realizado neste projeto.

## 5. Referências
Instruções Bitcraze para Deck Driver: [Repositório Bitcraze Deck Driver ](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/development/howto/)

Especificações do Sensor: [Datasheet SGP40 ](https://sensirion.com/media/documents/296373BB/6203C5DF/Sensirion_Gas_Sensors_Datasheet_SGP40.pdf)

