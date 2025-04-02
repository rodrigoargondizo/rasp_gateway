# Gateway Modbus TCP/DNP3
Projeto de TCC - Engenharia de Controle e Automação (UFU)

Descrição:
Este projeto consiste no desenvolvimento de um gateway embarcado capaz de se comunicar com dispositivos Modbus TCP e transmitir os dados utilizando o protocolo DNP3.
O objetivo é integrar equipamentos industriais em sistemas SCADA, otimizando a interoperabilidade entre diferentes protocolos.


Resumo dos projetos:

Main_Project: Este é um ambiente de testes que simula diversos pontos de dados, permitindo validar o funcionamento geral do gateway antes de testá-lo com equipamentos reais. Serve como uma bancada de desenvolvimento para verificar a comunicação entre Modbus TCP e DNP3, garantindo que o sistema funcione corretamente.

Real_Demo_Project: Este projeto demonstra o gateway em operação real, comunicando-se com equipamentos de energia, como inversores fotovoltaicos, medidores de energia ou outros dispositivos compatíveis com Modbus TCP. O objetivo é validar a funcionalidade do gateway em um cenário de aplicação prática, garantindo sua compatibilidade e confiabilidade no ambiente SCADA.

Slave_Modbus_TCP_ESP8266: Implementação de um dispositivo escravo Modbus TCP rodando em um ESP8266. Ele é utilizado para testes do gateway, simulando dispositivos reais de campo. Esse recurso facilita a validação da comunicação do gateway sem a necessidade de ter um equipamento industrial disponível.
