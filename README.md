# Projeto: Tarefa de Revisão – Raspberry Pi Pico

[![Vídeo de Demonstração](https://img.shields.io/badge/Assista%20no-YouTube-red?logo=youtube)](https://youtube.com/shorts/C3Dsy58RiMg?feature=share)

## 📋 Sumário

- [🎯 Objetivo](#🎯-objetivo)
- [⚙️ Funcionalidades Implementadas](#️-funcionalidades-implementadas)
- [🔌 Componentes Utilizados](#-componentes-utilizados)
- [📦 Instruções de Importação e Implementação](#-instruções-de-importação-e-implementação)
- [👨‍💻 Autoria](#-autoria)
- [📝 Descrição do Projeto](#-descrição-do-projeto)

## 🎯 Objetivo

Desenvolver uma aplicação embarcada utilizando a Raspberry Pi Pico, com foco na revisão de conceitos básicos de programação embarcada e interação com hardware aprendidas na fase 1 da capacitação.

## ⚙️ Funcionalidades Implementadas

- Contador numerico na matriz de LEDs 5x5 
- Controle de LEDs por PWM proporcionalmente ao movimento do joystick
- Exibição no display SSD1306 de um quadrado 8x8 pixels centralizado que se move proporcionalmente ao joystick 
- Comunicação via UART para controle dos buzzers
- Os botões A e B incrementam e decrementam, respectivamente, o valor na matriz de LEDs 

## 🔌 Componentes Utilizados

- Microcontrolador **Raspberry Pi Pico W** (BitDogLab)
- Display: OLED SSD1306 (128x64, I2C)  
- Matriz LED: 5x5 (WS2812B).  
- Joystick: Eixos analógicos + botão.  
- LED RGB: PWM (Vermelho, Verde, Azul)  
- Buzzer: Feedback sonoro.  
- Cabo micro USB para conexão com o computador  

## 📦 Instruções de Importação e Implementação

### 1. Pré-requisitos

- Instale o [Visual Studio Code](https://code.visualstudio.com/)  
- Instale a extensão **Raspberry Pi Pico (pico-sdk)** no VS Code  

### 2. Clone o repositório

```bash
git clone https://github.com/EderRenato/Tarefa_Revisao.git
```

### 3. Abra o projeto no VS Code

- Vá em `Arquivo > Abrir Pasta` e selecione a pasta clonada.

### 4. Configure o ambiente

- Siga as instruções da extensão Raspberry Pi Pico para configurar o SDK e o CMake.

### 5. Compile o projeto

- Use o botão **Build** ou pressione `Ctrl+Shift+B`.

### 6. Carregue na placa
- Mantenha o botão **BOOTSEL** pressionado enquanto conecta a Raspberry Pi Pico no PC.
#### Opção 1:
- Após conectar no modo **BOOTSEL**, pressione o botão **Run**
#### Opção 2:
- Após conectar no modo **BOOTSEL**, Copie o arquivo `.uf2` gerado para a unidade que aparecerá.

## 👨‍💻 Autoria

## OBS:
**Para a execução das ações com os buzzers deve-se utilizar o Monitor Serial do VS Code no modo Terminal ou utilizar o Putty**

Projeto desenvolvido por **Eder Renato** como parte das atividades de revisão para o ambiente de desenvolvimento com Raspberry Pi Pico na Fase 2 da Capacitação **CEPEDI/Embarcatech** .

## 📝 Descrição do Projeto

Este projeto tem como finalidade revisar os principais conceitos de desenvolvimento embarcado utilizando a Raspberry Pi Pico.  
Ele aborda tópicos como estruturação de código em C, uso do SDK oficial, manipulação de GPIOs, controle de tempo e lógica condicional, utilizando todos os periféricos utilizados na fase 1 da capacitação em sistemas embarcados **CEPEDI/Embarcatech** promovendo uma base sólida para futuros projetos mais complexos no ambiente da computação física.

