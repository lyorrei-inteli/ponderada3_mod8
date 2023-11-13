# Chatbot Simples para Controle de Robô

## Objetivo

Este projeto visa assimilar os conceitos relacionados a chatbots baseados em regras, especialmente aplicados no contexto de robótica. O objetivo principal é desenvolver um chatbot que possa interpretar comandos em linguagem natural para interagir com um robô de serviço fictício.

## Enunciado

O chatbot, implementado como um nó do ROS (Robot Operating System), deve ser capaz de compreender comandos escritos em linguagem natural. Estes comandos permitem ao usuário enviar instruções de posicionamento para um robô de forma intuitiva, como "Vá para a secretaria", "Dirija-se ao laboratório", ou "Me leve para a biblioteca".

## Funcionalidades

- Interpretação de comandos em linguagem natural.
- Extração de intenções do usuário através de expressões regulares.
- Mapeamento de intenções a ações específicas que o robô deve executar.
- Feedback ao usuário sobre a compreensão e execução dos comandos.

## Tecnologias Utilizadas

- ROS2 (Robot Operating System) para comunicação e controle do robô.
- Python para scripting e lógica do chatbot.
- Bibliotecas como `rclpy` e `std_msgs`.

## Instalação e Execução

Siga as instruções abaixo para instalar e executar o chatbot em seu ambiente ROS2:

1. Compile o projeto:

   ```
   colcon build
   ```

2. Dê um source do arquivo de setup (caso esteja utilizando zsh, mude o final do comando):

   ```
   source install/local_setup.bash 
   ```

3. Execute o nó do chatbot:

   ```
   ros2 run chatbot chatbot
   ```

4. Em um novo terminal, execute o nó de entrada de comandos:

   ```
   ros2 run chatbot input_node
   ```

5. Insira os comandos no nó de entrada para interagir com o chatbot.

## Demonstração

