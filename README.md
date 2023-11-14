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

## Explicação do Código

O projeto consiste em dois scripts principais: `chatbot.py` e `input_node.py`. Abaixo, os componentes chave de cada um desses arquivos são detalhados.

### chatbot.py

Este arquivo contém a lógica principal do chatbot.

#### Classe `CommandProcessor`
```python
class CommandProcessor:
    def __init__(self, intencoes, acoes):
        self.intencoes = intencoes
        self.acoes = acoes

    def processar_comando(self, comando):
        # Processamento do comando aqui
```

- **Função `__init__`**: Inicializa o processador de comandos com dois dicionários: `intencoes` e `acoes`. `intencoes` mapeia padrões de expressão regular para identificar as intenções dos usuários. `acoes` associa essas intenções a funções específicas que o chatbot executará.
- **Função `processar_comando`**: Analisa o comando do usuário, verifica se ele corresponde a algum padrão em `intencoes`, e executa a ação correspondente em `acoes`. Se o comando não for reconhecido, retorna uma mensagem de erro.

#### Classe `ChatbotNode`

```python
class ChatbotNode(Node):
    def __init__(self, command_processor):
        # Inicialização do nó aqui

    def listener_callback(self, msg):
        # Lógica para receber e responder a comandos
```

- **Função `__init__`**: Define o nó ROS, inicializa o processador de comandos e configura um publisher e um subscriber.
- **Função `listener_callback`**: Recebe mensagens do tópico `chat_commands`, processa essas mensagens usando `CommandProcessor`, e publica as respostas no tópico `chat_responses`.

### input_node.py

Este arquivo cria um nó para receber entrada do usuário.

#### Classe `InputNode`

```python
class InputNode(Node):
    def __init__(self):
        # Configuração inicial do nó

    def run(self):
        # Loop para receber entrada do usuário

    def publish_command(self, command):
        # Publica comandos no tópico ROS
```

- **Função `__init__`**: Configura o nó ROS e inicializa um publisher para enviar comandos ao chatbot.
- **Função `run`**: Mantém o nó rodando, aguardando entradas do usuário. Quando uma entrada é recebida, ela é publicada no tópico `chat_commands`.
- **Função `publish_command`**: Publica o comando do usuário no tópico ROS.



### Trechos de Código Relevantes

- **Padrões de expressões regulares em `intencoes`**: Define como o chatbot interpreta os comandos do usuário. Por exemplo, padrões como `r"V[áa] para (.+)"` ou `r"Me leve para (.+)"` são usados para identificar a intenção do usuário de mover o robô para um local específico.

   ```python
   intencoes = {
      "ir_para": [
         r"V[áa] para (?:o|a|os|as)? (.+)",
         # Outras expressões aqui
      ],
   }
   ```

- **Publicação e subscrição ROS**: O chatbot utiliza o sistema de publicação e subscrição do ROS para receber comandos (`chat_commands`) e enviar respostas (`chat_responses`).

   ```python
   # Configuração do publicador e assinante no ChatbotNode
   self.publisher_ = self.create_publisher(String, 'chat_responses', 10)
   self.subscription = self.create_subscription(String, 'chat_commands', self.listener_callback, 10)
   ```

## Demonstração
https://youtu.be/UQHfzfAOxk4