# MPU6050Lib

# Biblioteca MPU6050 para ESP-IDF

Esta biblioteca fornece uma interface para utilizar o sensor MPU6050 (Acelerômetro e Giroscópio) com ESP-IDF. Todo o processo de comunicação I2C e cálculo matemático é feito em segundo plano, permitindo que você consulte os dados processados e prontos para uso no seu código principal.

--------------------------------

## Requisitos e Configuração

Para compilar o projeto corretamente, o arquivo `CMakeLists.txt` deve registrar o componente e declarar as dependências necessárias, que são `driver` e `esp_timer`.

A biblioteca utiliza o barramento I2C do ESP32. Os pinos estão definidos internamente na biblioteca(Para esp32-c3):
* **SDA**: GPIO 21
* **SCL**: GPIO 22

--------------------------------

## Estrutura/Tarefas

A biblioteca gerencia duas tarefas internas de forma invisível para o usuário:

1. **sensor_task**: Conecta-se ao sensor via I2C, coleta as leituras brutas e as coloca em uma fila de processamento.
2. **processing_task**: Pega os dados da fila, calcula os ângulos reais usando integração e filtros matemáticos (Filtro Complementar), define o status de movimento e salva a snapshot.

--------------------------------

## Referência da API (Arquivo `.h`)

Abaixo estão detalhadas todas as estruturas, variáveis e funções que você utilizará no seu código principal.

### Estrutura de Leitura: `mpu_snapshot_t`

Para ler o sensor, você precisa criar uma variável do tipo `mpu_snapshot_t`. Esta estrutura armazena todos os dados mastigados e calculados. Ela possui as seguintes variáveis internas:

* **Aceleração (Unidade: Força G)**
  * `ax`: Aceleração no eixo X.
  * `ay`: Aceleração no eixo Y.
  * `az`: Aceleração no eixo Z.

* **Giroscópio (Unidade: Graus por segundo - °/s)**
  * `gx`: Rotação no eixo X.
  * `gy`: Rotação no eixo Y.
  * `gz`: Rotação no eixo Z.

* **Métricas Derivadas (Análise de Força)**
  * `magnitude`: Representa a força resultante (tamanho do vetor aceleração).
  * `delta`: É a diferença da magnitude atual para a anterior, mostrando se houve uma mudança brusca na aceleração.

* **Orientação (Unidade: Graus)**
  * `pitch`: Inclinação do sensor para frente ou para trás (eixo de arfagem).
  * `roll`: Inclinação do sensor para a esquerda ou para a direita (eixo de rolagem).

* **Estado do Movimento**
  * `estado`: Uma variável de texto que classifica a situação atual do sensor. Pode retornar quatro estados dependendo da força registrada: `"PARADO"`, `"LEVE"`, `"BRUSCO"` ou `"IMPACTO"`.

* **Tempo de Leitura**
  * `timestamp`: O tempo exato em que a leitura foi feita, medido em microssegundos.

--------------------------------

### Funções de Controle

A biblioteca expõe apenas duas funções principais para o seu uso diário:

#### 1. `void mpu_main(void)`
* **O que faz:** Função de inicialização. Ela configura os pinos I2C, acorda o MPU6050, cria as filas de comunicação, bloqueio de segurança (Mutex) e inicia as tarefas em segundo plano.
* **Como usar:** Deve ser chamada apenas uma vez, no início do seu programa (geralmente dentro da `app_main()`).

#### 2. `bool mpu_get_snapshot(mpu_snapshot_t *out)`
* **O que faz:** Função para copiar os dados mais recentes do sensor para a sua variável. Ela é segura contra falhas de concorrência (thread-safe), garantindo que você não receba um dado pela metade caso as tarefas de fundo estejam atualizando a estrutura no mesmo momento.
* **Como usar:** Passe o endereço da sua variável `mpu_snapshot_t` como parâmetro. 
* **Retorno:** Retorna `true` se a cópia foi feita com sucesso, ou `false` se houve falha ou demora na comunicação.

--------------------------------

## Exemplo Prático de Uso

```c
#include <stdio.h>
#include "mpu6050.h"

void app_main(void) {
    // 1. Inicializa a biblioteca (roda apenas uma vez)
    mpu_main();

    // 2. Cria a variável que vai receber os dados
    mpu_snapshot_t dados_do_sensor;

    while (1) {
        // 3. Captura o estado atual
        if (mpu_get_snapshot(&dados_do_sensor)) {
            // 4. Utiliza as variáveis explicadas acima
            printf("Inclinação: Pitch=%.2f, Roll=%.2f\n", dados_do_sensor.pitch, dados_do_sensor.roll);
            printf("Estado atual: %s\n", dados_do_sensor.estado);
        }
        
        vTaskDelay(100 / portTICK_PERIOD_MS); // Espera um pouco antes de ler de novo
    }
}
