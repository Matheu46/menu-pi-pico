#include <stdio.h>        // Biblioteca padrão de entrada e saída
#include "hardware/adc.h" // Biblioteca para manipulação do ADC no RP2040
#include "hardware/pwm.h" // Biblioteca para controle de PWM no RP2040
#include "pico/stdlib.h"  // Biblioteca padrão do Raspberry Pi Pico
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include "pico/binary_info.h"
#include "inc/ssd1306.h"
#include "hardware/i2c.h"

const uint I2C_SDA = 14;
const uint I2C_SCL = 15;

// Definição dos pinos usados para o joystick e LEDs
const int VRY = 26;          // Pino de leitura do eixo Y do joystick (conectado ao ADC)
const int VRX = 27;          // Pino de leitura do eixo X do joystick (conectado ao ADC)
const int ADC_CHANNEL_0 = 1; // Canal ADC para o eixo X do joystick
const int ADC_CHANNEL_1 = 0; // Canal ADC para o eixo Y do joystick
const int SW = 22;           // Pino de leitura do botão do joystick

const int LED_B = 13;                    // Pino para controle do LED azul via PWM
const int LED_R = 11;                    // Pino para controle do LED vermelho via PWM
const float DIVIDER_PWM = 16.0;          // Divisor fracional do clock para o PWM
const uint16_t PERIOD = 4096;            // Período do PWM (valor máximo do contador)
uint16_t led_b_level, led_r_level = 100; // Inicialização dos níveis de PWM para os LEDs
uint slice_led_b, slice_led_r;           // Variáveis para armazenar os slices de PWM correspondentes aos LEDs

// Função para configurar o joystick (pinos de leitura e ADC)
void setup_joystick()
{
  // Inicializa o ADC e os pinos de entrada analógica
  adc_init();         // Inicializa o módulo ADC
  adc_gpio_init(VRX); // Configura o pino VRX (eixo X) para entrada ADC
  adc_gpio_init(VRY); // Configura o pino VRY (eixo Y) para entrada ADC

  // Inicializa o pino do botão do joystick
  gpio_init(SW);             // Inicializa o pino do botão
  gpio_set_dir(SW, GPIO_IN); // Configura o pino do botão como entrada
  gpio_pull_up(SW);          // Ativa o pull-up no pino do botão para evitar flutuações
}

// Função para configurar o PWM de um LED (genérica para azul e vermelho)
void setup_pwm_led(uint led, uint *slice, uint16_t level)
{
  gpio_set_function(led, GPIO_FUNC_PWM); // Configura o pino do LED como saída PWM
  *slice = pwm_gpio_to_slice_num(led);   // Obtém o slice do PWM associado ao pino do LED
  pwm_set_clkdiv(*slice, DIVIDER_PWM);   // Define o divisor de clock do PWM
  pwm_set_wrap(*slice, PERIOD);          // Configura o valor máximo do contador (período do PWM)
  pwm_set_gpio_level(led, level);        // Define o nível inicial do PWM para o LED
  pwm_set_enabled(*slice, true);         // Habilita o PWM no slice correspondente ao LED
}

// Função de configuração geral
void setup()
{
  stdio_init_all();                                // Inicializa a porta serial para saída de dados
  setup_joystick();                                // Chama a função de configuração do joystick
  setup_pwm_led(LED_B, &slice_led_b, led_b_level); // Configura o PWM para o LED azul
  setup_pwm_led(LED_R, &slice_led_r, led_r_level); // Configura o PWM para o LED vermelho
}

void updateMenu(uint8_t *ssd, struct render_area *frame_area, int item){
  // Parte do código para exibir a mensagem no display (opcional: mudar ssd1306_height para 32 em ssd1306_i2c.h)
  char *text[3];
  
  if(item == 1) {
    text[0] = "X  Item 1";
    text[1] = "   Item 2";
    text[2] = "   Item 3";
  }
  if(item == 2) {
    text[0] = "   Item 1";
    text[1] = "X  Item 2";
    text[2] = "   Item 3";
  }
  if(item == 3) {
    text[0] = "   Item 1";
    text[1] = "   Item 2";
    text[2] = "X  Item 3";
  }

  int y = 0;
  for (uint i = 0; i < count_of(text); i++)
  {
      ssd1306_draw_string(ssd, 5, y, text[i]);
      y += 8;
  }
  render_on_display(ssd, frame_area);
}

void LimparDisplay(uint8_t *ssd, struct render_area *frame_area) {
  memset(ssd, 0, ssd1306_buffer_length);
  render_on_display(ssd, frame_area);
}

// Função para ler os valores dos eixos do joystick (X e Y)
void joystick_read_axis(uint16_t *vry_value,  uint16_t *vrx_value, uint16_t *sw_value) {

  // Leitura do valor do eixo X do joystick
  adc_select_input(ADC_CHANNEL_0); // Seleciona o canal ADC para o eixo X
  sleep_us(2);                     // Pequeno delay para estabilidade
  *vrx_value = adc_read();         // Lê o valor do eixo X (0-4095)

  // Leitura do valor do eixo Y do joystick
  adc_select_input(ADC_CHANNEL_1); // Seleciona o canal ADC para o eixo Y
  sleep_us(2);                     // Pequeno delay para estabilidade
  *vry_value = adc_read();         // Lê o valor do eixo Y (0-4095)

  *sw_value = gpio_get(SW);
}

void joystick_led_main() {
  uint16_t vrx_value, vry_value, sw_value; // Variáveis para armazenar os valores do joystick (eixos X e Y) e botão
  setup();                                 // Chama a função de configuração
  printf("Joystick-PWM\n");                // Exibe uma mensagem inicial via porta serial
  // Loop principal
  while (1)
  {
    joystick_read_axis(&vry_value, &vrx_value, &sw_value); // Lê os valores dos eixos do joystick
    // Ajusta os níveis PWM dos LEDs de acordo com os valores do joystick
    pwm_set_gpio_level(LED_B, vrx_value); // Ajusta o brilho do LED azul com o valor do eixo X
    pwm_set_gpio_level(LED_R, vry_value); // Ajusta o brilho do LED vermelho com o valor do eixo Y

    // Pequeno delay antes da próxima leitura
    sleep_ms(100); // Espera 100 ms antes de repetir o ciclo
  }
}

// Função principal
int main() {
  uint16_t vrx_value, vry_value, sw_value; // Variáveis para armazenar os valores do joystick (eixos X e Y) e botão
  setup();                                 // Chama a função de configuração
  
  i2c_init(i2c1, ssd1306_i2c_clock * 1000);
  gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
  gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
  gpio_pull_up(I2C_SDA);
  gpio_pull_up(I2C_SCL);

  // Processo de inicialização completo do OLED SSD1306
  ssd1306_init();

  // Preparar área de renderização para o display (ssd1306_width pixels por ssd1306_n_pages páginas)
  struct render_area frame_area = {
      start_column : 0,
      end_column : ssd1306_width - 1,
      start_page : 0,
      end_page : ssd1306_n_pages - 1
  };

  calculate_render_area_buffer_length(&frame_area);

  uint8_t ssd[ssd1306_buffer_length];
  LimparDisplay(ssd, &frame_area); 
  
  int item_proximo = 1;
  int item_selecionado = 1;
  updateMenu(ssd, &frame_area, item_proximo);

  while (1) {
    joystick_read_axis(&vry_value, &vrx_value, &sw_value); // Lê os valores dos eixos do joystick
    // pwm_set_gpio_level(LED_R, vry_value); // Ajusta o brilho do LED vermelho com o valor do eixo Y
    
    if (vry_value < 1000) { //descendo
      if (item_proximo == 1) {
        updateMenu(ssd, &frame_area, item_proximo);
        item_selecionado = item_proximo;
        item_proximo = 2;

      } else if (item_proximo == 2) {
        updateMenu(ssd, &frame_area, item_proximo);
        item_selecionado = item_proximo;
        item_proximo = 3;

      } else if (item_proximo == 3) {
        updateMenu(ssd, &frame_area, item_proximo);
        item_selecionado = item_proximo;
        item_proximo = 1;
      }
    }

    if (vry_value > 3000) { //subindo
      if (item_proximo == 1) {
        updateMenu(ssd, &frame_area, item_proximo);
        item_selecionado = item_proximo;
        item_proximo = 3;

      } else if (item_proximo == 2) {
        updateMenu(ssd, &frame_area, item_proximo);
        item_selecionado = item_proximo;
        item_proximo = 1;

      } else if (item_proximo == 3) {
        updateMenu(ssd, &frame_area, item_proximo);
        item_selecionado = item_proximo;
        item_proximo = 2;
      }
    }

    if (sw_value == 0) {
      if (item_selecionado == 1) {
        LimparDisplay(ssd, &frame_area);
        joystick_led_main();
        // rodar projeto
      }
      if (item_selecionado == 2) {
        LimparDisplay(ssd, &frame_area);
        // rodar projeto
      }
      if (item_selecionado == 3) {
        LimparDisplay(ssd, &frame_area);
        // rodar projeto
      }
    }

    // Pequeno delay antes da próxima leitura
    sleep_ms(150); // Espera 100 ms antes de repetir o ciclo
  }
}
