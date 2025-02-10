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
#include "hardware/clocks.h"

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

// Configuração do pino do buzzer
#define BUZZER_PIN 21

const uint LED = 12;            // Pino do LED conectado
const uint16_t PERIOD_LED_0 = 2000;   // Período do PWM (valor máximo do contador)
const float DIVIDER_PWM_LED_0 = 16.0; // Divisor fracional do clock para o PWM
const uint16_t LED_STEP = 100;  // Passo de incremento/decremento para o duty cycle do LED
uint16_t led_level = 100;       // Nível inicial do PWM (duty cycle)

void setup_pwm()
{
    uint slice;
    gpio_set_function(LED, GPIO_FUNC_PWM); // Configura o pino do LED para função PWM
    slice = pwm_gpio_to_slice_num(LED);    // Obtém o slice do PWM associado ao pino do LED
    pwm_set_clkdiv(slice, DIVIDER_PWM_LED_0);    // Define o divisor de clock do PWM
    pwm_set_wrap(slice, PERIOD_LED_0);           // Configura o valor máximo do contador (período do PWM)
    pwm_set_gpio_level(LED, led_level);    // Define o nível inicial do PWM para o pino do LED
    pwm_set_enabled(slice, true);          // Habilita o PWM no slice correspondente
}

// Notas musicais para a música tema de Star Wars
const uint star_wars_notes[] = {
    330, 330, 330, 262, 392, 523, 330, 262,
    392, 523, 330, 659, 659, 659, 698, 523,
    415, 349, 330, 262, 392, 523, 330, 262,
    392, 523, 330, 659, 659, 659, 698, 523,
    415, 349, 330, 523, 494, 440, 392, 330,
    659, 784, 659, 523, 494, 440, 392, 330,
    659, 659, 330, 784, 880, 698, 784, 659,
    523, 494, 440, 392, 659, 784, 659, 523,
    494, 440, 392, 330, 659, 523, 659, 262,
    330, 294, 247, 262, 220, 262, 330, 262,
    330, 294, 247, 262, 330, 392, 523, 440,
    349, 330, 659, 784, 659, 523, 494, 440,
    392, 659, 784, 659, 523, 494, 440, 392
};

// Duração das notas em milissegundos
const uint note_duration[] = {
    500, 500, 500, 350, 150, 300, 500, 350,
    150, 300, 500, 500, 500, 500, 350, 150,
    300, 500, 500, 350, 150, 300, 500, 350,
    150, 300, 650, 500, 150, 300, 500, 350,
    150, 300, 500, 150, 300, 500, 350, 150,
    300, 650, 500, 350, 150, 300, 500, 350,
    150, 300, 500, 500, 500, 500, 350, 150,
    300, 500, 500, 350, 150, 300, 500, 350,
    150, 300, 500, 350, 150, 300, 500, 500,
    350, 150, 300, 500, 500, 350, 150, 300,
};

// Inicializa o PWM no pino do buzzer
void pwm_init_buzzer(uint pin) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(pin);
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 4.0f); // Ajusta divisor de clock
    pwm_init(slice_num, &config, true);
    pwm_set_gpio_level(pin, 0); // Desliga o PWM inicialmente
}

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
}

void updateMenu(uint8_t *ssd, struct render_area *frame_area, int item){
  // Parte do código para exibir a mensagem no display (opcional: mudar ssd1306_height para 32 em ssd1306_i2c.h)
  char *text[3];
  
  if(item == 1) {
    text[0] = "X  Programa 1";
    text[1] = "   Programa 2";
    text[2] = "   Programa 3";
  }
  if(item == 2) {
    text[0] = "   Programa 1";
    text[1] = "X  Programa 2";
    text[2] = "   Programa 3";
  }
  if(item == 3) {
    text[0] = "   Programa 1";
    text[1] = "   Programa 2";
    text[2] = "X  Programa 3";
  }

  int y = 0;
  for (uint i = 0; i < count_of(text); i++)
  {
      ssd1306_draw_string(ssd, 5, y, text[i]);
      y += 16;
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


// Toca uma nota com a frequência e duração especificadas
void play_tone(uint pin, uint frequency, uint duration_ms) {
    uint slice_num = pwm_gpio_to_slice_num(pin);
    uint32_t clock_freq = clock_get_hz(clk_sys);
    uint32_t top = clock_freq / frequency - 1;

    pwm_set_wrap(slice_num, top);
    pwm_set_gpio_level(pin, top / 2); // 50% de duty cycle

    sleep_ms(duration_ms);

    pwm_set_gpio_level(pin, 0); // Desliga o som após a duração
    sleep_ms(50); // Pausa entre notas
}

// Função principal para tocar a música
int play_star_wars(uint pin) {

  for (int i = 0; i < sizeof(star_wars_notes) / sizeof(star_wars_notes[0]); i++) {
    if (gpio_get(SW) == 0) {
     sleep_ms(100);
     return 0;
    }

    if (star_wars_notes[i] == 0) {
        sleep_ms(note_duration[i]);
    } else {
        play_tone(pin, star_wars_notes[i], note_duration[i]);
    }
  }
  return 1;
}

// Main's dos outros códigos
int main_pwm_led()
{
  uint up_down = 1; // Variável para controlar se o nível do LED aumenta ou diminui

  setup_pwm();      // Configura o PWM
  while (true)
  {
    pwm_set_gpio_level(LED, led_level); // Define o nível atual do PWM (duty cycle)

    // Divide o atraso de 1 segundo em 10 intervalos de 100ms
    for (int i = 0; i < 10; i++) {
        sleep_ms(100); // Atraso de 100ms

        if (gpio_get(SW) == 0) {
            pwm_set_gpio_level(LED, 0); // Desliga o LED antes de sair
            sleep_ms(100);
            break; // Sai do loop e encerra o programa
        }
    }

    if (gpio_get(SW) == 0) {
        // Se o botão foi pressionado durante o loop de 1 segundo, sair do loop principal
        break;
    }

    if (up_down)
    {
        led_level += LED_STEP; // Incrementa o nível do LED
        if (led_level >= PERIOD)
            up_down = 0; // Muda direção para diminuir quando atingir o período máximo
    }
    else
    {
        led_level -= LED_STEP; // Decrementa o nível do LED
        if (led_level <= LED_STEP)
            up_down = 1; // Muda direção para aumentar quando atingir o mínimo
    }
  }
}

int main_buzzer_pwm() {
  // uint16_t sw_value;
  // stdio_init_all();
  pwm_init_buzzer(BUZZER_PIN);
  int control = 1;

  while(control){
    // *sw_value = gpio_get(SW);
    control = play_star_wars(BUZZER_PIN);
  }
  
  return 0;
}

void main_joystick_led() {
  setup_pwm_led(LED_B, &slice_led_b, led_b_level); // Configura o PWM para o LED azul
  setup_pwm_led(LED_R, &slice_led_r, led_r_level); // Configura o PWM para o LED vermelho

  uint16_t vrx_value, vry_value, sw_value; // Variáveis para armazenar os valores do joystick (eixos X e Y) e botão
  // Loop principal
  while (1)
  {
    joystick_read_axis(&vry_value, &vrx_value, &sw_value); // Lê os valores dos eixos do joystick
    // Ajusta os níveis PWM dos LEDs de acordo com os valores do joystick
    pwm_set_gpio_level(LED_B, vrx_value); // Ajusta o brilho do LED azul com o valor do eixo X
    pwm_set_gpio_level(LED_R, vry_value); // Ajusta o brilho do LED vermelho com o valor do eixo Y

    if (gpio_get(SW) == 0) {
      sleep_ms(200);
      setup_pwm_led(LED_B, &slice_led_b, 0);//desliga os leds antes de sair 
      setup_pwm_led(LED_R, &slice_led_r, 0); 
      break;
    }
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
    
    if (vry_value < 1000) { //descendo
      item_selecionado = (item_selecionado % 3) + 1;
      updateMenu(ssd, &frame_area, item_selecionado);
    }

    if (vry_value > 4000) { //subindo
      item_selecionado = (item_selecionado == 1) ? 3 : item_selecionado - 1;
      updateMenu(ssd, &frame_area, item_selecionado);
    }

    if (sw_value == 0) {
      if (item_selecionado == 1) {
        LimparDisplay(ssd, &frame_area);
        sleep_ms(200);
        main_joystick_led();
        updateMenu(ssd, &frame_area, 1);
        item_proximo, item_selecionado = 1;
      }
      if (item_selecionado == 2) {
        LimparDisplay(ssd, &frame_area);
        sleep_ms(200);
        main_buzzer_pwm();
        updateMenu(ssd, &frame_area, 1);
        item_proximo, item_selecionado = 1;
      }
      if (item_selecionado == 3) {
        LimparDisplay(ssd, &frame_area);
        sleep_ms(200);
        main_pwm_led();
        updateMenu(ssd, &frame_area, 1);
        item_proximo, item_selecionado = 1;
      }
    }

    // Pequeno delay antes da próxima leitura
    sleep_ms(150); // Espera 100 ms antes de repetir o ciclo
  }
}
