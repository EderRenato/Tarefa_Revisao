#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/i2c.h"
#include "hardware/adc.h"
#include "inc/ssd1306.h"
#include "inc/font.h"
#include "hardware/pio.h"
#include "ws2818b.pio.h"
#include "pico/multicore.h"

//Notas para as musicas do buzzer
#define D4 293.66f
#define E4 329.63f
#define G4 392.0f
#define A4 440.0f
#define B4 493.88f
#define E5 659.26f
#define D5 587.33f
#define C_SHARP5 554.37f
#define D_SHARP4 311.13f
#define C_SHARP3 138.59f
#define F_SHARP3 184.99f
#define G_SHARP3 207.65f
#define G5 784.0f
#define F_SHARP5 739.98f
//pinos dos buzzers
#define BUZZER1 21
#define BUZZER2 10
// pinos e dados do display
#define I2C_PORT i2c1
#define DISPLAY_SDA 14
#define DISPLAY_SCL 15
#define ENDERECO 0x3C
//pinos da matriz de leds
#define LED_COUNT 25        // Número de LEDs na matriz
#define LED_PIN 13          // Pino do LED simples
#define MATRIX_PIN 7       // Pino da matriz de LEDs
bool cor = true;
// pinos do rgb
const uint RED_PIN = 13;
const uint BLUE_PIN = 12;
const uint GREEN_PIN = 11;
// botoes e joystick
const uint BUTTON_A = 5;
const uint BUTTON_B = 6;
const uint JOYSTICK_BUTTON = 22;
const uint X_AXIS = 26;
const uint Y_AXIS = 27;
// debounce e pwm
const uint DEBOUNCE_DELAY = 300;
const uint FREQUENCY = 50;   // Frequência do PWM
const uint WRAP = (1000000 / FREQUENCY);  // Período em microssegundos
// valores padrões do joystick da minha placa
int min_x = 14, max_x = 4080, center_x = 2040;
int min_y = 14, max_y = 4084, center_y = 2050;
// variaveis para controle trava e destrava do pwm
int contador = 0;

ssd1306_t ssd; // Inicialização a estrutura do display

struct pixel_t {
    uint8_t G, R, B;        // Componentes de cor: Verde, Vermelho e Azul
};

typedef struct pixel_t pixel_t; // Alias para a estrutura pixel_t
typedef pixel_t npLED_t;        // Alias para facilitar o uso no contexto de LEDs

npLED_t leds[LED_COUNT];        // Array para armazenar o estado de cada LED
PIO np_pio;                     // Variável para referenciar a instância PIO usada
uint sm;                        // Variável para armazenar o número do state machine usado

// Função para calcular o índice do LED na matriz
int getIndex(int x, int y);

// Função para inicializar o PIO para controle dos LEDs
void npInit(uint pin);

// Função para definir a cor de um LED específico
void npSetLED(const uint index, const uint8_t r, const uint8_t g, const uint8_t b);

// Função para limpar (apagar) todos os LEDs
void npClear();

// Função para atualizar os LEDs no hardware
void npWrite();

void pins_config(); // configuração dos pinos

void display_init(); // configuração do display

void init_pwm(uint gpio); // configuração do pwm

void set_pulse(uint gpio, uint16_t percentage); // envio de dados do pwm

float normalize_value(int value, int min, int max, int center); // normalizção das coordenadas capturadas

uint16_t read_adc(uint channel); // leitura do adc

int map_value(int value, int min, int max, int display_min, int display_max); // Mapeia os valores do joystick para coordenadas do display

void button_callback(uint gpio, uint32_t events); // callback dos botoes

void display_numerico(int frame);

void init_buzzer_pwm(uint gpio);

void set_buzzer_tone(uint gpio, uint freq);

void stop_buzzer(uint gpio);

void gran_vals();

void sweet_child();

void core1_entry();

int main()
{
    stdio_init_all();
    adc_init();
    i2c_init(I2C_PORT, 400e3);
    pins_config();
    
    init_pwm(RED_PIN);
    init_pwm(BLUE_PIN);
    npInit(MATRIX_PIN);
    multicore_launch_core1(core1_entry);

    gpio_set_irq_enabled_with_callback(BUTTON_A, GPIO_IRQ_EDGE_FALL, true, button_callback);
    gpio_set_irq_enabled(BUTTON_B, GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(JOYSTICK_BUTTON, GPIO_IRQ_EDGE_FALL, true);
    display_init();
    while (true) {
        display_numerico(contador);
        uint16_t vrx_value, vry_value;
        vrx_value = read_adc(1);
        vry_value = read_adc(0);

        uint16_t vrx_norm = normalize_value(vrx_value, min_x, max_x, center_x);
        uint16_t vry_norm = normalize_value(vry_value, min_y, max_y, center_y);

        int x = map_value(vrx_value, min_x, max_x, 0, 127);
        int y = map_value(vry_value, min_y, max_y, 63, 0);

        set_pulse(RED_PIN, vrx_norm);
        set_pulse(BLUE_PIN, vry_norm);

        ssd1306_fill(&ssd, !cor); // Limpa o display
        ssd1306_draw_char(&ssd, '.', x, y); // Desenha uma string   
        ssd1306_send_data(&ssd); // Atualiza o display
        sleep_ms(10);  // Delay para suavizar transições
    }

}

void pins_config(){
    // Configuração do rgb
    gpio_init(RED_PIN);
    gpio_init(BLUE_PIN);
    gpio_init(GREEN_PIN);
    gpio_set_dir(RED_PIN, GPIO_OUT);
    gpio_set_dir(BLUE_PIN, GPIO_OUT);
    gpio_set_dir(GREEN_PIN, GPIO_OUT);
    //configuração dos botões
    gpio_init(BUTTON_A);
    gpio_init(BUTTON_B);
    gpio_init(JOYSTICK_BUTTON);
    gpio_set_dir(BUTTON_A, GPIO_IN);
    gpio_set_dir(BUTTON_B, GPIO_IN);
    gpio_set_dir(JOYSTICK_BUTTON, GPIO_IN);
    gpio_pull_up(BUTTON_A);
    gpio_pull_up(BUTTON_B);
    gpio_pull_up(JOYSTICK_BUTTON);
    //configuração dos eixos
    adc_gpio_init(X_AXIS);
    adc_gpio_init(Y_AXIS);
    // configuração do display
    gpio_set_function(DISPLAY_SDA, GPIO_FUNC_I2C);
    gpio_set_function(DISPLAY_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(DISPLAY_SDA);
    gpio_pull_up(DISPLAY_SCL);
    init_buzzer_pwm(BUZZER1);
    init_buzzer_pwm(BUZZER2);
}

void display_init(){
    ssd1306_init(&ssd, WIDTH, HEIGHT, false, ENDERECO, I2C_PORT); // Inicialização do display
    ssd1306_config(&ssd); // Configura display
    ssd1306_send_data(&ssd); // envia dados para o display

    ssd1306_fill(&ssd, false); // limpa o display
    ssd1306_send_data(&ssd);
 }

void button_callback(uint gpio, uint32_t events) {
    uint32_t last_time_button_a = 0;
    uint32_t last_time_button_b = 0;
    uint32_t last_time_joystick = 0;
    uint32_t current_time = to_ms_since_boot(get_absolute_time());

    if (gpio == BUTTON_A) {  // Trata o Botão A
        if (current_time - last_time_button_a > DEBOUNCE_DELAY) {
            contador++;
            if (contador>9){
                contador=0;
            }
            last_time_button_a = current_time;
        }
    }
    if (gpio == BUTTON_B){
        if (current_time - last_time_button_b > DEBOUNCE_DELAY){
            contador--;
            if (contador<0){
                contador=9;
            }
            last_time_button_b = current_time;
        }
    }
    else if (gpio == JOYSTICK_BUTTON) {  // Trata o Botão do Joystick
        if (current_time - last_time_joystick > DEBOUNCE_DELAY) {
            bool GREEN_STATE = gpio_get(GREEN_PIN);
            GREEN_STATE = !GREEN_STATE;
            gpio_put(GREEN_PIN, GREEN_STATE);
            last_time_joystick = current_time;
        }
    }
}

void init_pwm(uint gpio) {
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(gpio);
    pwm_set_wrap(slice_num, WRAP);
    pwm_set_clkdiv(slice_num, 125.0f); // Divisor de clock para 1 MHz
    pwm_set_enabled(slice_num, true);
}

void set_pulse(uint gpio, uint16_t percentage) {
    uint slice_num = pwm_gpio_to_slice_num(gpio);
    uint16_t level = (uint16_t)((WRAP * percentage) / 100);  // Converte 0-100% para 0-WRAP
    pwm_set_gpio_level(gpio, level);
}

float normalize_value(int value, int min, int max, int center) {
    if (value < center) {
        return ((float)(center - value) / (center - min)) * 100;  // Mapeia de 0 a 100
    } else {
        return ((float)(value - center) / (max - center)) * 100;  // Mapeia de 0 a 100
    }
}
uint16_t read_adc(uint channel) {
    adc_select_input(channel);
    uint32_t sum = 0;
    const int samples = 10;  // Coleta 10 amostras e faz a média
    for (int i = 0; i < samples; i++) {
        sum += adc_read();
        sleep_us(500);  // Pequeno atraso para suavizar leituras
    }
    return sum / samples;
}

int map_value(int value, int min, int max, int display_min, int display_max) {
    return display_min + (value - min) * (display_max - display_min) / (max - min);
}

void display_numerico(int frame) {
    int matriz[10][5][5][3] = {
                {
                    {{0, 0, 0}, {193, 192, 191}, {193, 192, 191}, {193, 192, 191}, {0, 0, 0}},
                    {{0, 0, 0}, {193, 192, 191}, {0, 0, 0}, {193, 192, 191}, {0, 0, 0}},
                    {{0, 0, 0}, {193, 192, 191}, {0, 0, 0}, {193, 192, 191}, {0, 0, 0}},
                    {{0, 0, 0}, {193, 192, 191}, {0, 0, 0}, {193, 192, 191}, {0, 0, 0}},
                    {{0, 0, 0}, {193, 192, 191}, {193, 192, 191}, {193, 192, 191}, {0, 0, 0}}
                },
                {
                    {{0, 0, 0}, {0, 101, 13}, {0, 101, 13}, {0, 0, 0}, {0, 0, 0}},
                    {{0, 0, 0}, {0, 0, 0}, {0, 101, 13}, {0, 0, 0}, {0, 0, 0}},
                    {{0, 0, 0}, {0, 0, 0}, {0, 101, 13}, {0, 0, 0}, {0, 0, 0}},
                    {{0, 0, 0}, {0, 0, 0}, {0, 101, 13}, {0, 0, 0}, {0, 0, 0}},
                    {{0, 0, 0}, {0, 101, 13}, {0, 101, 13}, {0, 101, 13}, {0, 0, 0}}
                },
                {
                    {{0, 0, 0}, {101, 0, 0}, {101, 0, 0}, {101, 0, 0}, {0, 0, 0}},
                    {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {101, 0, 0}, {0, 0, 0}},
                    {{0, 0, 0}, {101, 0, 0}, {101, 0, 0}, {101, 0, 0}, {0, 0, 0}},
                    {{0, 0, 0}, {101, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}},
                    {{0, 0, 0}, {101, 0, 0}, {101, 0, 0}, {101, 0, 0}, {0, 0, 0}}
                },
                {
                    {{0, 0, 0}, {0, 16, 101}, {0, 16, 101}, {0, 16, 101}, {0, 0, 0}},
                    {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 16, 101}, {0, 0, 0}},
                    {{0, 0, 0}, {0, 16, 101}, {0, 16, 101}, {0, 16, 101}, {0, 0, 0}},
                    {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 16, 101}, {0, 0, 0}},
                    {{0, 0, 0}, {0, 16, 101}, {0, 16, 101}, {0, 16, 101}, {0, 0, 0}}
                },
                {
                    {{0, 0, 0}, {97, 0, 90}, {0, 0, 0}, {97, 0, 90}, {0, 0, 0}},
                    {{0, 0, 0}, {97, 0, 90}, {0, 0, 0}, {97, 0, 90}, {0, 0, 0}},
                    {{0, 0, 0}, {97, 0, 90}, {97, 0, 90}, {97, 0, 90}, {0, 0, 0}},
                    {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {97, 0, 90}, {0, 0, 0}},
                    {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {97, 0, 90}, {0, 0, 0}}
                },
                {
                    {{0, 0, 0}, {0, 92, 97}, {0, 92, 97}, {0, 92, 97}, {0, 0, 0}},
                    {{0, 0, 0}, {0, 92, 97}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}},
                    {{0, 0, 0}, {0, 92, 97}, {0, 92, 97}, {0, 92, 97}, {0, 0, 0}},
                    {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 92, 97}, {0, 0, 0}},
                    {{0, 0, 0}, {0, 92, 97}, {0, 92, 97}, {0, 92, 97}, {0, 0, 0}}
                },
                {
                    {{0, 0, 0}, {101, 0, 80}, {101, 0, 80}, {101, 0, 80}, {0, 0, 0}},
                    {{0, 0, 0}, {101, 0, 80}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}},
                    {{0, 0, 0}, {101, 0, 80}, {101, 0, 80}, {101, 0, 80}, {0, 0, 0}},
                    {{0, 0, 0}, {101, 0, 80}, {0, 0, 0}, {101, 0, 80}, {0, 0, 0}},
                    {{0, 0, 0}, {101, 0, 80}, {101, 0, 80}, {101, 0, 80}, {0, 0, 0}}
                },
                {
                    {{0, 0, 0}, {248, 255, 0}, {248, 255, 0}, {248, 255, 0}, {0, 0, 0}},
                    {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {248, 255, 0}, {0, 0, 0}},
                    {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {248, 255, 0}, {0, 0, 0}},
                    {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {248, 255, 0}, {0, 0, 0}},
                    {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {248, 255, 0}, {0, 0, 0}}
                },
                {
                    {{0, 0, 0}, {34, 255, 0}, {34, 255, 0}, {34, 255, 0}, {0, 0, 0}},
                    {{0, 0, 0}, {34, 255, 0}, {0, 0, 0}, {34, 255, 0}, {0, 0, 0}},
                    {{0, 0, 0}, {34, 255, 0}, {34, 255, 0}, {34, 255, 0}, {0, 0, 0}},
                    {{0, 0, 0}, {34, 255, 0}, {0, 0, 0}, {34, 255, 0}, {0, 0, 0}},
                    {{0, 0, 0}, {34, 255, 0}, {34, 255, 0}, {34, 255, 0}, {0, 0, 0}}
                },
                {
                    {{0, 0, 0}, {255, 162, 0}, {255, 162, 0}, {255, 162, 0}, {0, 0, 0}},
                    {{0, 0, 0}, {255, 162, 0}, {0, 0, 0}, {255, 162, 0}, {0, 0, 0}},
                    {{0, 0, 0}, {255, 162, 0}, {255, 162, 0}, {255, 162, 0}, {0, 0, 0}},
                    {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {255, 162, 0}, {0, 0, 0}},
                    {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {255, 162, 0}, {0, 0, 0}}
                }};
    
    // Desenhar o número na matriz de LEDs
    for (int linha = 0; linha < 5; linha++) {
        for (int coluna = 0; coluna < 5; coluna++) {
            int posicao = getIndex(linha, coluna);
            npSetLED(posicao, matriz[frame][coluna][linha][0], matriz[frame][coluna][linha][1], matriz[frame][coluna][linha][2]);
        }
    }
    npWrite(); // Atualizar LEDs no hardware
}

int getIndex(int x, int y) {
    x = 4 - x; // Inverte as colunas (0 -> 4, 1 -> 3, etc.)
    y = 4 - y; // Inverte as linhas (0 -> 4, 1 -> 3, etc.)
    if (y % 2 == 0) {
        return y * 5 + x;       // Linha par (esquerda para direita)
    } else {
        return y * 5 + (4 - x); // Linha ímpar (direita para esquerda)
    }
}

void npInit(uint pin) {
    uint offset = pio_add_program(pio0, &ws2818b_program); // Carregar o programa PIO
    np_pio = pio0;                                         // Usar o primeiro bloco PIO

    sm = pio_claim_unused_sm(np_pio, false);              // Tentar usar uma state machine do pio0
    if (sm < 0) {                                         // Se não houver disponível no pio0
        np_pio = pio1;                                    // Mudar para o pio1
        sm = pio_claim_unused_sm(np_pio, true);           // Usar uma state machine do pio1
    }

    ws2818b_program_init(np_pio, sm, offset, pin, 800000.f); // Inicializar state machine para LEDs

    for (uint i = 0; i < LED_COUNT; ++i) {                // Inicializar todos os LEDs como apagados
        leds[i].R = 0;
        leds[i].G = 0;
        leds[i].B = 0;
    }
}

void npSetLED(const uint index, const uint8_t r, const uint8_t g, const uint8_t b) {
    leds[index].R = r;                                    // Definir componente vermelho
    leds[index].G = g;                                    // Definir componente verde
    leds[index].B = b;                                    // Definir componente azul
}

void npClear() {
    for (uint i = 0; i < LED_COUNT; ++i) {                // Iterar sobre todos os LEDs
        npSetLED(i, 0, 0, 0);                             // Definir cor como preta (apagado)
    }
    npWrite();                                            // Atualizar LEDs no hardware
}

void npWrite() {
    for (uint i = 0; i < LED_COUNT; ++i) {                // Iterar sobre todos os LEDs
        pio_sm_put_blocking(np_pio, sm, leds[i].G);       // Enviar componente verde
        pio_sm_put_blocking(np_pio, sm, leds[i].R);       // Enviar componente vermelho
        pio_sm_put_blocking(np_pio, sm, leds[i].B);       // Enviar componente azul
    }
}

void init_buzzer_pwm(uint gpio) {
    gpio_set_function(gpio, GPIO_FUNC_PWM); // Configura o GPIO como PWM
    uint slice_num = pwm_gpio_to_slice_num(gpio);
    pwm_set_clkdiv(slice_num, 125.0f);     // Define o divisor do clock para 1 MHz
    pwm_set_wrap(slice_num, 1000);        // Define o TOP para frequência de 1 kHz
    pwm_set_chan_level(slice_num, pwm_gpio_to_channel(gpio), 0); // Razão cíclica inicial
    pwm_set_enabled(slice_num, true);     // Habilita o PWM
}

void set_buzzer_tone(uint gpio, uint freq) {
    uint slice_num = pwm_gpio_to_slice_num(gpio);
    uint top = 1000000 / freq;            // Calcula o TOP para a frequência desejada
    pwm_set_wrap(slice_num, top);
    pwm_set_chan_level(slice_num, pwm_gpio_to_channel(gpio), top / 2); // 50% duty cycle
}

void stop_buzzer(uint gpio) {
    uint slice_num = pwm_gpio_to_slice_num(gpio);
    pwm_set_chan_level(slice_num, pwm_gpio_to_channel(gpio), 0); // Desliga o PWM
}

void gran_vals(){
    float notas[13] = {E5, D5, F_SHARP3, G_SHARP3, C_SHARP5, B4, D4, E4, B4, A4, C_SHARP3, E4, A4};
    float intervalo[13] = {125, 125, 250, 250, 125, 125, 250, 250, 125, 125, 250, 250, 500};
    for (int i = 0; i < 13; i++){
        set_buzzer_tone(BUZZER1, notas[i]);
        sleep_ms(intervalo[i]);
    }
    stop_buzzer(BUZZER1);
}

void sweet_child(){
    float notas[48] = {D4, D5, A4, G4, G5, A4, F_SHARP5, A4,
                       D4, D5, A4, G4, G5, A4, F_SHARP5, A4,
                       E4, D5, A4, G4, G5, A4, F_SHARP5, A4,
                       E4, D5, A4, G4, G5, A4, F_SHARP5, A4,
                       G4, D5, A4, G4, G5, A4, F_SHARP5, A4,
                       G4, D5, A4, G4, G5, A4, F_SHARP5, A4};
    float brk[13] = {D5, A4, E5, A4, F_SHARP5, A4,
                       G5, A4, F_SHARP5, A4, E5, A4, D5};
    float intervalo = 250;
    for (int j = 0; j<1; j++){
        for (int i = 0; i<48; i++){
            set_buzzer_tone(BUZZER1, notas[i]);
            sleep_ms(intervalo);
        }
    }
    for (int i = 0; i<13; i++){
            set_buzzer_tone(BUZZER1, brk[i]);
            sleep_ms(intervalo);
    }  
    stop_buzzer(BUZZER1);
}

void core1_entry(){
    // Inicializar o PIO para controle dos LEDs
   while (true){
       char buffer[128];
       scanf("%1024s", buffer);
       if (!strcmp(buffer, "buzzer_a")){
           sweet_child();
       }
       if (!strcmp(buffer, "buzzer_b")){
           gran_vals();
       }
       sleep_ms(100);
   }
}
