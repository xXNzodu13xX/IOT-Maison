#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <stdio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/adc.h>

#include "../inc/lcd_screen_i2c.h"

#define LED_ORANGE_NODE DT_ALIAS(led_orange)
#define AFFICHEUR_NODE DT_ALIAS(afficheur_print)
#define MODE_ALARME "Mode Alarme  "
#define MODE_INTRU "Intru detecte"
#define DT_SPEC_AND_COMMA(node_id, prop, idx) \
	ADC_DT_SPEC_GET_BY_IDX(node_id, idx),

#define BUZZER_PIN DT_ALIAS(buzz)
#define BUZZER_TOGGLE_PERIOD K_MSEC(1)
#define ALARM_NODE DT_ALIAS(alar)

const struct gpio_dt_spec led_orange_gpio = GPIO_DT_SPEC_GET_OR(LED_ORANGE_NODE, gpios, {0});
const struct i2c_dt_spec afficheur = I2C_DT_SPEC_GET(AFFICHEUR_NODE);
const struct device *const dht11 = DEVICE_DT_GET_ONE(aosong_dht);
const struct gpio_dt_spec buzzer_gpio = GPIO_DT_SPEC_GET_OR(BUZZER_PIN, gpios, {0});
const struct gpio_dt_spec presence_sensor_gpio = GPIO_DT_SPEC_GET_OR(ALARM_NODE, gpios, {0});

// Lecture température et humidité
struct sensor_value temp_value;
struct sensor_value humidity_value;

static const struct adc_dt_spec adc_channels[] = {DT_FOREACH_PROP_ELEM(DT_PATH(zephyr_user), io_channels, DT_SPEC_AND_COMMA)};

static struct gpio_callback button_callback_data;
static struct gpio_callback button_callback_data2;

static bool button_pressed_flag = false;
static bool affichage_off = false;

void button_pressed()
{
	printk("Bouton 16 pressé !\n");
	button_pressed_flag = true;
}

void button_pressed2()
{
	printk("Bouton 27 pressé !\n");
	affichage_off = true;
}

int main(void)
{
	gpio_pin_configure_dt(&led_orange_gpio, GPIO_OUTPUT_HIGH);
	k_sleep(K_SECONDS(1));
	gpio_pin_configure_dt(&led_orange_gpio, GPIO_OUTPUT_LOW);
	k_sleep(K_SECONDS(1));
	gpio_pin_configure_dt(&led_orange_gpio, GPIO_OUTPUT_HIGH);
	k_sleep(K_SECONDS(1));
	gpio_pin_configure_dt(&led_orange_gpio, GPIO_OUTPUT_LOW);
	k_sleep(K_SECONDS(1));
	gpio_pin_configure_dt(&led_orange_gpio, GPIO_OUTPUT_HIGH);

	// write_lcd(&afficheur, HELLO_MSG, LCD_LINE_1);

	int err;
	uint16_t buf;
	struct adc_sequence sequence = {
		.buffer = &buf,
		/* buffer size in bytes, not number of samples */
		.buffer_size = sizeof(buf),
	};

	err = adc_channel_setup_dt(&adc_channels[0]);

	// Configurer la broche GPIO du bouton pour les interruptions
	const struct gpio_dt_spec button_gpio = GPIO_DT_SPEC_GET_OR(DT_ALIAS(sw16), gpios, {0});

	gpio_pin_configure(button_gpio.port, button_gpio.pin, GPIO_INPUT | button_gpio.dt_flags);
	gpio_init_callback(&button_callback_data, button_pressed, BIT(button_gpio.pin));
	gpio_add_callback(button_gpio.port, &button_callback_data);
	gpio_pin_interrupt_configure(button_gpio.port, button_gpio.pin, GPIO_INT_EDGE_TO_ACTIVE);

	const struct gpio_dt_spec button_gpio2 = GPIO_DT_SPEC_GET_OR(DT_ALIAS(sw27), gpios, {0});

	gpio_pin_configure(button_gpio2.port, button_gpio2.pin, GPIO_INPUT | button_gpio2.dt_flags);
	gpio_init_callback(&button_callback_data2, button_pressed2, BIT(button_gpio2.pin));
	gpio_add_callback(button_gpio2.port, &button_callback_data2);
	gpio_pin_interrupt_configure(button_gpio2.port, button_gpio2.pin, GPIO_INT_EDGE_TO_ACTIVE);

	while (1)
	{
		int ret = sensor_sample_fetch(dht11);
		ret = sensor_channel_get(dht11, SENSOR_CHAN_AMBIENT_TEMP, &temp_value);
		ret = sensor_channel_get(dht11, SENSOR_CHAN_HUMIDITY, &humidity_value);

		double temperature = sensor_value_to_double(&temp_value);
		double humidity = sensor_value_to_double(&humidity_value);

		printf("Température : %f °C\n", temperature);
		printf("Humidité : %f %%\n", humidity);

		(void)adc_sequence_init_dt(&adc_channels[0], &sequence);

		// Lecture analogique humidité
		int16_t adc_value;
		int err = adc_read_dt(&adc_channels[0], &sequence);

		if (err == 0)
		{
			uint32_t val_mv;
			val_mv = (int32_t)buf;

			err = adc_raw_to_millivolts_dt(&adc_channels[0], &val_mv);

			if (err == 0)
			{
				double voltage = val_mv / 1000.0; // Convertir en volts
				printf("Tension analogique (humidité) : %f V\n", voltage);
			}
			else
			{
				printf("Erreur lors de la conversion de tension (humidité)\n");
			}
		}
		else
		{
			printf("Erreur lors de la lecture analogique (humidité)\n");
		}

		k_sleep(K_SECONDS(10));
	}
}

void alarm_button_thread()
{
	int count = 0;

	init_lcd(&afficheur);

	while (1)
	{
		gpio_pin_configure_dt(&presence_sensor_gpio, GPIO_INPUT);

		// Lire la valeur du capteur de présence
		int presence = gpio_pin_get(presence_sensor_gpio.port, presence_sensor_gpio.pin);

		if (button_pressed_flag)
		{
			//write_lcd(&afficheur, MODE_ALARME, LCD_LINE_1);
			
			if (presence)
			{
				printk("Présence détectée\n");
				k_sleep(BUZZER_TOGGLE_PERIOD);
				gpio_pin_configure_dt(&buzzer_gpio, GPIO_OUTPUT_LOW);
				k_sleep(BUZZER_TOGGLE_PERIOD);
				// gpio_pin_configure_dt(&buzzer_gpio, GPIO_OUTPUT_HIGH);
				k_sleep(BUZZER_TOGGLE_PERIOD);

				count++;

				if (count >= 5)
				{
					write_lcd(&afficheur, MODE_INTRU, LCD_LINE_1);
				}
			}
			else
			{
				printk("Pas de présence\n");
				//write_lcd(&afficheur, HELLO_MSG, LCD_LINE_1);
				write_lcd(&afficheur, MODE_ALARME, LCD_LINE_1);
				count = 0;
			}

			//button_pressed_flag = false;
		}

		if (affichage_off)
		{
			write_lcd(&afficheur, HELLO_MSG, LCD_LINE_1);
			button_pressed_flag = false;
			affichage_off = false;
		}

		k_sleep(BUZZER_TOGGLE_PERIOD);
		//k_sleep(BUZZER_TOGGLE_PERIOD);
	}
}

/*
void buzzer_thread(void)
{
	while (1)
	{

	}
}*/

K_THREAD_DEFINE(alarm_button_id, 521, alarm_button_thread, NULL, NULL, NULL, 9, 0, 0);
// K_THREAD_DEFINE(buzzer_id, 521, buzzer_thread, NULL, NULL, NULL, 9,0,0);
