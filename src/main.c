#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <stdio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/adc.h>

#include "../inc/lcd_screen_i2c.h"

#define LED_ORANGE_NODE DT_ALIAS(led_orange)
#define AFFICHEUR_NODE DT_ALIAS(afficheur_print)

#define DT_SPEC_AND_COMMA(node_id, prop, idx) \
	ADC_DT_SPEC_GET_BY_IDX(node_id, idx),

const struct gpio_dt_spec led_orange_gpio = GPIO_DT_SPEC_GET_OR(LED_ORANGE_NODE, gpios, {0});
const struct i2c_dt_spec afficheur = I2C_DT_SPEC_GET(AFFICHEUR_NODE);
const struct device *const dht11 = DEVICE_DT_GET_ONE(aosong_dht);

// Lecture température et humidité
struct sensor_value temp_value;
struct sensor_value humidity_value;

static const struct adc_dt_spec adc_channels[] = { DT_FOREACH_PROP_ELEM(DT_PATH(zephyr_user), io_channels, DT_SPEC_AND_COMMA)};

static struct gpio_callback button_callback_data;
static struct gpio_callback button_callback_data2;

void button_pressed()
{
    printk("Bouton 16 pressé !\n");
}

void button_pressed2()
{
    printk("Bouton 27 pressé !\n");
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

	// Initialize the LCD
	init_lcd(&afficheur); // Replace with the correct LCD device specification

	// Write "COucou" to the LCD
	write_lcd(&afficheur, HELLO_MSG, LCD_LINE_1);

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
