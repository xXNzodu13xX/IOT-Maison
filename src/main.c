#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <stdio.h>
#include <zephyr/drivers/i2c.h>

#include <zephyr/drivers/sensor.h>

#include "../inc/lcd_screen_i2c.h"

#define LED_ORANGE_NODE DT_ALIAS(led_orange)
#define AFFICHEUR_NODE DT_ALIAS(afficheur_print)

const struct gpio_dt_spec led_orange_gpio = GPIO_DT_SPEC_GET_OR(LED_ORANGE_NODE, gpios, {0});
const struct i2c_dt_spec afficheur = I2C_DT_SPEC_GET(AFFICHEUR_NODE);
const struct device *const dht11 = DEVICE_DT_GET_ONE(aosong_dht);

struct sensor_value temp_value;
struct sensor_value humidity_value;

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

	while (1)
	{
		int ret = sensor_sample_fetch(dht11);
		if (ret)
		{
			printk("Erreur collecte des échantillons du capteur\n");
		}

		ret = sensor_channel_get(dht11, SENSOR_CHAN_AMBIENT_TEMP, &temp_value);
		if (ret)
		{
			printk("Erreur récupération de la température\n");
		}

		ret = sensor_channel_get(dht11, SENSOR_CHAN_HUMIDITY, &humidity_value);
		if (ret)
		{
			printk("Erreur récupération de l'humidité\n");
		}

		double temperature = sensor_value_to_double(&temp_value);
		double humidity = sensor_value_to_double(&humidity_value);

		printf("Température : %f °C\n", temperature);
		printf("Humidité : %f %%\n", humidity);

		k_sleep(K_SECONDS(5));
	}
}
