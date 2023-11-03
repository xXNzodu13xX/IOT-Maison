#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <stdio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/adc.h>

#include <zephyr/net/net_if.h>
#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/net/net_event.h>
#include <zephyr/net/http/client.h>
#include <errno.h>
#include <zephyr/net/socket.h>

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

#define SSID "iPhone de Tom"
#define PSK "homedetom"

#define HTTP_PORT "5000"
#define HTTP_HOST "172.20.10.2"

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

static K_SEM_DEFINE(wifi_connected, 0, 1);
static K_SEM_DEFINE(ipv4_address_obtained, 0, 1);

static struct net_mgmt_event_callback wifi_cb;
static struct net_mgmt_event_callback ipv4_cb;

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

static void handle_wifi_connect_result(struct net_mgmt_event_callback *cb)
{
	const struct wifi_status *status = (const struct wifi_status *)cb->info;
	if (status->status)
	{
		printk("Connection request failed (%d)\n", status->status);
	}
	else
	{
		printk("Connected\n");
		k_sem_give(&wifi_connected);
	}
}

static void handle_wifi_disconnect_result(struct net_mgmt_event_callback *cb)
{
	const struct wifi_status *status = (const struct wifi_status *)cb->info;
	if (status->status)
	{
		printk("Disconnection request (%d)\n", status->status);
	}
	else
	{
		printk("Disconnected\n");
		k_sem_take(&wifi_connected, K_NO_WAIT);
	}
}

static void handle_ipv4_result(struct net_if *iface)
{
	int i = 0;
	for (i = 0; i < NET_IF_MAX_IPV4_ADDR; i++)
	{
		char buf[NET_IPV4_ADDR_LEN];
		if (iface->config.ip.ipv4->unicast[i].addr_type != NET_ADDR_DHCP)
		{
			continue;
		}
		printk("IPv4 address: %s\n",
			   net_addr_ntop(AF_INET,
							 &iface->config.ip.ipv4->unicast[i].address.in_addr,
							 buf, sizeof(buf)));
		printk("Subnet: %s\n", net_addr_ntop(AF_INET,
											 &iface->config.ip.ipv4->netmask,
											 buf, sizeof(buf)));
		printk("Router: %s\n",
			   net_addr_ntop(AF_INET,
							 &iface->config.ip.ipv4->gw,
							 buf, sizeof(buf)));
	}
	k_sem_give(&ipv4_address_obtained);
}

static void wifi_mgmt_event_handler(struct net_mgmt_event_callback *cb, uint32_t mgmt_event, struct net_if *iface)
{
	switch (mgmt_event)
	{
	case NET_EVENT_WIFI_CONNECT_RESULT:
		handle_wifi_connect_result(cb);
		break;
	case NET_EVENT_WIFI_DISCONNECT_RESULT:
		handle_wifi_disconnect_result(cb);
		break;
	case NET_EVENT_IPV4_ADDR_ADD:
		handle_ipv4_result(iface);
		break;
	default:
		break;
	}
}

void wifi_connect(void)
{
	struct net_if *iface = net_if_get_default();
	struct wifi_connect_req_params wifi_params = {0};
	wifi_params.ssid = SSID;
	wifi_params.psk = PSK;
	wifi_params.ssid_length = strlen(SSID);
	wifi_params.psk_length = strlen(PSK);
	wifi_params.channel = WIFI_CHANNEL_ANY;
	wifi_params.security = WIFI_SECURITY_TYPE_PSK;
	wifi_params.band = WIFI_FREQ_BAND_2_4_GHZ;
	wifi_params.mfp = WIFI_MFP_OPTIONAL;
	printk("Connecting to SSID: %s\n", wifi_params.ssid);
	if (net_mgmt(NET_REQUEST_WIFI_CONNECT, iface, &wifi_params, sizeof(struct wifi_connect_req_params)))
	{
		printk("WiFi Connection Request Failed\n");
	}
}

void wifi_status(void)
{
	struct net_if *iface = net_if_get_default();
	struct wifi_iface_status status = {0};
	if (net_mgmt(NET_REQUEST_WIFI_IFACE_STATUS, iface, &status, sizeof(struct wifi_iface_status)))
	{
		printk("WiFi Status Request Failed\n");
	}
	printk("\n");
	if (status.state >= WIFI_STATE_ASSOCIATED)
	{
		printk("SSID: %-32s\n", status.ssid);
		printk("Channel: %d\n", status.channel);
		printk("RSSI: %d\n", status.rssi);
	}
}

static void response_cb(struct http_response *rsp,
                        enum http_final_call final_data,
                        void *user_data)
{
    if (final_data == HTTP_DATA_MORE) {
        printk("Partial data received (%zd bytes)", rsp->data_len);
    } else if (final_data == HTTP_DATA_FINAL) {
        printk("All the data received (%zd bytes)", rsp->data_len);
    }

    printk("Response status %s", rsp->http_status);

        if (rsp->body_found) {
            printk("Response body:\n");
			printk("%s\n", rsp->body_frag_start);
        }
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

	net_mgmt_init_event_callback(
		&wifi_cb,
		wifi_mgmt_event_handler,
		NET_EVENT_WIFI_CONNECT_RESULT | NET_EVENT_WIFI_DISCONNECT_RESULT);

	net_mgmt_init_event_callback(
		&ipv4_cb,
		wifi_mgmt_event_handler,
		NET_EVENT_IPV4_ADDR_ADD);

	net_mgmt_add_event_callback(&wifi_cb);
	net_mgmt_add_event_callback(&ipv4_cb);

	wifi_connect();

	k_sem_take(&wifi_connected, K_FOREVER);
	wifi_status();
	k_sem_take(&ipv4_address_obtained, K_FOREVER);

	static struct addrinfo hints;
	struct addrinfo *res;
	int st, sock, ret;
	hints.ai_family = AF_INET;
	hints.ai_socktype = SOCK_STREAM;
	st = getaddrinfo(HTTP_HOST, HTTP_PORT, &hints, &res);
	printf("getaddrinfo status: %d\n", st);

	if (st != 0)
	{
		printf("Unable to resolve address, quitting\n");
		return 0;
	}

	struct http_request req = { 0 };
	static uint8_t recv_buf[512];

	sock = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
	zsock_connect(sock, res->ai_addr, res->ai_addrlen);
	req.method = HTTP_POST;
	req.url = "/";
	req.host = HTTP_HOST;
	req.protocol = "HTTP/1.1";
	req.response = response_cb;
	req.recv_buf = recv_buf;
	req.recv_buf_len = sizeof(recv_buf);
	ret = http_client_req(sock, &req, 5000, NULL);
	zsock_close(sock);

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
	gpio_pin_configure_dt(&buzzer_gpio, GPIO_OUTPUT_LOW);

	init_lcd(&afficheur);

	while (1)
	{
		gpio_pin_configure_dt(&presence_sensor_gpio, GPIO_INPUT);

		// Lire la valeur du capteur de présence
		int presence = gpio_pin_get(presence_sensor_gpio.port, presence_sensor_gpio.pin);

		if (button_pressed_flag)
		{

			if (presence)
			{
				printk("Présence détectée\n");
				k_sleep(BUZZER_TOGGLE_PERIOD);
				// gpio_pin_configure_dt(&buzzer_gpio, GPIO_OUTPUT_LOW);
				gpio_pin_set_dt(&buzzer_gpio, 0);
				k_sleep(BUZZER_TOGGLE_PERIOD);
				// gpio_pin_configure_dt(&buzzer_gpio, GPIO_OUTPUT_HIGH);
				gpio_pin_set_dt(&buzzer_gpio, 1);
				// k_sleep(BUZZER_TOGGLE_PERIOD);

				count++;

				if (count == 5)
				{
					write_lcd(&afficheur, MODE_INTRU, LCD_LINE_1);
				}
			}
			else
			{
				printk("Pas de présence\n");
				write_lcd(&afficheur, MODE_ALARME, LCD_LINE_1);
				count = 0;
			}
		}

		if (affichage_off)
		{
			write_lcd(&afficheur, HELLO_MSG, LCD_LINE_1);
			button_pressed_flag = false;
			affichage_off = false;
		}

		k_sleep(BUZZER_TOGGLE_PERIOD);
	}
}

K_THREAD_DEFINE(alarm_button_id, 521, alarm_button_thread, NULL, NULL, NULL, 9, 0, 0);