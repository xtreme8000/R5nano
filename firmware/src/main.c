#ifndef STM32F1
#define STM32F1
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/usb/usbd.h>

#include <FreeRTOS.h>
#include <queue.h>
#include <semphr.h>
#include <task.h>

#define PORT_CRESET_B GPIOB
#define PIN_CRESET_B GPIO1

#define PORT_SPI_SS GPIOB
#define PIN_SPI_SS GPIO12

#define PORT_CDONE GPIOB
#define PIN_CDONE GPIO0

#define PORT_USB_EN GPIOB
#define PIN_USB_EN GPIO3

#define PORT_LED1 GPIOB
#define PIN_LED1 GPIO6

#define PORT_LED2 GPIOB
#define PIN_LED2 GPIO5

uint8_t usbd_control_buffer[64]; // buffer for control requests

#define STREAM_PAYLOAD_DATA_SIZE 256

struct stream_payload {
	size_t length;
	uint8_t data[STREAM_PAYLOAD_DATA_SIZE];
};

struct usb_device_descriptor dev_descr = {
	.bLength = USB_DT_DEVICE_SIZE,
	.bDescriptorType = USB_DT_DEVICE,
	.bcdUSB = 0x0200,
	.bDeviceClass = 0,
	.bDeviceSubClass = 0,
	.bDeviceProtocol = 0,
	.bMaxPacketSize0 = 64, // max size
	.idVendor = 0x1209,
	.idProduct = 0x000E,
	.bcdDevice = 0x0100,
	.iManufacturer = 1,
	.iProduct = 2,
	.iSerialNumber = 0,
	.bNumConfigurations = 1,
};

struct usb_endpoint_descriptor usb_endpoints[] = {
	{
		.bLength = USB_DT_ENDPOINT_SIZE,
		.bDescriptorType = USB_DT_ENDPOINT,
		.bEndpointAddress = USB_ENDPOINT_ADDR_OUT(1),
		.bmAttributes = USB_ENDPOINT_ATTR_BULK,
		.wMaxPacketSize = STREAM_PAYLOAD_DATA_SIZE,
	},
	{
		.bLength = USB_DT_ENDPOINT_SIZE,
		.bDescriptorType = USB_DT_ENDPOINT,
		.bEndpointAddress = USB_ENDPOINT_ADDR_IN(2),
		.bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
		.wMaxPacketSize = 1,
		.bInterval = 5,
	},
};

struct usb_interface_descriptor usb_iface = {
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 0,
	.bAlternateSetting = 0,
	.bNumEndpoints = 2,
	.bInterfaceClass = USB_CLASS_VENDOR,
	.bInterfaceSubClass = 0,
	.bInterfaceProtocol = 0,
	.iInterface = 0,
	.endpoint = usb_endpoints,
};

struct usb_config_descriptor usb_config = {
	.bLength = USB_DT_CONFIGURATION_SIZE,
	.bDescriptorType = USB_DT_CONFIGURATION,
	.wTotalLength = 0,
	.bNumInterfaces = 1,
	.bConfigurationValue = 1,
	.iConfiguration = 0,
	.bmAttributes = USB_CONFIG_ATTR_DEFAULT,
	.bMaxPower = 100 / 2, // 100mA

	.interface = (struct usb_interface[]) {{
		.num_altsetting = 1,
		.altsetting = &usb_iface,
	}},
};

const char* usb_strings[3] = {
	"xtreme8000",
	"R5nano",
};

QueueHandle_t incoming_data;
QueueHandle_t available_buffers;
QueueHandle_t usb_host_interrupt;

void usb_endpoint_callback(usbd_device* dev, uint8_t ep) {
	if(ep != USB_ENDPOINT_ADDR_OUT(1))
		return;

	struct stream_payload* e;
	xQueueReceive(available_buffers, &e, portMAX_DELAY);
	e->length = usbd_ep_read_packet(dev, ep, e->data, sizeof(e->data));
	xQueueSend(incoming_data, &e, portMAX_DELAY);
}

enum usbd_request_return_codes
usb_creq_callback(usbd_device* dev, struct usb_setup_data* req, uint8_t** buf,
				  uint16_t* len,
				  void (**complete)(usbd_device*, struct usb_setup_data*)) {
	if(req->bmRequestType
	   != (USB_REQ_TYPE_OUT | USB_REQ_TYPE_VENDOR | USB_REQ_TYPE_INTERFACE))
		return USBD_REQ_NOTSUPP;

	switch(req->bRequest) {
		case 10:
			if(req->wLength != 4)
				return USBD_REQ_NOTSUPP;

			uint32_t payload_length = ((*buf)[3] << 24) | ((*buf)[2] << 16)
				| ((*buf)[1] << 8) | (*buf)[0];

			struct stream_payload* e;
			xQueueReceive(available_buffers, &e, portMAX_DELAY);
			e->length = payload_length;
			xQueueSend(incoming_data, &e, portMAX_DELAY);

			return USBD_REQ_HANDLED;
		default: return USBD_REQ_NOTSUPP;
	}
}

static void usb_set_config(usbd_device* dev, uint16_t wValue) {
	usbd_register_control_callback(
		dev, USB_REQ_TYPE_VENDOR | USB_REQ_TYPE_INTERFACE,
		USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT, usb_creq_callback);
	usbd_ep_setup(dev, USB_ENDPOINT_ADDR_OUT(1), USB_ENDPOINT_ATTR_BULK,
				  STREAM_PAYLOAD_DATA_SIZE, usb_endpoint_callback);
	usbd_ep_setup(dev, USB_ENDPOINT_ADDR_IN(2), USB_ENDPOINT_ATTR_INTERRUPT, 1,
				  NULL);
}

void task_usb(void* args) {
	gpio_clear(PORT_USB_EN, PIN_USB_EN);
	vTaskDelay(pdMS_TO_TICKS(100));
	gpio_set(PORT_USB_EN, PIN_USB_EN);

	usbd_device* usbd_dev = usbd_init(
		&st_usbfs_v1_usb_driver, &dev_descr, &usb_config, usb_strings, 2,
		usbd_control_buffer, sizeof(usbd_control_buffer));
	usbd_register_set_config_callback(usbd_dev, usb_set_config);

	while(1) {
		usbd_poll(usbd_dev);

		uint8_t signal;
		if(xQueueReceive(usb_host_interrupt, &signal, 0)) {
			// raise host interrupt
			usbd_ep_write_packet(usbd_dev, USB_ENDPOINT_ADDR_IN(2), &signal,
								 sizeof(signal));
		}
	}
}

void task_blink(void* args) {
	while(1) {
		gpio_toggle(PORT_LED1, PIN_LED1);
		vTaskDelay(pdMS_TO_TICKS(500));
	}
}

TaskHandle_t task_upload_handle;

struct stream_payload* payload_current = NULL;
uint32_t payload_total = 0;
uint32_t payload_transmitted = 0;

void dma1_channel5_isr(void) {
	dma_disable_channel(DMA1, DMA_CHANNEL5);
	dma_clear_interrupt_flags(DMA1, DMA_CHANNEL5, DMA_TCIF);

	BaseType_t task_woken = pdFALSE;

	struct stream_payload* e;
	if(payload_transmitted < payload_total
	   && xQueueReceiveFromISR(incoming_data, &e, &task_woken)) {
		dma_set_memory_address(DMA1, DMA_CHANNEL5, (uint32_t)e->data);
		dma_set_number_of_data(DMA1, DMA_CHANNEL5, e->length);
		dma_enable_channel(DMA1, DMA_CHANNEL5);
	} else {
		e = NULL;
		vTaskNotifyGiveFromISR(task_upload_handle, &task_woken);
	}

	if(payload_current) {
		payload_transmitted += payload_current->length;
		xQueueSendFromISR(available_buffers, &payload_current, &task_woken);
	}

	payload_current = e;
	portYIELD_FROM_ISR(task_woken);
}

void spi_wait_idle(uint32_t spi) {
	while(SPI_SR(spi) & SPI_SR_BSY)
		;
}

void task_upload(void* args) {
	while(1) {
		struct stream_payload* e;
		xQueueReceive(incoming_data, &e, portMAX_DELAY);
		payload_total = e->length;
		payload_transmitted = 0;
		xQueueSend(available_buffers, &e, portMAX_DELAY);

		gpio_set(PORT_LED2, PIN_LED2);

		gpio_clear(PORT_CRESET_B, PIN_CRESET_B);
		gpio_clear(PORT_SPI_SS, PIN_SPI_SS);
		// clear cresetb for 1-2ms
		vTaskDelay(pdMS_TO_TICKS(2));
		gpio_set(PORT_CRESET_B, PIN_CRESET_B);
		// clear spi_ss for 2-3ms
		vTaskDelay(pdMS_TO_TICKS(3));
		gpio_set(PORT_SPI_SS, PIN_SPI_SS);
		// generate 8 clock cycles
		spi_send(SPI2, 0);
		spi_wait_idle(SPI2);
		gpio_clear(PORT_SPI_SS, PIN_SPI_SS);

		// TODO: error handling, e.g. stream ends early

		while(payload_transmitted < payload_total) {
			xQueueReceive(incoming_data, &e, portMAX_DELAY);
			payload_current = e;
			dma_set_memory_address(DMA1, DMA_CHANNEL5, (uint32_t)e->data);
			dma_set_number_of_data(DMA1, DMA_CHANNEL5, e->length);
			dma_enable_channel(DMA1, DMA_CHANNEL5);
			ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		}

		spi_wait_idle(SPI2);
		gpio_set(PORT_SPI_SS, PIN_SPI_SS);

		for(size_t k = 0; k < 20; k++)
			spi_send(SPI2, k);

		spi_wait_idle(SPI2);

		uint8_t success = gpio_get(PORT_CDONE, PIN_CDONE) ? 1 : 0;
		xQueueSend(usb_host_interrupt, &success, portMAX_DELAY);

		gpio_clear(PORT_LED2, PIN_LED2);
	}
}

int main() {
	rcc_clock_setup_pll(rcc_hse_configs + RCC_CLOCK_HSE12_72MHZ);

	rcc_periph_clock_enable(RCC_USART1);
	rcc_periph_clock_enable(RCC_AFIO);
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOC);

	gpio_primary_remap(AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_ON, 0);

	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
				  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO8);
	rcc_set_mco(RCC_CFGR_MCO_HSE);

	gpio_set_mode(GPIO_BANK_USART1_TX, GPIO_MODE_OUTPUT_50_MHZ,
				  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_TX);

	usart_set_baudrate(USART1, 115200);
	usart_set_databits(USART1, 8);
	usart_set_stopbits(USART1, USART_STOPBITS_1);
	usart_set_parity(USART1, USART_PARITY_NONE);
	usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
	usart_set_mode(USART1, USART_MODE_TX);
	usart_enable(USART1);

	// usb setup
	gpio_set_mode(PORT_USB_EN, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL,
				  PIN_USB_EN);
	gpio_clear(PORT_USB_EN, PIN_USB_EN);

	// blink setup
	gpio_set_mode(PORT_LED1, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL,
				  PIN_LED1);
	gpio_clear(PORT_LED1, PIN_LED1);
	gpio_set_mode(PORT_LED2, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL,
				  PIN_LED2);
	gpio_clear(PORT_LED2, PIN_LED2);

	// upload setup
	rcc_periph_clock_enable(RCC_SPI2);
	rcc_periph_clock_enable(RCC_DMA1);

	gpio_set_mode(GPIO_BANK_SPI2_SCK, GPIO_MODE_OUTPUT_50_MHZ,
				  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_SPI2_SCK);
	gpio_set_mode(GPIO_BANK_SPI2_MOSI, GPIO_MODE_OUTPUT_50_MHZ,
				  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_SPI2_MOSI);
	gpio_set_mode(GPIO_BANK_SPI2_MISO, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT,
				  GPIO_SPI2_MISO);

	spi_init_master(
		SPI2, SPI_CR1_BAUDRATE_FPCLK_DIV_32, SPI_CR1_CPOL_CLK_TO_1_WHEN_IDLE,
		SPI_CR1_CPHA_CLK_TRANSITION_2, SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);
	spi_enable_software_slave_management(SPI2);
	spi_set_nss_high(SPI2);
	spi_enable_tx_dma(SPI2);
	spi_enable(SPI2);

	dma_channel_reset(DMA1, DMA_CHANNEL5);
	dma_set_memory_size(DMA1, DMA_CHANNEL5, DMA_CCR_MSIZE_8BIT);
	dma_set_peripheral_size(DMA1, DMA_CHANNEL5, DMA_CCR_PSIZE_8BIT);
	dma_set_peripheral_address(DMA1, DMA_CHANNEL5, (uint32_t)&SPI2_DR);
	dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL5);
	dma_disable_peripheral_increment_mode(DMA1, DMA_CHANNEL5);
	dma_set_read_from_memory(DMA1, DMA_CHANNEL5);

	dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL5);
	nvic_set_priority(NVIC_DMA1_CHANNEL5_IRQ, 0);
	nvic_enable_irq(NVIC_DMA1_CHANNEL5_IRQ);

	gpio_set_mode(PORT_CRESET_B, GPIO_MODE_OUTPUT_50_MHZ,
				  GPIO_CNF_OUTPUT_PUSHPULL, PIN_CRESET_B);
	gpio_set_mode(PORT_SPI_SS, GPIO_MODE_OUTPUT_50_MHZ,
				  GPIO_CNF_OUTPUT_PUSHPULL, PIN_SPI_SS);
	gpio_set_mode(PORT_CDONE, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, PIN_CDONE);
	gpio_set(PORT_CRESET_B, PIN_CRESET_B);
	gpio_set(PORT_SPI_SS, PIN_SPI_SS);

	available_buffers = xQueueCreate(8, sizeof(struct stream_payload*));
	incoming_data = xQueueCreate(8, sizeof(struct stream_payload*));
	usb_host_interrupt = xQueueCreate(8, sizeof(uint8_t));

	for(size_t k = 0; k < 8; k++) {
		struct stream_payload* e = pvPortMalloc(sizeof(struct stream_payload));
		xQueueSend(available_buffers, &e, portMAX_DELAY);
	}

	xTaskCreate(task_upload, "upload", configMINIMAL_STACK_SIZE, NULL, 4,
				&task_upload_handle);
	xTaskCreate(task_usb, "usb", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
	xTaskCreate(task_blink, "blink", configMINIMAL_STACK_SIZE, NULL, 3, NULL);
	vTaskStartScheduler();

	return 0;
}