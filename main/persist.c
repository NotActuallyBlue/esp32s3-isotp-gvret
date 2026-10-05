#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_task_wdt.h"
#include "ble_server.h"
#include "constants.h"
#include "persist.h"
#include "isotp.h"
#include "isotp_link_containers.h"
#include "connection_handler.h"
#include "isotp_bridge.h"
#include "esp_timer.h"
#include "display.h"

#define PERSIST_TAG	"Persist"

typedef struct {
	send_message_t messages[PERSIST_MAX_MESSAGE];
	uint16_t			number;
	uint16_t			count;
	uint16_t			position;
	uint16_t			rxID;
	uint16_t			txID;
	uint32_t			next_packet;
	SemaphoreHandle_t	data_mutex;
	SemaphoreHandle_t	task_mutex;
	SemaphoreHandle_t	send_sema;
} persist_t;

static SemaphoreHandle_t    persist_settings_mutex	= NULL;
static bool16				persist_run_task		= false;
static uint16_t				persist_msg_delay		= PERSIST_DEFAULT_MESSAGE_DELAY;
static uint16_t				persist_msg_qdelay		= PERSIST_DEFAULT_QUEUE_DELAY;
static bool16				persist_msg_enabled		= false;
static persist_t			persist_msgs[PERSIST_COUNT];

void persist_task(void *arg);

void persist_set_run_task(bool16 allow)
{
	tMUTEX(persist_settings_mutex);
		persist_run_task = allow;
	rMUTEX(persist_settings_mutex);
}

bool16 persist_allow_run_task()
{
	tMUTEX(persist_settings_mutex);
		bool16 run_task = persist_run_task;
	rMUTEX(persist_settings_mutex);

	return run_task;
}

void persist_init()
{
	persist_deinit();

	persist_settings_mutex = xSemaphoreCreateMutex();

	for (uint16_t i = 0; i < PERSIST_COUNT; i++) {
		persist_t* pPersist			= &persist_msgs[i];
		pPersist->number			= i;
		pPersist->count				= 0;
		pPersist->position			= 0;
		pPersist->next_packet		= 0;
		pPersist->rxID				= isotp_link_containers[i].link.send_arbitration_id;
		pPersist->txID				= isotp_link_containers[i].link.receive_arbitration_id;
		pPersist->data_mutex		= xSemaphoreCreateMutex();
		pPersist->task_mutex		= xSemaphoreCreateMutex();
		pPersist->send_sema			= xSemaphoreCreateBinary();
	}

	ESP_LOGI(PERSIST_TAG, "Init");
}

void persist_deinit()
{
	bool16 didDeInit = false;

	for (uint16_t i = 0; i < PERSIST_COUNT; i++) {
		if (persist_msgs[i].data_mutex) {
			vSemaphoreDelete(persist_msgs[i].data_mutex);
			persist_msgs[i].data_mutex = NULL;
			didDeInit = true;
		}

		if (persist_msgs[i].task_mutex) {
			vSemaphoreDelete(persist_msgs[i].task_mutex);
			persist_msgs[i].task_mutex = NULL;
			didDeInit = true;
		}

		if (persist_msgs[i].send_sema) {
			vSemaphoreDelete(persist_msgs[i].send_sema);
			persist_msgs[i].send_sema = NULL;
			didDeInit = true;
		}
	}

	if (persist_settings_mutex) {
		vSemaphoreDelete(persist_settings_mutex);
		persist_settings_mutex = NULL;
		didDeInit = true;
	}

	if(didDeInit)
		ESP_LOGI(PERSIST_TAG, "Deinit");
}

void persist_start_task()
{
	persist_stop_task();
	persist_set_run_task(true);

	ESP_LOGI(PERSIST_TAG, "Tasks starting");
	xSemaphoreTake(sync_task_sem, 0);
	for (uint16_t i = 0; i < PERSIST_COUNT; i++) {
		persist_t* pPersist = &persist_msgs[i];
		xTaskCreate(persist_task, "PERSIST_process", TASK_STACK_SIZE, pPersist, PERSIST_TSK_PRIO, NULL);
		xSemaphoreTake(sync_task_sem, portMAX_DELAY);
	}
	ESP_LOGI(PERSIST_TAG, "Tasks started");
}

void persist_stop_task()
{
	if (persist_allow_run_task()) {
		persist_set_run_task(false);

		for (uint16_t i = 0; i < PERSIST_COUNT; i++) {
			persist_t* pPersist = &persist_msgs[i];
			tMUTEX(pPersist->task_mutex);
			rMUTEX(pPersist->task_mutex);
		}

		persist_clear();

		ESP_LOGI(PERSIST_TAG, "Tasks stopped");
	}
}

void persist_allow_send(uint16_t persist)
{
	xSemaphoreGive(persist_msgs[persist].send_sema);
}

uint16_t persist_enabled()
{
	tMUTEX(persist_settings_mutex);
		uint16_t msg_enabled = persist_msg_enabled;
	rMUTEX(persist_settings_mutex);

	return msg_enabled;
}

static volatile uint32_t persist_enabled_at_ms = 0;

// Per-frame lines are plentiful (one per CAN frame), so the whole boot gets a fixed budget of them. Without
// the budget a retrying app keeps the window open and the log fills with the same exchange.
#define PERSIST_LOG_BUDGET  1200
static volatile uint32_t persist_log_budget = PERSIST_LOG_BUDGET;

bool persist_log_window(void)
{
	uint32_t started = persist_enabled_at_ms;
	if (!started || ((esp_timer_get_time() / 1000UL) - started) >= 1500 || persist_log_budget == 0)
		return false;
	persist_log_budget--;
	return true;
}

// How each persist session went: the app enables persist, waits a fixed time for every registered request to
// answer, then disables it. A reply that comes after that is too late for the "create PID frame" check.
static volatile uint32_t session_start_ms;
static volatile uint32_t session_end_ms;
static volatile uint32_t first_reply_ms[PERSIST_COUNT];
static volatile uint32_t reply_count[PERSIST_COUNT];

static uint32_t now_ms(void)
{
	return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

void persist_note_reply(uint16_t link)
{
	if (link >= PERSIST_COUNT || !session_start_ms)
		return;
	if (reply_count[link]++ == 0)
		first_reply_ms[link] = now_ms() - session_start_ms;
}

void persist_note_late_reply(uint16_t link)
{
	if (link >= PERSIST_COUNT || !session_end_ms)
		return;
	uint32_t late = now_ms() - session_end_ms;
	if (late < 400 && reply_count[link] == 0)       // only a link that said nothing during the session counts as a miss
		ESP_LOGW(PERSIST_TAG, "Link %u first answered %lu ms after persist was switched off: too late for the app's check", link, (unsigned long)late);
}

static void persist_session_begin(void)
{
	session_start_ms = now_ms();
	session_end_ms = 0;
	for (int i = 0; i < PERSIST_COUNT; i++) {
		first_reply_ms[i] = 0;
		reply_count[i] = 0;
	}
}

static void persist_session_end(void)
{
	if (!session_start_ms || session_end_ms)
		return;
	session_end_ms = now_ms();
	ESP_LOGI(PERSIST_TAG, "Session: persist on for %lu ms | ECU: %lu replies, first after %lu ms | TCU: %lu replies, first after %lu ms",
	         (unsigned long)(session_end_ms - session_start_ms),
	         (unsigned long)reply_count[0], (unsigned long)first_reply_ms[0],
	         (unsigned long)reply_count[1], (unsigned long)first_reply_ms[1]);
}

void persist_set(uint16_t enable)
{
	if (enable) {
		persist_enabled_at_ms = (esp_timer_get_time() / 1000UL) | 1;
		persist_session_begin();
	} else {
		persist_session_end();
	}

	tMUTEX(persist_settings_mutex);
		persist_msg_enabled = enable;
	rMUTEX(persist_settings_mutex);

	ESP_LOGI(PERSIST_TAG, "Enabled: %d", enable);
	display_set_persist(enable);

	// Wake the persist tasks so the first request goes out now instead of after their idle wait
	if (enable) {
		for (uint16_t i = 0; i < PERSIST_COUNT; i++) {
			if (persist_msgs[i].send_sema)
				xSemaphoreGive(persist_msgs[i].send_sema);
		}
	}
}

int16_t persist_add(uint16_t rx, uint16_t tx, const void* src, size_t size)
{
	//check for valid message
	if (!src || size == 0)
		return false;

	// Start the detailed logging window at the first PID so the whole setup burst is captured
	persist_enabled_at_ms = (esp_timer_get_time() / 1000UL) | 1;

	//set persist pointer
	persist_t* pPersist = NULL;
	uint16_t pCount = 0;
	for (uint16_t i = 0; i < PERSIST_COUNT; i++)
	{
		persist_t* persist = &persist_msgs[i];
		tMUTEX(persist->data_mutex);
			if (persist->rxID == rx && persist->txID == tx) {
				pPersist	= persist;
				pCount		= persist->count;
			}
		rMUTEX(persist->data_mutex);
		if (pPersist)
			break;
	}
	if (pPersist == NULL || pCount >= PERSIST_MAX_MESSAGE)
		return false;

	//add new persist message
	bool16 msg_added = false;
	tMUTEX(pPersist->data_mutex);
		send_message_t* pMsg	= &pPersist->messages[pPersist->count++];
		pMsg->rxID				= rx;
		pMsg->txID				= tx;
		pMsg->msg_length		= 0;
		pMsg->buffer			= malloc(size);
		if(pMsg->buffer){
			memcpy(pMsg->buffer, src, size);
			pMsg->msg_length = size;
			msg_added = true;
		}
	rMUTEX(pPersist->data_mutex);

	if (msg_added) {
		ESP_LOGI(PERSIST_TAG, "Message added with size: %04X", size);
	}
	else {
		ESP_LOGI(PERSIST_TAG, "Error adding message: malloc error %s %d", __func__, __LINE__);
	}

	return msg_added;
}

void persist_clear()
{
	persist_set(false);
	for (uint16_t d = 0; d < PERSIST_COUNT; d++) {
		//clear all messages
		persist_t* pPersist = &persist_msgs[d];
		tMUTEX(pPersist->data_mutex);
			for (uint16_t i = 0; i < pPersist->count; i++)
			{
				send_message_t* pMsg = &pPersist->messages[i];
				if (pMsg->buffer)
				{
					free(pMsg->buffer);
					pMsg->buffer = NULL;
				}
				pMsg->msg_length = 0;
			}
			pPersist->position = 0;
			pPersist->count = 0;
		rMUTEX(pPersist->data_mutex);
	}

	ESP_LOGI(PERSIST_TAG, "Messages cleared");
}

// Clears the persist messages of the link matching rx/tx and leaves persist mode untouched.
// If no link matches, every link is cleared.
void persist_clear_link(uint16_t rx, uint16_t tx)
{
	bool16 found = false;
	for (uint16_t d = 0; d < PERSIST_COUNT; d++) {
		persist_t* pPersist = &persist_msgs[d];
		tMUTEX(pPersist->data_mutex);
			if (pPersist->rxID == rx && pPersist->txID == tx) {
				found = true;
				for (uint16_t i = 0; i < pPersist->count; i++) {
					send_message_t* pMsg = &pPersist->messages[i];
					if (pMsg->buffer) {
						free(pMsg->buffer);
						pMsg->buffer = NULL;
					}
					pMsg->msg_length = 0;
				}
				pPersist->position = 0;
				pPersist->count = 0;
			}
		rMUTEX(pPersist->data_mutex);
	}

	if (found) {
		ESP_LOGI(PERSIST_TAG, "Messages cleared for rx 0x%04X / tx 0x%04X", rx, tx);
	} else {
		persist_clear();
	}
}

bool16 persist_send(persist_t* pPersist)
{
	tMUTEX(pPersist->data_mutex);
		//If persist mode is disabled abort
		if (!persist_enabled() || !pPersist->count)
			goto return_false;

		//don't flood the queues
		if (!bridge_send_available() || !ble_queue_spaces())
		{
			ESP_LOGW(PERSIST_TAG, "Unable to send message: queues are full");
			xSemaphoreGive(pPersist->send_sema);
			goto return_false;
		}

		//Cycle through messages
		if (pPersist->position >= pPersist->count)
			pPersist->position = 0;

		//Make sure the message has length
		uint16_t mPos = pPersist->position++;
		send_message_t* pMsg = &pPersist->messages[mPos];
		if (pMsg->msg_length == 0)
		{
			ESP_LOGW(PERSIST_TAG, "Unable to send message: message length 0");
			xSemaphoreGive(pPersist->send_sema);
			goto return_false;
		}

		//We are good, lets send the message!
		send_message_t msg;
		msg.buffer = malloc(pMsg->msg_length);
		if (msg.buffer == NULL) {
			ESP_LOGW(PERSIST_TAG, "Unable to send message: malloc error %s %d", __func__, __LINE__);
			xSemaphoreGive(pPersist->send_sema);
			goto return_false;
		}

		//copy persist message into a new container
		memcpy(msg.buffer, pMsg->buffer, pMsg->msg_length);
		msg.msg_length = pMsg->msg_length;
		msg.rxID = pMsg->rxID;
		msg.txID = pMsg->txID;
	rMUTEX(pPersist->data_mutex);

	//if we fail to place message into the queue free the memory!
	if (bridge_send_isotp(&msg) != pdTRUE) {
		free(msg.buffer);
		xSemaphoreGive(pPersist->send_sema);
		return false;
	}
	else {
		ESP_LOGD(PERSIST_TAG, "Message sent with size: %04X", msg.msg_length);
	}

	return true;

return_false:
	rMUTEX(pPersist->data_mutex);
	return false;
}

void persist_task(void *arg)
{
	//subscribe to WDT
	ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
	ESP_ERROR_CHECK(esp_task_wdt_status(NULL));

	persist_t *pPersist = ((persist_t*)arg);
	tMUTEX(pPersist->task_mutex);
		tMUTEX(pPersist->data_mutex);
			ESP_LOGI(PERSIST_TAG, "Task started: %d", pPersist->number);
		rMUTEX(pPersist->data_mutex);
		xSemaphoreGive(sync_task_sem);
		while(persist_allow_run_task()) {
			xSemaphoreTake(pPersist->send_sema, pdMS_TO_TICKS(TIMEOUT_NORMAL));
			uint32_t current_delay = 1;
			uint32_t required_delay = persist_get_delay() + (ble_queue_waiting() * persist_get_q_delay());
			uint32_t current_time = (esp_timer_get_time() / 1000UL) & 0xFFFFFFFF;
			if(persist_send(pPersist)) {
				tMUTEX(pPersist->data_mutex);
					pPersist->next_packet += required_delay;
					if(pPersist->next_packet <= current_time) {
						pPersist->next_packet = current_time + 1;
					} else {
						current_delay = pPersist->next_packet - current_time;
						if(current_delay > required_delay) {
							current_delay			= required_delay;
							pPersist->next_packet	= current_time + required_delay;
						}
					}
				rMUTEX(pPersist->data_mutex);
			} else {
				current_delay = required_delay;
				tMUTEX(pPersist->data_mutex);
					pPersist->next_packet = current_time + required_delay;
				rMUTEX(pPersist->data_mutex);
			}

			//reset the WDT and wait our delay
			esp_task_wdt_reset();
			vTaskDelay(pdMS_TO_TICKS(current_delay));
		}
		tMUTEX(pPersist->data_mutex);
			ESP_LOGI(PERSIST_TAG, "Task stopped: %d", pPersist->number);
		rMUTEX(pPersist->data_mutex);
	rMUTEX(pPersist->task_mutex);

	//unsubscribe to WDT and delete task
	ESP_ERROR_CHECK(esp_task_wdt_delete(NULL));
	vTaskDelete(NULL);
}

void persist_set_delay(uint16_t delay)
{
	tMUTEX(persist_settings_mutex);
		persist_msg_delay = delay;
	rMUTEX(persist_settings_mutex);

	ESP_LOGI(PERSIST_TAG, "Set delay: %d", delay);
}

void persist_set_q_delay(uint16_t delay)
{
	tMUTEX(persist_settings_mutex);
		persist_msg_qdelay = delay;
	rMUTEX(persist_settings_mutex);

	ESP_LOGI(PERSIST_TAG, "Set q delay: %d", delay);
}

uint16_t persist_get_delay()
{
	tMUTEX(persist_settings_mutex);
		uint16_t msg_delay = persist_msg_delay;
	rMUTEX(persist_settings_mutex);

	return msg_delay;
}

uint16_t persist_get_q_delay()
{
	tMUTEX(persist_settings_mutex);
		uint16_t q_delay = persist_msg_qdelay;
	rMUTEX(persist_settings_mutex);

	return q_delay;
}