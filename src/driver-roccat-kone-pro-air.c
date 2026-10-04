/*
 * Copyright © 2026 Welnyr.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

/*
 * Roccat Kone Pro Air (USB cable 1e7d:2c92, wireless dongle 1e7d:2c8e).
 *
 * The protocol was derived from USB captures of the vendor software. It is
 * spoken over 64-byte unnumbered interrupt reports on the vendor HID
 * interface (usage page 0xff00). Requests start with a direction byte (0x90
 * read, 0x10 write) and a command byte, replies echo both.
 *
 * Settings live in five chunks per profile. The write of chunk 0 selects the
 * profile that chunks 1..4 are written to. A chunk is re-read right before it
 * is written and only the bytes this driver owns are changed.
 *
 * Not exposed: macros, debounce, distance control, the Easy-Shift layer, the
 * energy-saving options and the profile colour.
 */

#include "config.h"

#include <errno.h>
#include <poll.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "libratbag-private.h"
#include "libratbag-hidraw.h"

#define PA_REPORT_SIZE		64

#define PA_DIR_READ		0x90
#define PA_DIR_WRITE		0x10

#define PA_CMD_SET_ACTIVE_PROFILE 0x03	/* write: profile that the mouse uses */
#define PA_CMD_SELECT_PROFILE	0x13	/* write: select the profile that is read back */
#define PA_CMD_WRITE_CHUNK	0x50	/* write: profile chunk, byte 3 is the chunk */

enum pa_chunk {
	PA_CHUNK_SETTINGS,	/* polling rate, DPI stages */
	PA_CHUNK_LIGHTING,
	PA_CHUNK_FUNCTIONS,	/* per-button function bytes */
	PA_CHUNK_BUTTONS,	/* button table */
	PA_CHUNK_BUTTONS_EASY_SHIFT,
	PA_NUM_CHUNKS,
};

/* The read command and the number of data bytes of every chunk. */
static const struct {
	uint8_t read_cmd;
	uint8_t len;
} pa_chunks[PA_NUM_CHUNKS] = {
	[PA_CHUNK_SETTINGS] = { 0x00, 0x14 },
	[PA_CHUNK_LIGHTING] = { 0x01, 0x0b },
	[PA_CHUNK_FUNCTIONS] = { 0x02, 0x1c },
	[PA_CHUNK_BUTTONS] = { 0x04, 0x18 },
	[PA_CHUNK_BUTTONS_EASY_SHIFT] = { 0x05, 0x18 },
};

/* Data starts after a 4-byte frame header, chunk 0 has 6 more header bytes. */
#define PA_REPLY_HEADER		4
#define PA_C0_EXTRA_HEADER	6
#define PA_C0_REPLY_LEN		0x26	/* data length of a full chunk 0 reply */

#define PA_RETRIES		3
#define PA_RETRY_DELAY_MS	100
#define PA_MAX_SKIPPED_REPLIES	8	/* unrelated frames to skip per request */

/* Offsets in the data of chunk 0. */
#define PA_C0_PROFILE_INDEX	0
#define PA_C0_POLLING_RATE	1	/* 0 = 1000 Hz ... 3 = 125 Hz */
#define PA_C0_ANGLE_SNAPPING	2
#define PA_C0_ACTIVE_STAGE	3
#define PA_C0_DPI_STAGE_0	4	/* little endian u16, units of 50 dpi, 0 = disabled */
#define PA_C0_DEBOUNCE		19	/* write position, read back at frame index 41 */
#define PA_C0_DEBOUNCE_READ	41	/* frame index in the reply */

/* Offsets in the data of chunk 1. */
#define PA_C1_EFFECT		0
#define PA_C1_BRIGHTNESS	3	/* percent */
#define PA_C1_SPEED		4	/* 1 (slowest) .. 11 */
#define PA_C1_COLOR_0		5	/* two RGB triplets, one per zone */

#define PA_LED_OFF		0x00
#define PA_LED_ON		0x01
#define PA_LED_BREATHING	0x02
#define PA_LED_WAVE		0x06

#define PA_SPEED_MIN		1
#define PA_SPEED_MAX		11
#define PA_SPEED_MS_BASE	6500	/* ms = base - speed * step */
#define PA_SPEED_MS_STEP	500

#define PA_FN_NONE		0x00
#define PA_FN_ORDINARY		0x40	/* the entry in the button table applies */
#define PA_FN_DEFAULT		0xff	/* read-only marker, the default button */
#define PA_ENTRY_SIZE		3
#define PA_ENTRY_MOUSE		0x00
#define PA_ENTRY_KEY		0x01
#define PA_HID_MODIFIER_0	0xe0	/* HID usage of left control, bit 0 of the mask */

#define PA_NUM_PROFILES		4
#define PA_NUM_DPI_STAGES	5
#define PA_NUM_BUTTONS		8
#define PA_NUM_LEDS		2
#define PA_DPI_STEP		50
#define PA_DPI_MIN		50
#define PA_DPI_MAX		19000

static const unsigned int pa_report_rates[] = { 125, 250, 500, 1000 };

/* Function bytes (chunk 2) that map to a libratbag special action. */
static const struct {
	uint8_t fn;
	enum ratbag_button_action_special special;
} pa_functions[] = {
	{ 0x41, RATBAG_BUTTON_ACTION_SPECIAL_PROFILE_CYCLE_UP },
	{ 0x46, RATBAG_BUTTON_ACTION_SPECIAL_PROFILE_UP },
	{ 0x47, RATBAG_BUTTON_ACTION_SPECIAL_PROFILE_DOWN },
	{ 0x42, RATBAG_BUTTON_ACTION_SPECIAL_RESOLUTION_CYCLE_UP },
	{ 0x52, RATBAG_BUTTON_ACTION_SPECIAL_RESOLUTION_UP },
	{ 0x53, RATBAG_BUTTON_ACTION_SPECIAL_RESOLUTION_DOWN },
};

/* Mouse button table entries `00 N 00` that are not a plain button. */
static const struct {
	uint8_t arg;
	enum ratbag_button_action_special special;
} pa_mouse_specials[] = {
	{ 6, RATBAG_BUTTON_ACTION_SPECIAL_DOUBLECLICK },
	{ 7, RATBAG_BUTTON_ACTION_SPECIAL_WHEEL_UP },
	{ 8, RATBAG_BUTTON_ACTION_SPECIAL_WHEEL_DOWN },
};

struct pa_profile {
	/* raw replies as read at probe time, only used to fill libratbag */
	uint8_t chunks[PA_NUM_CHUNKS][PA_REPORT_SIZE];
};

static uint8_t *
pa_chunk_data(uint8_t *reply, enum pa_chunk chunk)
{
	if (chunk == PA_CHUNK_SETTINGS)
		return reply + PA_REPLY_HEADER + PA_C0_EXTRA_HEADER;
	return reply + PA_REPLY_HEADER;
}

static bool
pa_reply_matches(const uint8_t *req, const uint8_t *reply)
{
	return reply[0] == req[0] && reply[1] == req[1];
}

/*
 * hidraw gives every process that has the node open a copy of every input
 * report. Replies meant for another reader (a dump script, the vendor
 * software) would otherwise be taken for the reply to our next request.
 */
static void
pa_drain_input(struct ratbag_device *device)
{
	struct pollfd pfd = { .fd = device->hidraw[0].fd, .events = POLLIN };
	uint8_t junk[PA_REPORT_SIZE];

	while (poll(&pfd, 1, 0) > 0 && (pfd.revents & POLLIN)) {
		if (read(pfd.fd, junk, sizeof(junk)) <= 0)
			break;
	}
}

/*
 * Send a request and wait for the reply with the same two first bytes.
 * Unsolicited frames (button events, battery status) arrive on the same
 * endpoint and are skipped.
 */
static int
pa_xfer(struct ratbag_device *device, const uint8_t *req, uint8_t *reply)
{
	uint8_t out[PA_REPORT_SIZE];
	int rc, retries = PA_MAX_SKIPPED_REPLIES;

	memcpy(out, req, sizeof(out));

	/* the vendor software waits 5 ms before every write */
	if (req[0] == PA_DIR_WRITE)
		msleep(5);

	pa_drain_input(device);

	rc = ratbag_hidraw_output_report(device, out, sizeof(out));
	if (rc < 0)
		return rc;

	do {
		rc = ratbag_hidraw_read_input_report(device, reply,
						     PA_REPORT_SIZE, NULL);
		if (rc < 0)
			return rc;
		if (rc == PA_REPORT_SIZE && pa_reply_matches(req, reply))
			return 0;
	} while (--retries);

	return -EIO;
}

/* A write is acknowledged with `10 <cmd> 03 00 ec ac`, errors are `ec fa NN`. */
static int
pa_check_ack(struct ratbag_device *device, const uint8_t *reply)
{
	if (reply[2] == 0x03 && reply[4] == 0xec && reply[5] == 0xac)
		return 0;

	if (reply[2] == 0x03 && reply[4] == 0xec && reply[5] == 0xfa)
		log_error(device->ratbag, "command %02x failed with error %02x\n",
			  reply[1], reply[6]);

	return -EIO;
}

static int
pa_read_chunk(struct ratbag_device *device, enum pa_chunk chunk, uint8_t *reply)
{
	uint8_t req[PA_REPORT_SIZE] = { PA_DIR_READ, pa_chunks[chunk].read_cmd };
	int need = PA_REPLY_HEADER + pa_chunks[chunk].len;
	int rc = -EIO;

	/* the profile chunk 0 request carries a constant payload */
	if (chunk == PA_CHUNK_SETTINGS) {
		static const uint8_t payload[] = { 0x04, 0x00, 0x18, 0x25, 0x51, 0x32 };

		memcpy(&req[2], payload, sizeof(payload));
		/* the reply is longer than the data written back, it also has the debounce */
		need = PA_REPLY_HEADER + PA_C0_REPLY_LEN;
	}

	for (int attempt = 0; attempt < PA_RETRIES; attempt++) {
		if (attempt)
			msleep(PA_RETRY_DELAY_MS);

		rc = pa_xfer(device, req, reply);
		if (rc)
			continue;

		/* an error reply (`ec fa NN`) is shorter than the data */
		if (reply[2] + PA_REPLY_HEADER >= need)
			return 0;
		rc = -EIO;
	}

	return rc;
}

static int
pa_write_req(struct ratbag_device *device, const uint8_t *req)
{
	uint8_t reply[PA_REPORT_SIZE];
	int rc = -EIO;

	/* all writes are idempotent, so a failed exchange can be repeated */
	for (int attempt = 0; attempt < PA_RETRIES; attempt++) {
		if (attempt)
			msleep(PA_RETRY_DELAY_MS);

		rc = pa_xfer(device, req, reply);
		if (!rc)
			rc = pa_check_ack(device, reply);
		if (!rc)
			return 0;
	}

	return rc;
}

static int
pa_write_chunk(struct ratbag_device *device, enum pa_chunk chunk,
	       const uint8_t *data)
{
	uint8_t req[PA_REPORT_SIZE] = { PA_DIR_WRITE, PA_CMD_WRITE_CHUNK,
					pa_chunks[chunk].len, chunk };

	memcpy(&req[PA_REPLY_HEADER], data, pa_chunks[chunk].len);

	return pa_write_req(device, req);
}

/* Commands with a single data byte: `10 <cmd> 01 00 <value>`. */
static int
pa_write_byte(struct ratbag_device *device, uint8_t cmd, uint8_t value)
{
	uint8_t req[PA_REPORT_SIZE] = { PA_DIR_WRITE, cmd, 0x01, 0x00, value };

	return pa_write_req(device, req);
}

static int
pa_select_profile(struct ratbag_device *device, unsigned int index)
{
	return pa_write_byte(device, PA_CMD_SELECT_PROFILE, index);
}

static unsigned int
pa_dpi_stage(const uint8_t *c0, unsigned int stage)
{
	unsigned int off = PA_C0_DPI_STAGE_0 + stage * 2;

	return (c0[off] | (c0[off + 1] << 8)) * PA_DPI_STEP;
}

static int
pa_read_profile_raw(struct ratbag_device *device, struct pa_profile *p)
{
	int rc;

	/* the Easy-Shift table is not exposed, so it is not read */
	for (enum pa_chunk chunk = 0; chunk < PA_CHUNK_BUTTONS_EASY_SHIFT; chunk++) {
		rc = pa_read_chunk(device, chunk, p->chunks[chunk]);
		if (rc)
			return rc;
	}

	return 0;
}

static int
pa_rate_to_index(unsigned int hz)
{
	for (size_t i = 0; i < ARRAY_LENGTH(pa_report_rates); i++) {
		if (pa_report_rates[i] == hz)
			return ARRAY_LENGTH(pa_report_rates) - 1 - i;
	}

	return -1;
}

static bool
pa_effect_to_mode(uint8_t effect, enum ratbag_led_mode *mode)
{
	switch (effect) {
	case PA_LED_OFF:
		*mode = RATBAG_LED_OFF;
		return true;
	case PA_LED_ON:
		*mode = RATBAG_LED_ON;
		return true;
	case PA_LED_BREATHING:
		*mode = RATBAG_LED_BREATHING;
		return true;
	case PA_LED_WAVE:
		*mode = RATBAG_LED_CYCLE;
		return true;
	default:
		/* heartbeat, blinking, battery, DPI colour: shown as ON */
		return false;
	}
}

static int
pa_mode_to_effect(enum ratbag_led_mode mode)
{
	switch (mode) {
	case RATBAG_LED_OFF:
		return PA_LED_OFF;
	case RATBAG_LED_ON:
		return PA_LED_ON;
	case RATBAG_LED_BREATHING:
		return PA_LED_BREATHING;
	case RATBAG_LED_CYCLE:
		return PA_LED_WAVE;
	default:
		return -1;
	}
}

static unsigned int
pa_speed_to_ms(unsigned int speed)
{
	return PA_SPEED_MS_BASE - speed * PA_SPEED_MS_STEP;
}

static unsigned int
pa_ms_to_speed(unsigned int ms)
{
	int speed = (PA_SPEED_MS_BASE - (int)ms + PA_SPEED_MS_STEP / 2) / PA_SPEED_MS_STEP;

	return min(max(speed, PA_SPEED_MIN), PA_SPEED_MAX);
}

static unsigned int
pa_percent_to_brightness(unsigned int percent)
{
	return (percent * 255 + 50) / 100;
}

static void
pa_fill_led(struct ratbag_led *led, const uint8_t *c1)
{
	enum ratbag_led_mode mode;
	const uint8_t *rgb = &c1[PA_C1_COLOR_0 + led->index * 3];

	if (!pa_effect_to_mode(c1[PA_C1_EFFECT], &mode))
		mode = RATBAG_LED_ON;
	led->mode = mode;
	led->colordepth = RATBAG_LED_COLORDEPTH_RGB_888;
	led->color.red = rgb[0];
	led->color.green = rgb[1];
	led->color.blue = rgb[2];
	led->brightness = pa_percent_to_brightness(c1[PA_C1_BRIGHTNESS]);
	led->ms = pa_speed_to_ms(min(max(c1[PA_C1_SPEED], PA_SPEED_MIN), PA_SPEED_MAX));

	ratbag_led_set_mode_capability(led, RATBAG_LED_OFF);
	ratbag_led_set_mode_capability(led, RATBAG_LED_ON);
	ratbag_led_set_mode_capability(led, RATBAG_LED_BREATHING);
	ratbag_led_set_mode_capability(led, RATBAG_LED_CYCLE);
}

static void
pa_fill_button(struct ratbag_button *button, const uint8_t *fns,
	       const uint8_t *entries)
{
	uint8_t fn = fns[button->index];
	const uint8_t *e = &entries[button->index * PA_ENTRY_SIZE];
	struct ratbag_button_action *action = &button->action;

	ratbag_button_enable_action_type(button, RATBAG_BUTTON_ACTION_TYPE_NONE);
	ratbag_button_enable_action_type(button, RATBAG_BUTTON_ACTION_TYPE_BUTTON);
	ratbag_button_enable_action_type(button, RATBAG_BUTTON_ACTION_TYPE_SPECIAL);
	ratbag_button_enable_action_type(button, RATBAG_BUTTON_ACTION_TYPE_KEY);

	/* anything that is not understood is kept as it is on writes */
	action->type = RATBAG_BUTTON_ACTION_TYPE_UNKNOWN;

	if (fn == PA_FN_NONE && !e[0] && !e[1] && !e[2]) {
		action->type = RATBAG_BUTTON_ACTION_TYPE_NONE;
		return;
	}

	for (size_t i = 0; i < ARRAY_LENGTH(pa_functions); i++) {
		if (pa_functions[i].fn != fn)
			continue;
		action->type = RATBAG_BUTTON_ACTION_TYPE_SPECIAL;
		action->action.special = pa_functions[i].special;
		return;
	}

	if (fn != PA_FN_ORDINARY && fn != PA_FN_DEFAULT)
		return;

	if (e[0] == PA_ENTRY_MOUSE) {
		for (size_t i = 0; i < ARRAY_LENGTH(pa_mouse_specials); i++) {
			if (pa_mouse_specials[i].arg != e[1])
				continue;
			action->type = RATBAG_BUTTON_ACTION_TYPE_SPECIAL;
			action->action.special = pa_mouse_specials[i].special;
			return;
		}
		if (e[1] >= 1 && e[1] <= 5 && e[2] == 0) {
			action->type = RATBAG_BUTTON_ACTION_TYPE_BUTTON;
			action->action.button = e[1];
		}
	} else if (e[0] == PA_ENTRY_KEY) {
		unsigned int usage = e[2];
		unsigned int key;

		/* a single modifier is `01 <mask> 00`, shortcuts stay unknown */
		if (e[1]) {
			unsigned int bit;

			if (e[2])
				return;
			for (bit = 0; bit < 8; bit++) {
				if (e[1] == 1 << bit)
					break;
			}
			if (bit == 8)
				return;
			usage = PA_HID_MODIFIER_0 + bit;
		}

		key = ratbag_hidraw_get_keycode_from_keyboard_usage(button->profile->device, usage);

		if (key) {
			action->type = RATBAG_BUTTON_ACTION_TYPE_KEY;
			action->action.key = key;
		}
	}
}

static void
pa_fill_profile(struct ratbag_profile *profile, struct pa_profile *p)
{
	struct ratbag_resolution *res;
	struct ratbag_button *button;
	struct ratbag_led *led;
	const uint8_t *c0 = pa_chunk_data(p->chunks[PA_CHUNK_SETTINGS], PA_CHUNK_SETTINGS);
	const uint8_t *c1 = pa_chunk_data(p->chunks[PA_CHUNK_LIGHTING], PA_CHUNK_LIGHTING);
	const uint8_t *fns = pa_chunk_data(p->chunks[PA_CHUNK_FUNCTIONS], PA_CHUNK_FUNCTIONS);
	const uint8_t *entries = pa_chunk_data(p->chunks[PA_CHUNK_BUTTONS], PA_CHUNK_BUTTONS);
	unsigned int rate_idx = c0[PA_C0_POLLING_RATE];

	ratbag_profile_set_report_rate_list(profile, pa_report_rates,
					    ARRAY_LENGTH(pa_report_rates));
	if (rate_idx < ARRAY_LENGTH(pa_report_rates))
		profile->hz = pa_report_rates[ARRAY_LENGTH(pa_report_rates) - 1 - rate_idx];

	profile->angle_snapping = c0[PA_C0_ANGLE_SNAPPING];

	ratbag_profile_for_each_resolution(profile, res) {
		unsigned int dpi = pa_dpi_stage(c0, res->index);

		ratbag_resolution_set_resolution(res, dpi, dpi);
		ratbag_resolution_set_dpi_list_from_range(res, PA_DPI_MIN, PA_DPI_MAX);
		ratbag_resolution_set_cap(res, RATBAG_RESOLUTION_CAP_DISABLE);
		if (dpi == 0)
			res->is_disabled = true;
		res->is_active = (res->index == c0[PA_C0_ACTIVE_STAGE]);
	}

	ratbag_profile_for_each_button(profile, button)
		pa_fill_button(button, fns, entries);

	ratbag_profile_for_each_led(profile, led)
		pa_fill_led(led, c1);
}

/* Change the bytes of chunk 0 that libratbag owns. */
static void
pa_apply_settings(struct ratbag_profile *profile, uint8_t *c0)
{
	struct ratbag_resolution *res;
	int rate_idx;

	if (profile->rate_dirty) {
		rate_idx = pa_rate_to_index(profile->hz);
		if (rate_idx >= 0)
			c0[PA_C0_POLLING_RATE] = rate_idx;
	}

	if (profile->angle_snapping_dirty)
		c0[PA_C0_ANGLE_SNAPPING] = !!profile->angle_snapping;

	ratbag_profile_for_each_resolution(profile, res) {
		unsigned int off = PA_C0_DPI_STAGE_0 + res->index * 2;
		unsigned int raw;

		if (!res->dirty)
			continue;

		/* the firmware moves the active stage itself, only follow libratbag on a change */
		if (res->is_active)
			c0[PA_C0_ACTIVE_STAGE] = res->index;

		raw = res->is_disabled ? 0 : res->dpi_x / PA_DPI_STEP;
		c0[off] = raw & 0xff;
		c0[off + 1] = raw >> 8;
	}
}

/* Change the bytes of chunk 1 that libratbag owns. */
static void
pa_apply_lighting(struct ratbag_profile *profile, uint8_t *c1)
{
	struct ratbag_led *led;
	enum ratbag_led_mode shown_mode;
	unsigned int shown_brightness = pa_percent_to_brightness(c1[PA_C1_BRIGHTNESS]);
	uint8_t effect_before = c1[PA_C1_EFFECT];

	/*
	 * The effect, speed and brightness are shared by both zones. Compare
	 * every LED with what was shown before the edit, not with the byte an
	 * earlier LED has already changed, or the second LED would undo it.
	 */
	if (!pa_effect_to_mode(effect_before, &shown_mode))
		shown_mode = RATBAG_LED_ON;

	ratbag_profile_for_each_led(profile, led) {
		uint8_t *rgb = &c1[PA_C1_COLOR_0 + led->index * 3];
		int effect;

		if (!led->dirty)
			continue;

		rgb[0] = led->color.red;
		rgb[1] = led->color.green;
		rgb[2] = led->color.blue;

		/* the byte is a percent: only touch it if libratbag's value differs from what we showed */
		if (shown_brightness != led->brightness)
			c1[PA_C1_BRIGHTNESS] = min((led->brightness * 100 + 127) / 255, 100u);

		/* one effect and speed for both zones, keep effects we do not know */
		effect = pa_mode_to_effect(led->mode);
		if (effect >= 0 && shown_mode != led->mode)
			c1[PA_C1_EFFECT] = effect;
		if (led->mode == RATBAG_LED_BREATHING || led->mode == RATBAG_LED_CYCLE)
			c1[PA_C1_SPEED] = pa_ms_to_speed(led->ms);
	}
}

/* Encode a libratbag action, returns false if it is not supported. */
static bool
pa_encode_action(struct ratbag_device *device,
		 const struct ratbag_button_action *action,
		 uint8_t *fn, uint8_t *entry)
{
	unsigned int usage;

	memset(entry, 0, PA_ENTRY_SIZE);

	switch (action->type) {
	case RATBAG_BUTTON_ACTION_TYPE_NONE:
		*fn = PA_FN_NONE;
		return true;
	case RATBAG_BUTTON_ACTION_TYPE_BUTTON:
		if (action->action.button < 1 || action->action.button > 5)
			return false;
		*fn = PA_FN_ORDINARY;
		entry[0] = PA_ENTRY_MOUSE;
		entry[1] = action->action.button;
		return true;
	case RATBAG_BUTTON_ACTION_TYPE_KEY:
		usage = ratbag_hidraw_get_keyboard_usage_from_keycode(device, action->action.key);
		if (!usage)
			return false;
		*fn = PA_FN_ORDINARY;
		entry[0] = PA_ENTRY_KEY;
		if (usage >= PA_HID_MODIFIER_0 && usage < PA_HID_MODIFIER_0 + 8)
			entry[1] = 1 << (usage - PA_HID_MODIFIER_0);
		else
			entry[2] = usage;
		return true;
	case RATBAG_BUTTON_ACTION_TYPE_SPECIAL:
		for (size_t i = 0; i < ARRAY_LENGTH(pa_functions); i++) {
			if (pa_functions[i].special != action->action.special)
				continue;
			*fn = pa_functions[i].fn;
			return true;
		}
		for (size_t i = 0; i < ARRAY_LENGTH(pa_mouse_specials); i++) {
			if (pa_mouse_specials[i].special != action->action.special)
				continue;
			*fn = PA_FN_ORDINARY;
			entry[0] = PA_ENTRY_MOUSE;
			entry[1] = pa_mouse_specials[i].arg;
			return true;
		}
		return false;
	default:
		return false;
	}
}

static bool
pa_has_dirty_button(const struct ratbag_profile *profile)
{
	const struct ratbag_button *button;

	list_for_each(button, &profile->buttons, link) {
		if (button->dirty)
			return true;
	}

	return false;
}

static bool
pa_has_dirty_led(const struct ratbag_profile *profile)
{
	const struct ratbag_led *led;

	list_for_each(led, &profile->leds, link) {
		if (led->dirty)
			return true;
	}

	return false;
}

/*
 * The core also marks the old and the new active profile dirty when the
 * active profile changes. That needs only the switch command, not a rewrite.
 */
static bool
pa_has_changes(const struct ratbag_profile *profile)
{
	const struct ratbag_resolution *resolution;

	if (profile->rate_dirty || profile->angle_snapping_dirty ||
	    profile->debounce_dirty)
		return true;

	list_for_each(resolution, &profile->resolutions, link) {
		if (resolution->dirty)
			return true;
	}

	return pa_has_dirty_led(profile) || pa_has_dirty_button(profile);
}

static int
pa_write_buttons(struct ratbag_profile *profile)
{
	struct ratbag_device *device = profile->device;
	struct ratbag_button *button;
	uint8_t fn_reply[PA_REPORT_SIZE], entry_reply[PA_REPORT_SIZE];
	uint8_t *fns = pa_chunk_data(fn_reply, PA_CHUNK_FUNCTIONS);
	uint8_t *entries = pa_chunk_data(entry_reply, PA_CHUNK_BUTTONS);
	uint8_t es_reply[PA_REPORT_SIZE];
	uint8_t *es = pa_chunk_data(es_reply, PA_CHUNK_BUTTONS_EASY_SHIFT);
	int rc;

	rc = pa_read_chunk(device, PA_CHUNK_FUNCTIONS, fn_reply);
	if (rc)
		return rc;
	rc = pa_read_chunk(device, PA_CHUNK_BUTTONS, entry_reply);
	if (rc)
		return rc;
	rc = pa_read_chunk(device, PA_CHUNK_BUTTONS_EASY_SHIFT, es_reply);
	if (rc)
		return rc;

	ratbag_profile_for_each_button(profile, button) {
		uint8_t fn, entry[PA_ENTRY_SIZE];

		if (!button->dirty)
			continue;

		/* unknown actions (macros, shortcuts, ...) stay as they are */
		if (!pa_encode_action(device, &button->action, &fn, entry)) {
			log_debug(device->ratbag, "button %u: action not supported, keeping it\n",
				  button->index);
			continue;
		}

		fns[button->index] = fn;
		memcpy(&entries[button->index * PA_ENTRY_SIZE], entry, PA_ENTRY_SIZE);
	}

	rc = pa_write_chunk(device, PA_CHUNK_FUNCTIONS, fns);
	if (rc)
		return rc;

	rc = pa_write_chunk(device, PA_CHUNK_BUTTONS, entries);
	if (rc)
		return rc;

	/*
	 * The vendor software always ends the button chunks with chunk 4 (the
	 * Easy-Shift table). The mouse acknowledges chunks 2 and 3 alone but
	 * does not keep them, so write chunk 4 back unchanged.
	 */
	return pa_write_chunk(device, PA_CHUNK_BUTTONS_EASY_SHIFT, es);
}

static int
pa_write_profile(struct ratbag_profile *profile)
{
	struct ratbag_device *device = profile->device;
	uint8_t reply[PA_REPORT_SIZE];
	uint8_t *c0 = pa_chunk_data(reply, PA_CHUNK_SETTINGS);
	uint8_t c1_reply[PA_REPORT_SIZE];
	uint8_t *c1 = pa_chunk_data(c1_reply, PA_CHUNK_LIGHTING);
	int rc;

	/* the device numbers profiles from 1 */
	rc = pa_select_profile(device, profile->index + 1);
	if (rc)
		return rc;

	rc = pa_read_chunk(device, PA_CHUNK_SETTINGS, reply);
	if (rc)
		return rc;

	/* never write a chunk 0 that belongs to another profile */
	if (c0[PA_C0_PROFILE_INDEX] != profile->index + 1) {
		log_error(device->ratbag,
			  "chunk 0 is for profile %u, expected %u, not writing\n",
			  c0[PA_C0_PROFILE_INDEX], profile->index + 1);
		return -EIO;
	}

	pa_apply_settings(profile, c0);

	/*
	 * The debounce time is read back from a different place than it is
	 * written to: carry the value over, or it would be reset to 0.
	 */
	c0[PA_C0_DEBOUNCE] = reply[PA_C0_DEBOUNCE_READ];

	/* chunk 0 also selects the profile that chunks 1..4 are written to */
	rc = pa_write_chunk(device, PA_CHUNK_SETTINGS, c0);
	if (rc)
		return rc;

	if (pa_has_dirty_led(profile)) {
		rc = pa_read_chunk(device, PA_CHUNK_LIGHTING, c1_reply);
		if (rc)
			return rc;
		pa_apply_lighting(profile, c1);
		rc = pa_write_chunk(device, PA_CHUNK_LIGHTING, c1);
		if (rc)
			return rc;
	}

	if (pa_has_dirty_button(profile))
		return pa_write_buttons(profile);

	return 0;
}

static int
pa_commit(struct ratbag_device *device)
{
	struct ratbag_profile *profile, *active = NULL;
	bool wrote = false;
	int rc = 0, rc_select = 0;

	ratbag_device_for_each_profile(device, profile) {
		if (!profile->dirty || !pa_has_changes(profile))
			continue;

		log_debug(device->ratbag, "profile %u changed, rewriting\n",
			  profile->index);

		wrote = true;
		rc = pa_write_profile(profile);
		if (rc)
			break;
	}

	/* leave the read selector on the active profile, like the vendor software */
	if (wrote) {
		ratbag_device_for_each_profile(device, profile) {
			if (profile->is_active)
				active = profile;
		}
		if (active)
			rc_select = pa_select_profile(device, active->index + 1);
	}

	return rc ? rc : rc_select;
}

static int
pa_set_active_profile(struct ratbag_device *device, unsigned int index)
{
	return pa_write_byte(device, PA_CMD_SET_ACTIVE_PROFILE, index + 1);
}

static int
pa_test_hidraw(struct ratbag_device *device)
{
	/* only the vendor interface speaks the settings protocol */
	return ratbag_hidraw_has_vendor_page(device);
}

static int
pa_probe(struct ratbag_device *device)
{
	struct ratbag_profile *profile;
	uint8_t reply[PA_REPORT_SIZE];
	unsigned int active_idx;
	int rc;

	rc = ratbag_find_hidraw(device, pa_test_hidraw);
	if (rc)
		return rc;

	/*
	 * The first profile read returns the profile that is currently
	 * active on the mouse. The wireless mouse does not answer while it
	 * sleeps or is switched off.
	 */
	rc = pa_read_chunk(device, PA_CHUNK_SETTINGS, reply);
	if (rc) {
		/* not an error: a sleeping mouse is skipped, not reported as broken */
		log_info(device->ratbag, "no answer from the mouse (%d)\n", rc);
		rc = -ENODEV;
		goto err_close;
	}
	active_idx = pa_chunk_data(reply, PA_CHUNK_SETTINGS)[PA_C0_PROFILE_INDEX];
	if (active_idx < 1 || active_idx > PA_NUM_PROFILES) {
		log_error(device->ratbag, "unexpected active profile %u, assuming 1\n",
			  active_idx);
		active_idx = 1;
	}

	ratbag_device_init_profiles(device, PA_NUM_PROFILES, PA_NUM_DPI_STAGES,
				    PA_NUM_BUTTONS, PA_NUM_LEDS);

	ratbag_device_for_each_profile(device, profile) {
		struct pa_profile p;

		/* the device numbers profiles from 1 */
		rc = pa_select_profile(device, profile->index + 1);
		if (rc)
			goto err_close;

		rc = pa_read_profile_raw(device, &p);
		if (rc)
			goto err_close;

		pa_fill_profile(profile, &p);
		profile->is_active = (profile->index + 1 == active_idx);
	}

	/* leave the read selector where we found it, like the vendor software */
	rc = pa_select_profile(device, active_idx);
	if (rc)
		goto err_close;

	return 0;

err_close:
	ratbag_close_hidraw(device);
	return rc;
}

static void
pa_remove(struct ratbag_device *device)
{
	ratbag_close_hidraw(device);
}

struct ratbag_driver roccat_kone_pro_air_driver = {
	.name = "Roccat Kone Pro Air",
	.id = "roccat-kone-pro-air",
	.probe = pa_probe,
	.remove = pa_remove,
	.commit = pa_commit,
	.set_active_profile = pa_set_active_profile,
};
