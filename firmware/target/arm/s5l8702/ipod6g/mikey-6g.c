/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Source     |       _//  _ \_/ ___\|  |/ /| __ \ /  _ \  \/  /
 *   Jukebox    |    |   (  <_> )  \___|    < | \_\ (  <_> > <  <
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 *
 * Copyright (C) 2026 by Hemant Kumar
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This software is distributed on an "AS IS" basis, WITHOUT WARRANTY OF ANY
 * KIND, either express or implied.
 *
 ****************************************************************************/

/* "Mikey" is the internal controller for the headphone jack microphone and
 * the inline earphone remote (I2C bus 0, address 0x72).
 *
 * Register protocol (reverse engineered on-device, 2026-07):
 *   reg0 = 0x2f     remote-reporting mode. The low 3 bits are the mic bias
 *                   level (7, same as the recording path uses); 0x28 arms
 *                   the button detection engine. The chip resets on jack
 *                   removal (all registers read zero afterwards), so the
 *                   mode is re-armed on insertion.
 *   reg4 & 0x05     center (play/pause) held; a level signal on a 0x40
 *                   base. Can flicker while the mode is (re)armed, hence
 *                   the "armed" guard below.
 *   reg5            volume edge events, each readable for ~100ms:
 *                   0x04 vol+ press, 0x08 vol+ release,
 *                   0x01 vol- press, 0x02 vol- release.
 *                   0x30 is the accessory-ID event (ignored).
 * The remote itself signals volume as static DC loads on the mic line and
 * identifies via a one-time ultrasonic chirp (the same scheme David Carne
 * documented for the shuffle 3G remote); Mikey does that level and chirp
 * detection in hardware and reports the results as above. */

#include "system.h"
#include "cpu.h"
#include "kernel.h"
#include "thread.h"
#include "button.h"
#include "i2c-s5l8702.h"
#include "mikey-target.h"

#define MIKEY_ADDR      0x72
#define MIKEY_REG_MODE  0       /* mic bias + detection-engine mode */
#define MIKEY_REG_BTN   4       /* center-button level */
#define MIKEY_REG_EVT   5       /* volume press/release edge events */

#define MIKEY_MODE_OFF      0x05    /* reset default, mic line unpowered */
#define MIKEY_MODE_MIC      0x07    /* mic bias raised, for recording */
#define MIKEY_MODE_REMOTE   0x2f    /* mic bias + button detection */

#define MIKEY_BTN_CENTER    0x05
#define MIKEY_EVT_VOLUP_DN  0x04
#define MIKEY_EVT_VOLUP_UP  0x08
#define MIKEY_EVT_VOLDN_DN  0x01
#define MIKEY_EVT_VOLDN_UP  0x02

/* A press+release pair landing in a single poll is stretched over this
 * many polls, so the button driver's debounce (two identical consecutive
 * reads at the 10ms button tick) still accepts the click. */
#define MIKEY_CLICK_POLLS   2

/* Center-button handling: click-only. reg4's rising edge is always
 * immediate, but after ~5s of button inactivity the chip's event engine
 * naps and reports the release (reg4 fall, reg6 events) ~1.8s late, so
 * the press duration is unreliable. Every press is therefore reported
 * as a fixed-length click at the rise; there is no hold/long-press. A
 * release must be seen for several consecutive polls before a new rise
 * counts, so bounce or an I2C NAK can't double-fire a click. */
#define MIKEY_CENTER_PULSE_POLLS  3     /* reported click length, 60ms */
#define MIKEY_CENTER_OFF_POLLS    3     /* release debounce, 60ms */

unsigned char mikey_read(int address)
{
    /* default to "no press, no events" if the transfer fails: the chip
     * NAKs while it resets itself around jack removal/insertion */
    unsigned char val = 0;
    i2c_read(0, MIKEY_ADDR, address, 1, &val);
    return val;
}

int mikey_write(int address, unsigned char val)
{
    return i2c_write(0, MIKEY_ADDR, address, 1, &val);
}

void mikey_reset(void)
{
    mikey_write(MIKEY_REG_MODE, MIKEY_MODE_OFF);
    mikey_write(1, 0x80);
}

static int mikey_btn = BUTTON_NONE;
static volatile bool mic_active = false;
static long mikey_stack[DEFAULT_STACK_SIZE/2/sizeof(long)];

/* The recording path owns the chip while the jack mic is enabled; the
 * polling thread backs off and re-arms the remote mode afterwards. */
void mikey_set_mic_capture(bool enable)
{
    mic_active = enable;
    if (enable)
        mikey_write(MIKEY_REG_MODE, MIKEY_MODE_MIC);
    else
        mikey_reset();
}

/* Decode one volume button's press/release edge events into a held state.
 * A real hold only produces the two edges, so between them *held carries
 * the state; *click stretches a same-poll press+release (see above). */
static void mikey_decode_vol(unsigned char evt, unsigned char dn,
                             unsigned char up, bool *held, int *click)
{
    if (*click > 0 && --*click == 0)
        *held = false;

    if (evt & dn)
    {
        *held = true;
        *click = (evt & up) ? MIKEY_CLICK_POLLS : 0;
    }
    else if ((evt & up) && *click == 0)
        *held = false;
}

static void mikey_thread(void)
{
    bool powered = false;
    bool armed = false;   /* need a released reading before first press */
    bool vol_up = false, vol_dn = false;
    int up_click = 0, dn_click = 0;
    bool center_down = false;   /* debounced "press already reported" */
    int center_off = MIKEY_CENTER_OFF_POLLS;   /* consecutive released */
    int center_pulse = 0;       /* click pulse countdown */

    while (1)
    {
        if (!headphones_inserted() || mic_active)
        {
            if (powered && !mic_active)
                mikey_reset();
            powered = false;
            armed = false;
            vol_up = vol_dn = false;
            up_click = dn_click = 0;
            center_down = false;
            center_off = MIKEY_CENTER_OFF_POLLS;
            center_pulse = 0;
            mikey_btn = BUTTON_NONE;
            sleep(HZ/2);   /* nothing to poll, check back at leisure */
            continue;
        }
        powered = true;

        /* Re-arm when the mode readback disagrees (first pass, chip reset
         * on jack removal, or the recording path rewrote it). Arming can
         * flicker reg4 and emits a spurious accessory-ID event, so drop
         * the state and settle for one poll. */
        if (mikey_read(MIKEY_REG_MODE) != MIKEY_MODE_REMOTE)
        {
            mikey_write(MIKEY_REG_MODE, MIKEY_MODE_REMOTE);
            armed = false;
            vol_up = vol_dn = false;
            up_click = dn_click = 0;
            center_down = false;
            center_off = MIKEY_CENTER_OFF_POLLS;
            center_pulse = 0;
            mikey_btn = BUTTON_NONE;
            sleep(HZ/50);
            continue;
        }

        unsigned char evt = mikey_read(MIKEY_REG_EVT);
        mikey_decode_vol(evt, MIKEY_EVT_VOLUP_DN, MIKEY_EVT_VOLUP_UP,
                         &vol_up, &up_click);
        mikey_decode_vol(evt, MIKEY_EVT_VOLDN_DN, MIKEY_EVT_VOLDN_UP,
                         &vol_dn, &dn_click);

        /* reg4 can read pressed while the mode settles; don't report the
         * center button until it has been seen released once */
        bool raw = (mikey_read(MIKEY_REG_BTN) & MIKEY_BTN_CENTER) != 0;
        if (!raw)
            armed = true;

        /* click-only: emit a fixed pulse at each debounced rise */
        if (raw && armed)
        {
            if (!center_down && center_off >= MIKEY_CENTER_OFF_POLLS)
                center_pulse = MIKEY_CENTER_PULSE_POLLS;
            center_down = true;
            center_off = 0;
        }
        else
        {
            if (center_off < MIKEY_CENTER_OFF_POLLS)
                center_off++;
            if (center_off >= MIKEY_CENTER_OFF_POLLS)
                center_down = false;
        }

        bool center = false;
        if (center_pulse > 0)
        {
            center = true;
            center_pulse--;
        }

        /* center reports as a multimedia key: handled globally by
         * default_event_handler on every screen (play/pause/resume),
         * so it can never act as select like BUTTON_RC_PLAY would */
        mikey_btn = (center ? BUTTON_MULTIMEDIA_PLAYPAUSE : 0)
                  | (vol_up ? BUTTON_RC_VOL_UP : 0)
                  | (vol_dn ? BUTTON_RC_VOL_DOWN : 0);

        /* volume edge events stay readable for ~100ms; 20ms leaves a
         * comfortable margin against scheduling jitter */
        sleep(HZ/50);
    }
}

int mikey_button_read(void)
{
    return mikey_btn;
}

void mikey_init(void)
{
    mikey_reset();
    create_thread(mikey_thread, mikey_stack, sizeof(mikey_stack), 0,
                  "mikey" IF_PRIO(, PRIORITY_SYSTEM) IF_COP(, CPU));
}
