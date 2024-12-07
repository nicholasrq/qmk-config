#include QMK_KEYBOARD_H
#include "version.h"
#include "features/achordion.h"
#include "keymap_steno.h"
#include "keymaps/colemakdh.h"

// remap for DH
#undef CM_B
#undef CM_G
#undef CM_D
#undef CM_H
#undef CM_M
#define CM_B  KC_T
#define CM_G  KC_G
#define CM_D  KC_V
#define CM_H  KC_M
#define CM_M  KC_H

#define MOON_LED_LEVEL LED_LEVEL
#define ML_SAFE_RANGE SAFE_RANGE

enum custom_keycodes {
    RGB_SLD = ML_SAFE_RANGE,
    MACRO_P_SESS,
    MACRO_N_SESS,
    MACRO_SELECT,
    MACRO_ZOOM,
    MACRO_SESSIONS,
    MACRO_P_WIN,
    MACRO_N_WIN,
    ST_MACRO_7,
    ST_MACRO_8,
    MAC_LOCK,
};

enum tap_dance_codes {
    DANCE_UNDS,
    DANCE_CAR,
    DANCE_PER,
    DANCE_DLR,
};

enum layers {
    _DEFAULT,
    _NUM,
    _TMUX,
};

#define THUMB_SPACE LT(2, KC_SPACE)
#define THUMB_TAB LT(3, KC_TAB)
#define THUMB_ENTER LT(4, KC_ENTER)

#define MT_LGUI MT(MOD_LGUI, KC_A)
#define MT_LALT MT(MOD_LALT, KC_S)
#define MT_LCTL MT(MOD_LCTL, KC_D)
#define MT_LSFT MT(MOD_LSFT, KC_F)

#define MT_RSFT MT(MOD_RSFT, KC_J)
#define MT_RCTL MT(MOD_RCTL, KC_K)
#define MT_RALT MT(MOD_LALT, KC_L)
#define MT_RGUI MT(MOD_RGUI, KC_SCLN)


const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
    [_DEFAULT] = LAYOUT_voyager(
        KC_NO,          KC_NO,          KC_NO,          KC_NO,          KC_NO,          KC_NO,                                          KC_NO,          KC_NO,          KC_NO,          KC_NO,          KC_NO,          MAC_LOCK,
        KC_TILD,        KC_Q,           KC_W,           KC_E,           KC_R,           KC_T,                                           KC_Y,           KC_U,           KC_I,           KC_O,           KC_P,           KC_BSLS,
        KC_ESCAPE,      MT_LGUI,        MT_LALT,        MT_LCTL,        MT_LSFT,        KC_G,                                           KC_H,           MT_RSFT,        MT_RCTL,        MT_RALT,        MT_RGUI,        KC_QUOTE,
        CW_TOGG,        KC_Z,           KC_X,           KC_C,           KC_V,           KC_B,                                           KC_N,           KC_M,           KC_COMMA,       KC_DOT,         KC_SLASH,       CW_TOGG,
                                                                        LT(_TMUX,KC_SPACE), LT(_NUM,KC_TAB),                            LT(_NUM,KC_ENTER),LT(_TMUX,KC_BSPC)
    ),
    [_NUM] = LAYOUT_voyager(
        RGB_MODE_FORWARD,RGB_TOG,       RGB_VAD,        RGB_VAI,        KC_NO,          KC_NO,                                          KC_NO,          KC_NO,          KC_NO,          KC_NO,          KC_NO,          KC_NO,
        KC_RCBR,        KC_NO,          KC_NO,          KC_NO,          KC_NO,          KC_NO,                                          KC_LBRC,        KC_7,           KC_8,           KC_9,           KC_0,           KC_RBRC,
        KC_PLUS,        KC_LGUI,        KC_LALT,        KC_LCTL,        KC_LSFT,        KC_NO,                                          KC_SCLN,        KC_4,           KC_5,           KC_6,           KC_MINUS,       KC_EQUAL,
        KC_NO,          KC_NO,          KC_NO,          KC_NO,          KC_NO,          KC_NO,                                          KC_GRAVE,       KC_1,           KC_2,           KC_3,           KC_BSLS,        KC_ENTER,
                                                                        KC_TRNS,        KC_TRNS,                                        KC_TRNS,        KC_TRNS
    ),
    [_TMUX] = LAYOUT_voyager(
        KC_NO,          KC_NO,          KC_NO,          KC_MS_BTN1,     KC_MS_BTN2,     KC_MS_BTN3,                                     KC_NO,          KC_NO,          KC_NO,          KC_NO,          KC_NO,          KC_NO,
        KC_NO,          MACRO_P_WIN,    MACRO_ZOOM,     MACRO_SELECT,   MACRO_SESSIONS, MACRO_N_WIN,                                    KC_NO,          KC_NO,          KC_NO,          KC_NO,          KC_NO,          KC_NO,
        KC_NO,          KC_LGUI,        KC_LALT,        KC_LCTL,        KC_LSFT,        KC_NO,                                          KC_LEFT,        KC_DOWN,        KC_UP,          KC_RIGHT,       KC_NO,          KC_NO,
        KC_NO,          KC_NO,          KC_NO,          KC_NO,          KC_NO,          KC_NO,                                          KC_NO,          KC_NO,          KC_NO,          KC_NO,          KC_NO,          KC_NO,
                                                                        KC_TRNS,        KC_TRNS,                                        KC_TRNS,        KC_TRNS
    ),
};

const uint16_t PROGMEM combo0[] = { MT_LCTL, MT_LSFT, COMBO_END};
const uint16_t PROGMEM combo1[] = { MT_RSFT, MT_RCTL, COMBO_END};
const uint16_t PROGMEM combo2[] = { MT_LALT, MT_LCTL, COMBO_END};
const uint16_t PROGMEM combo3[] = { MT_RCTL, MT_RALT, COMBO_END};
const uint16_t PROGMEM combo4[] = { KC_E, KC_R, COMBO_END};
const uint16_t PROGMEM combo5[] = { KC_U, KC_I, COMBO_END};
const uint16_t PROGMEM combo6[] = { KC_M, KC_COMMA, COMBO_END};
const uint16_t PROGMEM combo7[] = { KC_COMMA, KC_DOT, COMBO_END};
const uint16_t PROGMEM combo8[] = { KC_DOT, KC_SLASH, COMBO_END};
const uint16_t PROGMEM combo9[] = { KC_M, KC_SLASH, COMBO_END};
const uint16_t PROGMEM combo10[] = { KC_M, KC_COMMA, KC_DOT, COMBO_END};
const uint16_t PROGMEM combo11[] = { KC_COMMA, KC_SLASH, KC_DOT, COMBO_END};
const uint16_t PROGMEM combo12[] = { KC_X, KC_C, COMBO_END};
const uint16_t PROGMEM combo13[] = { KC_C, KC_V, COMBO_END};

combo_t key_combos[COMBO_COUNT] = {
    COMBO(combo0, CM_LPRN),
    COMBO(combo1, CM_RPRN),
    COMBO(combo2, CM_LBRC),
    COMBO(combo3, CM_RBRC),
    COMBO(combo4, KC_MINUS),
    COMBO(combo5, KC_EQUAL),
    COMBO(combo6, KC_AUDIO_VOL_DOWN),
    COMBO(combo7, KC_AUDIO_VOL_UP),
    COMBO(combo8, KC_MEDIA_PLAY_PAUSE),
    COMBO(combo9, KC_AUDIO_MUTE),
    COMBO(combo10, KC_MEDIA_PREV_TRACK),
    COMBO(combo11, KC_MEDIA_NEXT_TRACK),
    COMBO(combo12, KC_BRMD),
    COMBO(combo13, KC_BRMU),
};

uint16_t get_tapping_term(uint16_t keycode, keyrecord_t *record) {
    switch (keycode) {
        case MT(MOD_LGUI, KC_A):
            return TAPPING_TERM + 20;
        case MT(MOD_LALT, KC_S):
            return TAPPING_TERM + 20;
        case MT(MOD_LCTL, KC_D):
            return TAPPING_TERM + 20;
        case MT(MOD_LSFT, KC_F):
            return TAPPING_TERM + 20;
        case MT(MOD_RSFT, KC_J):
            return TAPPING_TERM + 20;
        case MT(MOD_RCTL, KC_K):
            return TAPPING_TERM + 20;
        case MT(MOD_RALT, KC_L):
            return TAPPING_TERM + 20;
        case MT(MOD_RGUI, KC_SCLN):
            return TAPPING_TERM + 20;
        default:
            return TAPPING_TERM;
    }
}

extern rgb_config_t rgb_matrix_config;

void keyboard_post_init_user(void) {
    rgb_matrix_enable();
}

const uint8_t PROGMEM ledmap[][RGB_MATRIX_LED_COUNT][3] = {
    [2] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 255, 255}, {0, 255, 255}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 255, 255}, {0, 255, 255}, {0, 255, 255}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 255, 255}, {0, 255, 255}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}},

    [3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {171, 224, 181}, {171, 224, 181}, {171, 224, 181}, {171, 224, 181}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {23, 242, 223}, {23, 242, 223}, {23, 242, 223}, {23, 242, 223}, {0, 0, 0}, {0, 0, 0}, {23, 242, 223}, {23, 242, 223}, {23, 242, 223}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {23, 242, 223}, {23, 242, 223}, {23, 242, 223}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}},

    [4] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {197, 235, 186}, {197, 235, 186}, {197, 235, 186}, {197, 235, 186}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {197, 235, 186}, {197, 235, 186}, {197, 235, 186}, {197, 235, 186}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}},
};

void set_layer_color(int layer) {
    for (int i = 0; i < RGB_MATRIX_LED_COUNT; i++) {
        HSV hsv = {
            .h = pgm_read_byte(&ledmap[layer][i][0]),
            .s = pgm_read_byte(&ledmap[layer][i][1]),
            .v = pgm_read_byte(&ledmap[layer][i][2]),
        };
        if (!hsv.h && !hsv.s && !hsv.v) {
            rgb_matrix_set_color(i, 0, 0, 0);
        } else {
            RGB   rgb = hsv_to_rgb(hsv);
            float f   = (float)rgb_matrix_config.hsv.v / UINT8_MAX;
            rgb_matrix_set_color(i, f * rgb.r, f * rgb.g, f * rgb.b);
        }
    }
}

bool rgb_matrix_indicators_user(void) {
    if (rawhid_state.rgb_control) {
        return false;
    }
    if (keyboard_config.disable_layer_led) {
        return false;
    }
    switch (biton32(layer_state)) {
        case 2:
            set_layer_color(2);
            break;
        case 3:
            set_layer_color(3);
            break;
        case 4:
            set_layer_color(4);
            break;
        default:
            if (rgb_matrix_get_flags() == LED_FLAG_NONE) rgb_matrix_set_color_all(0, 0, 0);
            break;
    }
    return true;
}

void matrix_scan_user(void) {
    achordion_task();
}

void tmux_macro(uint16_t keycode) {
    register_code(KC_RCTL);
    tap_code(CM_S);
    unregister_code(KC_RCTL);
    tap_code16(keycode);
}

bool process_record_user(uint16_t keycode, keyrecord_t *record) {
    if (!process_achordion(keycode, record)) {
        return false;
    }
    switch (keycode) {
        case MACRO_P_SESS:
            if (record->event.pressed) {
                tmux_macro(LSFT(KC_9));
            }
            break;
        case MACRO_N_SESS:
            if (record->event.pressed) {
                tmux_macro(LSFT(KC_0));
            }
            break;
        case MACRO_SELECT:
            if (record->event.pressed) {
                tmux_macro(KC_LBRC);
            }
            break;
        case MACRO_ZOOM:
            if (record->event.pressed) {
                tmux_macro(CM_Z);
            }
            break;
        case MACRO_SESSIONS:
            if (record->event.pressed) {
                tmux_macro(LSFT(CM_T));
            }
            break;
        case MACRO_P_WIN:
            if (record->event.pressed) {
                tmux_macro(CM_P);
            }
            break;
        case MACRO_N_WIN:
            if (record->event.pressed) {
                tmux_macro(CM_N);
            }
            break;
        case MAC_LOCK:
            HCS(0x19E);

        case RGB_SLD:
            if (record->event.pressed) {
                rgblight_mode(1);
            }
            return false;
    }
    return true;
}

typedef struct {
    bool    is_press_action;
    uint8_t step;
} tap;

enum { SINGLE_TAP = 1, SINGLE_HOLD, DOUBLE_TAP, DOUBLE_HOLD, DOUBLE_SINGLE_TAP, MORE_TAPS };

static tap dance_state[2];

uint8_t dance_step(tap_dance_state_t *state);

uint8_t dance_step(tap_dance_state_t *state) {
    if (state->count == 1) {
        if (state->interrupted || !state->pressed)
            return SINGLE_TAP;
        else
            return SINGLE_HOLD;
    } else if (state->count == 2) {
        if (state->interrupted)
            return DOUBLE_SINGLE_TAP;
        else if (state->pressed)
            return DOUBLE_HOLD;
        else
            return DOUBLE_TAP;
    }
    return MORE_TAPS;
}


void on_dance_unds(tap_dance_state_t *state, void *user_data);
void dance_unds_finished(tap_dance_state_t *state, void *user_data);
void dance_unds_reset(tap_dance_state_t *state, void *user_data);

void on_dance_unds(tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_UNDS);
        tap_code16(KC_UNDS);
        tap_code16(KC_UNDS);
    }
    if(state->count > 3) {
        tap_code16(KC_UNDS);
    }
}

void dance_unds_finished(tap_dance_state_t *state, void *user_data) {
    dance_state[1].step = dance_step(state);
    switch (dance_state[1].step) {
        case SINGLE_TAP: register_code16(KC_UNDS); break;
        case SINGLE_HOLD: register_code16(KC_LEFT_GUI); break;
        case DOUBLE_TAP: register_code16(KC_UNDS); register_code16(KC_UNDS); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_UNDS); register_code16(KC_UNDS);
    }
}

void dance_unds_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[1].step) {
        case SINGLE_TAP: unregister_code16(KC_UNDS); break;
        case SINGLE_HOLD: unregister_code16(KC_LEFT_GUI); break;
        case DOUBLE_TAP: unregister_code16(KC_UNDS); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_UNDS); break;
    }
    dance_state[1].step = 0;
}

void on_dance_car(tap_dance_state_t *state, void *user_data);
void dance_car_finished(tap_dance_state_t *state, void *user_data);
void dance_car_reset(tap_dance_state_t *state, void *user_data);

void on_dance_car(tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_CIRC);
        tap_code16(KC_CIRC);
        tap_code16(KC_CIRC);
    }
    if(state->count > 3) {
        tap_code16(KC_CIRC);
    }
}

void dance_car_finished(tap_dance_state_t *state, void *user_data) {
    dance_state[1].step = dance_step(state);
    switch (dance_state[1].step) {
        case SINGLE_TAP: register_code16(KC_CIRC); break;
        case SINGLE_HOLD: register_code16(KC_LEFT_ALT); break;
        case DOUBLE_TAP: register_code16(KC_CIRC); register_code16(KC_CIRC); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_CIRC); register_code16(KC_CIRC);
    }
}

void dance_car_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[1].step) {
        case SINGLE_TAP: unregister_code16(KC_CIRC); break;
        case SINGLE_HOLD: unregister_code16(KC_LEFT_ALT); break;
        case DOUBLE_TAP: unregister_code16(KC_CIRC); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_CIRC); break;
    }
    dance_state[1].step = 0;
}

void on_dance_per(tap_dance_state_t *state, void *user_data);
void dance_per_finished(tap_dance_state_t *state, void *user_data);
void dance_per_reset(tap_dance_state_t *state, void *user_data);

void on_dance_per(tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_PERC);
        tap_code16(KC_PERC);
        tap_code16(KC_PERC);
    }
    if(state->count > 3) {
        tap_code16(KC_PERC);
    }
}

void dance_per_finished(tap_dance_state_t *state, void *user_data) {
    dance_state[1].step = dance_step(state);
    switch (dance_state[1].step) {
        case SINGLE_TAP: register_code16(KC_PERC); break;
        case SINGLE_HOLD: register_code16(KC_LEFT_CTRL); break;
        case DOUBLE_TAP: register_code16(KC_PERC); register_code16(KC_PERC); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_PERC); register_code16(KC_PERC);
    }
}

void dance_per_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[1].step) {
        case SINGLE_TAP: unregister_code16(KC_PERC); break;
        case SINGLE_HOLD: unregister_code16(KC_LEFT_CTRL); break;
        case DOUBLE_TAP: unregister_code16(KC_PERC); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_PERC); break;
    }
    dance_state[1].step = 0;
}

bool achordion_chord(uint16_t tap_hold_keycode,
                     keyrecord_t* tap_hold_record,
                     uint16_t other_keycode,
                     keyrecord_t* other_record) {

    if (tap_hold_keycode == THUMB_SPACE || tap_hold_keycode == THUMB_TAB || tap_hold_keycode == THUMB_ENTER || tap_hold_keycode == KC_BSPC) {
        return true;
    }

  return achordion_opposite_hands(tap_hold_record, other_record);
}

void on_dance_dlr(tap_dance_state_t *state, void *user_data);
void dance_dlr_finished(tap_dance_state_t *state, void *user_data);
void dance_dlr_reset(tap_dance_state_t *state, void *user_data);

void on_dance_dlr(tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_DLR);
        tap_code16(KC_DLR);
        tap_code16(KC_DLR);
    }
    if(state->count > 3) {
        tap_code16(KC_DLR);
    }
}

void dance_dlr_finished(tap_dance_state_t *state, void *user_data) {
    dance_state[1].step = dance_step(state);
    switch (dance_state[1].step) {
        case SINGLE_TAP: register_code16(KC_DLR); break;
        case SINGLE_HOLD: register_code16(KC_LEFT_SHIFT); break;
        case DOUBLE_TAP: register_code16(KC_DLR); register_code16(KC_DLR); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_DLR); register_code16(KC_DLR);
    }
}

void dance_dlr_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[1].step) {
        case SINGLE_TAP: unregister_code16(KC_DLR); break;
        case SINGLE_HOLD: unregister_code16(KC_LEFT_SHIFT); break;
        case DOUBLE_TAP: unregister_code16(KC_DLR); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_DLR); break;
    }
    dance_state[1].step = 0;
}

tap_dance_action_t tap_dance_actions[] = {
    [DANCE_UNDS] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_unds, dance_unds_finished, dance_unds_reset),
    [DANCE_CAR] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_car, dance_car_finished, dance_car_reset),
    [DANCE_PER] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_per, dance_per_finished, dance_per_reset),
    [DANCE_DLR] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_dlr, dance_dlr_finished, dance_dlr_reset),
};
