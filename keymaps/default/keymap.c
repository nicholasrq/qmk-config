#include QMK_KEYBOARD_H
#include "version.h"
#include "features/achordion.h"
#include "keymaps/colemakdh.h"
#include "layout.h" // Neat layout remap
#include "keys.h" // Macros for mod keys

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
    TX_PR_SESS,
    TX_NX_SESS,
    TX_SEL,
    TX_ZOOM,
    TX_SESSIONS,
    TX_PR_WIN,
    TX_NX_WIN,
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
    _SYM,
    _TMUX,
};

#define THUMB_SPACE LT(2, KC_SPACE)
#define THUMB_TAB LT(3, KC_TAB)
#define THUMB_ENTER LT(4, KC_ENTER)

#define HOME_A MT(MOD_LGUI, KC_A)
#define HOME_S MT(MOD_LALT, KC_S)
#define HOME_D MT(MOD_LCTL, KC_D)
#define HOME_F MT(MOD_LSFT, KC_F)

#define HOME_J MT(MOD_RSFT, KC_J)
#define HOME_K MT(MOD_RCTL, KC_K)
#define HOME_L MT(MOD_LALT, KC_L)
#define HOME_SEMI MT(MOD_RGUI, KC_SCLN)

// Symbol layer
#define HOME_V LT(_SYM, KC_V)
#define HOME_N LT(_SYM, KC_H)

#define LT1 LT(_TMUX,KC_SPACE)
#define LT2 LT(_NUM,KC_TAB)

#define RT1 LT(_TMUX,KC_BSPC)
#define RT2 LT(_NUM,KC_ENTER)

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
/**
* Enter on NUM layer
*/
    [_DEFAULT] = LAYOUT_LR(
        KC_BRMD,        KC_BRMU,        _______,        _______,        _______,        _______,
        KC_TILD,        KC_Q,           KC_W,           KC_E,           KC_R,           KC_T,
        KC_ESCAPE,      HOME_A,         HOME_S,         HOME_D,         HOME_F,         KC_G,
        CW_TOGG,        KC_Z,           KC_X,           KC_C,           HOME_V,         KC_B,
                                                                                        LT1,            LT2,

                        KC_VOLD,        KC_VOLU,        KC_MUTE,        KC_MPLY,        KC_MNXT,        MAC_LOCK,
                        KC_Y,           KC_U,           KC_I,           KC_O,           KC_P,           KC_LBRC,
                        KC_H,         HOME_J,         HOME_K,         HOME_L,         HOME_SEMI,      KC_QUOTE,
                        HOME_N,           KC_M,           KC_COMMA,       KC_DOT,         KC_SLASH,     CW_TOGG,
        RT2,            RT1
    ),
    [_NUM] = LAYOUT_LR(
        /// Left
        RGB_MODE_FORWARD,RGB_TOG,       RGB_VAD,        RGB_VAI,        _______,        _______,
        _______,        _______,        _______,        _______,        _______,        _______,
        _______,        KC_LGUI,        KC_LALT,        KC_LCTL,        KC_LSFT,        _______,
        _______,        _______,        _______,        _______,        _______,        _______,
                                                                                        KC_TRNS,        KC_TRNS,

        /// Right
                        _______,        _______,        _______,        _______,        _______,        _______,
                        KC_LBRC,        KC_7,           KC_8,           KC_9,           KC_0,           KC_RBRC,
                        KC_COLN,        KC_4,           KC_5,           KC_6,           KC_MINUS,       KC_EQUAL,
                        KC_COMM,        KC_1,           KC_2,           KC_3,           KC_PLUS,        KC_ENTER,
        KC_TRNS,        KC_TRNS
    ),
    [_SYM] = LAYOUT_LR(
        /// Left
        _______,        _______,        _______,        _______,        _______,        _______,
        _______,        KC_HASH,        KC_AT,          KC_EQL,         KC_LPAR,        KC_RPAR,
        _______,        KC_BTI,         KC_UNDS,        KC_AMP,         KC_LBRC,        KC_RBRC,
        _______,        _______,        KC_MINUS,       KC_DLR,         KC_LCUR,        KC_RCUR,
                                                                                        KC_TRNS,        KC_TRNS,

        /// Right
                        _______,        _______,        _______,        _______,        _______,        _______,
                        KC_COLN,        KC_LANG,        KC_EQL,         KC_RANG,        _______,        _______,
                        KC_SCLN,        KC_AMP,         KC_AST,         KC_BSLS,        KC_SLASH,        _______,
                        KC_GRAVE,       KC_PIPE,        KC_POW,         KC_PERC,        _______,        _______,
        COPY,           PASTE
    ),
    [_TMUX] = LAYOUT_LR(
        /// Left
        _______,        _______,        _______,        KC_MS_BTN1,     KC_MS_BTN2,     KC_MS_BTN3,
        _______,        TX_PR_WIN,      TX_ZOOM,        TX_SEL,         TX_SESSIONS,    TX_NX_WIN,
        _______,        KC_LGUI,        KC_LALT,        KC_LCTL,        KC_LSFT,        _______,
        _______,        _______,        _______,        _______,        _______,        _______,
                                                                                        KC_TRNS,        KC_TRNS,

        /// Right
                        _______,        _______,        _______,          _______,          _______,          _______,
                        _______,        _______,        _______,          _______,          _______,          _______,
                        KC_LEFT,        KC_DOWN,        KC_UP,            KC_RIGHT,         _______,          _______,
                        _______,        _______,        _______,          _______,          _______,          _______,
        KC_TRNS,        KC_TRNS
    ),
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
            set_layer_color(_TMUX);
            break;
        case 3:
            set_layer_color(_NUM);
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
        case TX_PR_SESS:
            if (record->event.pressed) {
                tmux_macro(LSFT(KC_9));
            }
            break;
        case TX_NX_SESS:
            if (record->event.pressed) {
                tmux_macro(LSFT(KC_0));
            }
            break;
        case TX_SEL:
            if (record->event.pressed) {
                tmux_macro(KC_LBRC);
            }
            break;
        case TX_ZOOM:
            if (record->event.pressed) {
                tmux_macro(CM_Z);
            }
            break;
        case TX_SESSIONS:
            if (record->event.pressed) {
                tmux_macro(LSFT(CM_T));
            }
            break;
        case TX_PR_WIN:
            if (record->event.pressed) {
                tmux_macro(CM_P);
            }
            break;
        case TX_NX_WIN:
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

bool caps_word_press_user(uint16_t keycode) {
    switch(keycode) {
        // Characters
        case KC_A ... KC_Z:
        // Exception for Colemak
        case CM_O:
            add_weak_mods(MOD_BIT(KC_LSFT));
            return true;
        // Numbers
        case KC_1 ... KC_0:
            return true;
        // Symbols to ignore in CAPS_WORD
        case KC_BSPC:
        case KC_DEL:
        case KC_UNDS:
            return true;

        default:
            return false;
    }
}
