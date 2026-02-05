#include "flight.h"

#include "stdint.h"
#include "math.h"
#include "stdatomic.h"

#include "flight_config.h"
#include "driver_pyro.h"
#include "nvs_interface.h"
#include "command.h"

// static uint8_t flight_state = FS_PREFLIGHT;
static uint8_t flight_state = FS_ON_PAD;

#define APPO PYRO_CHANNEL_1
#define MAINS PYRO_CHANNEL_2

_Atomic flight_config_t flight_config;

void flight_config_init(void) {
    flight_config_t cfg;
    if (nvs_retreive_flight_config(&cfg) != ESP_OK) {
        cfg = (flight_config_t){
            .lift_off_acceleration_threshold = 9.81*3,
            .lift_off_tick_count = 5,
            .apogee_acceleration_threshold = 9.81/2,
            .apogee_tick_count = 5,
            .burnout_acceleration_threshold = 0,
            .burnout_tick_count = 5,
            .recovery_burnout_counter = 1, // This condition will allow us to select the number of stages in multistage rockets
            .arm_at_boot = false,
            .Appo_Channel = PYRO_CHANNEL_1,
            .Mains_Channel = PYRO_CHANNEL_2,
            .Separation_Channel = 0,
            .Ignition_Channel = 0,
            .Aux_1_Channel = 0,
            .Aux_2_Channel = 0,
            .Aux_3_Channel = 0,
            .Aux_4_Channel = 0,
            .panic_velocity_threshold = -240.0f/3.28,
            .main_deployment_altitude = 1000/3.28,
            .main_deployment_tick_count = 5,
            .highest_ground_elevation = 100,
            .landed_velocity_threshold = 2,
            .landed_tick_count = 255,
        };
        nvs_set_flight_config(&cfg);
    }
    atomic_store(&flight_config, cfg);
}

void flight_config_set(flight_config_t *cfg) {
    flight_config_t old_cfg = atomic_load(&flight_config);
    if(cfg->arm_at_boot != old_cfg.arm_at_boot && cfg->arm_at_boot) {
        // activate_txlock();
    }
    else if(cfg->arm_at_boot != old_cfg.arm_at_boot && !cfg->arm_at_boot) {
        // deactivate_txlock();
    }

    atomic_store(&flight_config, *cfg);
}

static bool deploy(pyro_channel_t channel)
{
    bool cont;

    // disable pyros for daq firmware
    
    // for (int i = 0; i < 2; i++) {
    //     cont = pyro_continuity(channel);
    //     if (cont) {
    //         pyro_activate(channel, 150*(i+1), 0);
    //         // vTaskDelay(50 / portTICK_PERIOD_MS);
    //         cont = pyro_continuity(channel);
    //         if (!cont) return true;
    //     }
    // }

    return true;
}

// VS code may say that this is an error bc it can't see APPO_GS but it will compile
#define APPO_COND (average_barometric_velocity < 0 && fabs(xacc) < APPO_GS)

bool flight_update(
    float barometric_agl,
    float barometric_velocity,
    float average_barometric_velocity,
    float xacc
) {
    // this function will be called at 50 Hz durring flight, thus every tick is 20 ms

    static int count1 = 0;
    static int count2 = 0;

    uint8_t pre = flight_state;
    static bool loaded_flight_config = false;
    // Take an atomic snapshot of the config for consistent reads in this tick
    static flight_config_t cfg;
    if (loaded_flight_config == false) {
        cfg = atomic_load(&flight_config);
        loaded_flight_config = true;
    }

    switch (flight_state) {
        case FS_PREFLIGHT:
            // if (should_wake_up()) {
            //     flight_state = FS_ON_PAD;
            // }
            // break;

        case FS_ON_PAD:
            if (xacc > cfg.lift_off_acceleration_threshold) {
                count1++;
            } else {
                count1 = 0;
            }

            if (count1 >= cfg.lift_off_tick_count) {
                flight_state = FS_BOOSTER;
                if(!is_tx_lock()) {
                    enable_txlock();
                    printf("Activated txlock from FSM\n");
                }
            // } else if (!should_wake_up() && count1 == 0) {
            //     flight_state = FS_PREFLIGHT;
            }
            break;

        case FS_BOOSTER:
            if (xacc < cfg.burnout_acceleration_threshold) {
                count1++;
            } else {
                count1 = 0;
            }

            if (count1 >= cfg.burnout_tick_count) {
                flight_state = FS_COAST_BOOSTER;
            }
            break;

        case FS_COAST_BOOSTER:
            if (xacc > cfg.lift_off_acceleration_threshold) {
                count1++;
            } else {
                count1 = 0;
            }

            if (APPO_COND) {
                count2++;
            } else {
                count2 = 0;
            }

            if (count2 >= cfg.apogee_tick_count) {
                deploy(cfg.Appo_Channel);
                printf("Deploy appo\n");
                flight_state = FS_UNDER_DROGUES;
            } else if (count1 >= cfg.lift_off_tick_count) {
                flight_state = FS_SUSTAINER;
            }
            break;

        case FS_SUSTAINER:
            if (xacc < cfg.burnout_acceleration_threshold) {
                count1++;
            } else {
                count1 = 0;
            }

            if (count1 >= cfg.burnout_tick_count) {
                flight_state = FS_COAST_SUSTAINER;
            }
            break;

        case FS_COAST_SUSTAINER:
            if (APPO_COND) {
                count1++;
            } else {
                count1 = 0;
            }

            if (count1 >= cfg.apogee_tick_count) {
                deploy(cfg.Appo_Channel);
                printf("Deploy appo\n");
                flight_state = FS_UNDER_DROGUES;
            }
            break;

        case FS_UNDER_DROGUES:
            if (average_barometric_velocity < cfg.panic_velocity_threshold) {
                count1++;
            } else {
                count1 = 0;
            }

            if (barometric_agl < cfg.main_deployment_altitude) {
                count2++;
            } else {
                count2 = 0;
            }

            if (count1 >= 50 || count2 >= cfg.main_deployment_tick_count) {
                deploy(cfg.Mains_Channel);
                printf("Deploy Mains\n");
                flight_state = FS_UNDER_MAINS;
            }
            break;

        case FS_UNDER_MAINS:
            if (barometric_agl < cfg.highest_ground_elevation && fabs(average_barometric_velocity) < cfg.landed_velocity_threshold) {
                count1++;
            } else {
                count1 = 0;
            }

            // for 6 seconds
            if (count1 >= cfg.landed_tick_count) {
                flight_state = FS_LANDED;
            }
            break;

        case FS_LANDED:
            break;

        default: break;
    }

    // reset the counters if we changed flight states
    // the next state needs to have the counters at zero
    if (flight_state != pre) {
        count1 = 0;
        count2 = 0;

        printf("Changing flight state, now: %s\n", get_flight_state_name());
    }

    return flight_state != pre;

}

uint8_t get_flight_state(void) {
    return flight_state;
}

const char* get_flight_state_name(void) {
    switch (flight_state) {
    case FS_ON_PAD: return "FS_ON_PAD";
    case FS_BOOSTER: return "FS_BOOSTER";
    case FS_COAST_BOOSTER: return "FS_COAST_BOOSTER";
    case FS_SUSTAINER: return "FS_SUSTAINER";
    case FS_COAST_SUSTAINER: return "FS_COAST_SUSTAINER";
    case FS_UNDER_DROGUES: return "FS_UNDER_DROGUES";
    case FS_UNDER_MAINS: return "FS_UNDER_MAINS";
    case FS_LANDED: return "FS_LANDED";
    case FS_PREFLIGHT: return "FS_PREFLIGHT";
    }
    return "UNKNOWN";
}

void print_flight_config(){
    flight_config_t cfg = flight_config;
    printf("\n=================   Flight Config   ====================\n");
    printf("-> lift_off_acceleration_threshold: %f\n", cfg.lift_off_acceleration_threshold);
    printf("-> lift_off_tick_count: %d\n", cfg.lift_off_tick_count);
    printf("-> apogee_acceleration_threshold: %f\n", cfg.apogee_acceleration_threshold);
    printf("-> apogee_tick_count: %d\n", cfg.apogee_tick_count);
    printf("-> burnout_acceleration_threshold: %f\n", cfg.burnout_acceleration_threshold);
    printf("-> burnout_tick_count: %d\n", cfg.burnout_tick_count);
    printf("-> recovery_burnout_counter: %d\n", cfg.recovery_burnout_counter);
    printf("-> arm_at_boot: %d\n", cfg.arm_at_boot);
    printf("-> Appo_Channel: %d\n", cfg.Appo_Channel);
    printf("-> Mains_Channel: %d\n", cfg.Mains_Channel);
    printf("-> Separation_Channel: %d\n", cfg.Separation_Channel);
    printf("-> Ignition_Channel: %d\n", cfg.Ignition_Channel);
    printf("-> Aux_1_Channel: %d\n", cfg.Aux_1_Channel);
    printf("-> Aux_2_Channel: %d\n", cfg.Aux_2_Channel);
    printf("-> Aux_3_Channel: %d\n", cfg.Aux_3_Channel);
    printf("-> Aux_4_Channel: %d\n", cfg.Aux_4_Channel);
    printf("-> panic_velocity_threshold: %f\n", cfg.panic_velocity_threshold);
    printf("-> main_deployment_altitude: %f\n", cfg.main_deployment_altitude);
    printf("-> main_deployment_tick_count: %d\n", cfg.main_deployment_tick_count);
    printf("-> highest_ground_elevation: %f\n", cfg.highest_ground_elevation);
    printf("-> landed_velocity_threshold: %f\n", cfg.landed_velocity_threshold);
    printf("-> landed_tick_count: %d\n", cfg.landed_tick_count);
    printf("==================================================\n\n");
}