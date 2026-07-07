from pathlib import Path

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.const import CONF_ID

CODEOWNERS = ["@teemow"]
DEPENDENCIES = ["esp32"]

plan_bridge_ns = cg.esphome_ns.namespace("plan_bridge")
PlanBridge = plan_bridge_ns.class_("PlanBridge", cg.Component)

CONF_TX_PIN = "tx_pin"
CONF_RX_PIN = "rx_pin"
CONF_DE_PIN = "de_pin"
CONF_BAUD_RATE = "baud_rate"
CONF_UART_NUM = "uart_num"
CONF_STOP_BITS = "stop_bits"
CONF_GAP_MS = "gap_ms"
CONF_REPEAT = "repeat"
CONF_REPEAT_INTERVAL_MS = "repeat_interval_ms"
CONF_ARMED = "armed"
CONF_ENROLL = "enroll"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(PlanBridge),
        # RX is used to detect an idle transmit slot; DE (RTS) + TX drive the bus.
        cv.Required(CONF_RX_PIN): pins.internal_gpio_input_pin_number,
        cv.Required(CONF_TX_PIN): pins.internal_gpio_output_pin_number,
        # MAX3485 DE+RE tied to one GPIO, driven directly by the RX ISR
        # around each 9-bit transmit.
        cv.Required(CONF_DE_PIN): pins.internal_gpio_output_pin_number,
        cv.Optional(CONF_BAUD_RATE, default=62500): cv.int_range(min=1200, max=1000000),
        cv.Optional(CONF_UART_NUM, default=1): cv.int_range(min=0, max=2),
        # Ignored: the pLAN wire format is 8 data + 9th address bit, which the
        # component handles internally (RX as 8E1, TX via per-byte parity).
        cv.Optional(CONF_STOP_BITS, default=2): cv.one_of(1, 2, int=True),
        # Stream-drain tick (ms): the capture-record boundary.
        cv.Optional(CONF_GAP_MS, default=3): cv.int_range(min=1, max=1000),
        # Keypad reports sent per single press. A real pGD tap is exactly ONE
        # report (hold counter 0x01); >1 emulates holding the key.
        cv.Optional(CONF_REPEAT, default=1): cv.int_range(min=1, max=20),
        cv.Optional(CONF_REPEAT_INTERVAL_MS, default=60): cv.int_range(min=1, max=1000),
        # Compile-time default for the write-enable gate. Leave false and arm at
        # runtime (e.g. a template switch calling set_armed) so a reboot never
        # comes up able to move the heat pump.
        cv.Optional(CONF_ARMED, default=False): cv.boolean,
        # Join the pLAN as a second terminal (address 31) at boot: answer the
        # controller's roll-call and its polls. Requires 31 in the uPC's
        # terminal list (P:01 config screen). Enrollment itself is passive
        # w.r.t. the heat pump -- key injection stays gated behind `armed`.
        cv.Optional(CONF_ENROLL, default=False): cv.boolean,
    }
).extend(cv.COMPONENT_SCHEMA)


def _require_esp_idf(config):
    # The component uses the esp-idf UART driver + RS485 mode directly; arduino
    # won't link.
    from esphome.core import CORE

    framework = getattr(CORE, "target_framework", None)
    is_idf = (framework == "esp-idf") if framework is not None else getattr(
        CORE, "using_esp_idf", False
    )
    if not is_idf:
        raise cv.Invalid("plan_bridge requires the esp-idf framework")
    return config


FINAL_VALIDATE_SCHEMA = _require_esp_idf


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    # The pure pLAN protocol logic (frame grammar, terminal state machine,
    # screen-snapshot cache) is the planterm library -- and this component
    # ships inside the planterm repo, so the headers come from this very
    # checkout: component and library version-lock by construction. Consumers
    # pick both up with one `external_components` ref. Components building on
    # top of this one (stream listener / press_key_internal consumers) must
    # NOT add their own planterm pin -- one shared library instance per build.
    repo_root = Path(__file__).resolve().parents[3]
    cg.add_library("planterm", None, f"symlink://{repo_root}")
    cg.add(var.set_rx_pin(config[CONF_RX_PIN]))
    cg.add(var.set_tx_pin(config[CONF_TX_PIN]))
    cg.add(var.set_de_pin(config[CONF_DE_PIN]))
    cg.add(var.set_baud_rate(config[CONF_BAUD_RATE]))
    cg.add(var.set_uart_num(config[CONF_UART_NUM]))
    cg.add(var.set_stop_bits(config[CONF_STOP_BITS]))
    cg.add(var.set_gap_ms(config[CONF_GAP_MS]))
    cg.add(var.set_repeat(config[CONF_REPEAT]))
    cg.add(var.set_repeat_interval_ms(config[CONF_REPEAT_INTERVAL_MS]))
    cg.add(var.set_armed(config[CONF_ARMED]))
    cg.add(var.set_enroll(config[CONF_ENROLL]))
