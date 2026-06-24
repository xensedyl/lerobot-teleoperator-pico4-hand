from lerobot.teleoperators.config import TeleoperatorConfig

from lerobot_teleoperator_pico4_hand import Pico4Hand, Pico4HandConfig


def test_pico4_hand_config_registered():
    cfg = Pico4HandConfig()

    assert cfg.type == "pico4_hand"
    assert TeleoperatorConfig.get_choice_class("pico4_hand") is Pico4HandConfig


def test_pico4_hand_can_be_instantiated():
    teleop = Pico4Hand(Pico4HandConfig())

    assert isinstance(teleop, Pico4Hand)
    assert teleop.name == "pico4_hand"
    assert "l_idx_prox.pos" in teleop.action_features
