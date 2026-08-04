from pathlib import Path
from xml.etree import ElementTree


def test_control_odom_relay_respawns_after_an_isolated_exit():
    launch_path = Path(__file__).parents[1] / 'launch' / 'estimation_mux.launch'
    root = ElementTree.parse(str(launch_path)).getroot()
    relays = [
        node for node in root.findall('node')
        if node.attrib.get('name') == 'robot_odom_relay'
    ]

    assert len(relays) == 1
    assert relays[0].attrib.get('respawn') == 'true'
    assert float(relays[0].attrib['respawn_delay']) > 0.0
