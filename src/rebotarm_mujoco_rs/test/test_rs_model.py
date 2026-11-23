from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import numpy as np

from rebotarm_mujoco_rs.mujoco_sync import load_model


class RsModelTest(unittest.TestCase):
    def test_bundled_scene_matches_bringup_urdf(self):
        package = Path(__file__).resolve().parents[1]
        model = load_model(package / "models" / "rs_grasp_scene.xml")
        urdf = ET.parse(
            package.parent / "rebotarm_bringup/description/RS/urdf/ReBot_Arm_RS.urdf"
        )
        for link in urdf.findall("link"):
            with self.subTest(link=link.attrib["name"]):
                inertial = link.find("inertial")
                body = model.body(link.attrib["name"])
                self.assertAlmostEqual(body.mass[0], float(inertial.find("mass").attrib["value"]))
                np.testing.assert_allclose(
                    body.ipos,
                    [float(value) for value in inertial.find("origin").attrib["xyz"].split()],
                    atol=1e-8,
                )
        for name in ("joint7", "joint_left", "joint_right"):
            np.testing.assert_allclose(model.joint(name).range, [0.0, 0.05])
