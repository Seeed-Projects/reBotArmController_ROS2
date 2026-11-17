"""Rendering-only URDF regression checks; no ROS nodes or motor I/O."""
import importlib.util
from collections import Counter
from pathlib import Path
import subprocess
import sys
import unittest
from xml.etree import ElementTree as ET

PACKAGE = Path(__file__).resolve().parents[1]
TOOL = PACKAGE / 'launch' / 'rviz_urdf_compat.py'
spec = importlib.util.spec_from_file_location('rviz_urdf_compat', TOOL)
compat = importlib.util.module_from_spec(spec)
spec.loader.exec_module(compat)


def semantic(element):
    if element is None:
        return None
    return (element.tag, tuple(sorted(element.attrib.items())), (element.text or '').strip(), tuple(semantic(child) for child in element))


class RvizUrdfCompatibilityTests(unittest.TestCase):
    def test_models_preserve_motion_collision_inertia_and_visual_poses(self):
        for filename in ('RS/urdf/ReBot_Arm_RS.urdf', 'DM/urdf/ReBot_Arm_DM.urdf'):
            with self.subTest(model=filename):
                xml = (PACKAGE / 'description' / filename).read_text()
                before = ET.fromstring(xml)
                after = ET.fromstring(compat.make_rviz_compatible(xml))
                original_links = {link.get('name'): link for link in before.findall('link')}
                result_links = {link.get('name'): link for link in after.findall('link')}
                result_joints = {joint.get('name'): joint for joint in after.findall('joint')}
                self.assertEqual(len(result_links), len(after.findall('link')))
                self.assertEqual(len(result_joints), len(after.findall('joint')))
                for joint in before.findall('joint'):
                    self.assertEqual(semantic(joint), semantic(result_joints[joint.get('name')]))
                extras = set(result_links) - set(original_links)
                extra_parents = {}
                for joint in after.findall('joint'):
                    child = joint.find('child').get('link')
                    if child in extras:
                        self.assertEqual(joint.get('type'), 'fixed')
                        self.assertEqual(joint.find('origin').attrib, {'xyz': '0 0 0', 'rpy': '0 0 0'})
                        extra_parents[child] = joint.find('parent').get('link')
                self.assertEqual(set(extra_parents), extras)
                for name, link in original_links.items():
                    current = result_links[name]
                    self.assertEqual(semantic(link.find('inertial')), semantic(current.find('inertial')))
                    self.assertEqual([semantic(c) for c in link.findall('collision')], [semantic(c) for c in current.findall('collision')])
                    visuals = list(current.findall('visual'))
                    for child, parent in extra_parents.items():
                        if parent == name:
                            visuals.extend(result_links[child].findall('visual'))
                    self.assertEqual(Counter(semantic(v) for v in link.findall('visual')), Counter(semantic(v) for v in visuals))
                for link in after.findall('link'):
                    self.assertLessEqual(len(link.findall('visual')), 1)
                    if link.get('name') in extras:
                        self.assertIsNone(link.find('collision'))
                        self.assertIsNone(link.find('inertial'))
                self.assertEqual(compat.make_rviz_compatible(ET.tostring(after, encoding='unicode')), ET.tostring(after, encoding='unicode'))

    def test_duplicate_mesh_names_get_unique_fixed_children(self):
        source = '<robot name="test"><link name="body">' + '<visual><geometry><mesh filename="part.stl"/></geometry></visual>' * 3 + '</link></robot>'
        result = ET.fromstring(compat.make_rviz_compatible(source))
        names = [link.get('name') for link in result.findall('link')]
        self.assertEqual(len(names), 3)
        self.assertEqual(len(set(names)), 3)

    def test_command_line_leaves_source_unchanged(self):
        source = PACKAGE / 'description/RS/urdf/ReBot_Arm_RS.urdf'
        before = source.read_bytes()
        output = subprocess.check_output([sys.executable, str(TOOL), str(source)], text=True)
        self.assertEqual(before, source.read_bytes())
        self.assertTrue(all(len(link.findall('visual')) <= 1 for link in ET.fromstring(output).findall('link')))


if __name__ == '__main__':
    unittest.main()
