from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]


class DynamicAttachmentCapacityTests(unittest.TestCase):
    def test_xacro_supports_the_full_generalized_scene_capacity(self):
        source = (ROOT / "urdf" / "panda_gz.urdf.xacro").read_text(encoding="utf-8")
        dynamic = (ROOT / "urdf" / "dynamic_attachments.xacro").read_text(encoding="utf-8")
        self.assertIn('name="fruit_attachment_count" default="3"', source)
        for target_id in range(4, 10):
            self.assertIn(f'target_id="{target_id}"', source)
        self.assertIn("strawberry_${target_id}", dynamic)
        self.assertIn("strawberry_stem_attachment", dynamic)
        self.assertIn("panda_link0", dynamic)
        self.assertIn("stem_attach", dynamic)
        self.assertIn("stem_detach", dynamic)

    def test_launch_derives_attachment_count_from_the_scene_manifest(self):
        source = (ROOT / "launch" / "sim.launch.py").read_text(encoding="utf-8")
        self.assertIn('fruits = scene_document.get("fruits")', source)
        self.assertIn("not 1 <= len(fruits) <= 9", source)
        self.assertIn('"fruit_attachment_count": str(fruit_attachment_count)', source)
        self.assertIn("stem_constraints_enabled=generalized_scene", source)
        self.assertIn('"enable_stem_attachment": "true" if generalized_scene and attachment_backend != "lazy" else "false"', source)
        self.assertIn("add_external_stem_support(external_world.getroot(), fruit_attachment_count)", source)
        self.assertIn('"stem_constraints_enabled": generalized_scene', source)
        self.assertIn('scene_document.get("generator")', source)
        self.assertIn(
            "150 if generalized_scene or fruit_attachment_count > 3 else 10",
            source,
        )

    def test_attachment_manager_emits_scoring_only_physical_lifecycle_events(self):
        source = (ROOT / "strawberry_sim" / "attachment_manager.py").read_text(
            encoding="utf-8"
        )

        self.assertIn('"/strawberry/ground_truth/harvest_events"', source)
        self.assertIn('"SIMULATION_GROUND_TRUTH_SCORING_ONLY"', source)
        self.assertIn('_publish_score_event("CONTACT_RESOLVED"', source)
        self.assertIn('_publish_score_event("PLACED"', source)


if __name__ == "__main__":
    unittest.main()
