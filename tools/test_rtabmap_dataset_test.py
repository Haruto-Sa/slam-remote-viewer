import importlib.util
from pathlib import Path
import tempfile
import unittest

spec = importlib.util.spec_from_file_location("rtabmap_dataset", Path(__file__).with_name("test-rtabmap-dataset.py"))
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class AssociationTests(unittest.TestCase):
    def test_nearest_depth_is_unique_and_ordered(self):
        rgb = [(1.0, "a"), (1.001, "b"), (2.0, "c"), (5.0, "d")]
        depth = [(0.999, "x"), (2.001, "y"), (8.0, "z")]
        self.assertEqual(module.associate(rgb, depth),
                         [(1.0, "a", 0.999, "x"), (2.0, "c", 2.001, "y")])

    def test_empty_input_and_no_matches(self):
        self.assertEqual(module.associate([], []), [])
        self.assertEqual(module.associate([(1, "a")], []), [])
        self.assertEqual(module.associate([(1, "a")], [(2, "b")]), [])

    def test_index_validates_order_finiteness_and_files(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary).resolve()
            (root / "a.png").touch()
            index = root / "rgb.txt"
            index.write_text("# comment\n1 a.png\n")
            self.assertEqual(module.read_index(index), [(1, root / "a.png")])
            for contents in ("nan a.png\n", "1 a.png\n1 a.png\n", "1 missing.png\n", "1 a.png extra\n"):
                index.write_text(contents)
                with self.assertRaises(ValueError):
                    module.read_index(index)


if __name__ == "__main__":
    unittest.main()
