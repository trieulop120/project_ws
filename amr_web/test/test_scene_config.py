import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from fastapi import HTTPException

from amr_web import web_server


class SceneConfigTest(unittest.TestCase):
    def test_map_config_precedence_missing_and_path_scope(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            maps = root / "maps"
            maps.mkdir()
            packaged = root / "resources" / "web" / "semantic_maps"
            packaged.mkdir(parents=True)
            configured = "Warehouse A.2"
            empty = "Kho B"
            for map_id in [configured, empty]:
                (maps / f"{map_id}.yaml").touch()

            local = maps / configured / "semantic_scene.json"
            local.parent.mkdir()
            fallback = packaged / f"{configured}.json"
            fallback.touch()

            with patch.object(web_server, "get_package_share_directory", return_value=str(root)):
                app = web_server.create_app(None, None)
            get_scene = next(route.endpoint for route in app.routes if route.path == "/api/map/scene/{map_id}")
            with patch.object(Path, "expanduser", return_value=maps):
                self.assertEqual(get_scene(configured).path, fallback)
                local.touch()
                self.assertEqual(get_scene(configured).path, local)
                for map_id, status in [(empty, 404), ("missing", 404), ("../x", 400)]:
                    with self.assertRaises(HTTPException) as error:
                        get_scene(map_id)
                    self.assertEqual(error.exception.status_code, status)
                local.unlink()
                local.symlink_to(root / "private.json")
                with self.assertRaises(HTTPException) as error:
                    get_scene(configured)
                self.assertEqual(error.exception.status_code, 400)


if __name__ == "__main__":
    unittest.main()
