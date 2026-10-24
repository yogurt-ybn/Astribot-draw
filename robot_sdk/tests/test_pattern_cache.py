import hashlib
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from astribot_draw_sdk.contracts import Trajectory
from astribot_draw_sdk.pattern_catalog import select_pattern
from astribot_draw_sdk.pattern_runner import cached_drawing, nearest_source


class PatternCacheTests(unittest.TestCase):
    def test_discards_recorded_approach_and_restores_stationary_exact_canvas_start(self):
        q = np.zeros((5,18)); q[:,0] = [-.8,-.4,.0001,.001,.002]
        v = np.zeros_like(q); v[2,0] = .02
        archived = Trajectory([0,.008,.016,.024,.032],q,velocity=v,
            phase=('approach_both','approach_both','draw_both','draw_both','draw_both'),scope='simulation')
        draw = cached_drawing(archived,dict(time_in_archived_plan_s=.015,position_rad=[0.]*18))
        np.testing.assert_allclose(draw.position[0],0)
        np.testing.assert_allclose(draw.velocity[0],0)
        np.testing.assert_allclose(draw.position[1:],archived.position[2:])
        np.testing.assert_allclose(draw.velocity[1:],archived.velocity[2:])
        np.testing.assert_allclose(draw.time,[0,.001,.009,.017])
        self.assertTrue(all(p=='draw_both' for p in draw.phase))
        with self.assertRaisesRegex(ValueError,'does not match'):
            cached_drawing(archived,dict(time_in_archived_plan_s=.001,position_rad=[0.]*18))
        with self.assertRaisesRegex(ValueError,'does not match'):
            cached_drawing(archived,dict(time_in_archived_plan_s=.015,position_rad=[1.]*18))

    def test_source_projection_uses_segments_not_just_vertices(self):
        source=np.array([[0.,0,0],[1.,0,0],[1.,1,0]])
        np.testing.assert_allclose(nearest_source(np.array([.4,.02,0]),source),[.4,0,0])

    def test_zero_based_numbering_and_hash_checks_precede_sdk_access(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);records={}
            for n in list(range(1,20))+list(range(21,24))+list(range(25,61)):
                identity=f'{n:03d}-test';folder=root/identity;folder.mkdir()
                assets={}
                for name in ('source.svg','sdk-plan.npz','sdk-plan.validation.json'):
                    blob=f'{identity}/{name}'.encode();(folder/name).write_bytes(blob)
                    assets[name]=dict(path=identity+'/'+name,sha256=hashlib.sha256(blob).hexdigest())
                records[identity]=dict(pattern_id=identity,number=f'{n:03d}',assets=assets)
            (root/'pattern_lookup.json').write_text(json.dumps(dict(count=58,patterns=dict(reversed(list(records.items()))))))
            self.assertEqual(select_pattern(root,0)[1]['number'],'001')
            self.assertEqual(select_pattern(root,19)[1]['number'],'021')
            self.assertEqual(select_pattern(root,57)[1]['number'],'060')
            for bad in (-1,58,True,1.5):
                with self.assertRaises(ValueError):select_pattern(root,bad)
            (root/'001-test/source.svg').write_text('changed')
            with self.assertRaisesRegex(ValueError,'SHA256'):select_pattern(root,0)


if __name__=='__main__': unittest.main()
