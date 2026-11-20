"""Checks for the simple base-frame Z max/min height measurement."""
import unittest
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
from amr_perception.pallet_detector_node import PalletDetectorNode as Detector


def det(x0,y0,x1,y1,name='box'):
    points=np.array([[x0,y0],[x1,y0],[x1,y1],[x0,y1]],np.float32)
    return {'class':name,'box':points,'mask':points.copy(),'center':points.mean(axis=0)}


class BoxZRangeTest(unittest.TestCase):
    def setUp(self):
        R=np.array([[0.,0.,1.],[-1.,0.,0.],[0.,-1.,0.]])
        self.R,self.t=R,np.array([0.,0.,1.])
        self.node=SimpleNamespace(
            fx=400.,fy=400.,cx=80.,cy=80.,MIN_DEPTH_M=.1,MAX_DEPTH_M=10.,
            box_height_min_pixels=30,use_box_height_class=True,
            box_height_threshold_m=.32,box_pallet_margin_px=15,
            camera_frame_param='camera',base_frame='base_footprint',
            lookup_rt=Mock(return_value=(self.R,self.t)))
        self.node._box_z_limits=lambda *args:Detector._box_z_limits(self.node,*args)
        self.node.is_pallet_loaded=lambda *args:Detector.is_pallet_loaded(self.node,*args)
        self.box=det(40,20,100,100)
        self.pallet=det(30,110,110,120,'pallet')
        self.depth=np.full((160,240),2000,np.uint16)
        self.header=SimpleNamespace(frame_id='camera')

    def measure(self,boxes=None,pallets=None):
        Detector.measure_box_heights(self.node,self.pallet,
            [self.box] if boxes is None else boxes,self.depth,'16UC1',self.header,pallets)

    def test_legacy_top_band_and_bottom_percentiles_in_base_frame(self):
        low,high=self.node._box_z_limits(self.box,self.depth,'16UC1',self.R,self.t)
        self.assertAlmostEqual(low,.92)
        self.assertAlmostEqual(high,1.26)
        self.measure()
        self.assertAlmostEqual(self.pallet['max_box_height_m'],.34)
        self.assertEqual(self.pallet['max_box_class'],'tall')

    def test_float_depth_and_detect_only_polygon(self):
        del self.box['mask']
        low,high=self.node._box_z_limits(self.box,self.depth/1000.,'32FC1',self.R,self.t)
        self.assertAlmostEqual(high-low,.34)

    def test_outlier_does_not_set_height_extreme(self):
        self.depth[30,60]=4000
        self.measure()
        self.assertAlmostEqual(self.pallet['max_box_height_m'],.34)

    def test_invalid_depth_is_excluded(self):
        depth=self.depth.astype(float)/1000
        depth[30,60]=np.nan
        depth[31,60]=np.inf
        depth[32,60]=0
        low,high=self.node._box_z_limits(self.box,depth,'32FC1',self.R,self.t)
        self.assertAlmostEqual(high-low,.34)

    def test_stacked_boxes_use_combined_z_range(self):
        boxes=[det(40,60,100,100),det(40,20,100,60)]
        self.measure(boxes)
        self.assertAlmostEqual(self.pallet['max_box_height_m'],.37)
        self.assertTrue(all(b['height_class']=='tall' for b in boxes))

    def test_other_pallet_does_not_contribute(self):
        other=det(150,110,230,120,'pallet')
        far_box=det(160,5,220,100)
        self.measure([self.box,far_box],[self.pallet,other])
        self.assertAlmostEqual(self.pallet['max_box_height_m'],.34)

    def test_medium_threshold_and_no_stale_height(self):
        self.depth[:]=1000
        self.measure()
        self.assertGreater(self.pallet['max_box_height_m'],.10)
        self.assertLess(self.pallet['max_box_height_m'],.20)
        self.assertEqual(self.pallet['max_box_class'],'medium')
        self.depth[:]=0
        self.measure()
        self.assertNotIn('max_box_height_m',self.pallet)

    def test_no_tf_does_not_create_a_height(self):
        self.node.lookup_rt.return_value=None
        self.measure()
        self.assertNotIn('max_box_height_m',self.pallet)

    def test_mask_outline_remains_unfilled(self):
        img=np.full((160,240,3),50,np.uint8)
        Detector._draw_detection_outline(img,self.box,(255,0,0))
        np.testing.assert_array_equal(img[50,60],[50,50,50])
        self.assertGreater(int(img[20,60,0]),50)


if __name__=='__main__':
    unittest.main()
