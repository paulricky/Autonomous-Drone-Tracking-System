#!/usr/bin/env python3
"""Measure the Mac landmark reference relative to the commissioned drone map axes."""
import argparse
import json
from pathlib import Path
import sys
import time
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src/robotics_tools'))
import numpy as np
from robotics_tools.hand_tracking.tracker import LatestFrameCamera, MediaPipeHandTracker


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--frame-id', required=True, help='same reference identifier commissioned on the Pi')
    parser.add_argument('--output', default='calibration_data/hand/pointing_reference.json')
    parser.add_argument('--camera', type=int, default=0)
    args=parser.parse_args()
    import cv2
    camera=LatestFrameCamera(args.camera)
    tracker=MediaPipeHandTracker()
    observations={}
    try:
        while True:
            image,captured=camera.latest()
            if image is None:
                time.sleep(.01)
                continue
            hand=tracker.process(image,captured)
            axis=None
            if hand.detected:
                scale=np.array([image.shape[1]/image.shape[0],1,image.shape[1]/image.shape[0]])
                p=hand.landmarks[[5,6,7,8]]*scale
                delta=np.diff(p,axis=0);length=np.linalg.norm(delta,axis=1)
                if np.min(length)>1e-5:
                    segment=delta/length[:,None]
                    if np.min(segment@segment[0])>.75:
                        _,_,vh=np.linalg.svd(p-p.mean(axis=0));axis=vh[0]
                        axis*=1 if axis@(p[-1]-p[0])>0 else -1
            overlay=tracker.draw(image,hand,str(axis))
            cv2.putText(overlay,'Point along measured map +X/+Y/+Z, press 1/2/3; W save; Q quit',(10,125),cv2.FONT_HERSHEY_SIMPLEX,.45,(0,255,255),1)
            cv2.putText(overlay,'Captured: '+str(sorted(observations)),(10,150),cv2.FONT_HERSHEY_SIMPLEX,.5,(0,255,255),1)
            cv2.imshow('Pointing reference calibration',overlay)
            key=cv2.waitKey(10)&255
            if key in (ord('1'),ord('2'),ord('3')) and axis is not None:
                observations[key-ord('1')]=axis.copy()
            if key==ord('q'):
                break
            if key==ord('w') and len(observations)==3:
                observed=np.stack([observations[i] for i in range(3)],axis=1)
                u,_,vh=np.linalg.svd(observed.T)
                correction=np.eye(3);correction[2,2]=np.linalg.det(u@vh)
                rotation=u@correction@vh
                error=np.arccos(np.clip(np.diag(rotation@observed),-1,1))
                if np.max(error)>np.deg2rad(15):
                    print('Calibration residual exceeds 15 degrees; repeat the measured directions.',flush=True)
                    continue
                path=Path(args.output);path.parent.mkdir(parents=True,exist_ok=True)
                path.write_text(json.dumps(dict(frame_id=args.frame_id,R_wc=rotation.tolist(),landmark_axis_scale=scale.tolist(),max_residual_rad=float(max(error)),date=time.strftime('%Y-%m-%d')),indent=2))
                print('Saved',path)
                break
    finally:
        tracker.close();camera.close();cv2.destroyAllWindows()


if __name__=='__main__':
    main()
