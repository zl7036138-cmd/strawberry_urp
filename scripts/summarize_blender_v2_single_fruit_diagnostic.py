#!/usr/bin/env python3
"""Summarize the three Blender-v2 single-fruit Shadow windows."""
import argparse, hashlib, json
from pathlib import Path

EXPECTED = {"strawberry_1": ("RIPE", 1), "strawberry_3": ("RIPE", 3), "strawberry_2": ("UNRIPE", 2)}
def sha(path): return hashlib.sha256(path.read_bytes()).hexdigest()
def main():
    p=argparse.ArgumentParser(); p.add_argument("--output-dir", type=Path, required=True); o=p.parse_args()
    records=[]
    for model,(maturity,identity) in EXPECTED.items():
        d=o.output_dir/model; wp=d/"shadow_window.json"; rp=d/"scene_receipt.json"
        w=json.loads(wp.read_text(encoding="utf-8")); r=json.loads(rp.read_text(encoding="utf-8"))
        if (r["visible_models"] != [model] or not r["truth_streams_retained"]
                or w["required_frames"] != 60 or w["warmup_frames_discarded"] != 10):
            raise ValueError(f"invalid isolation evidence for {model}")
        key="ripe_detection_count" if maturity=="RIPE" else "unripe_detection_count"
        correct=sum(int(f[key])>0 for f in w["frames"])
        pose=sum(identity in f["target_pose_ids"] for f in w["frames"])
        maturity_value=1 if maturity=="RIPE" else 2
        areas=[int(x["area_px"]) for f in w["frames"] for x in f["detection_rois"] if int(x["maturity"])==maturity_value]
        dr=correct/60; pr=pose/60
        diagnosis=("DEPTH_OR_TF_LOCALIZATION" if dr>0 and pr==0 else
                   "CANONICAL_PLANT_OCCLUSION_OR_VIEWPOINT" if model!="strawberry_1" and dr>=.9 else
                   "APPEARANCE_SIZE_POSE_OR_MODEL_DOMAIN" if dr<.1 else "MIXED_OR_INCONCLUSIVE")
        records.append({"target_model":model,"expected_maturity":maturity,"expected_target_id":identity,
          "correct_class_frames":correct,"correct_class_frame_rate":dr,"target_pose_frames":pose,
          "target_pose_frame_rate":pr,"roi_observation_count":len(areas),
          "roi_area_px":{"minimum":min(areas) if areas else None,"maximum":max(areas) if areas else None,
          "mean":sum(areas)/len(areas) if areas else None},
          "leaf_occlusion":{"status":"NOT_DIRECTLY_OBSERVABLE_WITH_CURRENT_RGB_D_TOPICS",
          "proxy":"isolated detection and ROI versus canonical three-fruit window","numeric_ratio":None},
          "diagnosis":diagnosis,"window_sha256":sha(wp),"world_sha256":r["materialized_world"]["sha256"]})
    result={"schema_version":1,"scope":"NON_ACCEPTANCE_NO_MOTION_BLENDER_V2_SINGLE_FRUIT_DIAGNOSTIC",
      "formal_acceptance":False,"held_out_test_consumed":False,"robot_motion_started":False,
      "training_started":False,"model_threshold_camera_lighting_fixed":True,
      "warmup_frames_per_scenario":10,"measurement_frames_per_scenario":60,"scenarios":records,
      "overall_diagnoses":sorted({x["diagnosis"] for x in records}),
      "interpretation_boundary":"A numeric leaf-occlusion ratio requires segmentation or a renderer visibility pass; RGB-D and detector boxes alone do not identify leaf pixels."}
    (o.output_dir/"summary.json").write_text(json.dumps(result,indent=2,sort_keys=True)+"\n",encoding="utf-8")
    print(json.dumps(result,indent=2,sort_keys=True)); return 0
if __name__=="__main__": raise SystemExit(main())
