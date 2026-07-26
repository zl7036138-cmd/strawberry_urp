# Dataset workflow

All commands below use only the Python standard library. The 1.5 GB archive is
not stored in Git and is never trusted until its pinned MD5 succeeds.

From the repository root:

```powershell
python tools/data/download_dataset.py --extract-dir data/raw/zenodo_6126677/extracted
python tools/data/verify_dataset.py
python tools/data/split_dataset.py \
  --resplit-all \
  --groups-csv data/manifests/zenodo_6126677_groups.csv \
  --exclusions-csv data/manifests/zenodo_6126677_exclusions.csv \
  --image-mode copy
python tools/data/split_dataset.py --verify-only
```

The formal v1 dataset combines the source's two directories and applies the
audited exclusions plus the fixed `20260710` grouped 70/15/15 split.  The
source directories cannot be treated as independent because the audit found
near-identical scenes crossing them.  The authoritative `image,group_id` CSV
keeps every related scene in one split.

Any split derived by this repository is fail-closed on scene identity. A
versioned `groups.csv` is required unless every source filename carries an
explicit non-empty scene prefix and frame token, for example
`greenhouse_a_frame0001.jpg`. Numeric names, generic `image_0001` names, and
arbitrary identifiers do not prove that samples are independent and are
rejected without `groups.csv`. An official three-way split may be kept as-is;
whenever an official training subset must be divided, the same grouping rule
applies. The CSV must assign every discovered image and has this format:

```csv
image,group_id
training/000001.jpg,recording_001
training/000002.jpg,recording_001
```

The processed output is a YOLO-standard `images/` + `labels/` view. Images are
isolated copies by default. Some source JPEGs omit the final EOI marker;
materialization appends only that two-byte marker to the processed copy because
Ultralytics would otherwise rewrite the file. The raw extraction remains byte
identical to the verified ZIP. Generated labels contain only class 0 (`ripe`)
and class 1 (`unripe`); the original class 2 (`peduncle`) annotations remain
untouched under `data/raw`. `--image-mode hardlink` is retained only for
diagnostics; incomplete JPEGs are still copied and normalized in that mode.

`split_manifest.json` records SHA-256 and byte size for every source image,
source label, materialized image, and materialized label. It also contains a
canonical digest for each split and for the complete dataset. Canonical digests
include relative paths, scene assignments, split assignments, filtered label
content, image content, declared JPEG normalization, seed, and class mapping.
They deliberately exclude the
informational absolute `source_root` and `output_root`, so an intact processed
dataset has the same identity after moving to another machine or directory.

Always verify the manifest before reading data for training, validation, or the
one-time test run. `--verify-only` re-hashes every materialized file and exits 2
with a machine-readable report on failure. It also checks that `dataset.yaml`
points exactly to `images/train`, `images/val`, and `images/test`, then compares
every supported image and every split `labels/*.txt` file in those directories
against the manifest. Thus an extra image, an extra label, a wrong-suffix input,
or a nested input directory cannot silently enter an Ultralytics run. Normal
`labels/train.cache`, `labels/val.cache`, and `labels/test.cache` metadata files
are ignored because they are not model inputs. Verification also detects
changes made through either side of a hard link. Python callers use the stable
API:

```python
from dataset_tools import verify_materialized_split_manifest

report = verify_materialized_split_manifest(path_to_split_manifest)
dataset_digest = report["canonical_dataset_sha256"]
```

Unit tests:

```powershell
python -m unittest discover -s tools/data/tests -p "test_*.py" -v
```
