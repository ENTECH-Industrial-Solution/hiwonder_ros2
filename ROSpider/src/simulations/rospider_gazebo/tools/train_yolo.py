#!/usr/bin/env python3
"""Train a YOLO model for the simulation's detector.

    python3 tools/train_yolo.py --data ~/datasets/cubes --name cubes

The dataset directory can come from tools/capture_dataset.py or from a
hand-labelled export -- Roboflow and labelImg both emit the same YOLO layout,
so there is one code path rather than two. It needs a data.yaml naming the
classes and the train/val image directories.

The result is written to models/yolo/<name>.pt, which is what
config/yolo.yaml points at by default.
"""

import argparse
import os
import shutil
import sys

PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data', required=True,
                        help='dataset directory containing data.yaml')
    parser.add_argument('--name', default='cubes')
    parser.add_argument('--base', default='yolo11n.pt',
                        help='starting weights; n is enough for a handful of '
                             'classes and keeps the committed file small')
    parser.add_argument('--epochs', type=int, default=60)
    parser.add_argument('--imgsz', type=int, default=640)
    parser.add_argument('--batch', type=int, default=16)
    parser.add_argument('--device', default='0',
                        help="'0' for the first GPU, 'cpu' to force CPU")
    args = parser.parse_args()

    try:
        from ultralytics import YOLO
    except ImportError:
        sys.exit('needs ultralytics and torch: pip install --user '
                 '--break-system-packages -r requirements-yolo.txt -- see '
                 'that file for the torch index URL a Blackwell GPU needs')

    data = os.path.join(os.path.expanduser(args.data), 'data.yaml')
    if not os.path.exists(data):
        sys.exit(f'{data} not found; capture_dataset.py writes one, and a '
                 'hand-labelled export needs one written by hand')

    # Run from the dataset directory. ultralytics writes runs/ and downloads
    # its base weights into the current working directory, and this tool is
    # normally invoked from inside the package -- which left runs/, weights/
    # and yolo11n.pt sitting in the source tree. Training artefacts belong
    # with the dataset they came from, not in the repository.
    os.chdir(os.path.dirname(data))
    result = YOLO(args.base).train(
        data=data, epochs=args.epochs, imgsz=args.imgsz, batch=args.batch,
        device=args.device, name=args.name)

    best = os.path.join(result.save_dir, 'weights', 'best.pt')
    out_dir = os.path.join(PKG_ROOT, 'models', 'yolo')
    os.makedirs(out_dir, exist_ok=True)
    out = os.path.join(out_dir, f'{args.name}.pt')
    shutil.copy(best, out)
    print(f'wrote {os.path.relpath(out, PKG_ROOT)}')
    print('point config/yolo.yaml model_path at it, then launch with '
          'detector:=yolo')


if __name__ == '__main__':
    main()
