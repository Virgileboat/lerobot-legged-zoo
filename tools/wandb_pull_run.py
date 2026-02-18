import os
import argparse
import wandb

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run-path", required=True, help="entity/project/run_id (ex: success-hf/mjlab/42brq2w3)")
    ap.add_argument("--out", default="wandb_runs", help="output directory")
    ap.add_argument("--glob", default=None, help="optional substring filter (ex: '.hydra' or '.pt' or '.onnx')")
    args = ap.parse_args()

    api = wandb.Api()
    run = api.run(args.run_path)

    out_dir = os.path.join(args.out, run.entity, run.project, run.id)
    os.makedirs(out_dir, exist_ok=True)

    print("Downloading files to:", out_dir)
    n = 0
    for f in run.files():
        if args.glob and args.glob not in f.name:
            continue
        # garde l’arborescence
        target_dir = os.path.join(out_dir, os.path.dirname(f.name))
        os.makedirs(target_dir, exist_ok=True)
        f.download(root=out_dir, replace=True)
        n += 1

    print(f"Done. Downloaded {n} files.")
    print("Tip: look for .hydra/config.yaml, model_*.pt, policy.onnx")

if __name__ == "__main__":
    main()
