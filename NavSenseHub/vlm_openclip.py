#!/usr/bin/env python3
"""
NavSense VLM Stage: OpenCLIP interactive loop.
Embeds every 10th frame from the session and opens a GUI for text queries.
Type 'close' in the box or close the window to exit.
"""
import os
import sys
import argparse
import torch
import open_clip
import cv2
import numpy as np
import tkinter as tk
from PIL import Image
from pathlib import Path

MODEL_NAME = "ViT-B-32"
PRETRAINED = "openai"
TARGET_H = 400
STRIDE = 10


def get_image_dir(session_dir: Path):
    d = session_dir / "mav0" / "cam0" / "data"
    return d if d.is_dir() else session_dir


def run_vlm(session_dir: Path):
    img_dir = get_image_dir(session_dir)
    if not img_dir.is_dir():
        print(f"ERROR: No image folder found at {img_dir}")
        return 1

    all_files = sorted([f for f in os.listdir(img_dir) if f.lower().endswith((".png", ".jpg", ".jpeg"))])
    if not all_files:
        print(f"ERROR: No images in {img_dir}")
        return 1

    images = all_files[::STRIDE]
    print(f"Total frames: {len(all_files)}, Selected every {STRIDE}th: {len(images)}")

    print(f"Loading {MODEL_NAME}...")
    model, _, preprocess = open_clip.create_model_and_transforms(MODEL_NAME, pretrained=PRETRAINED)
    tokenizer = open_clip.get_tokenizer(MODEL_NAME)
    model.eval()
    device = "cuda" if torch.cuda.is_available() else "cpu"
    model = model.to(device)

    print("Embedding images...")
    image_features = []
    valid_images = []
    with torch.no_grad():
        for fname in images:
            path = img_dir / fname
            try:
                img = preprocess(Image.open(path).convert("RGB")).unsqueeze(0).to(device)
                feat = model.encode_image(img)
                feat /= feat.norm(dim=-1, keepdim=True)
                image_features.append(feat.cpu())
                valid_images.append(fname)
                print(f"  {fname}")
            except Exception as e:
                print(f"  skip {fname}: {e}")

    if not valid_images:
        print("ERROR: No images could be embedded.")
        return 1

    image_features = torch.cat(image_features)
    print(f"Embedded {len(valid_images)} frames. Opening query window...")

    # ------------------------------------------------------------------ GUI
    root = tk.Tk()
    root.title(f"NavSense VLM — {session_dir.name}")
    root.geometry("520x260")

    tk.Label(root, text="Query (type 'close' to exit):").pack(pady=(10, 2))
    entry = tk.Entry(root, width=56)
    entry.pack(pady=4)
    entry.focus_set()

    out_text = tk.Text(root, height=9, width=66)
    out_text.pack(pady=6)

    def do_query(event=None):
        query = entry.get().strip()
        if not query:
            return
        if query.lower() == "close":
            root.destroy()
            return

        text = tokenizer([query]).to(device)
        with torch.no_grad():
            text_feat = model.encode_text(text)
            text_feat /= text_feat.norm(dim=-1, keepdim=True)

        sims = (image_features @ text_feat.cpu().T).squeeze()
        if sims.dim() == 0:
            sims = sims.unsqueeze(0)
        top_idx = sims.topk(min(20, len(sims))).indices.tolist()

        panels = []
        out_text.delete(1.0, tk.END)
        out_text.insert(tk.END, f"Top matches for: '{query}'\n")
        for idx in top_idx:
            if len(panels) == 6:
                break
            path = img_dir / valid_images[idx]
            img = cv2.imread(str(path))
            if img is None:
                continue
            score = sims[idx].item()
            h, w = img.shape[:2]
            new_w = int(w * TARGET_H / h)
            img = cv2.resize(img, (new_w, TARGET_H))
            cv2.putText(img, f"{score:.3f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
            panels.append(img)
            out_text.insert(tk.END, f"  {valid_images[idx]} ({score:.3f})\n")

        if not panels:
            return

        max_w = max(p.shape[1] for p in panels)
        panels = [np.pad(p, ((0, 0), (0, max_w - p.shape[1]), (0, 0))) for p in panels]
        mid = len(panels) // 2 or 1
        row1 = np.hstack(panels[:mid])
        row2 = np.hstack(panels[mid:]) if len(panels) > mid else np.zeros_like(row1)
        if row2.shape[1] != row1.shape[1]:
            row2 = np.pad(row2, ((0, 0), (0, row1.shape[1] - row2.shape[1]), (0, 0)))
        combined = np.vstack([row1, row2])

        cv2.imshow(f"Top matches: {query}", combined)
        cv2.waitKey(1)   # keep the window open; it updates on the next query

    entry.bind("<Return>", do_query)
    tk.Button(root, text="Search", command=do_query).pack(pady=4)
    tk.Button(root, text="Exit", command=root.destroy).pack(pady=2)

    root.mainloop()
    cv2.destroyAllWindows()
    print("VLM closed.")
    return 0


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--session", required=True, type=Path)
    args = ap.parse_args()
    sys.exit(run_vlm(args.session.resolve()))